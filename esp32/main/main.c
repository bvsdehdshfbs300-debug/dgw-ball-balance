/* 平衡球 ESP32-S3 端
 * MaixCAM2 --UDP广播--> 本机: "B,<seq>,<ball_cm>,<target_cm>\n" 或 "N,<seq>\n"
 * 本机 100Hz PD -> 目标摆角 -> Emm42_V5 串口绝对位置指令
 * 断联/丢球 -> 摆杆匀速回平
 *
 * 上电流程: 1) 手动把摆杆放平  2) 上电, 本程序清零编码器位置作为 0 度
 */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "config.h"
#include "emm42.h"
#include "mpu6050.h"

static const char *TAG = "ball";

/* ---------- 共享状态 (udp task 写, control task 读) ---------- */
typedef struct {
    float ball_cm;
    float target_cm;
    int64_t rx_time_us;     /* 最近一次有效球位置的时间 */
    bool have_ball;
} ball_state_t;

static ball_state_t g_state;
static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

/* 球速估计: 位置历史环形缓冲 */
#define HIST_N 16
static struct { int64_t t_us; float cm; } g_hist[HIST_N];
static int g_hist_len = 0;

/* ---------- WiFi ---------- */
static EventGroupHandle_t s_wifi_evt;
#define WIFI_CONNECTED_BIT BIT0
#include "freertos/event_groups.h"

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "wifi disconnected, retry...");
        xEventGroupClearBits(s_wifi_evt, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_evt, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    s_wifi_evt = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    /* 控制链路要低延迟, 关掉 WiFi modem 省电(否则 DTIM 休眠带来几十 ms 抖动) */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(s_wifi_evt, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ---------- UDP 接收 ---------- */
static void udp_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind failed");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "udp listening on %d", UDP_PORT);

    char buf[64];
    while (1) {
        int len = recvfrom(sock, buf, sizeof(buf) - 1, 0, NULL, NULL);
        if (len <= 0) continue;
        buf[len] = 0;
        int64_t now = esp_timer_get_time();

        if (buf[0] == 'B') {
            /* B,<seq>,<ball_cm>,<target_cm> */
            char *save = NULL;
            strtok_r(buf, ",", &save);            /* "B" */
            strtok_r(NULL, ",", &save);           /* seq, 暂不用 */
            char *s_ball = strtok_r(NULL, ",", &save);
            char *s_tgt = strtok_r(NULL, ",\n", &save);
            if (!s_ball || !s_tgt) continue;
            float ball = strtof(s_ball, NULL);
            float tgt = strtof(s_tgt, NULL);

            taskENTER_CRITICAL(&g_lock);
            g_state.ball_cm = ball;
            g_state.target_cm = tgt;
            g_state.rx_time_us = now;
            g_state.have_ball = true;
            /* 历史入队 */
            if (g_hist_len < HIST_N) {
                g_hist[g_hist_len].t_us = now;
                g_hist[g_hist_len].cm = ball;
                g_hist_len++;
            } else {
                memmove(&g_hist[0], &g_hist[1], sizeof(g_hist[0]) * (HIST_N - 1));
                g_hist[HIST_N - 1].t_us = now;
                g_hist[HIST_N - 1].cm = ball;
            }
            taskEXIT_CRITICAL(&g_lock);
        }
        /* 'N'(丢锁) 不立即处理: 按最后位置滑行, 超过 RX_TIMEOUT_MS 自然进 SAFE.
         * 边缘检测丢锁通常几帧内重锁, 立刻回平反而会猛拍把球弹出去 */
    }
}

/* 从历史窗口算球速 cm/s (约 SPEED_WINDOW_MS 基线) */
static float estimate_speed(int64_t now_us)
{
    float speed = 0.0f;
    taskENTER_CRITICAL(&g_lock);
    if (g_hist_len >= 2) {
        int i_old = 0;
        for (int i = g_hist_len - 1; i >= 0; i--) {
            if (now_us - g_hist[i].t_us >= (int64_t)SPEED_WINDOW_MS * 1000) {
                i_old = i;
                break;
            }
        }
        int i_new = g_hist_len - 1;
        float dt_s = (g_hist[i_new].t_us - g_hist[i_old].t_us) / 1e6f;
        if (dt_s > 0.005f)
            speed = (g_hist[i_new].cm - g_hist[i_old].cm) / dt_s;
    }
    taskEXIT_CRITICAL(&g_lock);
    if (speed > MAX_SPEED_CMS) speed = MAX_SPEED_CMS;
    if (speed < -MAX_SPEED_CMS) speed = -MAX_SPEED_CMS;
    return speed;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ---------- 100Hz 控制 ---------- */
static void control_task(void *arg)
{
    float cmd_angle = 0.0f;      /* 当前下发的摆角 */
    float trim = 0.0f;           /* 驻留积分: 当前目标位置的持衡角 */
    /* 分目标trim记忆: 任意目标菜单后按整cm分桶, -10..+10 共21槽.
     * 弯管上各位置持衡角实测相差可达4°, 共用一个trim会把球反推 */
    #define TRIM_SLOTS 21
    float trims[TRIM_SLOTS] = {0};
    bool trim_seen[TRIM_SLOTS] = {false};
    int trim_idx = 10;           /* 目标0对应槽10 */
    float last_target_cm = 1e9f;
    float boost = 0.0f;          /* 破卡积分: 卡住时快速累积, 球滚起来立刻清零 */
    int creep_ticks = 0;         /* 本次顶角持续时长计数 */
    bool creep_push = false;     /* true=正在顶角等球动, false=回平滑行/等停稳 */
    float creep_pulse = CREEP_PULSE_DEG0;  /* 自适应顶角 */
    float creep_start_err = 0.0f;
    int still_ticks = 0;         /* 连续停稳计数: 攒够才允许开新一击 */
    int32_t last_pulses = INT32_MIN;
    int64_t last_log_us = 0;
    float acc_f = 0.0f;          /* 纵向加速度低通值 m/s² */
    int64_t trim_freeze_until = 0;  /* 加速期间冻结trim学习的截止时刻 */
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
        int64_t now = esp_timer_get_time();

        /* ---- 加速度前馈: 小车加减速 = 假坡度, 视觉100ms滞后来不及, IMU毫秒级补上.
         * θ_ff = (a/g)*57.3° ≈ 5.8°/(m/s²). 传感器故障时读回0, 前馈自然失效 ---- */
        float acc_raw = mpu6050_read_accel_ms2();
        acc_f += MPU_LPF_ALPHA * (acc_raw - acc_f);
        float ff_deg = FF_SIGN * FF_GAIN * (acc_f / 9.80665f) * 57.2958f;
        ff_deg = clampf(ff_deg, -FF_MAX_DEG, FF_MAX_DEG);
        if (fabsf(acc_f) > FF_FREEZE_MS2)
            trim_freeze_until = now + (int64_t)FF_FREEZE_HOLD_MS * 1000;
        bool trim_frozen = now < trim_freeze_until;

        taskENTER_CRITICAL(&g_lock);
        ball_state_t st = g_state;
        taskEXIT_CRITICAL(&g_lock);

        bool link_ok = st.have_ball &&
                       (now - st.rx_time_us) < (int64_t)RX_TIMEOUT_MS * 1000;
        const char *mode;

        if (!link_ok) {
            /* 断联/丢球: 匀速回到前馈角(不是0! 行驶中颠簸丢检测时,
             * 回纯平等于放任加速度把球甩飞, 前馈至少兜住假坡度) */
            float safe_target = ANGLE_SIGN * ff_deg;
            float step = FAILSAFE_RETURN_DPS * CTRL_PERIOD_MS / 1000.0f;
            if (cmd_angle > safe_target + step) cmd_angle -= step;
            else if (cmd_angle < safe_target - step) cmd_angle += step;
            else cmd_angle = safe_target;
            boost = 0.0f;        /* trim 保留到重连 */
            creep_push = false;
            still_ticks = 0;
            mode = "SAFE";
        } else {
            float error = st.ball_cm - st.target_cm;
            float speed = estimate_speed(now);

            if (st.target_cm != last_target_cm) {
                /* 分目标trim: 相邻位置持衡角实测差可达4°, 带着旧trim换目标会把球反推回去
                 * (日志: 到-5后trim=-1.95把球从-6弹回-1.2). 按整cm分桶, 学过的直接调档;
                 * 没学过的沿用当前trim当种子(全局零偏是共享的) */
                int ni = (int)lroundf(st.target_cm) + 10;
                if (ni < 0) ni = 0;
                if (ni >= TRIM_SLOTS) ni = TRIM_SLOTS - 1;
                if (last_target_cm < 1e8f) {
                    trims[trim_idx] = trim;
                    trim_seen[trim_idx] = true;
                }
                if (trim_seen[ni]) trim = trims[ni];
                trim_idx = ni;
                creep_pulse = CREEP_PULSE_DEG0;  /* 各位置摩擦不同, 顶角重学 */
                last_target_cm = st.target_cm;
            }
            /* 驻留积分: 球停稳在目标附近时学当地持衡角.
             * 只在停稳时积: 球滚动时还积会一路积到误差过零, 必然过冲(相位滞后),
             * trim就退化成了主执行器 */
            /* 只在紧贴目标时学: 球被静摩擦卡在±1处的"停稳"不是平衡点,
             * 在那学trim会拧出2°偏差, HOLD时反而把球从死区推出去 */
            /* 加速期间冻结学习: 小车加减速造成的溜球不是坡度, 学了停车后会反拧 */
            /* 死区内也冻结: 球停在0.24已达标, 残余误差是静摩擦不是坡度. 继续积分
             * = 对卡死的球单向上弦(实测35s积到7°), 破卡瞬间球被弹飞5cm+全局失稳 */
            if (!trim_frozen &&
                fabsf(error) > CENTER_DEADZONE_CM &&
                fabsf(error) <= TRIM_LEARN_CM && !creep_push &&
                fabsf(speed) <= STILL_SPEED_CMS) {
                trim += KI_SLOW_DEG_PER_CM_S * error * CTRL_PERIOD_MS / 1000.0f;
                trim = clampf(trim, -TRIM_CLAMP_DEG, TRIM_CLAMP_DEG);
            }
            /* 滑坡学习: 放开后球往"远离目标"方向慢滑 => trim扛不住坡, 按溜的距离补角.
             * 朝目标滑的是刚顶出去的, 不学 */
            if (!trim_frozen &&
                fabsf(error) <= TRIM_SLIDE_ZONE_CM && !creep_push &&
                error * speed > 0.0f &&
                fabsf(speed) > STILL_SPEED_CMS && fabsf(speed) < TRIM_SLIDE_MAX_CMS) {
                trim += TRIM_SLIDE_DEG_PER_CM * speed * CTRL_PERIOD_MS / 1000.0f;
                trim = clampf(trim, -TRIM_CLAMP_DEG, TRIM_CLAMP_DEG);
            }

            if (fabsf(error) <= CENTER_DEADZONE_CM && fabsf(speed) <= STILL_SPEED_CMS) {
                /* 到位且静止: 主动保持在持衡角上(不能回0! 0在弯管上不是水平) */
                boost = 0.0f;
                creep_push = false;
                float target_angle = ANGLE_SIGN * clampf(trim + ff_deg, -MAX_CTRL_ANGLE_DEG, MAX_CTRL_ANGLE_DEG);
                float max_step = MAX_ANGLE_RATE_DPS * CTRL_PERIOD_MS / 1000.0f;
                cmd_angle += clampf(target_angle - cmd_angle, -max_step, max_step);
                mode = "HOLD";
            } else if (fabsf(error) <= GS_FAR_CM && fabsf(speed) < I_STUCK_SPEED_CMS) {
                /* 近距离步进定位: 顶角盯球, 一动立刻撤角回持衡角 => 无多余储能不过冲 */
                boost = 0.0f;
                /* 滑行阻尼: 破卡后的球带动能, 只回trim等于放任它冲过死区
                 * (0点每一击滑2cm, 从+0.9直接滑到-1.1). 带点kd把它按住 */
                float u = trim + KD_DEG_PER_CMS * speed;
                if (creep_push) {
                    creep_ticks++;
                    still_ticks = 0;
                    if (fabsf(error - creep_start_err) >= CREEP_MOVED_MIN_CM ||
                        fabsf(speed) > 1.2f) {
                        creep_push = false;      /* 破卡成功: 立即撤角滑行 */
                    } else {
                        if (creep_ticks % (CREEP_ESCALATE_MS / CTRL_PERIOD_MS) == 0)
                            creep_pulse = fminf(creep_pulse + 1.0f, CREEP_PULSE_MAX_DEG);
                        u = trim + ((error > 0.0f) ? creep_pulse : -creep_pulse);
                    }
                } else {
                    /* 停稳计数: 进本区间已保证|v|<1.5, 不再另设1.2门槛 —
                     * 视觉噪声若折算1.2~1.5cm/s, 原来既攒不了停稳也进不了PD,
                     * 卡死在GLD永远不出手. 须连续 CREEP_SETTLE_MS 才开新一击:
                     * 速度估计有~100ms滞后, 刚破卡瞬间会误读为"停稳" */
                    still_ticks++;
                    if (still_ticks >= CREEP_SETTLE_MS / CTRL_PERIOD_MS) {
                        if (fabsf(error - creep_start_err) > CREEP_MOVED_MAX_CM)
                            creep_pulse = CREEP_PULSE_DEG0;  /* 上一击劲太大: 回初始重爬 */
                        creep_start_err = error;
                        creep_ticks = 0;
                        creep_push = true;
                        u = trim + ((error > 0.0f) ? creep_pulse : -creep_pulse);
                    }
                }
                float target_angle = ANGLE_SIGN * clampf(u + ff_deg, -MAX_CTRL_ANGLE_DEG, MAX_CTRL_ANGLE_DEG);
                float max_step = MAX_ANGLE_RATE_DPS * CTRL_PERIOD_MS / 1000.0f;
                cmd_angle += clampf(target_angle - cmd_angle, -max_step, max_step);
                mode = creep_push ? "CRP" : "GLD";
            } else {
                creep_push = false;
                still_ticks = 0;
                if (fabsf(speed) < I_STUCK_SPEED_CMS) {
                    /* 卡住: 快速累积破卡角 */
                    boost += error * CTRL_PERIOD_MS / 1000.0f;
                    boost = clampf(boost,
                                   -I_CLAMP_DEG / KI_DEG_PER_CM_S,
                                    I_CLAMP_DEG / KI_DEG_PER_CM_S);
                } else if (fabsf(speed) > BOOST_RESET_SPEED_CMS) {
                    boost = 0.0f;  /* 球已滚动: 破卡角立刻扔掉, 只留PD管飞行, 防大冲 */
                }
                float kp = KP_DEG_PER_CM, kd = KD_DEG_PER_CMS;
                if (fabsf(error) > GS_FAR_CM) {
                    kp = KP_FAR_DEG_PER_CM;   /* 远端: 冲得快, 少刹车 */
                    kd = KD_FAR_DEG_PER_CMS;
                }
                float pd_u = kp * error
                           + kd * speed
                           + KI_DEG_PER_CM_S * boost;
                /* 静摩擦前馈: 卡住且误差够大才给突破角, 球一动就自动撤掉.
                 * 误差刚出死区就kick会在目标附近来回拍打 */
                if (fabsf(speed) < I_STUCK_SPEED_CMS && fabsf(error) > KICK_MIN_ERROR_CM)
                    pd_u += (error > 0.0f) ? STICTION_KICK_DEG : -STICTION_KICK_DEG;
                /* 限幅作用在相对trim的偏差上: 零偏不吃掉冲刺力 */
                pd_u = clampf(pd_u, -MAX_PD_ANGLE_DEG, MAX_PD_ANGLE_DEG);
                float target_angle = ANGLE_SIGN * clampf(trim + pd_u + ff_deg, -MAX_CTRL_ANGLE_DEG, MAX_CTRL_ANGLE_DEG);
                /* 摆角指令变化率限幅, 防止猛拍弹球 */
                float max_step = MAX_ANGLE_RATE_DPS * CTRL_PERIOD_MS / 1000.0f;
                cmd_angle += clampf(target_angle - cmd_angle, -max_step, max_step);
                mode = "PD";
            }
        }

        cmd_angle = clampf(cmd_angle, -MAX_ARM_ANGLE_DEG, MAX_ARM_ANGLE_DEG);
        int32_t pulses = (int32_t)lroundf(cmd_angle / 360.0f * PULSES_PER_REV);

        if (pulses != last_pulses) {
            emm42_move_abs(pulses, MOVE_SPEED_RPM, MOVE_ACC);
            last_pulses = pulses;
        }

        /* 5Hz 打印, 串口监视器看状态 */
        if (now - last_log_us > 200000) {
            last_log_us = now;
            ESP_LOGI(TAG, "%s ball=%.2f tgt=%.2f ang=%.2f trim=%.2f ff=%.2f a=%.2f pulses=%ld",
                     mode, st.ball_cm, st.target_cm, cmd_angle, trim, ff_deg, acc_f, (long)pulses);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    emm42_init();
    vTaskDelay(pdMS_TO_TICKS(500));   /* 等驱动器上电就绪 */
    emm42_enable(true);
    vTaskDelay(pdMS_TO_TICKS(100));
    emm42_zero_here();                /* 当前(水平)位置 = 0 度 */
    ESP_LOGI(TAG, "emm42 enabled, zeroed at level position");

    /* MPU6050: 与摆杆调零同一时机标定(都要求静止). 没接/坏了不影响原控制 */
    if (mpu6050_init())
        mpu6050_calibrate(MPU_CALIB_SAMPLES);
    else
        ESP_LOGW(TAG, "mpu6050 absent, accel feedforward disabled");

    wifi_init();

    xTaskCreate(udp_task, "udp", 4096, NULL, 5, NULL);
    xTaskCreate(control_task, "ctrl", 4096, NULL, 6, NULL);
}
