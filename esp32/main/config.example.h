#pragma once

/* ---------- WiFi: 改成你的手机热点 ---------- */
#define WIFI_SSID           "CHANGE_ME_SSID"
#define WIFI_PASS           "CHANGE_ME_PASSWORD"

/* ---------- UDP ---------- */
#define UDP_PORT            5005
#define RX_TIMEOUT_MS       300     /* 超过这个时间没收到球位置 -> 回平 */

/* ---------- Emm42_V5 串口 ---------- */
#define EMM42_UART_NUM      UART_NUM_1
#define EMM42_TX_PIN        17      /* ESP32 TX -> Emm42 RX */
#define EMM42_RX_PIN        18      /* ESP32 RX -> Emm42 TX */
#define EMM42_BAUD          115200  /* 与驱动器屏幕菜单一致 */
#define EMM42_ADDR          0x01    /* 驱动器地址, 菜单默认 1 */

/* ---------- MPU6050 加速度前馈 (第4~6问: 小车行驶时抵消加减速扰动) ---------- */
#define MPU_SDA_PIN         4
#define MPU_SCL_PIN         5
#define MPU_I2C_ADDR        0x68    /* AD0 悬空/接地 */
#define MPU_ACCEL_AXIS      0       /* 0=X 1=Y 2=Z: 选与行驶方向(=摆杆方向)平行的那根轴,
                                     * 安装时让模块丝印 X 箭头指向摆杆正方向最省事 */
#define MPU_CALIB_SAMPLES   200     /* 上电静止标定样本数(约1s) */
#define MPU_LPF_ALPHA       0.35f   /* 100Hz 下~10Hz 一阶低通: 滤车架振动, 保留加减速 */
/* 前馈物理: 加速度 a 等效假坡度, 补偿角 = (a/g)*57.3° ≈ 5.8°/(m/s²).
 * 增益从 1.0 起调; 符号看实车: 小车加速时球往哪边溜, 前馈就该把杆往那边压 */
#define FF_GAIN             1.6f    /* 理论1.0必亏: LPF+限速+电机响应共~100ms滞后,
                                     * 等角度到位球已被拽动, 需过补. 过了会反甩, 再回调 */
#define FF_SIGN             (+1.0f) /* 实测前馈方向反了(加速时球溜得更远)就改 -1.0f */
#define FF_MAX_DEG          12.0f   /* 前馈限幅: 12°≈1.3m/s²(增益1.6下), 急刹能到2m/s² */
#define FF_FREEZE_MS2       0.5f    /* |a|超过该值 => 冻结trim学习(防把加速度误学成坡度) */
#define FF_FREEZE_HOLD_MS   500     /* 加速结束后再多冻这么久(球还在晃, 学了也是脏的) */

/* ---------- 机械 ---------- */
#define MICROSTEP           16      /* 驱动器菜单细分, 必须一致 */
#define PULSES_PER_REV      (200 * MICROSTEP)   /* 3200 */
#define MAX_ARM_ANGLE_DEG   20.0f   /* 机械安全限幅 */
#define ANGLE_SIGN          (+1.0f) /* 球越推越远就改成 -1.0f */

/* ---------- 控制参数(调参主要动这里) ---------- */
#define CTRL_PERIOD_MS      10      /* 100Hz 控制周期 */
#define KP_DEG_PER_CM       0.6f
#define KD_DEG_PER_CMS      0.5f
/* 增益调度: |误差|>GS_FAR_CM 用远端增益. 巡航速度~kp/kd*|误差|,
 * 必须压在视觉能跟踪的范围(实测>30cm/s时检测丢帧, 刹车必然迟到) */
#define GS_FAR_CM           2.0f
#define KP_FAR_DEG_PER_CM   0.8f
#define KD_FAR_DEG_PER_CMS  0.6f
#define KI_DEG_PER_CM_S     3.0f    /* boost破卡积分累积速率: 卡住时快速顶角 */
#define I_STUCK_SPEED_CMS   1.5f    /* 球速低于该值视为"卡住": 积boost+加kick */
#define BOOST_RESET_SPEED_CMS 4.0f  /* 球速超过该值=已滚动, boost立刻清零防大冲 */
#define STICTION_KICK_DEG   2.0f    /* 静摩擦前馈: 卡住时立即加的突破角, 球动了就撤 */
#define KICK_MIN_ERROR_CM   0.8f    /* 误差大于该值才kick, 防止在目标附近拍打 */

/* 近距离步进定位: 顶角直到球动(每300ms没动加1度), 一动立即回平滑行, 停稳再评估下一击 */
#define CREEP_ESCALATE_MS   200     /* 顶着没动多久后加力 */
#define CREEP_SETTLE_MS     300     /* 球须连续停稳该时长才开新一击, 防对滚动球反向注能 */
#define CREEP_PULSE_DEG0    3.0f    /* 初始顶角(0点附近~5°破卡, ±4处8~10°, 靠加力自适应) */
#define CREEP_PULSE_MAX_DEG 12.0f   /* 加力上限(±4处实测破卡要8~10°, 即时释放敢给大) */
#define CREEP_MOVED_MIN_CM  0.15f   /* 挪动超过此值=破卡成功, 立刻撤角 */
#define CREEP_MOVED_MAX_CM  1.2f    /* 一击总滑动大于此=劲太大, 顶角直接回初始值 */
#define I_CLAMP_DEG         8.0f    /* boost限幅(度) */
/* 驻留积分: 学习当前目标位置的持衡角. 管子弯曲/支架下沉时各位置"水平"不同,
 * 例如-5处可能要持续+3度才拽得住球. 换目标清零重学 */
#define KI_SLOW_DEG_PER_CM_S 0.8f
#define TRIM_LEARN_CM        0.8f   /* 只在离目标这么近才积trim: 球卡在±1处不是平衡点, 在那学会拧歪 */
/* 滑坡学习: 球被放开后往远离目标方向慢滑 = trim扛不住当地坡度, 按滑动距离补角.
 * 治0→5爬坡: +5侧上坡GLD一放手球就溜回1.4, 每溜1cm补1°直到扛住.
 * 只认1.2~6cm/s的慢滑: 快飞是PD的事, 视觉噪声到不了1.2 */
#define TRIM_SLIDE_DEG_PER_CM 1.0f
#define TRIM_SLIDE_MAX_CMS    6.0f
#define TRIM_SLIDE_ZONE_CM    4.0f
#define TRIM_CLAMP_DEG       8.0f   /* 要盖住上电摆杆不平的全局零偏(实测可达5°) */
#define MAX_CTRL_ANGLE_DEG  16.0f   /* 总输出限幅(trim+破卡角), 机械硬限20 */
#define MAX_PD_ANGLE_DEG    13.0f   /* PD冲刺角限幅: 相对trim的偏差. 10时球卡在2.4处
                                     * ang饱和在trim-10=-8.3顶了7秒不动, 破卡帽子必须比
                                     * 实测最大静摩擦角(8~10°)高. 飞行时boost已清零碰不到帽 */
#define MAX_ANGLE_RATE_DPS  120.0f  /* 摆角指令变化率限幅. 80时破卡后从-9°转到刹车角要0.23s,
                                     * 球早冲出去了(实测滑到+7); 提速换刹车响应 */
#define CENTER_DEADZONE_CM  0.3f
#define STILL_SPEED_CMS     1.2f    /* 视觉噪声折算速度~1cm/s, 低于此阈值停稳门永远开不了 */
#define SPEED_WINDOW_MS     100     /* 球速差分窗口 */
#define MAX_SPEED_CMS       40.0f

/* ---------- 电机运动参数 ---------- */
#define MOVE_SPEED_RPM      150     /* 位置指令的运行速度 */
#define MOVE_ACC            0       /* 0 = 不用加减速曲线, 小角度伺服更跟手 */
#define FAILSAFE_RETURN_DPS 15.0f   /* 断联时回平角速度 */
