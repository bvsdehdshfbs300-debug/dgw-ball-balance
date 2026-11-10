#include "mpu6050.h"
#include "config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu6050";

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define ACC_LSB_PER_G    16384.0f   /* ±2g 量程 */
#define G_MS2            9.80665f

static i2c_master_dev_handle_t s_dev;
static float s_offset_ms2 = 0.0f;
static bool s_ok = false;

static bool wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, 2, 100) == ESP_OK;
}

static bool rd(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, len, 100) == ESP_OK;
}

/* 读一次原始加速度(沿 MPU_ACCEL_AXIS 轴), 返回 m/s², 未减零偏 */
static bool read_raw_ms2(float *out)
{
    uint8_t b[6];
    if (!rd(REG_ACCEL_XOUT_H, b, 6))
        return false;
    int i = MPU_ACCEL_AXIS * 2;
    int16_t raw = (int16_t)((b[i] << 8) | b[i + 1]);
    *out = raw / ACC_LSB_PER_G * G_MS2;
    return true;
}

bool mpu6050_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = MPU_SDA_PIN,
        .scl_io_num = MPU_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* 模块自带上拉的话也不冲突 */
    };
    i2c_master_bus_handle_t bus;
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed");
        return false;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &s_dev) != ESP_OK) {
        ESP_LOGE(TAG, "add device failed");
        return false;
    }

    /* WHO_AM_I: 原装MPU6050=0x68, 兼容片各不相同(MPU6515=0x74, MPU6500=0x70,
     * MPU9250=0x71...), 寄存器映射全兼容. 只要I2C读得通就放行, 0x00/0xFF才算没接 */
    uint8_t who = 0;
    if (!rd(REG_WHO_AM_I, &who, 1) || who == 0x00 || who == 0xFF) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X, 检查接线/AD0", who);
        return false;
    }
    if (who != 0x68)
        ESP_LOGW(TAG, "WHO_AM_I=0x%02X (兼容芯片, 非原装MPU6050, 正常使用)", who);
    bool ok = wr(REG_PWR_MGMT_1, 0x01)      /* 唤醒, 时钟=PLL */
           && wr(REG_SMPLRT_DIV, 0x00)      /* 1kHz 内部采样 */
           && wr(REG_CONFIG, 0x03)          /* DLPF 44Hz, 先滤掉车架高频振动 */
           && wr(REG_ACCEL_CONFIG, 0x00);   /* ±2g */
    if (!ok) {
        ESP_LOGE(TAG, "config failed");
        return false;
    }
    s_ok = true;
    ESP_LOGI(TAG, "init ok (SDA=%d SCL=%d axis=%d)", MPU_SDA_PIN, MPU_SCL_PIN, MPU_ACCEL_AXIS);
    return true;
}

void mpu6050_calibrate(int n_samples)
{
    if (!s_ok) return;
    float sum = 0.0f;
    int got = 0;
    for (int i = 0; i < n_samples; i++) {
        float a;
        if (read_raw_ms2(&a)) {
            sum += a;
            got++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (got > n_samples / 2) {
        s_offset_ms2 = sum / got;
        ESP_LOGI(TAG, "calibrated: offset=%.3f m/s² (%d samples)", s_offset_ms2, got);
    } else {
        s_ok = false;   /* 读不到数据, 关掉前馈 */
        ESP_LOGE(TAG, "calibrate failed, feedforward disabled");
    }
}

float mpu6050_read_accel_ms2(void)
{
    if (!s_ok) return 0.0f;
    float a;
    if (!read_raw_ms2(&a))
        return 0.0f;
    return a - s_offset_ms2;
}
