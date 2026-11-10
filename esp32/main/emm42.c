#include "emm42.h"
#include "config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "emm42";

static void send_frame(const uint8_t *buf, int len)
{
    uart_write_bytes(EMM42_UART_NUM, (const char *)buf, len);
    /* 回应模式 Receive 时驱动器会回 4 字节确认, 这里不阻塞等待,
     * 由 uart 缓冲区自然丢弃; 调试时可打开下面的读取打印 */
#if 0
    uint8_t rx[16];
    int n = uart_read_bytes(EMM42_UART_NUM, rx, sizeof(rx), pdMS_TO_TICKS(5));
    if (n > 0) ESP_LOG_BUFFER_HEX(TAG, rx, n);
#endif
}

void emm42_init(void)
{
    uart_config_t cfg = {
        .baud_rate = EMM42_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(EMM42_UART_NUM, 256, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(EMM42_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(EMM42_UART_NUM, EMM42_TX_PIN, EMM42_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "uart ready: tx=%d rx=%d baud=%d", EMM42_TX_PIN, EMM42_RX_PIN, EMM42_BAUD);
}

void emm42_enable(bool enable)
{
    uint8_t buf[6] = { EMM42_ADDR, 0xF3, 0xAB, enable ? 0x01 : 0x00, 0x00, 0x6B };
    send_frame(buf, sizeof(buf));
}

void emm42_zero_here(void)
{
    /* 角度清零: addr 0A 6D 6B  —— 若你的固件版本此命令无效,
     * 对照说明书"通讯控制"章节核对功能码 */
    uint8_t buf[4] = { EMM42_ADDR, 0x0A, 0x6D, 0x6B };
    send_frame(buf, sizeof(buf));
}

void emm42_move_abs(int32_t pulses, uint16_t rpm, uint8_t acc)
{
    /* 方向字节: 绝对模式下表示位置符号. 0=正方向, 1=负方向
     * 若实际转向与摆杆机械相反, 优先改 config.h 的 ANGLE_SIGN */
    uint8_t dir = 0;
    uint32_t p = (uint32_t)pulses;
    if (pulses < 0) {
        dir = 1;
        p = (uint32_t)(-pulses);
    }
    uint8_t buf[13] = {
        EMM42_ADDR, 0xFD, dir,
        (uint8_t)(rpm >> 8), (uint8_t)(rpm & 0xFF),
        acc,
        (uint8_t)(p >> 24), (uint8_t)(p >> 16), (uint8_t)(p >> 8), (uint8_t)(p & 0xFF),
        0x01,   /* 1 = 绝对位置模式 */
        0x00,   /* 多机同步: 不用 */
        0x6B
    };
    send_frame(buf, sizeof(buf));
}

void emm42_stop(void)
{
    uint8_t buf[5] = { EMM42_ADDR, 0xFE, 0x98, 0x00, 0x6B };
    send_frame(buf, sizeof(buf));
}
