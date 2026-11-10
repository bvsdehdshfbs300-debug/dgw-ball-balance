#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Emm42_V5.x 串口指令封装 (固定校验字节 0x6B)
 * 依据《Emm_V5.0 步进闭环驱动说明书 Rev1.3》
 * 驱动器菜单需设置: 串口波特率与 config.h 一致 / 地址 1 / 校验方式 0x6B / 回应模式 Receive
 */

void emm42_init(void);

/* 使能/失能电机 (addr F3 AB en sync 6B) */
void emm42_enable(bool enable);

/* 将当前位置清零, 作为 0 度水平基准 (addr 0A 6D 6B)
 * !! 上电前务必手动把摆杆放平 !! */
void emm42_zero_here(void);

/* 位置模式-绝对运动 (addr FD dir speedH speedL acc pulse[4] absFlag syncFlag 6B)
 * pulses: 带符号绝对位置(相对零点), rpm: 运行速度, acc: 加速度档位(0=直达) */
void emm42_move_abs(int32_t pulses, uint16_t rpm, uint8_t acc);

/* 立即停止 (addr FE 98 00 6B) */
void emm42_stop(void);
