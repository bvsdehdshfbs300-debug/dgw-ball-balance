#pragma once
#include <stdbool.h>

/* MPU6050 加速度计 (只用加速度, 不用陀螺)
 * 用途: 测小车纵向加速度, 前馈补偿"假坡度"
 * 接线: VCC->3.3V GND->GND SDA->GPIO4 SCL->GPIO5 (config.h 可改)
 * AD0 悬空/接地 => 地址 0x68
 */

/* 初始化 I2C + 唤醒芯片. 返回 false = 没找到芯片(检查接线/地址) */
bool mpu6050_init(void);

/* 上电零偏标定: 静止采 n 个样本取均值当零点.
 * !! 标定期间小车必须静止 !! (和摆杆放平是同一时机, 流程不变)
 * 零点同时吸收: 传感器零偏 + 安装倾斜带来的重力分量 */
void mpu6050_calibrate(int n_samples);

/* 读沿摆杆轴(=行驶方向)的加速度, 已减零偏, 单位 m/s².
 * 读失败返回 0 (前馈自然失效, 不影响原控制) */
float mpu6050_read_accel_ms2(void);
