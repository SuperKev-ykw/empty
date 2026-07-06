/**
 * @file    HMC5883L.h
 * @brief   HMC5883L 三轴磁力计驱动头文件
 * @details 通过 MPU6050 AUX I2C 旁路访问，I2C 地址 0x1E
 *
 * 函数清单：
 *   - HMC5883L_Init()    : 初始化（使能 BYPASS、配置量程、连续测量模式）
 *   - HMC5883L_GetData() : 一次性读取 3 轴磁场数据（X/Y/Z，单位 LSB）
 *
 * 使用方式：
 *   1. 先调用 MPU6050_Init() 初始化 I2C 总线
 *   2. 再调用 HMC5883L_Init() 初始化磁力计
 *   3. 循环调用 HMC5883L_GetData(&mx, &my, &mz) 读取磁场数据
 *
 * 注意：HMC5883L 数据寄存器顺序为 X, Z, Y（注意 Y 和 Z 的顺序）
 */

#ifndef __HMC5883L_H
#define __HMC5883L_H

#include <stdint.h>

void HMC5883L_Init(void);
void HMC5883L_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ);

#endif /* __HMC5883L_H */
