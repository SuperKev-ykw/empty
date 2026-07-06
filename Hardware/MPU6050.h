/**
 * @file    MPU6050.h
 * @brief   MPU6050 六轴传感器驱动头文件
 * @details 提供基础版 MPU6050 数据读取接口（不依赖 DMP）
 *          硬件：I2C1 (PB2=SCL, PB3=SDA)
 *
 * 函数清单：
 *   - MPU6050_Init()    : 初始化 MPU6050（配置时钟、量程、采样率）
 *   - MPU6050_GetID()   : 读 WHO_AM_I 寄存器（应返回 0x68）
 *   - MPU6050_GetData() : 一次性读取 6 轴原始数据（加速度+陀螺仪）
 *
 * 底层 I2C 接口（被 HMC5883L 复用）：
 *   - MPU6050_WriteReg() : 写一个寄存器
 *   - MPU6050_ReadReg()  : 读一个寄存器
 *   - MPU6050_ReadRegs() : 连续读多个寄存器
 *
 * 使用方式：
 *   1. MPU6050_Init();
 *   2. 循环调用 MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
 *
 * 注意：加速度原始值除以 2048 得到 g，陀螺仪原始值除以 16.4 得到 dps
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

/* ==================== 函数声明 ==================== */
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/* 底层 I2C 接口（HMC5883L 需要调用） */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count);

#endif /* __MPU6050_H */
