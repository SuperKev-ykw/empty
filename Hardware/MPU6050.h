/**
 * @file    MPU6050.h
 * @brief   MPU6050 六轴传感器驱动头文件
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
