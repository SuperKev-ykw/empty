/**
 * @file    MPU6050_Reg.h
 * @brief   MPU6050 寄存器地址定义
 * @details MPU6050 内部寄存器的 7 位地址常量，供 MPU6050.c 使用
 *          参考 InvenSense MPU-6000/MPU-6050 Register Map 文档
 *
 * 主要寄存器：
 *   - 0x19 SMPLRT_DIV   : 采样率分频器 (1kHz / (1+SMPLRT_DIV))
 *   - 0x1A CONFIG       : 配置（DLPF、FSYNC）
 *   - 0x1B GYRO_CONFIG  : 陀螺仪量程 (±250/500/1000/2000 dps)
 *   - 0x1C ACCEL_CONFIG : 加速度量程 (±2/4/8/16 g)
 *   - 0x37 INT_PIN_CFG  : 中断/旁路配置（磁力计 BYPASS 模式）
 *   - 0x3B-0x40         : 加速度 X/Y/Z 输出寄存器（H+L 16bit）
 *   - 0x41-0x42         : 温度输出寄存器
 *   - 0x43-0x48         : 陀螺仪 X/Y/Z 输出寄存器
 *   - 0x6B PWR_MGMT_1   : 电源管理 1（时钟源、复位、睡眠）
 *   - 0x6C PWR_MGMT_2   : 电源管理 2（各轴待机控制）
 *   - 0x75 WHO_AM_I     : 设备 ID 寄存器（MPU6050=0x68）
 */

#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H

#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_INT_PIN_CFG    0x37
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_ACCEL_XOUT_L   0x3C
#define MPU6050_ACCEL_YOUT_H   0x3D
#define MPU6050_ACCEL_YOUT_L   0x3E
#define MPU6050_ACCEL_ZOUT_H   0x3F
#define MPU6050_ACCEL_ZOUT_L   0x40
#define MPU6050_TEMP_OUT_H     0x41
#define MPU6050_TEMP_OUT_L     0x42
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_GYRO_XOUT_L    0x44
#define MPU6050_GYRO_YOUT_H    0x45
#define MPU6050_GYRO_YOUT_L    0x46
#define MPU6050_GYRO_ZOUT_H    0x47
#define MPU6050_GYRO_ZOUT_L    0x48
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_PWR_MGMT_2     0x6C
#define MPU6050_WHO_AM_I       0x75

#endif /* __MPU6050_REG_H */
