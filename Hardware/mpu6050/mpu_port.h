/**
 * @file    mpu_port.h
 * @brief   MPU6050 DMP 移植层头文件
 * @details 为 Invensense 官方 DMP 驱动（inv_mpu.c / inv_mpu_dmp_motion_driver.c）
 *          提供 MSPM0G3507 平台所需的底层 I2C 读写函数声明，
 *          并提供简化版的 DMP_Init() / DMP_Read_Data() 接口供测试程序使用。
 *
 * 硬件：I2C1 (GY_87_INST)  PB2(SCL) / PB3(SDA)
 *
 * 函数清单：
 *   - MPU_Write_Len() : DMP 库调用，写 MPU6050 寄存器（带超时）
 *   - MPU_Read_Len()  : DMP 库调用，读 MPU6050 寄存器（带超时）
 *   - delay_ms()      : 毫秒延时（实际由工程 System/delay.c 提供）
 *   - mget_ms()       : 获取当前毫秒计数（供 DMP 时间戳使用）
 *   - MPU_Tick()      : 1ms 定时器中断中调用，递增 sys_tick_ms
 *   - DMP_Init()      : 初始化 MPU6050 DMP 引擎
 *   - DMP_Read_Data() : 读取 DMP 解算的 Pitch/Roll/Yaw 角度（度）
 */

#ifndef _MPU_PORT_H_
#define _MPU_PORT_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 官方库需要的 4 个底层函数声明 */
int MPU_Write_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf);
int MPU_Read_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf);
void delay_ms(uint32_t num_ms);
void mget_ms(uint32_t *time);

/* 我们自己封装的 DMP 初始化和读取函数 */
void MPU_Tick(void);
int DMP_Init(void);
int DMP_Read_Data(float *pitch, float *roll, float *yaw);

/* 应用层：自动校准 + 增量累积（供主循环调用） */
extern uint8_t  mpu_ok;              /* MPU6050 初始化成功标志 */
extern uint8_t  mpu_calibrated;      /* YAW 校准完成标志 */
extern float    mpu_pitch;           /* 最新 Pitch（度） */
extern float    mpu_roll;            /* 最新 Roll（度） */
extern float    mpu_corrected_yaw;   /* 校准+补偿后的 YAW（度） */
void MPU_Update(void);
uint16_t MPU_GetCalibProgress(void); /* 校准进度 0~100% */

#endif

