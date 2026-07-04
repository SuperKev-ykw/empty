/**
 * @file    AHRS.h
 * @brief   姿态解算头文件 (加速度计+磁力计+陀螺仪互补融合)
 * @details 提供 Roll/Pitch/Yaw 姿态角获取接口
 */

#ifndef __AHRS_H
#define __AHRS_H

#include <stdint.h>

/* ==================== 函数声明 ==================== */

/**
 * @brief  磁力计硬铁校准（在线采集最大最小值）
 * @param  MagX 原始磁力计 X 值
 * @param  MagY 原始磁力计 Y 值
 * @param  MagZ 原始磁力计 Z 值
 * @note   采集 100 次后自动计算偏移量
 */
void AHRS_CalibrateMag(int16_t MagX, int16_t MagY, int16_t MagZ);

/**
 * @brief  手动设置磁力计偏移量
 */
void AHRS_SetMagOffset(float OffsetX, float OffsetY, float OffsetZ);

/**
 * @brief  获取当前磁力计偏移量
 */
void AHRS_GetMagOffset(float *OffsetX, float *OffsetY, float *OffsetZ);

/**
 * @brief  姿态更新（加速度计 + 陀螺仪 Z 轴 + 磁力计互补融合）
 * @param  AccX, AccY, AccZ  加速度计原始值
 * @param  GyroZ             陀螺仪 Z 轴原始值
 * @param  MagX, MagY, MagZ  磁力计原始值
 * @param  dt                上次更新到本次的时间间隔（秒）
 */
void AHRS_Update(int16_t AccX, int16_t AccY, int16_t AccZ,
                 int16_t GyroZ,
                 int16_t MagX, int16_t MagY, int16_t MagZ,
                 float dt);

/**
 * @brief  获取姿态角
 * @return 角度值，单位度
 * @note   Yaw: 0-360, Pitch/Roll: -180~180
 */
float AHRS_GetYaw(void);
float AHRS_GetPitch(void);
float AHRS_GetRoll(void);

#endif /* __AHRS_H */
