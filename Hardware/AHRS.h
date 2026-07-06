/**
 * @file    AHRS.h
 * @brief   姿态解算头文件（加速度计+磁力计+陀螺仪互补融合）
 * @details 提供 Roll/Pitch/Yaw 姿态角获取接口
 *
 * 函数清单：
 *   - AHRS_CalibrateMag() : 磁力计硬铁校准（在线采集最大最小值）
 *   - AHRS_SetMagOffset() : 手动设置磁力计偏移量
 *   - AHRS_GetMagOffset() : 获取当前磁力计偏移量
 *   - AHRS_Update()       : 姿态更新（融合加速度计+陀螺仪+磁力计）
 *   - AHRS_GetYaw()       : 获取 Yaw 角度（0~360°）
 *   - AHRS_GetPitch()     : 获取 Pitch 角度（-180~180°）
 *   - AHRS_GetRoll()      : 获取 Roll 角度（-180~180°）
 *
 * 使用方式：
 *   1. 调用 AHRS_CalibrateMag() 100+ 次采集磁力计最大最小值
 *      （用户需旋转模块 360°）
 *   2. 在 1ms / 10ms 定时器中周期性调用 AHRS_Update() 传入最新传感器数据
 *   3. 调用 AHRS_GetYaw() / AHRS_GetPitch() / AHRS_GetRoll() 获取姿态角
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
