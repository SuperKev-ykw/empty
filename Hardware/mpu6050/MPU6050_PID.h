/**
 * @file    MPU6050_PID.h
 * @brief   YAW 角度闭环控制头文件
 * @details 声明角度环 PID 变量、Yaw_PI_Control() 函数及工具函数。
 *          用于 MPU6050_Test.c 等需要角度闭环的测试程序。
 *
 * 蓝牙调参协议：
 *   [slider,YKp,值]  YAW 比例系数
 *   [slider,YKi,值]  YAW 积分系数
 *   [slider,YKd,值]  YAW 微分系数
 *   [slider,YDZ,值]  YAW 死区角度（°）
 *   [slider,YTgt,值] YAW 目标角度（°）
 */

#ifndef _MPU6050_PID_H_
#define _MPU6050_PID_H_

#include <stdint.h>

/* ==================== YAW PID 参数（蓝牙可调） ==================== */

extern float g_yaw_kp;          /* 比例系数 */
extern float g_yaw_ki;          /* 积分系数 */
extern float g_yaw_kd;          /* 微分系数 */
extern float g_yaw_dead_zone;   /* 死区角度（°） */
extern float g_yaw_target;      /* 目标 YAW 角度（度） */

/* ==================== 状态标志 ==================== */

extern volatile uint8_t g_yaw_ctrl_on;  /**< YAW 闭环使能 */

/* ==================== 函数声明 ==================== */

/**
 * @brief 将角度归一化到 [-180, 180] 范围
 */
float NormalizeAngle(float angle);

/**
 * @brief  YAW 角度 PID 控制器（应在 20ms 定时周期中调用）
 */
void Yaw_PI_Control(void);

/**
 * @brief  YAW 闭环状态重置（清零积分/微分，电机停转）
 */
void Yaw_Reset(void);

#endif
