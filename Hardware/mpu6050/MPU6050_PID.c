/**
 * @file    MPU6050_PID.c
 * @brief   YAW 角度闭环控制实现
 * @details 提供 YAW 角度 PID 控制器，将角度误差转为差速输出，
 *          经由 Motor_SetTargetSpeed() 送入速度内环执行。
 *
 * 控制链路：
 *   yaw_target --[PID]--> turn_deviation --[Motor_SetTargetSpeed]--> 速度内环(20ms) --> PWM
 *
 * 依赖：
 *   - Motor.h        : Motor_SetTargetSpeed(), Motor_Speed_PID_Reset()
 *   - mpu_port.h     : mpu_corrected_yaw, mpu_calibrated
 *   - math.h         : fabsf()
 */

#include "MPU6050_PID.h"
#include "mpu_port.h"
#include "Motor.h"
#include <math.h>

/* ==================== YAW PID 参数定义（蓝牙可调） ==================== */

float g_yaw_kp = 0.1f;          /* 比例系数 */
float g_yaw_ki = 0.07f;         /* 积分系数 */
float g_yaw_kd = 0.0f;          /* 微分系数 */
float g_yaw_dead_zone = 5.0f;   /* 死区角度（°） */
float g_yaw_target = 0.0f;      /* 目标 YAW 角度（度） */

/* ==================== 内部状态 ==================== */

volatile uint8_t g_yaw_ctrl_on = 0;     /**< YAW 闭环使能，校准后自动开启 */
float yaw_output = 0.0f;                /**< PID 输出值，供串口调试 */

/* PID 内部累加量 */
static float yaw_integral = 0.0f;
static float yaw_prev_error = 0.0f;

/* 限幅常数 */
#define YAW_I_LIMIT     100.0f      /* 积分限幅（抗积分饱和） */
#define YAW_OUT_LIMIT   800         /* 输出限幅（max turn deviation） */

/* ==================== 内部工具函数 ==================== */

/**
 * @brief 浮点限幅
 */
static float ClampFloat(float v, float max_v)
{
    if (v > max_v)  return max_v;
    if (v < -max_v) return -max_v;
    return v;
}

/* ==================== 接口函数实现 ==================== */

/**
 * @brief 将角度归一化到 [-180, 180] 范围
 */
float NormalizeAngle(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief  YAW 角度 PID 控制器
 * @details 计算 target - actual 误差，经 PID 输出差速值。
 *          输出经由 Motor_SetTargetSpeed() 送入速度内环。
 *
 *          角度归一化处理：target=350°, actual=10° 时取差值 -20°（最短路径），
 *          而非 340°（绕大圈）。
 *
 *          应在 20ms 定时周期中调用（与速度内环同频）。
 */
void Yaw_PI_Control(void)
{
    if (!g_yaw_ctrl_on || !mpu_calibrated)
    {
        /* 闭环未使能或未校准：输出 0 */
        yaw_integral = 0.0f;
        yaw_prev_error = 0.0f;
        Motor_SetTargetSpeed(0, 0);
        return;
    }

    /* 误差 = target - actual，归一化到 [-180, 180] */
    float error = NormalizeAngle(g_yaw_target - mpu_corrected_yaw);

    /* 死区：小误差不做响应，防止抖动（保留积分，不清零） */
    if (error > -g_yaw_dead_zone && error < g_yaw_dead_zone)
    {
        Motor_SetTargetSpeed(0, 0);
        yaw_output = 0.0f;
        return;
    }

    /* PID 计算 */
    yaw_integral += error;
    yaw_integral = ClampFloat(yaw_integral, YAW_I_LIMIT);

    float derivative = error - yaw_prev_error;
    float output = g_yaw_kp * error + g_yaw_ki * yaw_integral + g_yaw_kd * derivative;
    output = ClampFloat(output, (float)YAW_OUT_LIMIT);
    yaw_output = output;    /* 供串口调试 */

    /* 输出到速度内环：差速转向
     *   error > 0 → 需右转(顺时针) → 右轮后退(负)，左轮前进(正)
     *   error < 0 → 需左转(逆时针) → 右轮前进(正)，左轮后退(负) */
    int16_t speed = (int16_t)fabsf(output);
    int16_t target_l = 0, target_r = 0;
    if (error > 0)
    {
        /* 需右转：左轮前进，右轮停转（避免反转启动难/蜂鸣） */
        target_r = speed;
        target_l = 0;
    }
    else
    {
        /* 需左转：右轮前进，左轮停转（避免反转启动难/蜂鸣） */
        target_r = 0;
        target_l = speed;
    }
    Motor_SetTargetSpeed(target_l, target_r);

    yaw_prev_error = error;
}

/**
 * @brief  YAW 闭环状态重置
 * @details 清零积分/微分累加、电机停转，用于 Key2 回零等场景
 */
void Yaw_Reset(void)
{
    yaw_integral = 0.0f;
    yaw_prev_error = 0.0f;
    Motor_SetTargetSpeed(0, 0);
    Motor_Speed_PID_Reset();
}
