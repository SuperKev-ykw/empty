/**
 * @file    Motor.h
 * @brief   AT8236 直流电机驱动头文件
 * @details 驱动两路 AT8236 H 桥芯片，控制左右直流减速电机。
 *          使用 PWM_A(TIMA1/CCP1-PB1) + PWM_B(TIMG8/CCP0-PB15) 调速，
 *          方向引脚用 GPIO(PB0 / PB14) 控制。
 *
 * 硬件接线（左右不对称，已针对交叉布线适配）：
 *   左电机：IN1=PB4 (Motors_AIN, GPIO 方向), IN2=PB1 (TIMA1 CCP1=PWM_A)
 *   右电机：IN1=PB15 (TIMG8 CCP0=PWM_B), IN2=PB16 (Motors_BIN, GPIO 方向)
 *
 * 控制逻辑（AT8236 四条工作状态）:
 *   IN1=1, IN2=PWM  -> 正转, 慢衰减 (Slow Decay)   <- 左轮前进
 *   IN1=PWM, IN2=1  -> 反转, 慢衰减 (Slow Decay)   <- 右轮"前进"(因电机反装)
 *   IN1=0, IN2=PWM  -> 反转, 快衰减 (Fast Decay)   <- 左轮后退
 *   IN1=PWM, IN2=0  -> 正转, 快衰减 (Fast Decay)   <- 右轮"后退"
 *   IN1=1, IN2=1    -> 短路刹车 (主动制动)
 *   IN1=0, IN2=0    -> 自由滑行 (Coast)
 *
 * 小车前进时两轮均工作在慢衰减模式，后退时均工作在快衰减模式。
 *
 * 使用方式：
 *   1. SysConfig 中已配置 PWM_A(TIMA1, PB1) 和 PWM_B(TIMG8, PB15)
 *   2. main 开头调用 Motor_Init()
 *   3. 调用 Motor_SetPWM(MOTOR_LEFT, 500) 设置左轮正转 50% 占空比
 *   4. 调用 Motor_StopAll(MOTOR_BRAKE) 紧急刹车
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/* ==================== 电机编号 ==================== */
#define MOTOR_LEFT      1   /**< 左电机 */
#define MOTOR_RIGHT     2   /**< 右电机 */

/* ==================== 停机模式 ==================== */
#define MOTOR_COAST     0   /**< 自由滑行 (IN1=0, IN2=0) */
#define MOTOR_BRAKE     1   /**< 短路刹车 (IN1=1, IN2=1) */

/* ==================== PWM 限幅 ==================== */
#define MOTOR_PWM_MAX   8000    /**< 最大占空比 (对应 period=8000, 5kHz) */
#define MOTOR_PWM_MIN   (-8000) /**< 最小占空比 */

/* ==================== 速度环限幅（双环PID内环） ==================== */
#define MOTOR_SPEED_PWM_MAX   8000.0f
#define MOTOR_SPEED_PWM_MIN  (-8000.0f)

/* ==================== 电机速度控制结构体（双环PID内环） ==================== */
typedef struct {
    int16_t target_speed;     // 目标速度（外环设定，编码器脉冲/20ms）
    int16_t actual_speed;     // 实际速度（编码器测量，5点滤波后）
    float   pwm;              // PID输出PWM值
    float   error;            // 当前误差
    float   last_error;       // 上次误差（用于微分项）
    float   integral;         // 积分累加（带限幅）
} Motor_Speed_t;

/* 速度环结构体实例 */
extern Motor_Speed_t motor_l_ctrl;
extern Motor_Speed_t motor_r_ctrl;

/* ==================== 速度环PID参数（可调） ==================== */
extern float Motor_Kp;
extern float Motor_Ki;
extern float Motor_Kd;

/* ==================== 位置环PD参数（外环循迹） ==================== */
extern float Track_Kp;
extern float Track_Kd;

/* ==================== 函数声明（双环PID） ==================== */
void Motor_Speed_Init(void);           // 初始化速度PID状态
void Motor_Speed_PID_Reset(void);      // 重置速度PID状态
void Motor_Speed_PID_Update(void);     // 内环：速度PID更新（20ms中断中调用）
void Motor_SetTargetSpeed(int16_t left, int16_t right);  // 设置目标速度
void Racecar(float base_speed, float deviation);          // 外环：差速循迹控制

/* ==================== 函数声明（原有） ==================== */

/**
 * @brief 初始化两路 AT8236 电机驱动
 * @details 配置 PB0(P左IN1) / PB14(右IN2) 为推挽输出，默认低电平。
 *          PWM 已在 SysConfig 中由 SYSCFG_DL_init() 初始化。
 */
void Motor_Init(void);

/**
 * @brief 设置指定电机的 PWM 占空比和转向
 * @param n    电机编号：MOTOR_LEFT(1) 或 MOTOR_RIGHT(2)
 * @param PWM  占空比及转向控制：
 *             - 正数 (1~8000)：小车前进方向
 *             - 负数 (-8000~-1)：小车后退方向
 *             - 0：自由滑行停止
 *
 * @note 左右电机的 AT8236 硬件接线不对称，函数内部自动适配：
 *       - 左轮正转 = 前进(慢衰减)，左轮反转 = 后退(快衰减)
 *       - 右轮正转 = 后退(快衰减)，右轮反转 = 前进(慢衰减)
 *       对外接口统一为：PWM>0=前进，PWM<0=后退。
 */
void Motor_SetPWM(uint8_t n, int16_t PWM);

/**
 * @brief 停止指定电机
 * @param n    电机编号：MOTOR_LEFT(1) 或 MOTOR_RIGHT(2)
 * @param mode 停机模式：MOTOR_COAST(自由滑行) 或 MOTOR_BRAKE(短路刹车)
 */
void Motor_Stop(uint8_t n, uint8_t mode);

/**
 * @brief 停止所有电机
 * @param mode 停机模式：MOTOR_COAST(自由滑行) 或 MOTOR_BRAKE(短路刹车)
 */
void Motor_StopAll(uint8_t mode);

/* ==================== 距离计算 ==================== */

/** @brief 每次脉冲对应的距离（mm），由物理参数自动计算 */
extern const float Motor_Distance_Per_Pulse;

/**
 * @brief  根据编码器脉冲数计算行走距离（mm）
 * @param  pulse 编码器脉冲数（正=前进，负=后退）
 * @return 距离（mm）
 */
float Motor_PulseToDistance(int16_t pulse);

/**
 * @brief  累加左右轮平均行走距离
 * @param  left_pulse  左编码器脉冲数（Encoder_GetCount(ENCODER_LEFT)）
 * @param  right_pulse 右编码器脉冲数（Encoder_GetCount(ENCODER_RIGHT)）
 * @note   在主循环中每帧调用，自动累加行走总距离
 */
void Motor_AddDistance(int16_t left_pulse, int16_t right_pulse);

/**
 * @brief  获取累加行走总距离（mm）
 */
float Motor_GetDistance(void);

/**
 * @brief  重置累加距离为 0
 */
void Motor_ResetDistance(void);

#endif
