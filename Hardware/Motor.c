/**
 * @file    Motor.c
 * @brief   AT8236 直流电机驱动实现
 * @details 通过硬件 PWM + GPIO 方向引脚驱动两路 AT8236 H 桥芯片。
 *          方向引脚由 SysConfig 配置为 GPIO 推挽输出（SYSCFG_DL_init 完成初始化）。
 *          PWM 也由 SysConfig 配置（PWM_A=TIMA1_CCP1/PB1, PWM_B=TIMG8_CCP0/PB15）。
 *
 * SysConfig 引脚分配：
 *   左电机：IN1=PB4 (Motors_AIN, GPIO 方向), IN2=PB1 (PWM_A, 调速)
 *   右电机：IN1=PB15 (PWM_B, 调速), IN2=PB16 (Motors_BIN, GPIO 方向)
 *
 * 控制逻辑（AT8236，接口互换后）：
 *   IN1=1, IN2=PWM → 正转慢衰减  ← 左轮前进（左驱动→右轮）
 *   IN1=PWM, IN2=1 → 反转慢衰减  ← 右轮前进（右驱动→左轮）
 *   IN1=0, IN2=PWM → 反转快衰减  ← 左轮后退
 *   IN1=PWM, IN2=0 → 正转快衰减  ← 右轮后退
 *   IN1=1, IN2=1 → 短路刹车
 *   IN1=0, IN2=0 → 自由滑行
 *
 * 参考工程：STM32_CAR/NEW 8.0 基础驱动 双环循迹+频率云台
 */

#include "ti_msp_dl_config.h"
#include "Motor.h"
#include "Encoder.h"

/* ==================== 引脚重映射（对应 SysConfig 命名） ==================== */

/* 左电机：IN1 = 方向 = Motors_AIN (PB4) */
#define LEFT_DIR_PORT           Motors_PORT
#define LEFT_DIR_PIN            Motors_AIN_PIN

/* 右电机：IN2 = 方向 = Motors_BIN (PB16) */
#define RIGHT_DIR_PORT          Motors_PORT
#define RIGHT_DIR_PIN           Motors_BIN_PIN

/* ==================== 速度控制结构体实例 ==================== */
Motor_Speed_t motor_l_ctrl = {0};
Motor_Speed_t motor_r_ctrl = {0};

/* ==================== 速度环PID参数（内环） ==================== */
float Motor_Kp = 130.0f;
float Motor_Ki = 20.0f;
float Motor_Kd = 15.0f;

/* ==================== 位置环PD参数（外环循迹） ==================== */
float Track_Kp = 1.2f;
float Track_Kd = 0.0f;

static float last_deviation = 0;

/* ==================== 距离计算参数 ==================== */
const float Motor_Distance_Per_Pulse = 3.14159265f * 68.0f / (28.0f * 26.0f);
static float g_total_distance_mm = 0.0f;

/* ==================== Motor_Init ==================== */

void Motor_Init(void)
{
    /* 方向引脚已由 SYSCFG_DL_init() 完成 GPIO 初始化，默认低电平。
     * 此处仅做二次确认：确保方向引脚为低电平（电机停止/滑行状态）。 */
    DL_GPIO_clearPins(LEFT_DIR_PORT, LEFT_DIR_PIN);
    DL_GPIO_clearPins(RIGHT_DIR_PORT, RIGHT_DIR_PIN);
}

/* ==================== Motor_SetPWM ==================== */

void Motor_SetPWM(uint8_t n, int16_t PWM)
{
    int16_t pwm_val = PWM;

    /* 限幅保护 */
    if (pwm_val > MOTOR_PWM_MAX)  pwm_val = MOTOR_PWM_MAX;
    if (pwm_val < MOTOR_PWM_MIN)  pwm_val = MOTOR_PWM_MIN;

    /* 死区处理：慢衰减(前进)低速段电流不连续导致震荡，低于阈值直接停。
     * 快衰减(后退)电流连续响应线性，不加死区。 */
    if (pwm_val > 0 && pwm_val < MOTOR_PWM_DEAD)  pwm_val = 0;

    /* 左右驱动接口互换后，重映射电机编号 */
    if (n == MOTOR_LEFT)
        n = MOTOR_RIGHT;
    else if (n == MOTOR_RIGHT)
        n = MOTOR_LEFT;

    /* ---------- 左电机 (MOTOR_LEFT) — 接口互换后驱动右轮 ---------- */
    /* 接线：IN1=PB4(GPIO), IN2=PB1(PWM_A/CCP1)
     *   PWM>0(前进): IN1=1, IN2=PWM → 正转慢衰减 → 右轮前进
     *   PWM<0(后退): IN1=0, IN2=PWM → 反转快衰减 → 右轮后退
     *   PWM=0(停止): IN1=0, IN2=0    → 自由滑行
     *
     * PWM 输出极性 (INIT_VAL_LOW)：count<CC → HIGH, count≥CC → LOW
     * 慢衰减(IN1=1): LOW→驱动 → CC取反(MAX-pwm)
     * 快衰减(IN1=0): HIGH→驱动 → CC直通(pwm) */
    if (n == MOTOR_LEFT)
    {
        if (pwm_val > 0)
        {
            DL_GPIO_setPins(LEFT_DIR_PORT, LEFT_DIR_PIN);           /* IN1 = 1 → 慢衰减 */
            DL_TimerA_setCaptureCompareValue(PWM_A_INST,
                (uint16_t)(MOTOR_PWM_MAX - pwm_val),                /* CC取反: LOW→驱动 */
                DL_TIMER_CC_1_INDEX);                               /* IN2 = PWM */
        }
        else if (pwm_val < 0)
        {
            DL_GPIO_clearPins(LEFT_DIR_PORT, LEFT_DIR_PIN);         /* IN1 = 0 → 快衰减 */
            DL_TimerA_setCaptureCompareValue(PWM_A_INST,
                (uint16_t)(-pwm_val),                               /* CC直通: HIGH→驱动 */
                DL_TIMER_CC_1_INDEX);                               /* IN2 = PWM */
        }
        else
        {
            DL_GPIO_clearPins(LEFT_DIR_PORT, LEFT_DIR_PIN);         /* IN1 = 0 */
            DL_TimerA_setCaptureCompareValue(PWM_A_INST,
                0, DL_TIMER_CC_1_INDEX);                            /* IN2 = 0 */
        }
    }

    /* ---------- 右电机 (MOTOR_RIGHT) — 接口互换后驱动左轮 ---------- */
    /* 接线：IN1=PB15(PWM_B/CCP0), IN2=PB16(GPIO)
     *   PWM>0(前进): IN1=PWM, IN2=1 → 反转慢衰减 → 左轮前进
     *   PWM<0(后退): IN1=PWM, IN2=0 → 正转快衰减 → 左轮后退
     *   PWM=0(停止): IN1=0, IN2=0    → 自由滑行
     *
     * PWM 输出极性 (INIT_VAL_LOW)：count<CC → HIGH, count≥CC → LOW
     * 慢衰减(IN2=1): LOW→驱动 → CC取反(MAX-pwm)
     * 快衰减(IN2=0): HIGH→驱动 → CC直通(pwm) */
    else if (n == MOTOR_RIGHT)
    {
        if (pwm_val > 0)
        {
            DL_GPIO_setPins(RIGHT_DIR_PORT, RIGHT_DIR_PIN);         /* IN2 = 1 → 慢衰减 */
            DL_TimerG_setCaptureCompareValue(PWM_B_INST,
                (uint16_t)(MOTOR_PWM_MAX - pwm_val),                /* CC取反: LOW→驱动 */
                DL_TIMER_CC_0_INDEX);                               /* IN1 = PWM */
        }
        else if (pwm_val < 0)
        {
            DL_GPIO_clearPins(RIGHT_DIR_PORT, RIGHT_DIR_PIN);       /* IN2 = 0 → 快衰减 */
            DL_TimerG_setCaptureCompareValue(PWM_B_INST,
                (uint16_t)(-pwm_val),                               /* CC直通: HIGH→驱动 */
                DL_TIMER_CC_0_INDEX);                               /* IN1 = PWM */
        }
        else
        {
            DL_GPIO_clearPins(RIGHT_DIR_PORT, RIGHT_DIR_PIN);       /* IN2 = 0 */
            DL_TimerG_setCaptureCompareValue(PWM_B_INST,
                0, DL_TIMER_CC_0_INDEX);                            /* IN1 = 0 */
        }
    }
}

/* ==================== Motor_Stop ==================== */

void Motor_Stop(uint8_t n, uint8_t mode)
{
    if (n == MOTOR_LEFT)
    {
        if (mode == MOTOR_BRAKE)
        {
            DL_GPIO_setPins(LEFT_DIR_PORT, LEFT_DIR_PIN);           /* IN1 = 1 */
            DL_TimerA_setCaptureCompareValue(PWM_A_INST,
                MOTOR_PWM_MAX, DL_TIMER_CC_1_INDEX);                /* IN2 = 1 (满占空比=高) */
        }
        else
        {
            DL_GPIO_clearPins(LEFT_DIR_PORT, LEFT_DIR_PIN);         /* IN1 = 0 */
            DL_TimerA_setCaptureCompareValue(PWM_A_INST,
                0, DL_TIMER_CC_1_INDEX);                            /* IN2 = 0 */
        }
    }
    else if (n == MOTOR_RIGHT)
    {
        if (mode == MOTOR_BRAKE)
        {
            DL_GPIO_setPins(RIGHT_DIR_PORT, RIGHT_DIR_PIN);         /* IN2 = 1 */
            DL_TimerG_setCaptureCompareValue(PWM_B_INST,
                MOTOR_PWM_MAX, DL_TIMER_CC_0_INDEX);               /* IN1 = 1 */
        }
        else
        {
            DL_GPIO_clearPins(RIGHT_DIR_PORT, RIGHT_DIR_PIN);       /* IN2 = 0 */
            DL_TimerG_setCaptureCompareValue(PWM_B_INST,
                0, DL_TIMER_CC_0_INDEX);                           /* IN1 = 0 */
        }
    }
}

/* ==================== Motor_StopAll ==================== */

void Motor_StopAll(uint8_t mode)
{
    Motor_Stop(MOTOR_LEFT, mode);
    Motor_Stop(MOTOR_RIGHT, mode);
}

/* ===================== 速度控制函数 ===================== */

void Motor_Speed_Init(void)
{
    motor_l_ctrl.target_speed = 0;
    motor_l_ctrl.actual_speed = 0;
    motor_l_ctrl.pwm = 0;
    motor_l_ctrl.error = 0;
    motor_l_ctrl.last_error = 0;
    motor_l_ctrl.integral = 0;

    motor_r_ctrl.target_speed = 0;
    motor_r_ctrl.actual_speed = 0;
    motor_r_ctrl.pwm = 0;
    motor_r_ctrl.error = 0;
    motor_r_ctrl.last_error = 0;
    motor_r_ctrl.integral = 0;

    last_deviation = 0;
}

void Motor_Speed_PID_Reset(void)
{
    motor_l_ctrl.error = 0;
    motor_l_ctrl.last_error = 0;
    motor_l_ctrl.integral = 0;

    motor_r_ctrl.error = 0;
    motor_r_ctrl.last_error = 0;
    motor_r_ctrl.integral = 0;

    last_deviation = 0;
}

void Motor_SetTargetSpeed(int16_t left, int16_t right)
{
    motor_l_ctrl.target_speed = left;
    motor_r_ctrl.target_speed = right;
}

/**
 * @brief  速度环PID更新（内环，在20ms定时中断中调用）
 * @details error = target - actual
 *          integral += error（±500限幅抗积分饱和）
 *          output = Kp*error + Ki*integral + Kd*(error - last_error)
 *          限幅到 ±MOTOR_SPEED_PWM_MAX 后输出到 Motor_SetPWM
 */
void Motor_Speed_PID_Update(void)
{
    /* 读取编码器实际速度（5点滑动平均滤波后） */
    motor_l_ctrl.actual_speed = Encoder_GetSpeed(ENCODER_LEFT);
    motor_r_ctrl.actual_speed = Encoder_GetSpeed(ENCODER_RIGHT);

    /* ---- 左电机PID ---- */
    motor_l_ctrl.error = (float)(motor_l_ctrl.target_speed - motor_l_ctrl.actual_speed);
    motor_l_ctrl.integral += motor_l_ctrl.error;
    if (motor_l_ctrl.integral > 4000.0f)  motor_l_ctrl.integral = 4000.0f;
    if (motor_l_ctrl.integral < -4000.0f) motor_l_ctrl.integral = -4000.0f;

    float output_l = Motor_Kp * motor_l_ctrl.error
                   + Motor_Ki * motor_l_ctrl.integral
                   + Motor_Kd * (motor_l_ctrl.error - motor_l_ctrl.last_error);
    motor_l_ctrl.last_error = motor_l_ctrl.error;

    if (output_l > MOTOR_SPEED_PWM_MAX)  output_l = MOTOR_SPEED_PWM_MAX;
    if (output_l < MOTOR_SPEED_PWM_MIN)  output_l = MOTOR_SPEED_PWM_MIN;
    motor_l_ctrl.pwm = output_l;

    /* ---- 右电机PID ---- */
    motor_r_ctrl.error = (float)(motor_r_ctrl.target_speed - motor_r_ctrl.actual_speed);
    motor_r_ctrl.integral += motor_r_ctrl.error;
    if (motor_r_ctrl.integral > 4000.0f)  motor_r_ctrl.integral = 4000.0f;
    if (motor_r_ctrl.integral < -4000.0f) motor_r_ctrl.integral = -4000.0f;

    float output_r = Motor_Kp * motor_r_ctrl.error
                   + Motor_Ki * motor_r_ctrl.integral
                   + Motor_Kd * (motor_r_ctrl.error - motor_r_ctrl.last_error);
    motor_r_ctrl.last_error = motor_r_ctrl.error;

    if (output_r > MOTOR_SPEED_PWM_MAX)  output_r = MOTOR_SPEED_PWM_MAX;
    if (output_r < MOTOR_SPEED_PWM_MIN)  output_r = MOTOR_SPEED_PWM_MIN;
    motor_r_ctrl.pwm = output_r;

    /* ---- 输出PWM到电机 ---- */
    Motor_SetPWM(MOTOR_LEFT, (int16_t)output_l);
    Motor_SetPWM(MOTOR_RIGHT, (int16_t)output_r);
}

/**
 * @brief  差速循迹控制（外环，在主循环中调用）
 * @param  base_speed  基础速度
 * @param  deviation   灰度偏差值：
 *                     >0 线偏右（车偏左），<0 线偏左（车偏右）
 * @details target_l = base + value
 *          target_r = base - value
 *          线偏右(dev>0)→value正→左轮快+右轮慢→车右转修正
 */
void Racecar(float base_speed, float deviation)
{
    float value = deviation * Track_Kp + Track_Kd * (deviation - last_deviation);
    last_deviation = deviation;

    int16_t target_l = (int16_t)(base_speed + value);
    int16_t target_r = (int16_t)(base_speed - value);

    Motor_SetTargetSpeed(target_l, target_r);
}

/* ==================== 距离计算函数 ==================== */

float Motor_PulseToDistance(int16_t pulse)
{
    return (float)pulse * Motor_Distance_Per_Pulse;
}

void Motor_AddDistance(int16_t left_pulse, int16_t right_pulse)
{
    /* 左右轮脉冲平均值换算为车体中心总里程 */
    g_total_distance_mm += ((float)left_pulse + (float)right_pulse) / 2.0f * Motor_Distance_Per_Pulse;
}

float Motor_GetDistance(void)
{
    return g_total_distance_mm;
}

void Motor_ResetDistance(void)
{
    g_total_distance_mm = 0.0f;
}
