/**
 * @file    Grayscale.c
 * @brief   8 路灰度传感器驱动
 * @details 实现 8 路灰度传感器的数据读取和加权偏差计算
 *
 * 硬件引脚（GPIO 由 SysConfig 初始化）：
 *   Gray_1（最右）：GPIOB.6
 *   Gray_2：       GPIOB.7
 *   Gray_3：       GPIOB.8
 *   Gray_4：       GPIOB.9
 *   Gray_5：       GPIOB.10
 *   Gray_6：       GPIOB.11
 *   Gray_7：       GPIOB.12
 *   Gray_8（最左）：GPIOB.18
 *
 * 函数清单：
 *   - Gray_Sensor_Init()      : 初始化 GPIO（SysConfig 已完成，本函数保留接口）
 *   - Gray_Sensor_Read()      : 读取 8 路灰度传感器状态到全局变量
 *   - Grayscale_GetDeviation(): 计算灰度加权偏差值
 *
 * 加权偏差算法：
 *   偏差 = 加权和 / 检测到黑线的数量
 *   权重：Gray_8=-4, Gray_7=-3, Gray_6=-2, Gray_5=-1,
 *         Gray_4=+1, Gray_3=+2, Gray_2=+3, Gray_1=+4
 *   >0 表示线偏右侧（车头偏左，需左转）
 *   <0 表示线偏左侧（车头偏右，需右转）
 *    0 表示在中间 或 全白离线
 *
 * 使用方式：
 *   1. 调用 Gray_Sensor_Init()（可选，SysConfig 已初始化）
 *   2. 在循环中调用 Grayscale_GetDeviation() 直接获取偏差值
 *      或调用 Gray_Sensor_Read() 后读取 Gray_1 ~ Gray_8 全局变量
 */

#include "Grayscale.h"

/* ===================== 全局变量 ===================== */
uint8_t Gray_1 = 0;   // 最右边灰度
uint8_t Gray_2 = 0;
uint8_t Gray_3 = 0;
uint8_t Gray_4 = 0;
uint8_t Gray_5 = 0;
uint8_t Gray_6 = 0;
uint8_t Gray_7 = 0;
uint8_t Gray_8 = 0;   // 最左边灰度

/**
  * @brief  初始化8路灰度传感器GPIO
  * @details GPIO 已在 SYSCFG_DL_init() 中通过 SysConfig 配置为上拉数字输入
  *          本函数仅作为接口预留，硬件初始化由 SysConfig 自动完成
  *          参考 STM32 标准库 Gray_Sensor_Init() 风格
  */
void Gray_Sensor_Init(void)
{
    /* GPIO 初始化已由 SysConfig 在 SYSCFG_DL_init() 中完成：
     *   DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM23~29, PINCM44,
     *       DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
     *       DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
     * 此处保留函数声明以保持与 STM32 参考工程接口一致
     */
}

/**
  * @brief  读取8路灰度传感器数据
  * @details 保存到全局变量 Gray_1 ~ Gray_8
  *           Gray_1 对应最右边传感器，Gray_8 对应最左边传感器
  */
void Gray_Sensor_Read(void)
{
    Gray_1 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_1_PIN) ? 1 : 0;
    Gray_2 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_2_PIN) ? 1 : 0;
    Gray_3 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_3_PIN) ? 1 : 0;
    Gray_4 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_4_PIN) ? 1 : 0;
    Gray_5 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_5_PIN) ? 1 : 0;
    Gray_6 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_6_PIN) ? 1 : 0;
    Gray_7 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_7_PIN) ? 1 : 0;
    Gray_8 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_8_PIN) ? 1 : 0;
}

/**
  * @brief  计算灰度传感器加权偏差值（使用全部8路）
  * @retval 偏差值（加权平均）
  *         >0 表示线偏右侧（车头偏左，需左转）
  *         <0 表示线偏左侧（车头偏右，需右转）
  *         0 表示在中间 或 全白离线
  * @note   参考 STM32 标准库工程 Grayscale_GetDeviation() 实现
  *         偏差 = 加权和 / 检测到黑线的数量
  */
float Grayscale_GetDeviation(void)
{
    Gray_Sensor_Read();

    float weighted_sum = 0;
    uint8_t active_count = 0;

    if (Gray_8 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_8; active_count++; }
    if (Gray_7 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_7; active_count++; }
    if (Gray_6 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_6; active_count++; }
    if (Gray_5 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_5; active_count++; }
    if (Gray_4 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_4; active_count++; }
    if (Gray_3 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_3; active_count++; }
    if (Gray_2 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_2; active_count++; }
    if (Gray_1 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_1; active_count++; }

    if (active_count == 0) return 0;   // 全白离线，偏差返回0

    return weighted_sum / active_count;
}

/* ===================== 循迹控制：包含 Motor.h 以下内容 ===================== */
#include "Motor.h"
#include "Timer.h"

/* ===================== 循迹全局变量 ===================== */
uint8_t RunFlag = 0;
int16_t PWML = 0;
int16_t PWMR = 0;
uint8_t Turn_State = TURN_IDLE;
uint8_t Turn_Direction = 0;    /* 1=左弯，2=右弯 */

/* ===================== 软启动参数 ===================== */
#define SOFT_START_STEP      6
#define SOFT_START_INTERVAL 50

static uint8_t  soft_start_active = 0;
static int16_t  soft_current_speed = 0;
static int16_t  soft_target_speed = 0;
static uint16_t soft_start_counter = 0;
static uint8_t  soft_was_running = 0;

#define TURN_SOFT_START_STEP      5
#define TURN_SOFT_START_INTERVAL  30

static uint8_t  turn_soft_active = 0;
static int16_t  turn_soft_current = 0;
static uint16_t turn_soft_counter = 0;

/* ===================== 拐弯变量 ===================== */
static uint32_t Turn_Start_Tick = 0;

/* ===================== 蓝牙可调参数（默认值） ===================== */
uint16_t BaseSpeed = 20;  /* 默认值，蓝牙 [slider,BSp,xxx] 可在线修改 */
uint16_t TurnDecelTime = 200;
uint16_t TurnPower = 35;
uint16_t DecelSpeed = 65;
uint16_t JoyTurn = 15;     /* 摇杆转向倍率，蓝牙 [slider,JTurn,15] 可调 */

/**
 * @brief  计算循迹偏差（仅用中间6路传感器 Gray_2~Gray_7）
 * @retval 偏差值
 *         Gray_8(Gray_8)=最左拐弯检测，Gray_1=最右拐弯检测
 *         中间6路(Gray_7~Gray_2)用于偏差计算
 */
float Grayscale_GetDeviation_Track(void)
{
    Gray_Sensor_Read();

    float weighted_sum = 0;
    uint8_t active_count = 0;

    /* 中间6路：Gray_7=-3, Gray_6=-2, Gray_5=-1, Gray_4=+1, Gray_3=+2, Gray_2=+3 */
    if (Gray_7 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_7; active_count++; }
    if (Gray_6 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_6; active_count++; }
    if (Gray_5 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_5; active_count++; }
    if (Gray_4 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_4; active_count++; }
    if (Gray_3 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_3; active_count++; }
    if (Gray_2 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_2; active_count++; }

    if (active_count == 0) return 0;

    return weighted_sum / active_count;
}

/**
 * @brief  灰度传感器循迹控制主函数
 * @details 三段式拐弯状态机（适配8路传感器）：
 *          Gray_8(最左)/Gray_1(最右) 作为拐弯检测
 *          中间6路(Gray_7~Gray_2) 用于PD偏差计算
 *
 *          状态机：
 *          TURN_IDLE:   PD微调循迹
 *          TURN_DECEL:  外侧触发→减速直行
 *          TURN_ACTIVE: 差速拐弯，直到中间6路捕获到黑线
 */
void Gray_Track_Control(void)
{
    Gray_Sensor_Read();

    /* ---- 保护：8个全黑=压线/出界保护 ---- */
    if (Gray_1 == GRAY_BLACK && Gray_2 == GRAY_BLACK &&
        Gray_3 == GRAY_BLACK && Gray_4 == GRAY_BLACK &&
        Gray_5 == GRAY_BLACK && Gray_6 == GRAY_BLACK &&
        Gray_7 == GRAY_BLACK && Gray_8 == GRAY_BLACK)
    {
        Motor_SetTargetSpeed(0, 0);
        return;
    }

//    /* ============ 拐弯状态机（暂注释，等PID调好再开） ============ */
//    if (Turn_State == TURN_ACTIVE && ... ) { ... }
//    if (Turn_State == TURN_IDLE && ... ) { ... }
//    if (Turn_State == TURN_DECEL) { ... }
//    else if (Turn_State == TURN_ACTIVE) { ... }
//    else { ... PD循迹 ... }

    /* ============ RunFlag 上升沿检测：启动软启动 ============ */
    if (RunFlag && !soft_was_running)
    {
        soft_was_running = 1;
        soft_target_speed = BaseSpeed;
        soft_current_speed = 0;
        soft_start_active = 1;
        soft_start_counter = 0;
    }

    /* ============ 基本PD循迹（跳过拐弯状态机，仅用于PID调试） ============ */
    float deviation = Grayscale_GetDeviation_Track();
    int16_t speed = soft_start_active ? soft_current_speed : (int16_t)BaseSpeed;
    Racecar((float)speed, deviation);

    /* ---- 更新用于显示的 PWM 值 ---- */
    PWML = (int16_t)motor_l_ctrl.target_speed;
    PWMR = (int16_t)motor_r_ctrl.target_speed;
}

/**
 * @brief   软启动 Tick 函数（1ms 中断中调用）
 * @details 每 SOFT_START_INTERVAL(50ms) 增加一次速度，
 *          直到达到目标速度 BaseSpeed。
 *          同时处理拐弯软启动：每 TURN_SOFT_START_INTERVAL(30ms) 增加。
 */
void Gray_SoftStart_Tick(void)
{
    if (soft_start_active)
    {
        soft_start_counter++;
        if (soft_start_counter >= SOFT_START_INTERVAL)
        {
            soft_start_counter = 0;
            soft_current_speed += SOFT_START_STEP;
            if (soft_current_speed >= soft_target_speed)
            {
                soft_current_speed = soft_target_speed;
                soft_start_active = 0;
            }
        }
    }

    if (turn_soft_active)
    {
        turn_soft_counter++;
        if (turn_soft_counter >= TURN_SOFT_START_INTERVAL)
        {
            turn_soft_counter = 0;
            turn_soft_current += TURN_SOFT_START_STEP;
            if (turn_soft_current >= TurnPower)
            {
                turn_soft_current = TurnPower;
                turn_soft_active = 0;
            }
        }
    }
}

/**
 * @brief   重置软启动状态（按键停止时调用）
 */
void Gray_SoftStart_Reset(void)
{
    soft_was_running = 0;
    soft_start_active = 0;
    soft_current_speed = 0;
    soft_start_counter = 0;
    turn_soft_active = 0;
}
