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
 * 
 * 参数调节在六十多行，但是直行的PID在Motor.c里面
 * 
 */

#include "Grayscale.h"
#include "Motor.h"
#include "Timer.h"

/* ===================== 灰度传感器全局变量 ===================== */
uint8_t Gray_1 = 0;   // 最右边灰度
uint8_t Gray_2 = 0;
uint8_t Gray_3 = 0;
uint8_t Gray_4 = 0;
uint8_t Gray_5 = 0;
uint8_t Gray_6 = 0;
uint8_t Gray_7 = 0;
uint8_t Gray_8 = 0;   // 最左边灰度

/* ===================== 循迹控制全局变量 ===================== */
uint8_t RunFlag = 0;            /* 循迹运行标志 */
int16_t PWML = 0;               /* 左电机当前PWM值（显示用） */
int16_t PWMR = 0;               /* 右电机当前PWM值（显示用） */

/* ===================== 循迹模式 ===================== */
uint8_t Turn_Mode = 2;          /* 1=三段式拐弯,适合直角拐弯； 2=纯PD循迹(8路全参与,默认)，适用于曲线拐弯 */
uint8_t Turn_State = TURN_IDLE; /* 模式1状态机状态 */
uint8_t Turn_Direction = 0;     /* 模式1拐弯方向（1=左，2=右） */
uint32_t Turn_Start_Tick = 0;   /* 模式1拐弯触发时刻 */
uint8_t Gray_LineFound = 0;     /* 0=寻线中直行, 1=已捕获黑线进入循迹 */
uint8_t  M2_State     = M2_IDLE;/* 模式2状态机状态， M2_IDLE=直行， M2_TURN=弯 */

/* ===================== 通用蓝牙可调参数（两模式共用） ===================== */
uint16_t BaseSpeed = 30;        /* BSp   直行基础速度 */
uint16_t SoftStartTime =500;  /* SST   起步/出弯软加速时间（ms），越大加速越慢 */

/* ===================== 模式1参数（三段式拐弯） ===================== */
uint16_t TurnDecelTime = 150;   /* TdT   拐弯减速平滑时间（ms） */
uint16_t TurnPower = 10;        /* TdP   拐弯差速幅值 */
uint16_t DecelSpeed = 1000;     /* TdS   拐弯前减速目标速度 */
uint16_t JoyTurn = 10;          /* JTurn 摇杆转向倍率 */

/* ===================== 模式2参数（纯PD循迹） ===================== */
float TurnSlowRatio = 0.15f;    /* TSlo  过弯降速比例：越大拐弯基础速度越慢，为0时是以基础速度拐弯 */
float Gray_WeightScale = 1.5f;  /* WSc   Gray_1/8 乘 WSc, Gray_2/7 乘 (WSc+1)/2，放大纠偏力度 */

/* ===================== 速度斜坡实例（内部） ===================== */
static SpeedRamp_t soft_ramp;    /* 起步/出弯软启动：0 → BaseSpeed */
static SpeedRamp_t decel_ramp;   /* 转弯减速：当前速度 → 目标速度 */
static uint8_t  soft_was_running = 0;  /* 上次 RunFlag 状态（上升沿检测用） */

/**
  * @brief  初始化8路灰度传感器GPIO
  * @details 在 SysConfig 初始化基础上，将引脚改为下拉输入，
  *          使拔线时引脚固定为 LOW（GRAY_WHITE），
  *          确保全白丢线检测正确触发
  */
void Gray_Sensor_Init(void)
{
    /* SysConfig 已配置为 RESISTOR_NONE，此处重设为下拉 */
    DL_GPIO_setDigitalInternalResistor(Gray_Gray_1_IOMUX, DL_GPIO_RESISTOR_PULL_DOWN);
    DL_GPIO_setDigitalInternalResistor(Gray_Gray_2_IOMUX, DL_GPIO_RESISTOR_PULL_DOWN);
    DL_GPIO_setDigitalInternalResistor(Gray_Gray_3_IOMUX, DL_GPIO_RESISTOR_PULL_DOWN);
    DL_GPIO_setDigitalInternalResistor(Gray_Gray_4_IOMUX, DL_GPIO_RESISTOR_PULL_DOWN);
    DL_GPIO_setDigitalInternalResistor(Gray_Gray_5_IOMUX, DL_GPIO_RESISTOR_PULL_DOWN);
    DL_GPIO_setDigitalInternalResistor(Gray_Gray_6_IOMUX, DL_GPIO_RESISTOR_PULL_DOWN);
    DL_GPIO_setDigitalInternalResistor(Gray_Gray_7_IOMUX, DL_GPIO_RESISTOR_PULL_DOWN);
    DL_GPIO_setDigitalInternalResistor(Gray_Gray_8_IOMUX, DL_GPIO_RESISTOR_PULL_DOWN);
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

    float ws = Gray_WeightScale;                        /* 最外层倍率 */
    float ws2 = ws * 0.75f + 0.25f;                    /* 稍外层倍率（靠近 ws 侧） */
    float weighted_sum = 0;
    uint8_t active_count = 0;

    if (Gray_8 == GRAY_BLACK) { weighted_sum += (float)GRAY_WEIGHT_8 * ws;  active_count++; }
    if (Gray_7 == GRAY_BLACK) { weighted_sum += (float)GRAY_WEIGHT_7 * ws2; active_count++; }
    if (Gray_6 == GRAY_BLACK) { weighted_sum += (float)GRAY_WEIGHT_6;       active_count++; }
    if (Gray_5 == GRAY_BLACK) { weighted_sum += (float)GRAY_WEIGHT_5;       active_count++; }
    if (Gray_4 == GRAY_BLACK) { weighted_sum += (float)GRAY_WEIGHT_4;       active_count++; }
    if (Gray_3 == GRAY_BLACK) { weighted_sum += (float)GRAY_WEIGHT_3;       active_count++; }
    if (Gray_2 == GRAY_BLACK) { weighted_sum += (float)GRAY_WEIGHT_2 * ws2; active_count++; }
    if (Gray_1 == GRAY_BLACK) { weighted_sum += (float)GRAY_WEIGHT_1 * ws;  active_count++; }

    if (active_count == 0) return 0;   // 全白离线，偏差返回0

    return weighted_sum / active_count;
}

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

    /* ============ RunFlag 上升沿检测：启动软启动 + 进入寻线阶段 ============ */
    if (RunFlag && !soft_was_running)
    {
        soft_was_running = 1;
        Gray_LineFound = 0;     /* 开始直行寻线 */
        M2_State = M2_IDLE;     /* 模式2状态机复位 */
        SpeedRamp_Start(&soft_ramp, 0, (int16_t)BaseSpeed, SoftStartTime);
    }

    /* ============ 模式2：两段状态机（基于触发灰度判定） ============ */
    /*
     * M2_IDLE - 直行：只有中间 Gray_3/4/5/6 见线，全速 PD 循迹
     * M2_TURN - 转弯：Gray_1/2/7/8 中任意有黑线，降速 PD 过弯
     */
    if (Turn_Mode == 2)
    {
        float deviation = Grayscale_GetDeviation();

        /* ---- 全白处理：区分寻线中和循迹中丢线 ---- */
        if (Gray_1 != GRAY_BLACK && Gray_2 != GRAY_BLACK &&
            Gray_3 != GRAY_BLACK && Gray_4 != GRAY_BLACK &&
            Gray_5 != GRAY_BLACK && Gray_6 != GRAY_BLACK &&
            Gray_7 != GRAY_BLACK && Gray_8 != GRAY_BLACK)
        {
            if (Gray_LineFound)
            {
                /* 循迹中丢线：保持上一次左右轮速度 */
                goto track_done;
            }
            else
            {
                /* 寻线中：直行，软加速到 BaseSpeed */
                int16_t speed = SpeedRamp_IsActive(&soft_ramp) ? SpeedRamp_Update(&soft_ramp) : (int16_t)BaseSpeed;
                Motor_SetTargetSpeed(speed, speed);
                goto track_done;
            }
        }
        Gray_LineFound = 1;     /* 捕获到黑线，进入循迹 */

        /* ---- 判定当前状态：基于触发的灰度 ---- */
        uint8_t outer = (Gray_1 == GRAY_BLACK || Gray_2 == GRAY_BLACK ||
                         Gray_7 == GRAY_BLACK || Gray_8 == GRAY_BLACK);

        if (outer)// 外侧四个灰度触发
        {
            /* 外侧触发 → 转入弯道 */
            if (M2_State != M2_TURN)
            {
                M2_State = M2_TURN;
                int16_t avg = (int16_t)((motor_l_ctrl.target_speed + motor_r_ctrl.target_speed) / 2);
                int16_t turn_spd = (int16_t)(BaseSpeed * (1.0f - TurnSlowRatio));
                SpeedRamp_Start(&decel_ramp, avg, turn_spd, TurnDecelTime);
            }

            int16_t speed;
            if (SpeedRamp_IsActive(&decel_ramp))
                speed = SpeedRamp_Update(&decel_ramp);
            else
                speed = (int16_t)(BaseSpeed * (1.0f - TurnSlowRatio));
            Racecar((float)speed, deviation);
        }
        else
        {
            /* 仅中间触发 → 直道 */
            if (M2_State != M2_IDLE)
            {
                M2_State = M2_IDLE;
                int16_t cur = (int16_t)((motor_l_ctrl.target_speed + motor_r_ctrl.target_speed) / 2);
                if (cur < 10) cur = 10;
                SpeedRamp_Start(&soft_ramp, cur, (int16_t)BaseSpeed, SoftStartTime);
            }

            int16_t speed;
            if (SpeedRamp_IsActive(&soft_ramp))
            {
                int16_t rv = SpeedRamp_Update(&soft_ramp);
                speed = (rv < (int16_t)BaseSpeed) ? rv : (int16_t)BaseSpeed;
            }
            else
            {
                speed = (int16_t)BaseSpeed;
            }
            Racecar((float)speed, deviation);
        }
        goto track_done;
    }

    /* ============ 模式1：三段式拐弯状态机 ============ */

    /* P1：优先执行转弯（退出检查在 TURN_ACTIVE 内部处理） */
    if (Turn_State == TURN_DECEL)
    {
        /* 通用速度斜坡：从当前速度平滑降至 DecelSpeed */
        int16_t speed = SpeedRamp_Update(&decel_ramp);
        Motor_SetTargetSpeed(speed, speed);
        if (!SpeedRamp_IsActive(&decel_ramp))
            Turn_State = TURN_ACTIVE;
    }
    else if (Turn_State == TURN_ACTIVE)
    {
        /* 检查中间传感器是否重新捕获黑线 → 退出拐弯 */
        if (Gray_3 == GRAY_BLACK || Gray_6 == GRAY_BLACK)
        {
            Turn_State = TURN_IDLE;
            Turn_Direction = 0;
            /* 退出拐弯后软启动，从 0 平滑加速回 BaseSpeed */
            SpeedRamp_Start(&soft_ramp, 0, (int16_t)BaseSpeed, SoftStartTime);
        }
        else
        {
            /* 差速拐弯：左弯左停右转，右弯右停左转 */
            if (Turn_Direction == 1)
                Motor_SetTargetSpeed(0, (int16_t)TurnPower);
            else
                Motor_SetTargetSpeed((int16_t)TurnPower, 0);
        }
    }
    else if (Turn_State == TURN_IDLE)
    {
        /* 检测拐弯触发：最外侧灰度检测到黑线 */
        /* Gray_8(最左)→线偏左→需左转; Gray_1(最右)→线偏右→需右转 */
        if (Gray_8 == GRAY_BLACK)
        {
            Turn_Direction = 1;  /* 左弯 */
            Turn_State = TURN_DECEL;
            Turn_Start_Tick = System_Tick_Count;
            int16_t avg_speed = (int16_t)((motor_l_ctrl.target_speed + motor_r_ctrl.target_speed) / 2);
            SpeedRamp_Start(&decel_ramp, avg_speed, (int16_t)DecelSpeed, TurnDecelTime);
        }
        else if (Gray_1 == GRAY_BLACK)
        {
            Turn_Direction = 2;  /* 右弯 */
            Turn_State = TURN_DECEL;
            Turn_Start_Tick = System_Tick_Count;
            int16_t avg_speed = (int16_t)((motor_l_ctrl.target_speed + motor_r_ctrl.target_speed) / 2);
            SpeedRamp_Start(&decel_ramp, avg_speed, (int16_t)DecelSpeed, TurnDecelTime);
        }
        else
        {
            /* 未触发 → 正常 PD 循迹 */
            float deviation = Grayscale_GetDeviation_Track();
            int16_t speed = SpeedRamp_IsActive(&soft_ramp) ? SpeedRamp_Update(&soft_ramp) : (int16_t)BaseSpeed;
            Racecar((float)speed, deviation);
        }
    }

track_done:
    /* ---- 更新用于显示的 PWM 值 ---- */
    PWML = (int16_t)motor_l_ctrl.target_speed;
    PWMR = (int16_t)motor_r_ctrl.target_speed;
}

/* ===================== 通用速度斜坡函数实现 ===================== */

/**
 * @brief  启动速度斜坡
 * @param ramp  斜坡实例指针
 * @param from  起始速度
 * @param to    目标速度
 * @param duration_ms  过渡时长(ms)
 */
void SpeedRamp_Start(SpeedRamp_t *ramp, int16_t from, int16_t to, uint32_t duration_ms)
{
    ramp->start_val  = from;
    ramp->target_val = to;
    ramp->start_tick = System_Tick_Count;
    ramp->duration   = duration_ms;
    ramp->active     = 1;
}

/**
 * @brief  更新速度斜坡，返回当前速度
 * @param ramp  斜坡实例指针
 * @return 当前插值速度；斜坡完成后固定返回目标值
 */
int16_t SpeedRamp_Update(SpeedRamp_t *ramp)
{
    if (!ramp->active)
        return ramp->target_val;

    uint32_t elapsed = System_Tick_Count - ramp->start_tick;
    if (elapsed >= ramp->duration)
    {
        ramp->active = 0;
        return ramp->target_val;
    }

    /* 线性插值：start_val → target_val */
    float ratio = (float)elapsed / ramp->duration;
    return ramp->start_val + (int16_t)((ramp->target_val - ramp->start_val) * ratio);
}

/**
 * @brief  查询斜坡是否仍在过渡中
 * @param ramp  斜坡实例指针
 * @retval 1=仍在过渡，0=已完成
 */
uint8_t SpeedRamp_IsActive(SpeedRamp_t *ramp)
{
    return ramp->active;
}

/**
 * @brief  强制结束斜坡（后续 Update 直接返回目标值）
 */
void SpeedRamp_Stop(SpeedRamp_t *ramp)
{
    ramp->active = 0;
}

/**
 * @brief   重置软启动状态（按键停止时调用）
 * @note    兼容旧接口，内部使用 SpeedRamp_Stop
 */
void Gray_SoftStart_Reset(void)
{
    soft_was_running = 0;
    SpeedRamp_Stop(&soft_ramp);
    SpeedRamp_Stop(&decel_ramp);
}
