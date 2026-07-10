/**
 * @file    Timer.c
 * @brief   定时器驱动实现文件
 * @details 实现定时器中断计数功能
 *
 * 核心功能：
 *   1. Timer_Init() — 初始化定时器中断（NVIC 配置）
 *   2. TIMER_0_INST_IRQHandler() — 中断服务函数
 *
 * 定时器配置：
 *   - 中断周期：1ms
 *   - 模式：周期模式（自动重载）
 *
 * 使用方式：
 *   1. main() 中调用 Timer_Init()
 *   2. 通过外部变量 Count0（ms计数）、Count1（秒计数）获取计数值
 */

#include "ti_msp_dl_config.h"
#include "Timer.h"
#include "key.h"
#include "mpu6050/mpu_port.h"   /* MPU_Tick() */
#include "Grayscale.h"          /* Gray_SoftStart_Tick(), RunFlag */
#include "Motor.h"              /* Motor_Speed_PID_Update() */
#include "Encoder.h"            /* Encoder_CalcSpeed() */

/* 定时器计数值定义 */
volatile uint16_t Count0 = 0;
volatile uint16_t Count1 = 0;
volatile uint32_t System_Tick_Count = 0;  /* 1ms递增的系统滴答（用于循迹拐弯计时） */

/**
 * @brief  定时器初始化
 * @details 清除中断待处理标志，使能定时器中断
 */
void Timer_Init(void)
{
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

/**
 * @brief  定时器中断服务函数（1ms 周期）
 * @details 累加 Count0（ms 计数），每满 1000 次累加 Count1（秒计数），
 *          同时调用 Key_Tick() 进行按键扫描。
 *          每 1ms 调用 Gray_SoftStart_Tick() 用于循迹软启动。
 *          每 20ms 执行 Encoder_CalcSpeed() + Motor_Speed_PID_Update()。
 */
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        Key_Tick();
        MPU_Tick();             /* MPU6050 DMP 驱动计时 */

        Count0++;
        if (Count0 >= 1000)
        {
            Count0 = 0;
            Count1++;
        }

        /* 系统滴答递增 */
        System_Tick_Count++;

        /* 循迹软启动（每1ms步进计时） */
        Gray_SoftStart_Tick();

        /* 20ms调度：速度计算 + 内环速度PID */
        static uint16_t speed_pid_cnt = 0;
        speed_pid_cnt++;
        if (speed_pid_cnt >= 20)
        {
            speed_pid_cnt = 0;
            Encoder_CalcSpeed();
            if (RunFlag)
                Motor_Speed_PID_Update();
        }
    }
}


