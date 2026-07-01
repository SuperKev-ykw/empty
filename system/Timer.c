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
#include "Key.h"

/* 外部变量：定时器计数值 */
extern uint16_t Count0;
extern uint16_t Count1;

/**
 * @brief  定时器初始化
 * @details 清除中断待处理标志，使能定时器中断
 */
void Timer_Init(void)
{
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}


