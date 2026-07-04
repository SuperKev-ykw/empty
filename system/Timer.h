#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

/* 定时器计数值（在 Timer.c 中定义） */
extern volatile uint16_t Count0;   /* 1ms 递增计数 */
extern volatile uint16_t Count1;   /* 秒计数 */

/**
 * @brief 定时器初始化（配置 NVIC，使能定时器中断）
 */
void Timer_Init(void);

#endif
