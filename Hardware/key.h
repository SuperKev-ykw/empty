/**
 * @file    key.h
 * @brief   按键驱动头文件（非阻塞扫描）
 * @details 4 个独立按键（Key1~Key4），基于 1ms 定时器中断扫描
 *
 * 硬件引脚：Key1=PB21, Key2=PB0, Key3=PB19, Key4=PB17（上拉输入）
 *
 * 函数清单：
 *   - Key_Tick()  : 非阻塞扫描函数（需在 1ms 定时器中断中调用）
 *   - Key_GetNum(): 获取按键值（0=无, 1~4=对应按键，读取后自动清零）
 *
 * 使用方式：
 *   1. 在 TIMER_0_INST_IRQHandler 中每 1ms 调用一次 Key_Tick()
 *   2. 在主循环中调用 Key_GetNum() 读取按键值
 */

#ifndef __KEY_H
#define __KEY_H

/**
 * @brief 非阻塞按键扫描函数（需在 1ms 定时器中断中调用）
 * @details 内部 20ms 检测一次按键状态，释放时记录按键值
 */
void Key_Tick(void);

/**
 * @brief 获取按键编号（非阻塞，读取后自动清零）
 * @return 1=Key1, 2=Key2, 3=Key3, 4=Key4, 0=无按键
 */
uint8_t Key_GetNum(void);

#endif


