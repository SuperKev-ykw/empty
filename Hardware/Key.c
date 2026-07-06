/**
 * @file    Key.c
 * @brief   按键驱动实现文件（非阻塞扫描）
 * @details 实现四个独立按键的非阻塞扫描功能
 *
 * 核心功能：
 *   - Key_Tick()  : 非阻塞按键扫描（需在 1ms 定时器中断中调用）
 *   - Key_GetNum(): 获取按键值（读取后自动清零）
 *
 * 硬件连接：
 *   Key1：PB21（上拉输入）
 *   Key2：PB0 （上拉输入）
 *   Key3：PB19（上拉输入）
 *   Key4：PB17（上拉输入）
 *
 * 按键检测逻辑：
 *   - GPIO 配置为上拉输入
 *   - 未按下时：GPIO 读取为高电平
 *   - 按下时  ：GPIO 读取为低电平
 *   - 采用状态机方式检测释放事件，防止长按重复触发
 *   - 内部 20ms 检测一次状态
 *
 * 使用方式：
 *   1. 在 TIMER_0_INST_IRQHandler 中每 1ms 调用一次 Key_Tick()
 *   2. 在主循环中调用 Key_GetNum() 获取按键值（0=无, 1~4=对应按键）
 */

#include "ti_msp_dl_config.h"
#include "Key.h"

/* 按键引脚宏由 SysConfig 生成（ti_msp_dl_config.h）：
 *   Keys_PORT, Keys_Key1_PIN, Keys_Key2_PIN, Keys_Key3_PIN, Keys_Key4_PIN
 */

/* 全局变量：存储按键编号（1=Key1, 2=Key2, 3=Key3, 4=Key4, 0=无按键） */
static uint8_t Key_Num = 0;

/**
 * @brief  获取按键当前状态（底层）
 * @return 1=Key1按下, 2=Key2按下, 3=Key3按下, 4=Key4按下, 0=无按键
 */
static uint8_t Key_GetState(void)
{
    if (DL_GPIO_readPins(Keys_PORT, Keys_Key1_PIN) == 0) return 1;
    if (DL_GPIO_readPins(Keys_PORT, Keys_Key2_PIN) == 0) return 2;
    if (DL_GPIO_readPins(Keys_PORT, Keys_Key3_PIN) == 0) return 3;
    if (DL_GPIO_readPins(Keys_PORT, Keys_Key4_PIN) == 0) return 4;
    return 0;
}

/**
 * @brief  按键扫描函数（非阻塞，需在定时器中断中每1ms调用一次）
 * @details 采用状态机方式检测按键释放事件，防止长按重复触发
 */
void Key_Tick(void)
{
    static uint8_t Count = 0;
    static uint8_t CurrState = 0, PrevState = 0;
    
    Count++;
    // 每20次扫描（20ms）检测一次释放事件
    if (Count >= 20)
    {
        Count = 0;
        
        // 保存上一状态，读取当前状态
        PrevState = CurrState;
        CurrState = Key_GetState();
        
        // 释放时检测：当前无按键 && 之前有按键
        if (CurrState == 0 && PrevState != 0)
        {
            Key_Num = PrevState;  // 记录释放的按键
        }
    }
}

/**
 * @brief  获取按键编号（非阻塞）
 * @return 1=Key1, 2=Key2, 3=Key3, 4=Key4, 0=无按键
 * @note   读取后自动清零，防止重复触发
 */
uint8_t Key_GetNum(void)
{
    uint8_t Temp = Key_Num;
    Key_Num = 0;  // 清零，防止重复读取
    return Temp;
}
