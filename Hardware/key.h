#ifndef __KEY_H
#define __KEY_H

/**
 * @brief 非阻塞按键扫描函数（需在定时器中断中每10ms调用一次）
 * @details 记录按键按下/释放状态，释放时记录按键值
 */
void Key_Tick(void);

/**
 * @brief 获取按键编号（非阻塞，读取后自动清零）
 * @return 1=Key1, 2=Key2, 3=Key3, 4=Key4, 0=无按键
 */
uint8_t Key_GetNum(void);

#endif


