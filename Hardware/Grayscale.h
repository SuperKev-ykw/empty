/**
 * @file    Grayscale.h
 * @brief   8 路灰度传感器驱动接口
 * @details 定义灰度传感器引脚、电平宏、加权表和函数声明
 *
 * 硬件引脚：Gray_1=GPIOB.6（最右）, ..., Gray_8=GPIOB.18（最左）
 *
 * 加权偏差算法：
 *   偏差 = 加权和 / 检测到黑线的数量
 *   权重：Gray_8=-4, Gray_7=-3, Gray_6=-2, Gray_5=-1,
 *         Gray_4=+1, Gray_3=+2, Gray_2=+3, Gray_1=+4
 *
 * 函数清单：
 *   - Gray_Sensor_Init()      : 初始化 GPIO（SysConfig 已完成，本函数保留接口）
 *   - Gray_Sensor_Read()      : 读取 8 路灰度传感器状态到全局变量
 *   - Grayscale_GetDeviation(): 计算灰度加权偏差值
 *
 * 全局变量：Gray_1 ~ Gray_8（每个为 0=白底 或 1=黑线）
 */

#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ===================== 电平定义 ===================== */
#define GRAY_BLACK       1            // 检测到黑线（高电平）
#define GRAY_WHITE       0            // 检测到白底（低电平）

/* ===================== 传感器权重表 =====================
  * 传感器位置（从左到右）：Gray_8  Gray_7  Gray_6  Gray_5  Gray_4  Gray_3  Gray_2  Gray_1
  * 对应权重：              -4      -3      -2      -1      +1      +2      +3      +4
  * 偏差 = 加权和 / 检测到黑线的数量
  * 参考自 STM32 标准库工程 5路权重：-2, -1, 0, +1, +2
  * ==================================================== */
#define GRAY_WEIGHT_8    -4
#define GRAY_WEIGHT_7    -3
#define GRAY_WEIGHT_6    -2
#define GRAY_WEIGHT_5    -1
#define GRAY_WEIGHT_4     1
#define GRAY_WEIGHT_3     2
#define GRAY_WEIGHT_2     3
#define GRAY_WEIGHT_1     4

/* ===================== 全局变量 ===================== */
extern uint8_t Gray_1;   // 最右边灰度
extern uint8_t Gray_2;
extern uint8_t Gray_3;
extern uint8_t Gray_4;
extern uint8_t Gray_5;
extern uint8_t Gray_6;
extern uint8_t Gray_7;
extern uint8_t Gray_8;   // 最左边灰度

/* ===================== 函数声明 ===================== */
void Gray_Sensor_Init(void);     // 初始化8路灰度传感器GPIO
void Gray_Sensor_Read(void);     // 读取8路灰度传感器数据
float Grayscale_GetDeviation(void); // 获取加权偏差值

#endif
