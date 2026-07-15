/**
 * @file    Encoder.h
 * @brief   编码器驱动头文件
 * @details 使用 GPIO 外部中断模拟正交解码，读取两路 AB 相编码器脉冲。
 *
 * 硬件接线：
 *   左编码器：A1=PA7, A2=PA25（GPIOA 中断）
 *   右编码器：B1=PB13, B2=PB14（GPIOB 中断）
 *
 * 解码原理：
 *   当某相上升沿触发中断时，读取另一相电平判断方向：
 *     A↑ & B=1 → 正转 +1  |  A↑ & B=0 → 反转 -1
 *     B↑ & A=1 → 反转 -1  |  B↑ & A=0 → 正转 +1
 *
 * 使用方式：
 *   1. main() 开头调用 Encoder_Init()
 *   2. 调用 Encoder_GetCount() 读取 10ms 窗口内的脉冲增量
 *      （读取后自动清零，每次返回的都是最近 10ms 的增量）
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

#define ENCODER_LEFT    1   /**< 左编码器（PB13/PB14） */
#define ENCODER_RIGHT   2   /**< 右编码器（PA7/PA25） */

/* ==================== 诊断计数（在 Encoder.c 中定义） ==================== */
extern volatile uint32_t g_enc_gpioa_irq_cnt;   /* GPIOA 中断触发次数（右编码器，累加） */
extern volatile uint32_t g_enc_gpiob_irq_cnt;   /* GPIOB 中断触发次数（左编码器，累加） */

/**
 * @brief 获取 GPIO 中断在最近 10ms 内的触发次数（诊断用）
 * @param port 编码器编号：ENCODER_LEFT(1) 或 ENCODER_RIGHT(2)
 * @return 10ms 窗口内的中断触发次数，读取后自动清零
 */
int16_t Encoder_GetIRQCount(uint8_t port);

/**
 * @brief 初始化编码器驱动
 * @details 使能 GPIO 中断 NVIC、使能 TIMG12 10ms 定时器中断
 */
void Encoder_Init(void);

/**
 * @brief 获取编码器在最近 10ms 内的脉冲增量
 * @param n 编码器编号：ENCODER_LEFT(1) 或 ENCODER_RIGHT(2)
 * @return 脉冲增量（有符号，正=正转，负=反转），读取后自动清零
 *
 * @note 需在 TIMG12 10ms 定时器启动后调用
 */
int16_t Encoder_GetCount(uint8_t n);

/**
 * @brief 编码器速度计算（带5点滑动平均滤波）
 * @details 读取10ms脉冲增量并累加2次得到20ms窗口的速度值，
 *          经过5点滑动平均滤波平滑输出。
 *          应在20ms定时中断中调用。
 */
void Encoder_CalcSpeed(void);

/**
 * @brief 获取滤波后的编码器速度值
 * @param n 编码器编号：ENCODER_LEFT(1) 或 ENCODER_RIGHT(2)
 * @return 滤波后的速度值（脉冲/20ms）
 */
int16_t Encoder_GetSpeed(uint8_t n);

/**
 * @brief 获取从初始化开始的累计总脉冲数
 * @param n 编码器编号：ENCODER_LEFT(1) 或 ENCODER_RIGHT(2)
 * @return 累计脉冲总数（无符号累积，不因主循环周期而丢失）
 */
int32_t Encoder_GetTotalCount(uint8_t n);

#endif
