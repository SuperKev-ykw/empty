/**
 * @file    F32C_Serial.h
 * @brief   F32C 电机串口帧解析（从 Serial 环形缓冲区读取）
 * @details 将原 Serial.c ISR 中的 F32C 9 字节反馈帧解析逻辑，
 *          提取为独立模块，通过主循环轮询 Serial 环形缓冲区来解析。
 *
 * 使用方式：
 *   在主循环中周期调用 F32C_Serial_ParseFrame()，
 *   解析结果直接更新 F32C.h 中的全局变量：
 *     - motor1_current_position / motor1_current_speed
 *     - motor2_current_position / motor2_current_speed
 *
 * 注意：依赖 Serial.c 的环形缓冲区（Serial_GetRxCount / Serial_GetRxData）
 */

#ifndef __F32C_SERIAL_H
#define __F32C_SERIAL_H

#include <stdint.h>

/* ==================== F32C 帧解析调试计数 ==================== */
extern volatile uint32_t f32c_frame_header;     /* 找到帧头 0x7A 的次数 */
extern volatile uint32_t f32c_frame_full;       /* 收满 9 字节的次数 */
extern volatile uint32_t f32c_tail_err;         /* 帧尾 0x7B 校验失败 */
extern volatile uint32_t f32c_bcc_err;          /* BCC 校验失败 */
extern volatile uint32_t f32c_frame_ok;         /* 解析成功次数 */

/* ==================== 函数声明 ==================== */

/**
 * @brief  从 Serial 环形缓冲区轮询解析 F32C 9 字节反馈帧
 * @return 1=解析成功并更新了电机状态，0=未收到完整帧
 * @note   协议格式：0x7A ADDR TYPE DATA4 BCC 0x7B
 *         ADDR=1/2 → TYPE=0x01(角度) / 0x00(速度)
 *         在主循环中周期调用，与 Serial_GetKeyFrame() 类似
 */
uint8_t F32C_Serial_ParseFrame(void);

#endif /* __F32C_SERIAL_H */
