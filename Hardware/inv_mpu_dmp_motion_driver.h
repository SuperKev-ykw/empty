/**
 * @file    BlueSerial.h
 * @brief   蓝牙串口驱动头文件（UART0）
 * @details 使用环形缓冲区接收原始字节，供主循环解析数据包
 *
 * 硬件配置（SysConfig 已配置）：
 *   UART0：PA10(TX), PA11(RX), 115200-8N1
 */

#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/* ==================== 函数声明 ==================== */

void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);

/** @brief 获取接收缓冲区中的数据量 */
uint16_t BlueSerial_GetRxCount(void);

/** @brief 从接收缓冲区读取一个字节 */
uint8_t BlueSerial_GetRxData(void);

#endif
