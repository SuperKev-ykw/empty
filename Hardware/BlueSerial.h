/**
 * @file    BlueSerial.h
 * @brief   蓝牙串口驱动头文件（UART0）
 * @details 参考工程风格，中断直接接收字符串
 *
 * 硬件配置（SysConfig 已配置）：
 *   UART0：PA10(TX), PA11(RX), 115200-8N1
 */

#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/* ==================== 外部变量声明 ==================== */

extern volatile uint8_t BlueSerial_RxFlag;   /**< 一帧数据接收完成标志 */
extern char BlueSerial_RxPacket[100];         /**< 接收缓冲区（字符串） */

/* ==================== 函数声明 ==================== */

void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);
uint8_t BlueSerial_GetRxFlag(void);

#endif
