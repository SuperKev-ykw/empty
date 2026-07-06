/**
 * @file    BlueSerial.h
 * @brief   蓝牙串口驱动头文件（UART0）
 * @details 使用环形缓冲区接收原始字节，供主循环解析数据包
 *
 * 硬件配置（SysConfig 已配置）：
 *   UART0：PA10(TX), PA11(RX), 115200-8N1
 *
 * 函数清单（见 BlueSerial.c 中的详细说明）：
 *   - BlueSerial_Init()       : 初始化（使能 NVIC 中断）
 *   - BlueSerial_SendByte()   : 发送单个字节
 *   - BlueSerial_SendArray()  : 发送字节数组
 *   - BlueSerial_SendString() : 发送字符串
 *   - BlueSerial_SendNumber() : 按指定位数发送数字
 *   - BlueSerial_Printf()     : 格式化发送（类似 printf）
 *   - BlueSerial_GetRxCount() : 获取接收缓冲区数据量
 *   - BlueSerial_GetRxData()  : 从接收缓冲区读一个字节
 *
 * 使用方式：
 *   1. 初始化：BlueSerial_Init()
 *   2. 发送：BlueSerial_Printf("[plot,%d]\r\n", value);
 *   3. 接收：在主循环中调用 BlueSerial_GetRxCount() 检测，
 *           再用 BlueSerial_GetRxData() 逐字节读取并自行解析协议
 *      示例：接收 "Hello\n" 字符串
 *        if (BlueSerial_GetRxCount() > 0) {
 *            char c = BlueSerial_GetRxData();
 *            // 解析协议...
 *        }
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
