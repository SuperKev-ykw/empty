/**
 * @file    BlueSerial.h
 * @brief   蓝牙串口驱动头文件（UART0）
 * @details 使用环形缓冲区接收原始字节，供主循环解析数据包
 *          中断中自动提取 [ ... ] 数据包到 BlueSerial_RxPacket
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
 * 数据包接口（类比参考工程 "江科大平衡车/04-速度环"）：
 *   中断自动提取 [ ... ] 之间的内容到 BlueSerial_RxPacket，
 *   主循环轮询 BlueSerial_RxFlag，为 1 时解析处理。
 *   格式示例：[joystick,LH,LV,RH,RV]
 *
 * 使用方式：
 *   1. 初始化：BlueSerial_Init()
 *   2. 发送：BlueSerial_Printf("[plot,%d]\r\n", value);
 *   3. 数据包接收：轮询 BlueSerial_RxFlag，为 1 时解析 BlueSerial_RxPacket
 *   4. 原始字节接收：调用 BlueSerial_GetRxCount()/GetRxData() 逐字节读取
 */

#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/* ==================== 数据包接收（中断中自动提取） ==================== */

/** @brief 蓝牙接收数据包缓冲区（中断中填充 [ ... ] 之间的内容） */
extern char BlueSerial_RxPacket[100];

/** @brief 蓝牙接收数据包标志（1=新数据包到达，主循环解析后清 0） */
extern volatile uint8_t BlueSerial_RxFlag;

/* ==================== 蓝牙摇杆数据（解码后） ==================== */
extern int8_t  Joystick_LH;      /**< 左摇杆水平（-100~100） */
extern int8_t  Joystick_LV;      /**< 左摇杆垂直（-100~100；正=前进） */
extern int8_t  Joystick_RH;      /**< 右摇杆水平（-100~100；正=右转） */
extern int8_t  Joystick_RV;      /**< 右摇杆垂直（-100~100） */
extern uint8_t Joystick_Active;  /**< 摇杆激活标志（=1时有摇杆控制信号） */

/* ==================== 函数声明 ==================== */

void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);

/** @brief 蓝牙调参任务（在主循环中调用，解析 [slider,Name,Value] 协议） */
void BlueSerial_Tasks(void);

/** @brief 获取接收缓冲区中的数据量 */
uint16_t BlueSerial_GetRxCount(void);

/** @brief 从接收缓冲区读取一个字节 */
uint8_t BlueSerial_GetRxData(void);

#endif
