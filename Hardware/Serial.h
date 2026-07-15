/**
 * @file    Serial.h
 * @brief   串口通信驱动头文件
 * @details 提供串口初始化和收发功能（基于 MSPM0 SysConfig 配置）
 *
 * 硬件配置（SysConfig 已配置）：
 *   UART1：PA8(TX), PA9(RX), 115200-8N1
 *
 * 函数清单（详见 Serial.c）：
 *   - Serial_Init()        : 初始化（使能 NVIC 中断）
 *   - Serial_SendByte()    : 发送单个字节
 *   - Serial_SendArray()   : 发送字节数组
 *   - Serial_SendString()  : 发送字符串
 *   - Serial_SendNumber()  : 按指定位数发送数字
 *   - Serial_Printf()      : 格式化输出（类似 printf）
 *   - Serial_GetRxCount()  : 获取接收缓冲区数据量
 *   - Serial_GetRxData()   : 从接收缓冲区读一个字节
 *
 * 使用方式：
 *   1. SysConfig 已初始化串口外设
 *   2. 调用 Serial_Init() 使能 NVIC 中断
 *   3. 调用 Serial_SendByte() / Serial_SendString() / Serial_Printf() 发送
 *   4. 中断自动接收数据到环形缓冲区，调用 Serial_GetRxData() 读取
 *
 * 注意：与 BlueSerial（UART0）共用相同的 API 风格，
 *       仅硬件实例（UART_0_INST vs UART_1_INST）不同。
 */

#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/* ==================== 宏定义 ==================== */

/** @brief 环形缓冲区大小 */
#define RX_BUFFER_SIZE 256

/* ==================== 函数声明 ==================== */

/**
 * @brief  串口初始化（使能 NVIC 中断）
 * @note   SysConfig 已完成外设和 GPIO 初始化，此函数仅配置 NVIC
 */
void Serial_Init(void);

/**
 * @brief  串口发送单个字节
 * @param  Byte 要发送的字节数据
 */
void Serial_SendByte(uint8_t Byte);

/**
 * @brief  串口发送数组
 * @param  Array 数据数组指针
 * @param  Length 数据长度
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length);

/**
 * @brief  串口发送字符串
 * @param  String 以 '\0' 结尾的字符串
 */
void Serial_SendString(char *String);

/**
 * @brief  串口发送数字（按十进制位逐位发送）
 * @param  Number 要发送的数字
 * @param  Length 数字位数（不足补前导零）
 * @note   Serial_SendNumber(123, 5) 会发送 "00123"
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length);

/**
 * @brief  串口格式化输出（类似 printf）
 * @param  format 格式化字符串
 * @param  ... 可变参数
 * @note   内部缓冲区大小 128 字节，超长内容会被截断
 */
void Serial_Printf(char *format, ...);

/**
 * @brief  获取接收到的数据字节数（环形缓冲区中的数据量）
 * @return 缓冲区中未读取的字节数
 */
uint16_t Serial_GetRxCount(void);

/**
 * @brief  获取接收缓冲区中的数据量（调试用）
 */
uint16_t Serial_GetRxBufCount(void);

/**
 * @brief  从环形缓冲区读取一个字节
 * @return 读取到的字节（缓冲区为空时返回 0）
 */
uint8_t Serial_GetRxData(void);

/* ---- ISR 帧解析调试计数 ---- */
extern volatile uint32_t uart1_isr_count;   /* UART1 ISR 进入次数 */
extern volatile uint32_t dbg_rx_total;      /* ISR 收到的总字节数 */
extern volatile uint32_t dbg_frame_header;  /* 找到帧头 0x7A 的次数 */
extern volatile uint32_t dbg_frame_full;    /* 收满 9 字节的次数 */
extern volatile uint32_t dbg_tail_err;      /* 帧尾 0x7B 校验失败 */
extern volatile uint32_t dbg_bcc_err;       /* BCC 校验失败 */
extern volatile uint32_t dbg_frame_ok;      /* 解析成功次数 */

#endif
