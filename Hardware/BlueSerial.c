/**
 * @file    BlueSerial.c
 * @brief   蓝牙串口驱动实现文件（UART0）
 * @details 使用环形缓冲区接收原始字节，供主循环解析
 *
 * 硬件配置（SysConfig）：
 *   UART0：PA10(TX), PA11(RX), 115200-8N1
 *
 * 函数清单：
 *   - RingBuffer_Write()      : 环形缓冲区写入（内部）
 *   - RingBuffer_Read()       : 环形缓冲区读出（内部）
 *   - RingBuffer_GetCount()   : 获取缓冲区数据量（内部）
 *   - BlueSerial_Init()       : 初始化（使能 NVIC 中断）
 *   - BlueSerial_SendByte()   : 发送单个字节
 *   - BlueSerial_SendArray()  : 发送字节数组
 *   - BlueSerial_SendString() : 发送字符串
 *   - BlueSerial_SendNumber() : 按指定位数发送数字
 *   - BlueSerial_Printf()     : 格式化发送（类似 printf）
 *   - BlueSerial_GetRxCount() : 获取接收缓冲区数据量
 *   - BlueSerial_GetRxData()  : 从接收缓冲区读一个字节
 *   - UART_0_INST_IRQHandler(): UART0 接收中断服务函数
 *
 * 使用方式：
 *   1. 初始化：BlueSerial_Init()
 *   2. 发送：BlueSerial_Printf() / BlueSerial_SendString() / ...
 *   3. 接收：在主循环中调用 BlueSerial_GetRxCount() 检测，
 *           再用 BlueSerial_GetRxData() 逐字节读取并自行解析协议
 */

#include "ti_msp_dl_config.h"
#include "BlueSerial.h"
#include <stdio.h>
#include <stdarg.h>

/* ==================== 环形缓冲区 ==================== */

#define RX_BUFFER_SIZE 256

typedef struct {
    uint8_t buffer[RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} RingBuffer;

static RingBuffer rxBuffer = { .head = 0, .tail = 0, .count = 0 };

static void RingBuffer_Write(RingBuffer *rb, uint8_t data)
{
    if (rb->count < RX_BUFFER_SIZE) {
        rb->buffer[rb->head] = data;
        rb->head = (rb->head + 1) % RX_BUFFER_SIZE;
        rb->count++;
    }
}

static uint8_t RingBuffer_Read(RingBuffer *rb)
{
    if (rb->count > 0) {
        uint8_t data = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % RX_BUFFER_SIZE;
        rb->count--;
        return data;
    }
    return 0;
}

static uint16_t RingBuffer_GetCount(RingBuffer *rb)
{
    return rb->count;
}

/* ==================== 串口 API ==================== */

void BlueSerial_Init(void)
{
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void BlueSerial_SendByte(uint8_t Byte)
{
    while (DL_UART_isTXFIFOFull(UART_0_INST));
    DL_UART_transmitData(UART_0_INST, Byte);
}

void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++) {
        BlueSerial_SendByte(Array[i]);
    }
}

void BlueSerial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        BlueSerial_SendByte((uint8_t)String[i]);
    }
}

static uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        BlueSerial_SendByte((uint8_t)(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0'));
    }
}

void BlueSerial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    BlueSerial_SendString(String);
}

/* ==================== 接收接口（主循环使用） ==================== */

uint16_t BlueSerial_GetRxCount(void)
{
    return RingBuffer_GetCount(&rxBuffer);
}

uint8_t BlueSerial_GetRxData(void)
{
    return RingBuffer_Read(&rxBuffer);
}

/* ==================== 中断服务函数 ==================== */

void UART_0_INST_IRQHandler(void)
{
    uint8_t RxData = DL_UART_receiveData(UART_0_INST);
    RingBuffer_Write(&rxBuffer, RxData);
}
