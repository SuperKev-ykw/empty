/**
 * @file    Serial.c
 * @brief   串口通信驱动实现文件
 * @details 基于 MSPM0G3507 + SysConfig 配置的 UART 驱动
 * 
 * 本文件参考 STM32 标准库的串口驱动设计思路，但适配了 MSPM0 平台：
 *   - 外设和 GPIO 初始化由 SysConfig 完成（无需手动配置时钟和引脚）
 *   - 使用 MSPM0 DriverLib API（DL_UART_xxx）
 *   - 接收采用环形缓冲区 + 中断方式
 * 
 * 硬件配置（SysConfig）：
 *   UART1：PA8(TX), PA9(RX), 115200-8N1
 */

#include "ti_msp_dl_config.h"
#include "Serial.h"

/* ==================== 环形缓冲区实现 ==================== */

/** @brief 环形缓冲区结构体 */
typedef struct {
    uint8_t buffer[RX_BUFFER_SIZE];
    volatile uint16_t head;   /**< 写指针（中断中写入） */
    volatile uint16_t tail;   /**< 读指针（主循环读取） */
    volatile uint16_t count;  /**< 缓冲区中数据量 */
} RingBuffer;

/** @brief 接收环形缓冲区实例 */
static RingBuffer rxBuffer = {
    .head = 0,
    .tail = 0,
    .count = 0
};

/**
 * @brief  向环形缓冲区写入一个字节（在中断中调用）
 * @param  data 要写入的字节
 */
static void RingBuffer_Write(RingBuffer *rb, uint8_t data)
{
    if (rb->count < RX_BUFFER_SIZE) {
        rb->buffer[rb->head] = data;
        rb->head = (rb->head + 1) % RX_BUFFER_SIZE;
        rb->count++;
    } else {
        /* 缓冲区满时覆盖最旧的数据 */
        rb->buffer[rb->head] = data;
        rb->head = (rb->head + 1) % RX_BUFFER_SIZE;
        rb->tail = (rb->tail + 1) % RX_BUFFER_SIZE;
    }
}

/**
 * @brief  从环形缓冲区读取一个字节
 * @return 读取到的字节（缓冲区为空时返回 0）
 */
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

/**
 * @brief  获取环形缓冲区中的数据量
 */
static uint16_t RingBuffer_GetCount(RingBuffer *rb)
{
    return rb->count;
}

/* ==================== 串口 API 实现 ==================== */

/**
 * @brief  串口初始化
 * @note   SysConfig 已完成外设初始化，此函数仅使能 NVIC 中断
 */
void Serial_Init(void)
{
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/**
 * @brief  串口发送单个字节（阻塞）
 */
void Serial_SendByte(uint8_t Byte)
{
    DL_UART_transmitData(UART_1_INST, Byte);
}

/**
 * @brief  串口发送数组
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte(Array[i]);
    }
}

/**
 * @brief  串口发送字符串
 */
void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        Serial_SendByte((uint8_t)String[i]);
    }
}

/**
 * @brief  计算 10 的幂（内部函数）
 */
static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

/**
 * @brief  串口发送数字（按位发送）
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte((uint8_t)(Number / Serial_Pow(10, Length - i - 1) % 10 + '0'));
    }
}

/**
 * @brief  串口格式化输出（类似 printf）
 */
void Serial_Printf(char *format, ...)
{
    char String[128];
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    Serial_SendString(String);
}

/**
 * @brief  获取接收缓冲区中的数据量
 */
uint16_t Serial_GetRxCount(void)
{
    return RingBuffer_GetCount(&rxBuffer);
}

/**
 * @brief  从接收缓冲区读取一个字节
 */
uint8_t Serial_GetRxData(void)
{
    return RingBuffer_Read(&rxBuffer);
}

/* ==================== 中断服务函数 ==================== */

/**
 * @brief  UART1 接收中断服务函数
 * @note   SysConfig 生成的宏 UART_1_INST_IRQHandler 展开为 UART1_IRQHandler
 *         当接收到数据时，将数据写入环形缓冲区
 */
void UART_1_INST_IRQHandler(void)
{
    /* 读取接收到的数据（读取操作自动清除中断标志） */
    uint8_t received = DL_UART_receiveData(UART_1_INST);

    /* 写入环形缓冲区 */
    RingBuffer_Write(&rxBuffer, received);
}
