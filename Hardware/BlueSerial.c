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
#include <string.h>
#include <stdlib.h>
#include "Motor.h"
#include "Grayscale.h"

/* ==================== 全局变量定义 ==================== */

char BlueSerial_RxPacket[100];
volatile uint8_t BlueSerial_RxFlag = 0;

/* 摇杆数据 */
int8_t  Joystick_LH = 0;
int8_t  Joystick_LV = 0;
int8_t  Joystick_RH = 0;
int8_t  Joystick_RV = 0;
uint8_t Joystick_Active = 0;

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

    /* 原始字节写入环形缓冲区（供 0xAA 等协议解析） */
    RingBuffer_Write(&rxBuffer, RxData);

    /* 自动提取 [ ... ] 数据包（类比参考工程 江科大平衡车/04-速度环） */
    {
        static uint8_t RxState = 0;
        static uint8_t pRxPacket = 0;

        if (RxState == 0)
        {
            if (RxData == '[' && BlueSerial_RxFlag == 0)
            {
                RxState = 1;
                pRxPacket = 0;
            }
        }
        else if (RxState == 1)
        {
            if (RxData == ']')
            {
                RxState = 0;
                BlueSerial_RxPacket[pRxPacket] = '\0';
                BlueSerial_RxFlag = 1;
            }
            else if (pRxPacket < (int)(sizeof(BlueSerial_RxPacket) - 1))
            {
                BlueSerial_RxPacket[pRxPacket] = (char)RxData;
                pRxPacket++;
            }
            else
            {
                /* 数据包超过缓冲区，丢弃 */
                RxState = 0;
            }
        }
    }
}

/* ==================== 蓝牙调参任务（主循环中调用） ==================== */

/**
 * @brief  蓝牙调参任务
 * @details 解析手机APP发来的 [slider,Name,Value] 协议，更新PID等运行参数。
 *          支持以下参数：
 *          电机速度环：MKp, MKi, MKd
 *          循迹位置环：TKp, TKd
 *          循迹速度：  BSp (BaseSpeed)
 *          拐弯参数：  TdT (TurnDecelTime), TdP (TurnPower), TdS (DecelSpeed)
 *
 * @note  协议格式：手机发送 [slider,MKp,2.5]
 *        与参考工程 (STM32_CAR) 完全兼容
 */
void BlueSerial_Tasks(void)
{
    if (BlueSerial_RxFlag == 0)
        return;

    /* 拷贝到本地缓冲区再解析，避免与 ISR 写冲突 */
    char packet[100];
    uint16_t i;
    for (i = 0; i < sizeof(packet) - 1; i++)
    {
        packet[i] = BlueSerial_RxPacket[i];
        if (packet[i] == '\0')
            break;
    }
    packet[sizeof(packet) - 1] = '\0';
    BlueSerial_RxFlag = 0;

    /* 解析：格式为 "Tag,Name,Value" */
    char *Tag = strtok(packet, ",");
    if (Tag == NULL)
        return;

    /* ============ 滑杆指令：调参 ============ */
    if (strcmp(Tag, "slider") == 0)
    {
        char *Name = strtok(NULL, ",");
        char *Value = strtok(NULL, ",");
        if (Name == NULL || Value == NULL)
            return;

        float val_f = atof(Value);
        uint16_t val_u16 = (uint16_t)atoi(Value);

        /* 电机速度环PID */
        if      (strcmp(Name, "MKp") == 0) { Motor_Kp = val_f; }
        else if (strcmp(Name, "MKi") == 0) { Motor_Ki = val_f; }
        else if (strcmp(Name, "MKd") == 0) { Motor_Kd = val_f; }
        /* 循迹位置环PD */
        else if (strcmp(Name, "TKp") == 0) { Track_Kp = val_f; }
        else if (strcmp(Name, "TKd") == 0) { Track_Kd = val_f; }
        /* 循迹速度 */
        else if (strcmp(Name, "BSp") == 0) { BaseSpeed = val_u16; }
        /* 拐弯参数 */
        else if (strcmp(Name, "TdT") == 0) { TurnDecelTime = val_u16; }
        else if (strcmp(Name, "TdP") == 0) { TurnPower = val_u16; }
        else if (strcmp(Name, "TdS") == 0) { DecelSpeed = val_u16; }
        /* 摇杆转向灵敏度 */
        else if (strcmp(Name, "JTrn") == 0) { JoyTurn = val_u16; }
    }
    /* ============ 摇杆指令：手动控制 ============ */
    else if (strcmp(Tag, "joystick") == 0)
    {
        Joystick_LH = (int8_t)atoi(strtok(NULL, ","));
        Joystick_LV = (int8_t)atoi(strtok(NULL, ","));
        Joystick_RH = (int8_t)atoi(strtok(NULL, ","));
        Joystick_RV = (int8_t)atoi(strtok(NULL, ","));

        /* 左垂直（前后）或右水平（转向）有非零值 = 摇杆激活 */
        if (Joystick_LV != 0 || Joystick_RH != 0)
            Joystick_Active = 1;
        else
            Joystick_Active = 0;
    }
}
