/**
 * @file    BlueSerial.c
 * @brief   蓝牙串口驱动实现文件（UART0）
 * @details 参考工程风格，中断直接接收字符串
 *
 * 硬件配置（SysConfig）：
 *   UART0：PA10(TX), PA11(RX), 115200-8N1
 */

#include "ti_msp_dl_config.h"
#include "BlueSerial.h"
#include <stdio.h>
#include <stdarg.h>

/* ==================== 全局变量 ==================== */

char BlueSerial_RxPacket[100];
volatile uint8_t BlueSerial_RxFlag = 0;

/* ==================== 串口 API ==================== */

void BlueSerial_Init(void)
{
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void BlueSerial_SendByte(uint8_t Byte)
{
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

uint8_t BlueSerial_GetRxFlag(void)
{
    if (BlueSerial_RxFlag == 1) {
        BlueSerial_RxFlag = 0;
        return 1;
    }
    return 0;
}

/* ==================== 中断服务函数（参考工程风格） ==================== */

void UART_0_INST_IRQHandler(void)
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;

    uint8_t RxData = DL_UART_receiveData(UART_0_INST);

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
        else
        {
            BlueSerial_RxPacket[pRxPacket] = (char)RxData;
            pRxPacket++;
            if (pRxPacket >= sizeof(BlueSerial_RxPacket) - 1) {
                pRxPacket = 0;
                RxState = 0;
            }
        }
    }
}
