#if 0

/**
 * @file    Serial_Test.c
 * @brief   串口数据帧接收测试（0xAA DATA 0xFF）
 * @details 接收上位机发来的 3 字节数据帧，在 OLED 上显示数据值
 *
 * 数据包格式（3 字节）：
 *   帧头   数据    帧尾
 *   0xAA   DATA    0xFF
 *
 * DATA 范围：0x00 ~ 0xFF
 *
 * 串口参数：115200-8N1
 *
 * 测试对象：
 *   - 串口接收（Hardware/Serial.c）
 *   - 状态机协议解析
 *   - OLED 数值显示
 *
 * 结果现象：
 *   - 上电后 OLED 清空
 *   - 串口调试助手发送 "AA 04 FF" → OLED 显示 "RX:04 Cnt:1"
 *   - 串口调试助手发送 "AA 00 FF" → OLED 显示 "RX:00 Cnt:2"
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "Timer.h"

/* ==================== 帧格式宏 ==================== */
#define FRAME_HEADER    0xAA
#define FRAME_TAIL      0xFF

/* ==================== 接收数据 ==================== */
volatile uint8_t  Rx_Data = 0xFF;   // 最新收到的数据字节（0xFF 表示未收到）
volatile uint8_t  Data_Valid = 0;   // 数据有效标志
volatile uint16_t Rx_Cnt = 0;       // 接收计数

/* ==================== 数据包解析 ==================== */

/**
 * @brief  解析 0xAA DATA 0xFF 数据帧
 * @note   从环形缓冲区逐字节读取，状态机解析
 */
void Process_Received_Data(void)
{
    static uint8_t rx_state = 0;     // 状态机状态
    static uint8_t rx_data = 0;      // 暂存数据字节

    while (Serial_GetRxCount() > 0)
    {
        uint8_t received = Serial_GetRxData();

        switch (rx_state)
        {
            case 0:  /* 等待帧头 0xAA */
                if (received == 0xAA)
                {
                    rx_state = 1;
                }
                break;

            case 1:  /* 接收数据字节 */
                rx_data = received;
                rx_state = 2;
                break;

            case 2:  /* 等待帧尾 0xFF */
                if (received == 0xFF)
                {
                    Rx_Data = rx_data;
                    Rx_Cnt++;
                    Data_Valid = 1;  // 标记数据有效
                }
                else if (received == 0xAA)
                {
                    /* 帧尾错误但遇到了新的帧头，重新同步 */
                    rx_state = 1;
                    break;
                }
                else
                {
                    /* 帧尾错误，回到空闲继续找下一个帧头 */
                    rx_state = 0;
                }
                break;

            default:
                rx_state = 0;
                break;
        }
    }
}

/* ==================== 主函数 ==================== */

int main(void)
{
    uint32_t tick = 0;

    SYSCFG_DL_init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    Serial_Init();          // 初始化串口中断
    Timer_Init();           // 初始化定时器（Count0/Count1 依赖它）
    BlueSerial_Init();      // 初始化蓝牙串口（UART0）

    /* 1) 先排空硬件 UART RX FIFO（防止 SYSCFG_DL_init 期间的毛刺字节） */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST))
    {
        (void)DL_UART_Main_receiveData(UART_1_INST);
    }

    /* 2) 再排空软件环形缓冲区（如果 ISR 在 FIFO 排空前已写入了数据） */
    while (Serial_GetRxCount() > 0)
    {
        (void)Serial_GetRxData();
    }

    while (1)
    {
        uint32_t tick_loop = (uint32_t)Count1 * 1000 + Count0;
        uint32_t t_serial = 0, t_oled = 0;

        /* 解析数据包 */
        {
            uint32_t t0 = (uint32_t)Count1 * 1000 + Count0;
            Process_Received_Data();
            t_serial = (uint32_t)Count1 * 1000 + Count0 - t0;
        }

        /* 每 100ms 刷新一次 OLED */
        {
            uint32_t t0_oled = (uint32_t)Count1 * 1000 + Count0;
            if (++tick >= 10)
            {
                tick = 0;
                OLED_Printf(0, 0,  16, "RX:0x%02X",
                            (Rx_Cnt > 0) ? (uint16_t)Rx_Data : 0xFF);
                OLED_Printf(0, 16, 16, "Cnt:%d    ", (uint16_t)Rx_Cnt);

                OLED_Refresh();
            }
            Data_Valid = 0;
            t_oled = (uint32_t)Count1 * 1000 + Count0 - t0_oled;
        }

        delay_ms(10);

        uint32_t loop_ms = (uint32_t)Count1 * 1000 + Count0 - tick_loop;
        BlueSerial_Printf("%d,%d,%d\n", loop_ms, t_serial, t_oled);
    }
}

#endif
