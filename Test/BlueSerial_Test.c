/**
 * @file    BlueSerial_Test.c
 * @brief   蓝牙串口测试程序（调参测试）
 * @details 接收蓝牙调参数据包，解析 slider,Name,Value 格式
 *          在OLED上实时显示 XKp 等参数值
 *
 * 串口参数：115200-8N1 (PA10=TX, PA11=RX)
 * 数据包格式：[slider,Name,Value]
 * 示例：[slider,XKp,50]
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "BlueSerial.h"
#include <string.h>
#include <stdlib.h>

/* 调参变量 */
float XKp = 0.0f;

int main(void)
{
    SYSCFG_DL_init();
    BlueSerial_Init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    while (1)
    {
        /* 接收蓝牙串口数据包并解析 */
        if (BlueSerial_GetRxFlag())
        {
            char *Tag = strtok(BlueSerial_RxPacket, ",");

            if (strcmp(Tag, "slider") == 0)
            {
                char *Name = strtok(NULL, ",");
                char *Value = strtok(NULL, ",");

                if (strcmp(Name, "XKp") == 0)
                {
                    XKp = atof(Value);
                }
            }

            BlueSerial_RxFlag = 0;
        }

        /* OLED 实时显示 XKp */
        OLED_Printf(0, 0, 16, "XKp:%.2f", XKp);
        OLED_Refresh();

        delay_ms(10);
    }
}
