#if 0

/**
 * @file    OLED_LED_Test.c
 * @brief   OLED 显示 + LED 闪烁测试程序（参考示例）
 * @details 验证 OLED 字符串和浮点数显示功能 + LED 闪烁控制
 *
 * 测试对象：
 *   - OLED 字符串和浮点数显示（Hardware/oled.c）
 *   - LED 闪烁控制（SysConfig 生成的 LED_LED0_PIN）
 *
 * 结果现象：
 *   - OLED 持续显示：
 *       第一行："Hello World!"
 *       第二行："Temp:25.6C"（浮点数显示）
 *   - 板载 LED（LED0）以 50ms 周期闪烁（亮 50ms 灭 50ms）
 *
 * 注意：本文件当前完全注释掉，仅作为 API 使用示例参考。
 */

/*
 * OLED_LED_Test.c - OLED 显示 + LED 闪烁测试
 */
#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "key.h"

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    while (1) {
        OLED_Printf(0,0,16,"Hello World!");
        OLED_Printf(0, 16, 16, "Temp:%.1fC", 25.6);
        OLED_Refresh();

        delay_ms(50);
        DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
        delay_ms(50);
        DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);
    }
}

#endif