// /*

//  */

// #include "ti/driverlib/dl_gpio.h"
// #include "ti_msp_dl_config.h"
// #include "delay.h"
// #include "oled.h"
// #include "key.h"



// int main(void)
// {
//     SYSCFG_DL_init();

//     OLED_Init();
//     OLED_ColorTurn(0);
//     OLED_DisplayTurn(0);
//     OLED_Clear();

//     while (1) {
//         OLED_Printf(0,0,16,"Hello World!");
//         OLED_Printf(0, 16, 16, "Temp:%.1fC", 25.6);
//         OLED_Refresh();

//         delay_ms(50);
//         DL_GPIO_clearPins(LED_PORT, LED_LED0_PIN);
//         delay_ms(50);
//         DL_GPIO_setPins(LED_PORT, LED_LED0_PIN);

//         // if (Key_GetNum(1) == 1) { /* Key1 被按下 */
//         //     OLED_Printf(0, 32, 16, "Key1 Pressed");
//         //     OLED_Refresh();
//         // }
//         // if (Key_GetNum(2) == 1) { /* Key2 被按下 */
//         //     OLED_Printf(0, 32, 16, "Key2 Pressed");
//         //     OLED_Refresh();
//         // }
//         // if (Key_GetNum(3) == 1) { /* Key3 被按下 */
//         //     OLED_Printf(0, 32, 16, "Key3 Pressed");
//         //     OLED_Refresh();
//         // }
//         // if (Key_GetNum(4) == 1) { /* Key4 被按下 */
//         //     OLED_Printf(0, 32, 16, "Key4 Pressed");
//         //     OLED_Refresh();
//         // }

//     }
// }
