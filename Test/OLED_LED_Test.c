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
//     }
// }
