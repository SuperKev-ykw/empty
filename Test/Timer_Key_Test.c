// /*
//  * Timer_Key.c - 定时器+按键测试程序
//  * 功能：定时器中断计数 + 非阻塞按键扫描 + OLED显示
//  */

// #include "ti_msp_dl_config.h"
// #include "delay.h"
// #include "oled.h"
// #include "key.h"
// #include "Timer.h"

// uint16_t Count0;        // 定时器中断累计计数器，每1ms+1
// uint16_t Count1;        // 秒计数器，每1s+1
// uint8_t  KeyVal = 0;    // 按键值

// int main(void)
// {
//     SYSCFG_DL_init();
//     Timer_Init();       // 初始化定时器中断

//     OLED_Init();
//     OLED_ColorTurn(0);
//     OLED_DisplayTurn(0);
//     OLED_Clear();

//     while (1) {
//         // 获取按键值（非阻塞）
//         uint8_t key = Key_GetNum();
//         if (key != 0) {
//             KeyVal = key;
//         }

//         // OLED显示
//         OLED_Printf(0, 0, 16, "Count0:%d", Count0);
//         OLED_Printf(0, 16, 16, "Count1:%d", Count1);
//         OLED_Printf(0, 32, 16, "Key:%d", KeyVal);
//         OLED_Refresh();

//         delay_ms(10);
//     }
// }

// /**
//  * @brief  定时器中断服务函数（每 1ms 触发一次）
//  * @details
//  *   - Count0 每 1ms +1，满 1000 后清零
//  *   - Count1 每 1s +1
//  *   - 调用 Key_Tick() 进行按键扫描（内部 20ms 检测一次状态）
//  */
// void TIMER_0_INST_IRQHandler(void)
// {
//     Count0++;
//     if (Count0 >= 1000) {
//         Count0 = 0;
//         Count1++;
//     }

//     // 按键扫描（每 1ms 调用，内部 20ms 检测一次）
//     Key_Tick();
// }
