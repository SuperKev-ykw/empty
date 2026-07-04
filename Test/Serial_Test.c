// /**
//  * @file    Serial_Test.c
//  * @brief   串口数据包接收测试
//  * @details 接收上位机发来的坐标数据包，并在 OLED 上显示
//  *
//  * 数据包格式（6 字节）：
//  *   帧头   数据1   数据2   数据3   数据4   帧尾
//  *   0xAA   X_H    X_L     Y_H     Y_L     0xFF
//  *
//  * 坐标计算：
//  *   X = (X_H << 8) | X_L
//  *   Y = (Y_H << 8) | Y_L
//  *
//  * 串口参数：115200-8N1
//  *
//  * 使用串口调试助手发送（十六进制）：
//  *   AA 00 32 00 64 FF   →  X=50, Y=100
//  *   AA 01 2C 02 58 FF   →  X=300, Y=600
//  */

// #include "ti_msp_dl_config.h"
// #include "delay.h"
// #include "oled.h"
// #include "Serial.h"

// /* ==================== 坐标数据 ==================== */
// volatile uint16_t X_Data = 0;   // 接收到的 X 坐标
// volatile uint16_t Y_Data = 0;   // 接收到的 Y 坐标
// volatile uint8_t  Data_Valid = 0;  // 数据有效标志（主循环处理后清零）

// /* ==================== 数据包解析 ==================== */

// /**
//  * @brief  解析数据包（在主循环中调用）
//  * @note   从环形缓冲区逐字节读取，状态机解析
//  *         数据包格式：0xAA XH XL YH YL 0xFF
//  */
// void Process_Received_Data(void)
// {
//     static uint8_t rx_state = 0;     // 状态机状态
//     static uint8_t rx_buf[4];        // 4 字节数据缓冲区
//     static uint8_t rx_index = 0;     // 缓冲区索引

//     /* 循环读取所有可用数据 */
//     while (Serial_GetRxCount() > 0)
//     {
//         uint8_t received = Serial_GetRxData();

//         switch (rx_state)
//         {
//             case 0:  /* 等待帧头 0xAA */
//                 if (received == 0xAA)
//                 {
//                     rx_index = 0;
//                     rx_state = 1;
//                 }
//                 break;

//             case 1:  /* 接收 4 个数据字节 */
//                 rx_buf[rx_index++] = received;
//                 if (rx_index >= 4)
//                 {
//                     rx_state = 2;
//                 }
//                 break;

//             case 2:  /* 等待帧尾 0xFF */
//                 if (received == 0xFF)
//                 {
//                     /* 解析坐标：X = buf[0]<<8 | buf[1], Y = buf[2]<<8 | buf[3] */
//                     X_Data = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
//                     Y_Data = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
//                     Data_Valid = 1;  // 标记数据有效
//                 }
//                 else if (received == 0xAA)
//                 {
//                     /* 帧尾错误但遇到了新的帧头，重新同步 */
//                     rx_index = 0;
//                     rx_state = 1;
//                     break;
//                 }
//                 rx_state = 0;  // 无论帧尾是否正确，都回到等待帧头
//                 break;

//             default:
//                 rx_state = 0;
//                 rx_index = 0;
//                 break;
//         }
//     }
// }

// /* ==================== 主函数 ==================== */

// int main(void)
// {
//     SYSCFG_DL_init();
//     Serial_Init();          // 初始化串口中断
//     OLED_Init();
//     OLED_ColorTurn(0);
//     OLED_DisplayTurn(0);
//     OLED_Clear();

//     while (1)
//     {
//         /* 解析数据包 */
//         Process_Received_Data();

//         /* 有有效数据时更新 OLED 显示 */
//         if (Data_Valid)
//         {
//             Data_Valid = 0;
//             /* 不清屏，只更新显示区域，避免闪烁 */
//             OLED_Printf(0, 0, 16, "X:%d", X_Data);
//             OLED_Printf(0, 16, 16, "Y:%d", Y_Data);
//             OLED_Refresh();
//         }

//         delay_ms(10);
//     }
// }
