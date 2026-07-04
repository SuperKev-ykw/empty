// /**
//  * @file    Grayscale_Test.c
//  * @brief   灰度传感器测试程序
//  * @details 读取8路灰度传感器状态，在OLED上实时显示
//  *          OLED显示格式：
//  *            Gray:10100101
//  *            Dev: 0.00
//  *          最左边数值对应最左边灰度(Gray_8)，最右边数值对应最右边灰度(Gray_1)
//  *          1=检测到黑线，0=检测到白底
//  */

// #include "ti_msp_dl_config.h"
// #include "delay.h"
// #include "oled.h"
// #include "Grayscale.h"

// int main(void)
// {
//     SYSCFG_DL_init();

//     OLED_Init();
//     OLED_ColorTurn(0);
//     OLED_DisplayTurn(0);
//     OLED_Clear();

//     /* 初始化灰度传感器（SysConfig 已配置 GPIO，此调用仅为保持接口一致） */
//     Gray_Sensor_Init();

//     while (1)
//     {
//         /* 读取8路灰度传感器 */
//         Gray_Sensor_Read();

//         /* 计算加权偏差值 */
//         float deviation = Grayscale_GetDeviation();

//         /* 在OLED上显示灰度状态
//          * 第一行：显示8路灰度状态，从左到右：Gray_8 ~ Gray_1
//          * 第二行：显示偏差值
//          * 即最左边数值 = 最左边灰度状态，最右边数值 = 最右边灰度状态
//          */
//         OLED_Printf(0, 0, 16, "Gray:%d%d%d%d%d%d%d%d",
//             Gray_8, Gray_7, Gray_6, Gray_5,
//             Gray_4, Gray_3, Gray_2, Gray_1);
//         OLED_Printf(0, 16, 16, "Dev:%+5.2f", deviation);

//         /* 刷新OLED显示 */
//         OLED_Refresh();

//         delay_ms(50);
//     }
// }
