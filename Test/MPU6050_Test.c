// /**
//  * @file    MPU6050_Test.c
//  * @brief   MPU6050 DMP 测试程序（新版底层驱动）
//  * @details 使用新 DMP 驱动读取 MPU6050 四元数解算姿态角，
//  *          在 OLED 上实时显示 Pitch/Roll/Yaw，并通过蓝牙串口发送 YAW 波形。
//  *
//  * 测试对象：
//  *   - MPU6050 DMP 姿态解算（Hardware/mpu6050/mpu_port.c）
//  *   - 蓝牙串口波形发送（Hardware/BlueSerial.c）
//  *   - OLED 显示
//  *
//  * 结果现象：
//  *   - 上电后 OLED 显示 "MPU6050 DMP Init..."，初始化成功显示 "Calibrating..."
//  *   - YAW 漂移检测：连续 500ms 漂移 < 0.1° 后自动锁定零偏，显示 "Calib done!"
//  *   - 校准完成后 OLED 显示：
//  *       Pitch: 实时俯仰角（度，2位小数）
//  *       Roll:  实时横滚角（度，2位小数）
//  *       Yaw:   实时偏航角（度，2位小数，已补偿 + 去零偏）
//  *   - 蓝牙串口持续输出 "[plot,YAWx100,0]\r\n"（~100Hz），可在串口绘图器查看波形
//  *
//  * 硬件连接 (I2C1)：
//  *   GY-87 VCC -> 3.3V
//  *   GY-87 GND -> GND
//  *   GY-87 SCL -> PB2 (I2C1_SCL)
//  *   GY-87 SDA -> PB3 (I2C1_SDA)
//  */

// #include "ti_msp_dl_config.h"
// #include "delay.h"
// #include "oled.h"
// #include "key.h"
// #include "Timer.h"
// #include "mpu6050/mpu_port.h"
// #include "BlueSerial.h"

// /* 陀螺仪比例补偿：物理转 360° 只测出 327°，故补偿因子 = 360/327 ≈ 1.101 */
// #define GYRO_SCALE_CORR    (360.0f / 327.0f)

// int main(void)
// {
//     float pitch, roll, yaw;
//     float yaw_offset = 0.0f;
//     float yaw_last = 0.0f;
//     float yaw_drift = 0.0f;
//     float raw_yaw_prev = 0.0f;     /* 上一帧的去零偏原始 YAW，用于计算增量 */
//     float corrected_yaw = 0.0f;    /* delta 累积补偿后的 YAW */
//     uint8_t oled_cnt = 0;
//     uint16_t stable_cnt = 0;
//     uint8_t calibrated = 0;

//     SYSCFG_DL_init();
//     BlueSerial_Init();
//     Timer_Init();
//     OLED_Init();
//     OLED_ColorTurn(0);
//     OLED_DisplayTurn(0);
//     OLED_Clear();

//     OLED_Printf(0, 0, 16, "MPU6050 DMP Init...");
//     OLED_Refresh();


//     /* 初始化 DMP */
//     if (DMP_Init() != 0)
//     {
//         OLED_Clear();
//         OLED_Printf(0, 0, 16, "DMP Init FAIL!");
//         OLED_Refresh();
//         while (1);
//     }

//     OLED_Clear();
//     OLED_Printf(0, 0, 16, "Calibrating...");
//     OLED_Refresh();

//     while (1)
//     {
//         /* 读取 DMP 姿态角 */
//         if (DMP_Read_Data(&pitch, &roll, &yaw) == 0)
//         {
//             /* 自动校准：检测 YAW 漂移率，稳定后锁定零偏 */
//             if (!calibrated)
//             {
//                 yaw_drift = yaw - yaw_last;
//                 yaw_last = yaw;

//                 if (yaw_drift < 0.0f) yaw_drift = -yaw_drift;
//                 if (yaw_drift < 0.1f)
//                     stable_cnt++;
//                 else
//                     stable_cnt = 0;

//                 if (stable_cnt >= 50)  /* 连续 500ms 漂移 < 0.1° */
//                 {
//                     yaw_offset = yaw;
//                     raw_yaw_prev = 0.0f;      /* 校准完成瞬间，去零偏 yaw = 0 */
//                     corrected_yaw = 0.0f;     /* 累积补偿 YAW 归零 */
//                     calibrated = 1;
//                     OLED_Clear();
//                     OLED_Printf(0, 0, 16, "Calib done!");
//                     OLED_Refresh();
//                     delay_ms(300);
//                     OLED_Clear();
//                 }
//             }
//             else
//             {
//                 /* 校准完成：对每帧角度增量做补偿，避免绝对值跨 ±180° 边界跳变 */
//                 float raw = yaw - yaw_offset;          /* 去零偏的原始 YAW */
//                 float delta = raw - raw_yaw_prev;
//                 raw_yaw_prev = raw;

//                 if (delta > 180.0f) delta -= 360.0f;   /* 处理 ±180° 跨边界 */
//                 if (delta < -180.0f) delta += 360.0f;

//                 corrected_yaw += delta * GYRO_SCALE_CORR;  /* 增量补偿后累积 */

//                 if (corrected_yaw > 180.0f) corrected_yaw -= 360.0f;
//                 if (corrected_yaw < -180.0f) corrected_yaw += 360.0f;
//             }

//             /* 蓝牙每帧发送 corrected_yaw 波形（~100Hz） */
//             BlueSerial_Printf("[plot,%d,%d]\r\n", (int16_t)(corrected_yaw * 100), 0);

//             /* OLED 每 5 帧刷新一次（~20Hz，避免闪烁） */
//             oled_cnt++;
//             if (oled_cnt >= 5)
//             {
//                 oled_cnt = 0;
//                 OLED_Printf(0, 0, 16, "Pitch:%+07.2f", (double)pitch);
//                 OLED_Printf(0, 16, 16, "Roll:%+07.2f", (double)roll);
//                 if (!calibrated)
//                     OLED_Printf(0, 32, 16, "Calib:%d", (int)(stable_cnt * 2));
//                 else
//                     OLED_Printf(0, 32, 16, "Yaw:%+07.2f", (double)corrected_yaw);
//                 OLED_Refresh();
//             }
//         }

//         delay_ms(10);
//     }
// }
