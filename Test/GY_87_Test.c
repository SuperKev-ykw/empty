// /**
//  * @file    GY_87_Test.c
//  * @brief   GY-87 模块测试程序 (MPU6050 + HMC5883L)
//  * @details 读取MPU6050加速度计/陀螺仪和HMC5883L磁力计数据，
//  *          通过AHRS解算三轴角度，在OLED上实时显示：
//  *            Roll/Pitch 来自加速度计
//  *            Yaw 来自陀螺仪+磁力计互补滤波融合（抑制零漂）
//  *
//  *          I2C 传感器每 10ms 读取一次（约 100Hz），
//  *          AHRS 和串口回传每帧都跑满，不阻塞于 I2C。
//  *
//  * OLED显示分两页，按Key4切换：
//  *   页面0 - 姿态角: Roll, Pitch, Yaw
//  *   页面1 - 调试数据: 磁力计原始值 + 陀螺仪Z轴 + 磁力计偏移
//  *
//  * 硬件连接 (SysConfig I2C2 -> I2C1)：
//  *   GY-87 VCC -> 3.3V
//  *   GY-87 GND -> GND
//  *   GY-87 SCL -> PB2 (I2C1_SCL)
//  *   GY-87 SDA -> PB3 (I2C1_SDA)
//  */

// #include "ti_msp_dl_config.h"
// #include "delay.h"
// #include "oled.h"
// #include "MPU6050.h"
// #include "HMC5883L.h"
// #include "AHRS.h"
// #include "key.h"
// #include "Timer.h"
// #include "BlueSerial.h"
// #include "Serial.h"
// #include "KalmanFilter.h"

// /* 定时器计数值 (Timer.c 引用) */
// uint16_t Count0;
// uint16_t Count1;

// /* 三个卡尔曼滤波器实例（q=0.1, r=1.0） */
// static Kalman_t kf_roll, kf_pitch, kf_yaw;

// int main(void)
// {
//     int16_t AccX, AccY, AccZ;
//     int16_t GyroX, GyroY, GyroZ;
//     int16_t MagX, MagY, MagZ;

//     float roll, pitch, yaw;
//     float roll_kf, pitch_kf, yaw_kf;  /* 卡尔曼滤波后的角度 */
//     float offX, offY, offZ;

//     uint8_t page = 0;           /* 0=姿态页, 1=调试页 */
//     uint8_t oled_cnt = 0;       /* OLED刷新节流计数器 */

//     /* ---- 32 位毫秒时间戳（避免 Count0 回绕问题） ---- */
//     uint32_t tick_now;          /* 当前毫秒 (Count1*1000 + Count0) */
//     uint32_t tick_last = 0;     /* 上一帧毫秒快照 */
//     uint32_t sensor_tick = 0;   /* 下次读取传感器的时间戳 */
//     float dt;                   /* 实际帧间隔 (s) */

//     SYSCFG_DL_init();
//     Timer_Init();               /* 启动定时器中断，供按键扫描 */
//     BlueSerial_Init();          /* 初始化蓝牙串口 */
//     BlueSerial_Printf("GY-87 Test Start!\r\n");

//     /* 初始化OLED */
//     OLED_Init();
//     OLED_ColorTurn(0);
//     OLED_DisplayTurn(0);
//     OLED_Clear();

//     OLED_Printf(0, 0, 16, "GY-87 Init...");
//     OLED_Refresh();

//     /* 初始化MPU6050 (内部初始化 I2C1 + 配置寄存器) */
//     MPU6050_Init();

//     /* 初始化HMC5883L磁力计 */
//     HMC5883L_Init();

//     /* 检查MPU6050是否正常 */
//     if (MPU6050_GetID() != 0x68)
//     {
//         OLED_Printf(0, 0, 16, "MPU6050 ERROR!");
//         OLED_Printf(0, 16, 16, "ID:0x%02X", MPU6050_GetID());
//         OLED_Refresh();
//         while (1);
//     }

//     OLED_Clear();
//     OLED_Printf(0, 0, 16, "GY-87 OK!");
//     OLED_Refresh();
//     delay_ms(500);

//     /* ===== 磁力计校准 =====
//      * 提示：水平旋转模块一整圈，采集硬铁偏移 */
//     OLED_Clear();
//     OLED_Printf(0, 0, 16, "Mag Calibrating...");
//     OLED_Printf(0, 16, 16, "Rotate 360 deg");
//     OLED_Printf(0, 48, 16, "Press Key4 end");
//     OLED_Refresh();

//     /* 按键校准：第4键按下才结束，时间足够旋转一整圈 */
//     do
//     {
//         for (int i = 0; i < 50; i++)    /* 每50次循环打印一次. */
//         {
//             HMC5883L_GetData(&MagX, &MagY, &MagZ);
//             AHRS_CalibrateMag(MagX, MagY, MagZ);
//             delay_ms(10);
//         }
//         OLED_Printf(0, 32, 16, "Progress...");
//         OLED_Refresh();
//     } while (Key_GetNum() != 4);

//     AHRS_GetMagOffset(&offX, &offY, &offZ);
//     OLED_Clear();
//     OLED_Printf(0, 0, 16, "Mag Calib Done!");
//     OLED_Printf(0, 16, 16, "Xoff:%.0f", (double)offX);
//     OLED_Printf(0, 32, 16, "Yoff:%.0f", (double)offY);
//     OLED_Printf(0, 48, 16, "Zoff:%.0f", (double)offZ);
//     OLED_Refresh();
//     delay_ms(1500);

//     /* ---- 卡尔曼滤波器初始化 ---- */
//     Kalman_Init(&kf_roll,  0.1f, 1.0f);
//     Kalman_Init(&kf_pitch, 0.1f, 1.0f);
//     Kalman_Init(&kf_yaw,   0.1f, 1.0f);

//     OLED_Clear();

//     while (1)
//     {
//         /* ===== 32 位毫秒时间戳（无回绕风险） ===== */
//         tick_now = (uint32_t)Count1 * 1000 + Count0;
//         dt = (float)(tick_now - tick_last) * 0.001f;
//         tick_last = tick_now;

//         /* dt 限幅 */
//         if (dt < 0.0005f) dt = 0.0005f;
//         if (dt > 0.050f)  dt = 0.050f;

//         /* ===== 每 10ms 读取一次传感器（不拖累主循环） ===== */
//         if (tick_now - sensor_tick >= 10)
//         {
//             sensor_tick = tick_now;
//             MPU6050_GetData(&AccX, &AccY, &AccZ,
//                             &GyroX, &GyroY, &GyroZ);
//             HMC5883L_GetData(&MagX, &MagY, &MagZ);
//         }

//         /* ===== 每帧都做姿态解算（使用缓存值，无 I2C 阻塞） ===== */
//         AHRS_Update(AccX, AccY, AccZ, GyroZ,
//                     MagX, MagY, MagZ, dt);

//         /* 获取姿态角 */
//         roll  = AHRS_GetRoll();
//         pitch = AHRS_GetPitch();
//         yaw   = AHRS_GetYaw();

//         /* ===== 一维卡尔曼滤波平滑角度（dt单位ms） ===== */
//         roll_kf  = Kalman_Update(&kf_roll,  roll,  dt * 1000.0f);
//         pitch_kf = Kalman_Update(&kf_pitch, pitch, dt * 1000.0f);
//         yaw_kf   = Kalman_Update(&kf_yaw,   yaw,   dt * 1000.0f);

//         /* ===== 每帧都回传串口波形（蓝牙 + 调试串口同时发送） ===== */
//         if (page == 0)
//         {
//             BlueSerial_Printf("[plot,%d,%d,%d]\r\n",
//                 (int16_t)(roll_kf * 10.0f),
//                 (int16_t)(pitch_kf * 10.0f),
//                 (int16_t)(yaw_kf * 10.0f));
//             Serial_Printf("%d,%d,%d\n",
//                 (int16_t)(roll_kf * 10.0f),
//                 (int16_t)(pitch_kf * 10.0f),
//                 (int16_t)(yaw_kf * 10.0f));
//         }
//         else
//         {
//             BlueSerial_Printf("[plot,%d,%d,%d]\r\n", MagX, MagY, MagZ);
//             Serial_Printf("%d,%d,%d\n", MagX, MagY, MagZ);
//         }

//         /* ===== OLED 每5帧刷新一次（节流） ===== */
//         oled_cnt++;
//         if (oled_cnt >= 5)
//         {
//             oled_cnt = 0;

//             if (Key_GetNum() == 4)
//             {
//                 page = !page;
//                 OLED_Clear();
//             }

//             if (page == 0)
//             {
//                 OLED_Printf(0, 0, 16, "Roll:%+06.1f", (double)roll_kf);
//                 OLED_Printf(0, 16, 16, "Pitch:%+05.1f", (double)pitch_kf);
//                 OLED_Printf(0, 32, 16, "Yaw:%06.1f", (double)yaw_kf);
//             }
//             else
//             {
//                 AHRS_GetMagOffset(&offX, &offY, &offZ);
//                 OLED_Printf(0, 0, 16, "MagX:%+05d", MagX);
//                 OLED_Printf(0, 16, 16, "MagY:%+05d", MagY);
//                 OLED_Printf(0, 32, 16, "MagZ:%+05d", MagZ);
//                 OLED_Printf(0, 48, 12, "Gz:%+04d OfX:%.0f",
//                     GyroZ, (double)offX);
//             }

//             OLED_Refresh();
//         }
//     }
// }

// /**
//  * @brief  定时器中断服务函数（每 1ms 触发一次）
//  * @note   Count0/Count1 为 Timer.c 引用的外部变量
//  *         Key_Tick() 提供非阻塞按键扫描
//  */
// void TIMER_0_INST_IRQHandler(void)
// {
//     Count0++;
//     if (Count0 >= 1000) {
//         Count0 = 0;
//         Count1++;
//     }

//     Key_Tick();     /* 按键扫描（内部20ms检测一次） */
// }
