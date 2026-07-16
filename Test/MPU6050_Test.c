#if 1

/**
 * @file    MPU6050_Test.c
 * @brief   MPU6050 DMP 测试程序（使用 mpu_port 共享驱动）
 * @details 调用 mpu_port.c 的 MPU_Update()（FIFO 排空方案），
 *          在 OLED 上实时显示 Pitch/Roll/Yaw，并通过蓝牙串口发送 YAW 波形。
 *
 * 硬件连接 (I2C1)：
 *   GY-87 VCC -> 3.3V
 *   GY-87 GND -> GND
 *   GY-87 SCL -> PB2 (I2C1_SCL)
 *   GY-87 SDA -> PB3 (I2C1_SDA)
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "Timer.h"
#include "mpu6050/mpu_port.h"
#include "BlueSerial.h"

int main(void)
{
    uint8_t oled_cnt = 0;

    SYSCFG_DL_init();
    BlueSerial_Init();
    Timer_Init();
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    if (DMP_Init() != 0)
    {
        OLED_Printf(0, 0, 16, "DMP Init FAIL!");
        OLED_Refresh();
        while (1);
    }

    mpu_ok = 1;
    OLED_Printf(0, 0, 16, "Calibrating...");
    OLED_Refresh();

    while (1)
    {
        /* 使用和 empty.c 同一套 MPU_Update()（FIFO 排空累积） */
        MPU_Update();

        /* 蓝牙每帧发送 corrected_yaw 波形 */
        if (mpu_calibrated)
            BlueSerial_Printf("[plot,%d,%d]\r\n", (int16_t)(mpu_corrected_yaw * 100), 0);

        /* OLED 每 5 帧刷新一次 */
        oled_cnt++;
        if (oled_cnt >= 5)
        {
            oled_cnt = 0;
            OLED_Printf(0, 0, 16, "Pitch:%+07.2f", (double)mpu_pitch);
            OLED_Printf(0, 16, 16, "Roll:%+07.2f", (double)mpu_roll);
            if (!mpu_calibrated)
                OLED_Printf(0, 32, 16, "Calib:%3d%%", (int)MPU_GetCalibProgress());
            else
                OLED_Printf(0, 32, 16, "Yaw:%+07.2f", (double)mpu_corrected_yaw);
            OLED_Refresh();
        }

        // delay_ms(10);
    }
}

#endif
