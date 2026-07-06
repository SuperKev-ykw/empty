/**
 * @file    GY_87_Test.c
 * @brief   GY-87 ???????MPU6050 + HMC5883L ????
 * @details ????? MPU6050 ???????? + HMC5883L ???????
 *          ?? AHRS ???????? Roll/Pitch/Yaw?? OLED ????
 *          OLED ??????Key4 ?????? / ??????
 *          I2C ?? 10ms ??????AHRS ????????? I2C ????
 *
 * ?????
 *   - ??? MPU6050 ???Hardware/MPU6050.c?
 *   - HMC5883L ????Hardware/HMC5883L.c?
 *   - AHRS ?????Hardware/AHRS.c?
 *   - Key4 ????
 *   - ?? + ??????
 *
 * ?????
 *   - ??? OLED ?? "GY-87 Init..."?????? "GY-87 OK!"
 *   - ?? "Mag Calibrating... Rotate 360 deg"?? Key4 ????
 *   - ?? "Mag Calib Done!" + ???
 *   - ????????
 *       ? 0?Roll / Pitch / Yaw ?????
 *       ? 1?MagX / MagY / MagZ + ??? Z ? + ???
 *   - ?????? "[plot,Roll,Pitch,Yaw]\r\n"?10????
 *
 * ???? (SysConfig I2C1)?
 *   GY-87 VCC -> 3.3V
 *   GY-87 GND -> GND
 *   GY-87 SCL -> PB2 (I2C1_SCL)
 *   GY-87 SDA -> PB3 (I2C1_SDA)
 *
 * ????????? #if 0 ?????????? MPU6050_Test.c
 */

#if 0

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "AHRS.h"
#include "key.h"
#include "Timer.h"
#include "BlueSerial.h"
#include "Serial.h"

int main(void)
{
    int16_t AccX, AccY, AccZ;
    int16_t GyroX, GyroY, GyroZ;
    int16_t MagX, MagY, MagZ;

    float roll, pitch, yaw;
    float offX, offY, offZ;

    uint8_t page = 0;           /* 0=???, 1=????*/
    uint8_t oled_cnt = 0;       /* OLED????????*/

    /* ---- 32 ??????????Count0 ??????---- */
    uint32_t tick_now;          /* ???? (Count1*1000 + Count0) */
    uint32_t tick_last = 0;     /* ????????*/
    uint32_t sensor_tick = 0;   /* ????????????*/
    float dt;                   /* ??????(s) */

    SYSCFG_DL_init();
    Timer_Init();               /* ??????????????*/
    BlueSerial_Init();          /* ????????*/
    BlueSerial_Printf("GY-87 Test Start!\r\n");

    /* ???OLED */
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    OLED_Printf(0, 0, 16, "GY-87 Init...");
    OLED_Refresh();

    /* ???MPU6050 (??????I2C1 + ?????? */
    MPU6050_Init();

    /* ???HMC5883L????*/
    HMC5883L_Init();

    /* ??MPU6050???? */
    if (MPU6050_GetID() != 0x68)
    {
        OLED_Printf(0, 0, 16, "MPU6050 ERROR!");
        OLED_Printf(0, 16, 16, "ID:0x%02X", MPU6050_GetID());
        OLED_Refresh();
        while (1);
    }

    OLED_Clear();
    OLED_Printf(0, 0, 16, "GY-87 OK!");
    OLED_Refresh();
    delay_ms(500);

    /* ===== ??????=====
     * ????????????????????*/
    OLED_Clear();
    OLED_Printf(0, 0, 16, "Mag Calibrating...");
    OLED_Printf(0, 16, 16, "Rotate 360 deg");
    OLED_Printf(0, 48, 16, "Press Key4 end");
    OLED_Refresh();

    /* ??????4???????????????? */
    do
    {
        for (int i = 0; i < 50; i++)    /* ??0???????? */
        {
            HMC5883L_GetData(&MagX, &MagY, &MagZ);
            AHRS_CalibrateMag(MagX, MagY, MagZ);
            delay_ms(10);
        }
        OLED_Printf(0, 32, 16, "Progress...");
        OLED_Refresh();
    } while (Key_GetNum() != 4);

    AHRS_GetMagOffset(&offX, &offY, &offZ);
    OLED_Clear();
    OLED_Printf(0, 0, 16, "Mag Calib Done!");
    OLED_Printf(0, 16, 16, "Xoff:%.0f", (double)offX);
    OLED_Printf(0, 32, 16, "Yoff:%.0f", (double)offY);
    OLED_Printf(0, 48, 16, "Zoff:%.0f", (double)offZ);
    OLED_Refresh();
    delay_ms(1500);

    OLED_Clear();

    while (1)
    {
        /* ===== 32 ??????????????===== */
        tick_now = (uint32_t)Count1 * 1000 + Count0;
        dt = (float)(tick_now - tick_last) * 0.001f;
        tick_last = tick_now;

        /* dt ?? */
        if (dt < 0.0005f) dt = 0.0005f;
        if (dt > 0.050f)  dt = 0.050f;

        /* ===== ??10ms ??????????????? ===== */
        if (tick_now - sensor_tick >= 10)
        {
            sensor_tick = tick_now;
            MPU6050_GetData(&AccX, &AccY, &AccZ,
                            &GyroX, &GyroY, &GyroZ);
            HMC5883L_GetData(&MagX, &MagY, &MagZ);
        }

        /* ===== ?????????????????I2C ????===== */
        AHRS_Update(AccX, AccY, AccZ, GyroZ,
                    MagX, MagY, MagZ, dt);

        /* ????? */
        roll  = AHRS_GetRoll();
        pitch = AHRS_GetPitch();
        yaw   = AHRS_GetYaw();

        /* ===== ???????????? + ????????? ===== */
        if (page == 0)
        {
            BlueSerial_Printf("[plot,%d,%d,%d]\r\n",
                (int16_t)(roll * 10.0f),
                (int16_t)(pitch * 10.0f),
                (int16_t)(yaw * 10.0f));
            Serial_Printf("%d,%d,%d\n",
                (int16_t)(roll * 10.0f),
                (int16_t)(pitch * 10.0f),
                (int16_t)(yaw * 10.0f));
        }
        else
        {
            BlueSerial_Printf("[plot,%d,%d,%d]\r\n", MagX, MagY, MagZ);
            Serial_Printf("%d,%d,%d\n", MagX, MagY, MagZ);
        }

        /* ===== OLED ????????????===== */
        oled_cnt++;
        if (oled_cnt >= 5)
        {
            oled_cnt = 0;

            if (Key_GetNum() == 4)
            {
                page = !page;
                OLED_Clear();
            }

            if (page == 0)
            {
                OLED_Printf(0, 0, 16, "Roll:%+06.1f", (double)roll);
                OLED_Printf(0, 16, 16, "Pitch:%+05.1f", (double)pitch);
                OLED_Printf(0, 32, 16, "Yaw:%06.1f", (double)yaw);
            }
            else
            {
                AHRS_GetMagOffset(&offX, &offY, &offZ);
                OLED_Printf(0, 0, 16, "MagX:%+05d", MagX);
                OLED_Printf(0, 16, 16, "MagY:%+05d", MagY);
                OLED_Printf(0, 32, 16, "MagZ:%+05d", MagZ);
                OLED_Printf(0, 48, 12, "Gz:%+04d OfX:%.0f",
                    GyroZ, (double)offX);
            }

            OLED_Refresh();
        }
    }
}
#endif
