#if 0

/**
 * @file    MPU6050_Test.c
 * @brief   MPU6050 DMP 测试程序 + YAW 角度闭环
 * @details MPU6050 姿态显示 + 蓝牙波形 + YAW 角度闭环控制
 *
 * YAW 闭环功能：
 *   校准完成后当前 YAW = 0°，通过按键设定目标角度，小车自动旋转对准。
 *   类似电机速度环：目标 YAW → PI 控制器 → 差速输出 → 电机速度环 → 实际 YAW 跟随。
 *
 *   控制链路：
 *     yaw_target --[PI]--> turn_deviation --[Motor_SetTargetSpeed]--> 速度内环(20ms) --> PWM
 *
 * 按键功能：
 *   Key2 - 目标回零（对准初始方向）
 *   Key3 - 目标 +90°（右转 90°）
 *   Key4 - 目标 -90°（左转 90°）
 *
 * 硬件连接：
 *   MPU6050 (I2C1) : SCL=PB2, SDA=PB3
 *   电机驱动       : AT8236（同主工程）
 *   编码器         : 左=PB13/PB14, 右=PA7/PA25
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "Timer.h"
#include "key.h"
#include "mpu6050/mpu_port.h"
#include "BlueSerial.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"

#include "mpu6050/MPU6050_PID.h"   /* YAW PID 变量及函数 */

/* ==================== 主函数 ==================== */

int main(void)
{
    uint32_t tick_last = 0;
    uint8_t oled_cnt = 0;

    SYSCFG_DL_init();
    BlueSerial_Init();
    Timer_Init();
    OLED_Init();

    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    /* MPU6050 必须在外设中断使能前初始化，避免中断干扰 I2C 时序 */
    if (DMP_Init() != 0)
    {
        OLED_Printf(0, 0, 16, "DMP Init FAIL!");
        OLED_Refresh();
        while (1);
    }

    mpu_ok = 1;
    OLED_Printf(0, 0, 16, "Calibrating...");

    OLED_Refresh();

    /* DMP 初始化成功后再初始化电机/编码器（会开启 GPIO 中断和定时器中断） */
    Motor_Init();
    Encoder_Init();
    Motor_Speed_Init();

    /* 清除初始界面残留 */
    OLED_Clear();

    while (1)
    {
        /* ========== MPU6050 更新（FIFO 排空累积） ========== */
        MPU_Update();

        /* ========== 按键处理 ========== */
        uint8_t key = Key_GetNum();
        if (key != 0)
        {
            switch (key)
            {
            case 1:  /* Key1: 启停 YAW 闭环 */
                g_yaw_ctrl_on = !g_yaw_ctrl_on;
                if (!g_yaw_ctrl_on) Yaw_Reset();
                break;
            case 2:  /* Key2: 目标回零 */
                g_yaw_target = 0.0f;
                Yaw_Reset();
                break;
            case 3:  /* Key3: 目标 +90°（右转） */
                g_yaw_target = NormalizeAngle(g_yaw_target + 90.0f);
                break;
            case 4:  /* Key4: 目标 -90°（左转） */
                g_yaw_target = NormalizeAngle(g_yaw_target - 90.0f);
                break;
            }
        }

        /* ========== 蓝牙调参 ========== */
        BlueSerial_Tasks();

        /* ========== 校准完成后清屏 + 自动开启 YAW 闭环 ========== */
        if (mpu_calibrated && !g_yaw_ctrl_on)
        {
            g_yaw_ctrl_on = 1;
            OLED_Clear();
        }

        /* ========== 每 20ms：YAW 控制 + 速度内环 ========== */
        uint32_t tick_now = (uint32_t)Count1 * 1000 + Count0;
        if (tick_now - tick_last >= 20)
        {
            tick_last = tick_now;

            /* YAW 角度 PI 控制 → 设置目标速度 */
            Yaw_PI_Control();

            /* 速度内环：编码器 → PI → PWM（由 Timer.c ISR 也可调度，这里确保运行） */
            Motor_Speed_PID_Update();
        }

        /* ========== 调试输出（每帧发送） ========== */
        // BlueSerial_Printf("[plot,%d,%d,%d,%d]\r\n",
        //     motor_l_ctrl.target_speed,
        //     motor_r_ctrl.target_speed,
        //     motor_l_ctrl.actual_speed,
        //     motor_r_ctrl.actual_speed);

        Serial_Printf("%d,%d,%d,%d\n",
            motor_l_ctrl.target_speed,
            motor_r_ctrl.target_speed,
            motor_l_ctrl.actual_speed,
            motor_r_ctrl.actual_speed);

        /* ========== OLED 显示 ========== */
        oled_cnt++;
        if (oled_cnt >= 5)
        {
            oled_cnt = 0;

            float err = NormalizeAngle(g_yaw_target - mpu_corrected_yaw);

            OLED_Printf(0, 0, 8, "Pitch:%+07.2f", (double)mpu_pitch);
            OLED_Printf(0, 8, 8, "Roll:%+07.2f", (double)mpu_roll);

            if (!mpu_calibrated)
            {
                OLED_Printf(0, 16, 8, "Calib:%3d%%   ", (int)MPU_GetCalibProgress());
            }
            else
            {
                OLED_Printf(0, 16, 8, "Yaw:%+07.2f  ", (double)mpu_corrected_yaw);
                OLED_Printf(0, 24, 8, "T:%+06.1f E:%+06.1f",
                    (double)g_yaw_target, (double)err);
                OLED_Printf(0, 32, 8, "P:%.2f I:%.2f D:%.2f",
                    (double)g_yaw_kp, (double)g_yaw_ki, (double)g_yaw_kd);
            }

            OLED_Refresh();
        }
    }
}

#endif
