#if 0

/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    empty.c
 * @brief   主程序：循迹控制 + 蓝牙摇杆手动控制
 *
 * 模式切换（按 Key1 启停）：
 *   停止：RunFlag=0，电机停转
 *   循迹：RunFlag=1 + 摇杆居中 → 灰度传感器循迹
 *   手动：RunFlag=1 + 摇杆推动 → 摇杆控制前进/后退/转向
 *
 * 内环（速度环）由 TIMER_0 1ms 中断中每 20ms 执行：
 *   编码器测速(5点滑动平均) → Motor_Speed_PID_Update() → PWM输出
 *
 * 蓝牙协议（与江科大平衡车兼容）：
 *   [joystick,LH,LV,RH,RV]  摇杆控制
 *   [slider,Name,Value]      参数调节
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "Timer.h"
#include "Motor.h"
#include "Encoder.h"
#include "Grayscale.h"
#include "key.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "mpu6050/mpu_port.h"   /* MPU6050 DMP 姿态解算 */

/* 陀螺仪比例补偿：物理转 360° 只测出 327°，故补偿因子 = 360/327 ≈ 1.101 */
#define GYRO_SCALE_CORR    (360.0f / 327.0f)
/*所有程序写完要重新测量这个比例补偿系数，受代码长度影响*/


int main(void)
{
    SYSCFG_DL_init();

    /* ---- 外设初始化 ---- */
    Timer_Init();                        /* TIMER_0 1ms系统时基 */
    OLED_Init();                         /* OLED 128×64 */
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    Motor_Init();                        /* 电机GPIO+PWM */
    Encoder_Init();                      /* 编码器GPIO中断 */
    BlueSerial_Init();                   /* 蓝牙UART0中断 */
    Motor_Speed_Init();                  /* 速度环PID清零 */
    Gray_Sensor_Init();                  /* 灰度传感器 */

    /* ---- MPU6050 DMP 初始化（GY-87 陀螺仪） ---- */
    float pitch = 0, roll = 0, yaw = 0;
    float yaw_offset = 0.0f, yaw_last = 0.0f;
    float corrected_yaw = 0.0f, raw_yaw_prev = 0.0f;
    float yaw_drift = 0.0f;
    uint16_t stable_cnt = 0;
    uint8_t calib_done = 0;
    uint8_t disp_page = 1;              /* 0=小车状态, 1=陀螺仪 */

    OLED_Printf(0, 0, 16, "BlueCar + GY-87");
    OLED_Printf(0, 16, 16, "DMP Init...");
    OLED_Refresh();

    if (DMP_Init() != 0)
    {
        OLED_Clear();
        OLED_Printf(0, 0, 16, "DMP Init FAIL!");
        OLED_Refresh();
        while (1);
    }

    OLED_Clear();
    OLED_Printf(0, 0, 16, "Calibrating...");
    OLED_Printf(0, 16, 16, "Keep GY-87 still");
    OLED_Refresh();

    OLED_Clear();
    OLED_Refresh();

    while (1)
    {
        /* ---- 按键扫描 ---- */
        uint8_t key = Key_GetNum();

        /* Key1：启停翻转 */
        if (key == 1)
        {
            if (RunFlag == 0)
            {
                RunFlag = 1;
                Turn_State = TURN_IDLE;
                Turn_Direction = 0;
                Gray_SoftStart_Reset();
            }
            else
            {
                RunFlag = 0;
                Motor_SetTargetSpeed(0, 0);
                Gray_SoftStart_Reset();
                Motor_Speed_PID_Reset();
            }
        }

        /* Key4：切换显示页面 */
        if (key == 4)
        {
            disp_page = !disp_page;
            OLED_Clear();
        }

        /* ---- 蓝牙调参（解析 [slider,Name,Value] 和 [joystick,...]） ---- */
        BlueSerial_Tasks();

        /* ---- 蓝牙摇杆手动控制（独立于 RunFlag，即开即用） ---- */
        if (Joystick_Active)
        {
            int16_t base = (int16_t)((int32_t)Joystick_LV * BaseSpeed / 100);
            int16_t turn = (int16_t)((int32_t)Joystick_RH * JoyTurn / 100);
            Motor_SetTargetSpeed(base + turn, base - turn);

            PWML = (int16_t)motor_l_ctrl.target_speed;
            PWMR = (int16_t)motor_r_ctrl.target_speed;

            /* PID 由 Timer ISR 统一调度（每20ms），主循环只设目标速度 */
        }
        /* ---- 循迹控制（需 RunFlag=1） ---- */
        else if (RunFlag)
        {
            Gray_Track_Control();
        }
        /* ---- 停止 ---- */
        else
        {
            Motor_SetTargetSpeed(0, 0);
            Motor_Speed_PID_Reset();
            Motor_SetPWM(MOTOR_LEFT, 0);
            Motor_SetPWM(MOTOR_RIGHT, 0);
        }

        /* ---- 读取 DMP 姿态角（约 100Hz） ---- */
        if (DMP_Read_Data(&pitch, &roll, &yaw) == 0)
        {
            /* 自动校准：检测 YAW 漂移率，稳定后锁定零偏 */
            if (!calib_done)
            {
                yaw_drift = yaw - yaw_last;
                yaw_last = yaw;
                if (yaw_drift < 0.0f) yaw_drift = -yaw_drift;
                if (yaw_drift < 0.1f)
                    stable_cnt++;
                else
                    stable_cnt = 0;

                if (stable_cnt >= 50)  /* 连续 500ms 漂移 < 0.1° */
                {
                    yaw_offset = yaw;
                    raw_yaw_prev = 0.0f;
                    corrected_yaw = 0.0f;
                    calib_done = 1;
                    OLED_Clear();
                }
            }
            else
            {
                /* 校准完成：对每帧角度增量做补偿，避免绝对值跨 ±180° 边界跳变 */
                float raw = yaw - yaw_offset;
                float delta = raw - raw_yaw_prev;
                raw_yaw_prev = raw;

                if (delta > 180.0f) delta -= 360.0f;
                if (delta < -180.0f) delta += 360.0f;

                corrected_yaw += delta * GYRO_SCALE_CORR;

                if (corrected_yaw > 180.0f) corrected_yaw -= 360.0f;
                if (corrected_yaw < -180.0f) corrected_yaw += 360.0f;
            }
        }

        /* ---- OLED 显示（每5帧刷新一次，减少I2C总线竞争） ---- */
        static uint8_t oled_cnt = 0;
        oled_cnt++;
        if (oled_cnt >= 5)
        {
            oled_cnt = 0;
            if (disp_page == 0)
            {
                /* 页面0：小车状态（8号字体，固定宽度防残留） */
                OLED_Printf(0, 0,  8, "Track:%s", RunFlag ? "ON " : "OFF");
                if (Joystick_Active)
                    OLED_Printf(0, 0,  8, "JoyStick!  ");
                OLED_Printf(0, 8,  8, "TgtL:%-4d TgtR:%-4d", PWML, PWMR);
                OLED_Printf(0, 16, 8, "ActL:%-4d ActR:%-4d",
                    Encoder_GetSpeed(ENCODER_LEFT),
                    Encoder_GetSpeed(ENCODER_RIGHT));
                OLED_Printf(0, 24, 8, "Dev:%6.2f", Grayscale_GetDeviation_Track());
                OLED_Printf(0, 32, 8, "Dir:%-2d St:%-2d",
                    Turn_Direction, Turn_State);
                OLED_Printf(0, 40, 8, "Kp:%-3.1f Ki:%-3.1f Kd:%-3.1f",
                    Motor_Kp, Motor_Ki, Motor_Kd);
                OLED_Printf(0, 48, 8, "TKp:%-5.1f TKd:%-5.1f",
                    Track_Kp, Track_Kd);
                OLED_Printf(0, 56, 8, "BSp:%-4d JTrn:%-3d",
                    BaseSpeed, JoyTurn);
            }
            else
            {
                /* 页面1：陀螺仪姿态角（16号字体） */
                OLED_Printf(0, 0,  16, "Pitch:%+07.2f", (double)pitch);
                OLED_Printf(0, 16, 16, "Roll:%+07.2f", (double)roll);
                if (!calib_done)
                    OLED_Printf(0, 32, 16, "Calib:%d", (int)(stable_cnt * 2));
                else
                    OLED_Printf(0, 32, 16, "Yaw:%+07.2f", (double)corrected_yaw);
                OLED_Printf(0, 48, 16, "Key4<->Page");
            }
            OLED_Refresh();
        }

        /* ---- 串口1发送四参数：左目标,右目标,左实际,右实际 ---- */
        {
            Serial_Printf("%d,%d,%d,%d\n",
                (int16_t)motor_l_ctrl.target_speed,
                (int16_t)motor_r_ctrl.target_speed,
                Encoder_GetSpeed(ENCODER_LEFT),
                Encoder_GetSpeed(ENCODER_RIGHT));
        }  /* 串口输出结束 */

         BlueSerial_Printf("[plot,%d,%d]\r\n",
                (int16_t)(corrected_yaw * 100),
                0);

        delay_ms(10);
    }  /* while(1) 结束 */
}

#endif