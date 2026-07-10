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

    OLED_Printf(0, 0, 16, "Tracking Ready!");
    OLED_Refresh();
    delay_ms(500);

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

        /* ---- 蓝牙摇杆手动控制（独立于 RunFlag，即开即用） ---- */
        if (Joystick_Active)
        {
            int16_t base = (int16_t)((int32_t)Joystick_LV * BaseSpeed / 100);
            int16_t turn = (int16_t)((int32_t)Joystick_RH * JoyTurn / 100);
            Motor_SetTargetSpeed(base + turn, base - turn);

            PWML = (int16_t)motor_l_ctrl.target_speed;
            PWMR = (int16_t)motor_r_ctrl.target_speed;

            /* 手动调用速度PID（底层Timer.c只响应RunFlag，这里直接驱动电机） */
            Motor_Speed_PID_Update();
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

        /* ---- 蓝牙调参（解析 [slider,Name,Value] 和 [joystick,...]） ---- */
        BlueSerial_Tasks();

        /* ---- OLED显示（8号字体，固定宽度防残留） ---- */
        OLED_Printf(0, 0,  8, "Track:%s", RunFlag ? "ON " : "OFF");
        if (Joystick_Active)
            OLED_Printf(0, 0,  8, "JoyStick!  ");
        OLED_Printf(0, 8, 8, "TgtL:%-4d TgtR:%-4d", PWML, PWMR);
        OLED_Printf(0, 16, 8, "ActL:%-4d ActR:%-4d",
            Encoder_GetSpeed(ENCODER_LEFT),
            Encoder_GetSpeed(ENCODER_RIGHT));
        OLED_Printf(0, 24, 8, "Dev:%6.2f", Grayscale_GetDeviation_Track());
        OLED_Printf(0, 32, 8, "Dir:%-2d St:%-2d",
            Turn_Direction, Turn_State);
        OLED_Printf(0, 40, 8, "Kp:%-5.1f Ki:%-5.1f Kd:%-5.1f",
            Motor_Kp, Motor_Ki, Motor_Kd);
        OLED_Printf(0, 48, 8, "TKp:%-5.1f TKd:%-5.1f",
            Track_Kp, Track_Kd);
        OLED_Printf(0, 56, 8, "BSp:%-4d JTrn:%-3d",
            BaseSpeed, JoyTurn);
        OLED_Refresh();

        /* ---- 串口1发送四参数：左目标,右目标,左实际,右实际 ---- */
        {
            Serial_Printf("%d,%d,%d,%d\n",
                (int16_t)motor_l_ctrl.target_speed,
                (int16_t)motor_r_ctrl.target_speed,
                Encoder_GetSpeed(ENCODER_LEFT),
                Encoder_GetSpeed(ENCODER_RIGHT));
        }  /* 串口输出结束 */

    }  /* while(1) 结束 */
}
