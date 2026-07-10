// /*
//  * Copyright (c) 2021, Texas Instruments Incorporated
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * *  Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  *
//  * *  Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in the
//  *    documentation and/or other materials provided with the distribution.
//  *
//  * *  Neither the name of Texas Instruments Incorporated nor the names of
//  *    its contributors may be used to endorse or promote products derived
//  *    from this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
//  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */

// /**
//  * @file    empty.c
//  * @brief   主程序：双环PID循迹控制
//  * @details 基于 MSPM0G3507 的双环PID循迹小车
//  *
//  * 外环（位置环）：
//  *   8路灰度传感器 → Grayscale_GetDeviation_Track() 计算加权偏差
//  *   → Racecar() 差速分配 → 左右轮目标速度
//  *
//  * 内环（速度环）：
//  *   编码器10ms测速(5点滑动平均) → Motor_Speed_PID_Update()
//  *   → PWM输出(0~8000) → 电机驱动
//  *
//  * 控制周期：
//  *   外环：主循环中软实时执行
//  *   内环：TIMER_0 1ms中断中每20ms执行一次
//  *
//  * 拐弯策略：三段式状态机（IDLE → DECEL减速直行 → ACTIVE差速拐弯）
//  *   适配8路传感器：Gray_8(最左)/Gray_1(最右)作为拐弯检测
//  *                 Gray_7~Gray_2作为偏差计算
//  *
//  * 按键功能：
//  *   Key1 - 循迹启停
//  */

// #include "ti_msp_dl_config.h"
// #include "delay.h"
// #include "oled.h"
// #include "Timer.h"
// #include "Motor.h"
// #include "Encoder.h"
// #include "Grayscale.h"
// #include "key.h"
// #include "Serial.h"
// #include "BlueSerial.h"

// int main(void)
// {
//     SYSCFG_DL_init();

//     /* ---- 外设初始化 ---- */
//     Timer_Init();                        /* TIMER_0 1ms系统时基 */
//     OLED_Init();                         /* OLED 128×64 */
//     OLED_ColorTurn(0);
//     OLED_DisplayTurn(0);
//     OLED_Clear();

//     Motor_Init();                        /* 电机GPIO+PWM */
//     Encoder_Init();                      /* 编码器GPIO中断 */
//     BlueSerial_Init();                   /* 蓝牙UART0中断 */
//     Motor_Speed_Init();                  /* 速度环PID清零 */
//     Gray_Sensor_Init();                  /* 灰度传感器（SysConfig已初始化） */

//     OLED_Printf(0, 0, 16, "Tracking Ready!");
//     OLED_Refresh();
//     delay_ms(500);

//     OLED_Clear();  /* 进入主循环前清屏，避免 "Tracking Ready!" 比后续文字长而残留 */
//     OLED_Refresh();

//     while (1)
//     {
//         /* ---- 按键扫描 ---- */
//         uint8_t key = Key_GetNum();

//         /* Key1：循迹启停（翻转模式） */
//         if (key == 1)
//         {
//             if (RunFlag == 0)
//             {
//                 /* 启动 */
//                 RunFlag = 1;
//                 Turn_State = TURN_IDLE;
//                 Turn_Direction = 0;
//                 Gray_SoftStart_Reset();  /* 确保软启动从零开始 */
//             }
//             else
//             {
//                 /* 停止 */
//                 RunFlag = 0;
//                 Motor_SetTargetSpeed(0, 0);
//                 Gray_SoftStart_Reset();
//                 Motor_Speed_PID_Reset();
//             }
//         }

//         /* ---- 循迹控制 ---- */
//         if (RunFlag)
//         {
//             Gray_Track_Control();        /* 三段式拐弯 + 灰度偏差PD差速 */
//         }
//         else
//         {
//             Motor_SetTargetSpeed(0, 0);
//             Motor_Speed_PID_Reset();     /* 复位PID积分为零 */
//             Motor_SetPWM(MOTOR_LEFT, 0);
//             Motor_SetPWM(MOTOR_RIGHT, 0);
//         }

//         /* ---- 蓝牙调参（解析 [slider,Name,Value] 协议） ---- */
//         BlueSerial_Tasks();

//         /* ---- OLED显示（8号字体，固定宽度防残留） ---- */
//         OLED_Printf(0, 0,  8, "Track:%s", RunFlag ? "ON " : "OFF");
//         OLED_Printf(0, 8, 8, "TgtL:%-4d TgtR:%-4d", PWML, PWMR);
//         OLED_Printf(0, 16, 8, "ActL:%-4d ActR:%-4d",
//             Encoder_GetSpeed(ENCODER_LEFT),
//             Encoder_GetSpeed(ENCODER_RIGHT));
//         OLED_Printf(0, 24, 8, "Dev:%6.2f", Grayscale_GetDeviation_Track());
//         OLED_Printf(0, 32, 8, "Dir:%-2d St:%-2d",
//             Turn_Direction, Turn_State);
//         OLED_Printf(0, 40, 8, "Kp:%-3.1f Ki:%-3.1f Kd:%-3.1f",
//             Motor_Kp, Motor_Ki, Motor_Kd);
//         OLED_Printf(0, 48, 8, "TKp:%-5.1f TKd:%-5.1f",
//             Track_Kp, Track_Kd);
//         OLED_Refresh();

//         /* ---- 串口1发送四参数：左目标,右目标,左实际,右实际 ---- */
//         {
//             Serial_Printf("%d,%d,%d,%d\n",
//                 (int16_t)motor_l_ctrl.target_speed,
//                 (int16_t)motor_r_ctrl.target_speed,
//                 Encoder_GetSpeed(ENCODER_LEFT),
//                 Encoder_GetSpeed(ENCODER_RIGHT));
//         }
//     }
// }
