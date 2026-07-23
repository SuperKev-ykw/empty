#if 1
/**
 * @file    empty.c
 * @brief   MSPM0G3507 循迹小车主程序
 *
 * 硬件平台：MSPM0G3507（TI ARM Cortex-M0+）
 *
 * 外设及功能：
 *   1. 8路灰度传感器循迹（8路GPIO数字输入，白=0/黑=1）
 *      三段式拐弯状态机：TURN_IDLE(PD循迹) → TURN_DECEL(减速直行) → TURN_ACTIVE(差速拐弯)
 *   2. 电机驱动：AT8236（2路PWM + 2路GPIO方向控制）
 *      双环控制：PD外环（位置偏差）+ 速度PID内环（编码器反馈）
 *      通用 SpeedRamp 平滑速度过渡（起步/减速/出弯）
 *   3. 蓝牙遥控：UART0 通信协议 [joystick,LH,LV,RH,RV] / [slider,Name,Value]
 *      接收摇杆手动控制，接收滑块实时调节 Kp/Ki/Kd/BaseSpeed 等参数
 *   4. MPU6050 陀螺仪（I2C1）DMP 姿态解算，OLED 显示 Pitch/Roll/Yaw
 *   5. OLED 0.96寸（I2C0）显示实时数据：目标速度、PID参数、拐弯状态等
 *   6. 编码器速度闭环（Timer capture）+ 1ms系统定时中断（调度10ms循迹/20ms PID）
 *   7. 软启动 + 全黑出界保护
 *
 * 外设模块：
 *   - Hardware/Grayscale.c/h：灰度传感器 + 循迹状态机 + SpeedRamp
 *   - Hardware/Motor.c/h：PID速度闭环 + 正反转PWM控制
 *   - Hardware/Encoder.c/h：编码器速度测量
 *   - Hardware/OLED.c/h：OLED屏幕驱动
 *   - Hardware/BlueSerial.c/h：蓝牙串口通信协议处理
 *   - Hardware/mpu6050/：MPU6050 DMP 陀螺仪驱动
 *   - Hardware/Serial.c/h：备用UART1串口
 *   - System/Timer.c：1ms定时器中断调度
 *   - System/key.c：按键扫描驱动
 */

/**
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
#include "mpu6050/mpu_port.h"   /* DMP_Init(), MPU_Update(), mpu_ok */

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
    Serial_Init();                       /* 串口1 UART1中断 */

    /* 串口1双清：排空硬件RX FIFO和软件环形缓冲区，避免上电毛刺被误判为有效帧 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST))
    {
        (void)DL_UART_Main_receiveData(UART_1_INST);
    }
    while (Serial_GetRxCount() > 0)
    {
        (void)Serial_GetRxData();
    }

    Motor_Speed_Init();                  /* 速度环PID清零 */
    Gray_Sensor_Init();                  /* 灰度传感器 */

    /* ---- MPU6050 陀螺仪初始化（I2C1） ---- */
    if (DMP_Init() == 0)
    {
        mpu_ok = 1;
        OLED_Printf(0, 0, 16, "MPU6050 OK!");
    }
    else
    {
        mpu_ok = 0;
        OLED_Printf(0, 0, 16, "MPU6050 FAIL!");
    }
    OLED_Refresh();
    delay_ms(500);

    OLED_Printf(0, 0, 16, "Tracking Ready!");
    OLED_Refresh();
    delay_ms(500);

    OLED_Clear();
    OLED_Refresh();

    while (1)
    {
        /* ========== UART1 远程按键帧解析 ========== */
        {
            uint8_t u1_key = Serial_GetKeyFrame();
            if (u1_key != 0)
            {
                /* 收到新键值立即显示，不受 RunFlag 影响 */
                OLED_Printf(0, 24, 8, "U1: KEY=%d           ", u1_key);
                OLED_Refresh();
            }
            switch (u1_key)
            {
                case 1:
                    /* Key1：启动循迹 */
                    RunFlag = 1;
                    Turn_State = TURN_IDLE;
                    Turn_Direction = 0;
                    Gray_SoftStart_Reset();
                    break;

                case 2:
                    /* Key2：停止循迹 */
                    RunFlag = 0;
                    Motor_SetTargetSpeed(0, 0);
                    Gray_SoftStart_Reset();
                    Motor_Speed_PID_Reset();
                    break;

                default:
                    /* 其他键值：仅显示，不执行动作（保留扩展接口） */
                    break;
            }
        }

        /* ---- 按键扫描 ---- */
        uint8_t key = Key_GetNum();

        /* 按下按键时通过串口1发送 0xAA KEY 0xFF（与参考工程一致） */
        if (key != 0)
        {
            Serial_SendByte(0xAA);
            Serial_SendByte(key);
            Serial_SendByte(0xFF);
        }

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

            /* PID 由 Timer ISR 统一调度（每20ms），主循环只设目标速度 */
        }
        /* ---- 停止（RunFlag=0 且非摇杆控制） ---- */
        else if (!RunFlag)
        {
            Motor_SetTargetSpeed(0, 0);
            Motor_Speed_PID_Reset();
            Motor_SetPWM(MOTOR_LEFT, 0);
            Motor_SetPWM(MOTOR_RIGHT, 0);
        }
        /* 若 RunFlag=1 且非摇杆，Gray_Track_Control() 在 Timer ISR 20ms 调度中执行 */

        /* ---- 蓝牙调参（解析 [slider,Name,Value] 和 [joystick,...]） ---- */
        BlueSerial_Tasks();

        /* ---- MPU6050 更新（自动校准 + 增量累积） ---- */
        if (mpu_ok) MPU_Update();

        /* ---- 距离累加（总脉冲差值，不丢帧） ---- */
        {
            static int32_t prev_l = 0, prev_r = 0;
            int32_t total_l = Encoder_GetTotalCount(ENCODER_LEFT);
            int32_t total_r = Encoder_GetTotalCount(ENCODER_RIGHT);
            Motor_AddDistance((int16_t)(total_l - prev_l), (int16_t)(total_r - prev_r));
            prev_l = total_l;
            prev_r = total_r;
        }

        /* ---- 速度计算 ---- */
        Encoder_CalcSpeed();
        float spd_l = (float)Encoder_GetSpeed(ENCODER_LEFT)  * Motor_Distance_Per_Pulse * 0.05f;
        float spd_r = (float)Encoder_GetSpeed(ENCODER_RIGHT) * Motor_Distance_Per_Pulse * 0.05f;
        float car_speed = (spd_l + spd_r) / 2.0f;

    /* ---- 电机运动时跳过 OLED 刷新，避免刷新耗时影响控制时序 ---- */
    if (!RunFlag && !Joystick_Active)
    {
        /* ---- OLED显示（8号字体，固定宽度防残留） ---- */
        OLED_Printf(0, 0,  8, "Track:%s", RunFlag ? "ON " : "OFF");
        if (Joystick_Active)
            OLED_Printf(0, 0,  8, "JoyStick!  ");

        OLED_Printf(0, 8, 8, "TgtL:%-4d TgtR:%-4d", PWML, PWMR);    // 目标速度
        OLED_Printf(0, 16, 8, "ActL:%-4d ActR:%-4d",
            Encoder_GetSpeed(ENCODER_LEFT),
            Encoder_GetSpeed(ENCODER_RIGHT));    // 实际速度

        /* 第4行：串口1最后收到的按键值 */
        if (Serial_LastKey == 0)
            OLED_Printf(0, 24, 8, "U1: --               ");
        else
            OLED_Printf(0, 24, 8, "U1: KEY=%d           ", Serial_LastKey);

        /* 第5行：新加蓝牙参数 */
        OLED_Printf(0, 32, 8, "SST:%-3d TSlo:%.2f ", SoftStartTime, TurnSlowRatio);

        OLED_Printf(0, 40, 8, "Kp:%d Ki:%d Kd:%d",
            (int)Motor_Kp, (int)Motor_Ki, (int)Motor_Kd);// 速度环PID参数
        OLED_Printf(0, 48, 8, "TKp:%-5.1f TKd:%-5.1f",
            Track_Kp, Track_Kd);// 转向环PID参数
        OLED_Printf(0, 56, 8, "BSp:%-2d TP:%-2d WSc:%.1f",
            BaseSpeed, TurnPower, Gray_WeightScale);// 基础速度+转向功率+权重倍率
        OLED_Refresh();
    }

        //     /*蓝牙发送左右电机目标速度和实际速度（四参数）*/
        // BlueSerial_Printf("[plot,%d,%d,%d,%d]",
        //     (int16_t)motor_l_ctrl.target_speed,
        //     (int16_t)motor_r_ctrl.target_speed,
        //     Encoder_GetSpeed(ENCODER_LEFT),
        //     Encoder_GetSpeed(ENCODER_RIGHT));

        /* ---- 串口1发送左右电机目标速度/实际速度/整体速度 ---- */
        {
           Serial_Printf("%d,%d,%d,%d,%.2f\n",
               (int16_t)motor_l_ctrl.target_speed,
               (int16_t)motor_r_ctrl.target_speed,
               Encoder_GetSpeed(ENCODER_LEFT),
               Encoder_GetSpeed(ENCODER_RIGHT),
               car_speed);
        }  /* 串口输出结束 */

        // /*串口1发送运行状态：  0：IDLE  1：DECEL  2：ACTIVE*/
        // Serial_Printf("Turn_State:%d\n",
        //     Turn_State);

    }  /* while(1) 结束 */
}

#endif
