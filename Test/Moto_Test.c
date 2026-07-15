#if 0


/**
 * @file    Moto_Test.c
 * @brief   AT8236 直流电机驱动测试程序
 * @details 通过四个按键分别控制左右电机的转速和转向。
 *          使用 OLED 实时显示左右电机当前 PWM 占空比。
 *
 * 测试对象：
 *   - AT8236 H 桥驱动（Hardware/Motor.c）
 *   - 直流减速电机 × 2
 *   - 按键控制（Key1~Key4）
 *   - PWM_A(TIMA1) / PWM_B(TIMG8) 硬件输出
 *
 * 按键功能：
 *   Key1  - 左电机加速 (+500)
 *   Key2  - 左电机减速 (-500)
 *   Key3  - 右电机加速 (+500)
 *   Key4  - 右电机减速 (-500)
 *   同时按住 Key1+Key3 或 Key2+Key4 可双电机同时加减速。
 *
 * 接线说明（以 SysConfig 为准）：
 *   左电机 AT8236：IN1=PB4 (GPIO 方向), IN2=PB1 (PWM_A)
 *   右电机 AT8236：IN1=PB15 (PWM_B), IN2=PB16 (GPIO 方向)
 *   电机供电：电池正极 → AT8236 VM, GND 共地
 *
 * 结果现象：
 *   - OLED 第一行显示 "L: +0500"（左电机 PWM=500/8000）
 *   - OLED 第二行显示 "R: +0500"（右电机 PWM=500/8000）
 *   - PWM 为正数时前进（慢衰减），负数时后退（快衰减）
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "key.h"
#include "Motor.h"
#include "Timer.h"
#include "Encoder.h"
#include "Serial.h"

int main(void)
{
    int16_t speed_l = 0;        /* 左电机 PWM 值 (-8000~8000) */
    int16_t speed_r = 0;        /* 右电机 PWM 值 (-8000~8000) */

    SYSCFG_DL_init();

    OLED_Init();
    Motor_Init();
    Timer_Init();
    Encoder_Init();

    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    Motor_ResetDistance();      /* 清零累加距离 */

    OLED_Printf(0, 0, 16, "Motor Test");
    OLED_Refresh();
    delay_ms(500);

    OLED_Clear();                   /* 清除 "Motor Test"，准备显示实时 PWM */

    while (1)
    {
        /* ========== 按键处理 ========== */
        uint8_t key_val = Key_GetNum();

        switch (key_val)
        {
        case 1: /* Key1: 左电机加速 (+500) */
            speed_l += 500;
            if (speed_l > MOTOR_PWM_MAX) speed_l = MOTOR_PWM_MAX;
            Motor_SetPWM(MOTOR_LEFT, speed_l);
            break;

        case 2: /* Key2: 左电机减速 (-500) */
            speed_l -= 500;
            if (speed_l < MOTOR_PWM_MIN) speed_l = MOTOR_PWM_MIN;
            Motor_SetPWM(MOTOR_LEFT, speed_l);
            break;

        case 3: /* Key3: 右电机加速 (+500) */
            speed_r += 500;
            if (speed_r > MOTOR_PWM_MAX) speed_r = MOTOR_PWM_MAX;
            Motor_SetPWM(MOTOR_RIGHT, speed_r);
            break;

        case 4: /* Key4: 右电机减速 (-500) */
            speed_r -= 500;
            if (speed_r < MOTOR_PWM_MIN) speed_r = MOTOR_PWM_MIN;
            Motor_SetPWM(MOTOR_RIGHT, speed_r);
            break;

        default:
            break;
        }

        /* ========== OLED 显示 ========== */
        /* 读取编码器 10ms 脉冲增量 */
        int16_t enc_l = Encoder_GetCount(ENCODER_LEFT);
        int16_t enc_r = Encoder_GetCount(ENCODER_RIGHT);

        /* 读取 GPIO 中断 10ms 增量（诊断用） */
        int16_t irq_a = Encoder_GetIRQCount(ENCODER_RIGHT);  /* GPIOA = 右编码器 */
        int16_t irq_b = Encoder_GetIRQCount(ENCODER_LEFT);   /* GPIOB = 左编码器 */

        /* 显示左/右电机的 PWM 和编码器脉冲数 */
        OLED_Printf(0, 0, 8, "L:%+05d E:%+05d", speed_l, enc_l);
        OLED_Printf(0, 8, 8, "R:%+05d E:%+05d", speed_r, enc_r);

        /* 显示 GPIO 中断触发计数（诊断用） */
        OLED_Printf(0, 16, 8, "IRQ_A:%+04d R:%+04d", irq_a, enc_r);
        OLED_Printf(0, 24, 8, "IRQ_B:%+04d L:%+04d", irq_b, enc_l);

        /* 累加距离并显示（用总脉冲差值，不丢帧） */
        {
            static int32_t prev_l = 0, prev_r = 0;
            int32_t total_l = Encoder_GetTotalCount(ENCODER_LEFT);
            int32_t total_r = Encoder_GetTotalCount(ENCODER_RIGHT);
            Motor_AddDistance((int16_t)(total_l - prev_l), (int16_t)(total_r - prev_r));
            prev_l = total_l;
            prev_r = total_r;
        }
        OLED_Printf(0, 32, 8, "Dist:%.0fmm  ", Motor_GetDistance());

        /* 计算并显示速度（m/s） */
        Encoder_CalcSpeed();
        float spd_l = (float)Encoder_GetSpeed(ENCODER_LEFT)  * Motor_Distance_Per_Pulse * 0.05f;
        float spd_r = (float)Encoder_GetSpeed(ENCODER_RIGHT) * Motor_Distance_Per_Pulse * 0.05f;
        float spd   = (spd_l + spd_r) / 2.0f;       /* 小车整体速度 */
        OLED_Printf(0, 40, 8, "Speed:%.2fm/s  ", spd);
        Serial_Printf("%.2f\n", spd);               /* 串口发送 */

        OLED_Refresh();


        // Serial_Printf("%d,%d\n",
        // Encoder_GetSpeed(ENCODER_LEFT),
        // Encoder_GetSpeed(ENCODER_RIGHT));
    }
}

#endif
