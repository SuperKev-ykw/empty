#if 0

/**
 * @file    Gimbal_Test_Vel.c
 * @brief   F32C 云台视觉 PD 追踪（速度模式）
 * @details 参考 WHEELTEC 2025 电赛 E 题控制算法：
 *          - K230 发送 0xAA XH XL YH YL 0xFF 坐标（0~640 × 0~480）
 *          - UART0(BlueSerial) 接收，环形缓冲区 + 状态机解析
 *          - PD 比例微分控制，死区防抖 + 丢帧超时保护
 *          - F32C 电机切换至速度模式（MODE_SPEED），输出 RPM
 *
 * 硬件连线（与 Gimbal_Test_Posi 完全一致）：
 *   F32C TTL TX -> PA9  (UART1 RX)
 *   F32C TTL RX -> PA8  (UART1 TX)
 *   K230 TX      -> PA11 (UART0 RX)
 *   K230 RX      -> PA10 (UART0 TX)
 *
 * 按键功能：
 *   Key2 - 双电机回零（PID 状态 + 速度清 0）
 *   Key3 - 强制正转（调试用）
 *   Key4 - 强制反转（调试用）
 *
 * OLED 显示（8 号字体）：
 *   行 0: YAW:%+05d PTC:%+05d   (电机 RPM)
 *   行 1: ERR_X:%+04d ERR_Y:%+04d (像素偏差)
 *   行 2: RX_X:%-4d RX_Y:%-4d   (原始坐标)
 *   行 3: Lost:%03dms            (丢帧计时)
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "key.h"
#include "Timer.h"
#include "F32C.h"
#include "Serial.h"
#include "BlueSerial.h"

/* ==================== PD 控制参数 ==================== */
#define KP_VISION_X     0.1f     /* 比例系数 X → YAW */
#define KD_VISION_X     0.02f    /* 微分系数 X → YAW */
#define KP_VISION_Y     0.25f    /* 比例系数 Y → PITCH */
#define KD_VISION_Y     0.02f    /* 微分系数 Y → PITCH */
#define VISION_DEADZONE 10.0f    /* 死区（像素），小于此值输出归零防抖 */
#define VISION_LOST_MS  800      /* 连续无帧超时（ms），清空 PD 并停车 */
#define SPEED_LIMIT_YAW   200    /* YAW 速度限幅（±RPM） */
#define SPEED_LIMIT_PITCH 200    /* PITCH 速度限幅（±RPM） */

/* ==================== 全局状态 ==================== */
static uint16_t g_rx_x = 0;          /**< 最新收到的 X 坐标 */
static uint16_t g_rx_y = 0;          /**< 最新收到的 Y 坐标 */
static volatile uint8_t g_frame_ready = 0;  /**< 1=有新帧待消费，PD 控制消费后清 0 */

/* PD 微分记忆 */
static float prev_err_x = 0.0f;
static float prev_err_y = 0.0f;

/* ==================== 工具函数 ==================== */
static float abs_float(float v)
{
    return v < 0.0f ? -v : v;
}

static float clamp_float(float v, float max_v, float min_v)
{
    if (v > max_v) return max_v;
    if (v < min_v) return min_v;
    return v;
}

/* ==================== K230 坐标解析 ==================== */

/**
 * @brief  从 BlueSerial（UART0）解析 0xAA 格式数据包
 * @details 协议格式: 0xAA XH XL YH YL 0xFF
 *          解析成功后仅设 g_frame_ready=1，不在此做任何控制运算
 */
static void Process_Bluetooth_Data(void)
{
    static uint8_t rx_state = 0;
    static uint8_t rx_buf[4];
    static uint8_t rx_idx = 0;

    while (BlueSerial_GetRxCount() > 0)
    {
        uint8_t byte = BlueSerial_GetRxData();

        switch (rx_state)
        {
        case 0:                     /* 等待帧头 0xAA */
            if (byte == 0xAA)
            {
                rx_idx = 0;
                rx_state = 1;
            }
            break;

        case 1:                     /* 接收 4 字节数据 */
            rx_buf[rx_idx++] = byte;
            if (rx_idx >= 4)
                rx_state = 2;
            break;

        case 2:                     /* 等待帧尾 0xFF */
            if (byte == 0xFF)
            {
                g_rx_x = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
                g_rx_y = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
                g_frame_ready = 1;                  /* 通知 PD 控制有新帧 */
            }
            else if (byte == 0xAA)                  /* 帧中意外出现帧头，重新同步 */
            {
                rx_idx = 0;
                rx_state = 1;
                break;
            }
            rx_state = 0;                           /* 无效字节，重新等待帧头 */
            break;

        default:
            rx_state = 0;
            rx_idx = 0;
            break;
        }
    }
}

/* ==================== PD 速度控制 ==================== */

/**
 * @brief  视觉 PD 控制器（每 20ms 调用一次）
 * @details 参考 WHEELTEC Visual_Gimbal_Process 的结构：
 *          1. 丢帧检测（超时清空 PD 状态 + 停车）
 *          2. 消费新帧，计算像素误差
 *          3. PD 公式计算 RPM 速度指令
 *          4. 死区 + 限幅 + 发送到 F32C
 */
static void Gimbal_PD_Control(void)
{
    static uint32_t lost_ms = 0;        /* 连续丢帧毫秒数 */
    float err_x, err_y;
    float diff_err_x, diff_err_y;
    float cmd_yaw, cmd_pitch;
    int16_t rpm_yaw, rpm_pitch;

    /* ==================== 丢帧检测 ==================== */
    if (!g_frame_ready)
    {
        if (lost_ms < 0xFFFF)
            lost_ms += 20;              /* 本函数每 20ms 调用一次 */

        if (lost_ms >= VISION_LOST_MS)
        {
            /* 超时：清 PD 历史、停车 */
            prev_err_x = 0.0f;
            prev_err_y = 0.0f;
            Send_Speed(MOTOR1_ID, 0);
            Send_Speed(MOTOR2_ID, 0);
        }
        return;                         /* 无新帧，不执行 PD */
    }

    /* ==================== 消费新帧 ==================== */
    g_frame_ready = 0;
    lost_ms = 0;

    /* ==================== 误差计算 ==================== */
    /* err_x: 正=偏左，负=偏右；err_y: 正=偏上，负=偏下 */
    err_x = (float)(COORD_CENTER_X - (int32_t)g_rx_x);
    err_y = (float)(COORD_CENTER_Y - (int32_t)g_rx_y);

    /* ==================== PD 微分 ==================== */
    diff_err_x = err_x - prev_err_x;
    diff_err_y = err_y - prev_err_y;
    prev_err_x = err_x;
    prev_err_y = err_y;

    /* ==================== PD 公式 ==================== */
    cmd_yaw   = KP_VISION_X * err_x + KD_VISION_X * diff_err_x;
    cmd_pitch = KP_VISION_Y * err_y + KD_VISION_Y * diff_err_y;

    /* ==================== 死区 ==================== */
    if (abs_float(err_x) <= VISION_DEADZONE) cmd_yaw   = 0.0f;
    if (abs_float(err_y) <= VISION_DEADZONE) cmd_pitch = 0.0f;

    /* ==================== 限幅 ==================== */
    cmd_yaw   = clamp_float(cmd_yaw,   SPEED_LIMIT_YAW,   -SPEED_LIMIT_YAW);
    cmd_pitch = clamp_float(cmd_pitch, SPEED_LIMIT_PITCH, -SPEED_LIMIT_PITCH);

    /* ==================== 发送速度指令 ==================== */
    /* YAW (M1)：负号修正安装方向，使 err_x>0(偏左) → 左转（负RPM） */
    rpm_yaw   = (int16_t)(-cmd_yaw);
    rpm_pitch = (int16_t)( cmd_pitch);
    Send_Speed(MOTOR1_ID, rpm_yaw);
    Send_Speed(MOTOR2_ID, rpm_pitch);
}

/* ==================== 主函数 ==================== */

int main(void)
{
    uint32_t tick_last = 0;
    uint8_t  oled_cnt = 0;
    uint8_t  key_val = 0;

    SYSCFG_DL_init();

    Timer_Init();
    OLED_Init();
    Serial_Init();
    BlueSerial_Init();

    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    /* 发送唤醒字节，等待电机串口总线稳定 */
    Serial_SendByte(0x00);
    delay_ms(1500);

    /* ---- F32C 电机初始化（速度模式） ---- */
    Send_Enable(MOTOR1_ID);     delay_ms(5);
    Send_Enable(MOTOR2_ID);     delay_ms(5);
    Send_Mode(MOTOR1_ID, MODE_SPEED);  delay_ms(5);
    Send_Mode(MOTOR2_ID, MODE_SPEED);  delay_ms(5);
    Send_Speed(MOTOR1_ID, 0);          delay_ms(5);  /* 初始速度为 0 */
    Send_Speed(MOTOR2_ID, 0);          delay_ms(5);

    /* 清空上电期间积压的 K230 脏数据 */
    while (BlueSerial_GetRxCount() > 0)
        BlueSerial_GetRxData();

    while (1)
    {
        uint32_t tick_now = (uint32_t)Count1 * 1000 + Count0;

        /* ========== 按键处理 ========== */
        key_val = Key_GetNum();
        if (key_val != 0)
        {
            switch (key_val)
            {
            case 2:  /* Key2: 双电机归零停车 */
                prev_err_x = 0.0f;
                prev_err_y = 0.0f;
                g_frame_ready = 0;
                Send_Speed(MOTOR1_ID, 0);
                Send_Speed(MOTOR2_ID, 0);
                break;
            case 3:  /* Key3: 强制正转（调试） */
                Send_Speed(MOTOR1_ID,  50);
                Send_Speed(MOTOR2_ID,  50);
                delay_ms(100);
                Send_Speed(MOTOR1_ID, 0);
                Send_Speed(MOTOR2_ID, 0);
                break;
            case 4:  /* Key4: 强制反转（调试） */
                Send_Speed(MOTOR1_ID, -50);
                Send_Speed(MOTOR2_ID, -50);
                delay_ms(100);
                Send_Speed(MOTOR1_ID, 0);
                Send_Speed(MOTOR2_ID, 0);
                break;
            default:
                break;
            }
        }

        /* ========== K230 数据解析 ========== */
        Process_Bluetooth_Data();

        /* ========== 每 20ms：PD 控制 + 编码器反馈 ========== */
        if (tick_now - tick_last >= 20)
        {
            tick_last = tick_now;

            Gimbal_PD_Control();

            /* 可选：读取电机反馈用于显示 */
            Send_FeedbackRequest(MOTOR1_ID, FB_SPEED);
            delay_ms(1);
            Send_FeedbackRequest(MOTOR2_ID, FB_SPEED);
            delay_ms(1);
            F32C_PollFeedback();
        }

        /* ========== OLED 显示（约 100ms 刷新一次） ========== */
        oled_cnt++;
        if (oled_cnt >= 5)
        {
            oled_cnt = 0;

            int32_t disp_err_x = (int32_t)COORD_CENTER_X - (int32_t)g_rx_x;
            int32_t disp_err_y = (int32_t)COORD_CENTER_Y - (int32_t)g_rx_y;

            OLED_Printf(0, 0,  8, "YAW:%+05d PTC:%+05d   ",
                motor1_current_speed, motor2_current_speed);
            OLED_Printf(0, 8,  8, "ERR_X:%+04d ERR_Y:%+04d   ",
                (int)disp_err_x, (int)disp_err_y);
            OLED_Printf(0, 16, 8, "RX_X:%-4d RX_Y:%-4d   ",
                g_rx_x, g_rx_y);

            /* 丢帧计时显示 */
            OLED_Printf(0, 24, 8, "Lost:%03dms            ",
                g_frame_ready ? 0 : 999);

            OLED_Refresh();
        }
    }
}

#endif
