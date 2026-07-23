#if 0

/**
 * @file    Gimbal_Test.c
 * @brief   F32C 无刷云台电机 TTL 控制测试（蓝牙坐标控制）
 * @details 通过蓝牙接收 640×480 坐标数据包，换算成角度控制云台电机
 *
 * 控制模式：
 *   蓝牙坐标: 发送 0xAA XH XL YH YL 0xFF
 *   X: 误差 = (320-坐标), ±320 → ±100 (0.1°) = ±10.0°
 *   Y: 误差 = (240-坐标), ±240 → ±100 (0.1°) = ±10.0°
 *
 * 按键功能:
 *   Key2 - 双电机回零
 *   Key3 - M1 正转 10°, M2 正转 10°
 *   Key4 - M1 反转 10°, M2 反转 10°
 *
 * 硬件连接:
 *   F32C TTL TX -> PA9  (UART1 RX)
 *   F32C TTL RX -> PA8  (UART1 TX)
 *   F32C GND    -> GND
 *   蓝牙模块     -> PA10 (UART0 TX), PA11 (UART0 RX)
 *
 * 结果现象:
 *   - 上电后电机使能、进入位置闭环模式
 *   - Key2 双电机回零
 *   - OLED 显示 M1 Tgt / M1 Pos / M2 Tgt / M2 Pos / RX_X / RX_Y
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "key.h"
#include "Timer.h"
#include "F32C.h"
#include "Serial.h"
#include "BlueSerial.h"

extern volatile uint32_t uart1_isr_count;   /* 来自 Serial.c：UART1 ISR 接收计数 */

/* ==================== 最新收到的坐标 ==================== */
static uint16_t g_rx_x = 0;     /**< X 坐标 (0~640) */
static uint16_t g_rx_y = 0;     /**< Y 坐标 (0~480) */

/* 浮点累加器：保留小数部分跨帧积累，避免整数截断导致卡顿 */
static float motor1_accum = 0.0f;
static float motor2_accum = 0.0f;

/* ==================== 蓝牙 0xAA 坐标数据包解析 ==================== */

/**
 * @brief  从蓝牙串口(UART0)解析 0xAA 格式坐标数据包
 * @details 数据包格式: 0xAA XH XL YH YL 0xFF
 *          X = (XH<<8)|XL，映射到 motor1_target
 *          Y = (YH<<8)|YL，映射到 motor2_target
 *
 *          浮点累加增量积分：小数部分跨帧保留，小误差也能平滑推动电机
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
        case 0: /* 等待帧头 0xAA */
            if (byte == 0xAA)
            {
                rx_idx = 0;
                rx_state = 1;
            }
            break;

        case 1: /* 接收 4 个数据字节 */
            rx_buf[rx_idx++] = byte;
            if (rx_idx >= 4)
                rx_state = 2;
            break;

        case 2: /* 等待帧尾 0xFF */
            if (byte == 0xFF)
            {
                /* 解析坐标并计算误差值 */
                g_rx_x = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
                g_rx_y = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];

                int32_t err_x = (int32_t)COORD_CENTER_X - (int32_t)g_rx_x;
                int32_t err_y = (int32_t)COORD_CENTER_Y - (int32_t)g_rx_y;

                /* 浮点累加：err_x=5 时会加 5*60/640=0.47，跨帧累积到 1 */
                motor1_accum += (float)(-err_x) * ANGLE_RANGE / COORD_RANGE_X;
                motor2_accum += (float)( err_y) * ANGLE_RANGE / COORD_RANGE_Y;
                motor1_target = (int32_t)motor1_accum;
                motor2_target = (int32_t)motor2_accum;
            }
            else if (byte == 0xAA)
            {
                rx_idx = 0;
                rx_state = 1;
                break;
            }
            rx_state = 0;
            break;

        default:
            rx_state = 0;
            rx_idx = 0;
            break;
        }
    }
}

/* ==================== 回已保存的机械零点 ==================== */

/**
 * @brief  单圈模式回到已保存的 0°，停在机械零点
 * @note   先调试回零稳定性，再取消注释下方多圈切换代码
 */
static void Gimbal_Home(void)
{
    OLED_Printf(0, 0, 16, "Homing...");
    OLED_Refresh();

    /* 切单圈 → 设速度 → 发目标 0°，回到已保存的机械零点 */
    Send_Mode(MOTOR1_ID, MODE_POS_SINGLE);  delay_ms(1);
    Send_Mode(MOTOR2_ID, MODE_POS_SINGLE);  delay_ms(1);
    Send_Speed(MOTOR1_ID, 20);              delay_ms(1);
    Send_Speed(MOTOR2_ID, 20);              delay_ms(1);
    Send_Position_Single(MOTOR1_ID, 0);     delay_ms(1);
    Send_Position_Single(MOTOR2_ID, 0);     delay_ms(1);

    delay_ms(3000);  /* 等电机慢速转到机械零点 */

    /* ---- 参考工程方式：失能 → 配加速度/模式/位置 → 使能（避免带电切换异常） ---- */
    Send_Disable(MOTOR1_ID);              delay_us(250);
    Send_Disable(MOTOR2_ID);              delay_us(250);
    Send_Acc(MOTOR1_ID, 250);             delay_us(250);
    Send_Acc(MOTOR2_ID, 250);             delay_us(250);
    Send_Mode(MOTOR1_ID, MODE_POS);       delay_us(250);
    Send_Mode(MOTOR2_ID, MODE_POS);       delay_us(250);
    Send_Speed(MOTOR1_ID, 50);            delay_us(250);
    Send_Speed(MOTOR2_ID, 50);            delay_us(250);
    Send_MultiClear(MOTOR1_ID);           delay_us(250);
    Send_MultiClear(MOTOR2_ID);           delay_us(250);
    Send_Position(MOTOR1_ID, 0);          delay_us(250);
    Send_Position(MOTOR2_ID, 0);          delay_us(250);
    Send_Enable(MOTOR1_ID);               delay_ms(2);
    Send_Enable(MOTOR2_ID);               delay_ms(2);

    motor1_accum = 0.0f;
    motor2_accum = 0.0f;
    motor1_target = 0;
    motor2_target = 0;
    OLED_Clear();
}

int main(void)
{
    uint32_t tick_last = 0;
    uint8_t  oled_cnt = 0;
    uint8_t  key_val = 0;

    SYSCFG_DL_init();

    Timer_Init();
    OLED_Init();
    Serial_Init();

    /* UART1 RX 加上拉电阻，防止 F32C 电机 TX 空闲时浮空导致误接收 */
    // DL_GPIO_setDigitalInternalResistor(GPIO_UART_1_IOMUX_RX, DL_GPIO_RESISTOR_PULL_UP);

    BlueSerial_Init();

    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();

    /* 发送唤醒字节，等待电机串口总线稳定 */
    Serial_SendByte(0x00);
    delay_ms(1500);

    /* 电机使能 + 回机械零点 */
    Send_Enable(MOTOR1_ID);     delay_ms(2);
    Send_Enable(MOTOR2_ID);     delay_ms(2);
    Gimbal_Home();

    /* 上电初始化期间 UART0 缓冲区可能溢出积压脏数据，
       清空并丢弃，确保状态机从干净数据帧开始同步 */
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
            case 1:  /* Key1: 保存当前为机械零点（一次性校准） */
                OLED_Printf(0, 0, 16, "Set Zero...");
                OLED_Refresh();
                Send_Mode(MOTOR1_ID, MODE_POS_SINGLE);  delay_ms(5);
                Send_Mode(MOTOR2_ID, MODE_POS_SINGLE);  delay_ms(5);
                Send_SetZero(MOTOR1_ID);                delay_ms(5);
                Send_SetZero(MOTOR2_ID);                delay_ms(5);
                Send_Save(MOTOR1_ID);                   delay_ms(10);
                Send_Save(MOTOR2_ID);                   delay_ms(10);
                Send_Speed(MOTOR1_ID, 20);              delay_ms(5);
                Send_Speed(MOTOR2_ID, 20);              delay_ms(5);
                Send_Position_Single(MOTOR1_ID, 0);     delay_ms(5);
                Send_Position_Single(MOTOR2_ID, 0);     delay_ms(5);
                delay_ms(500);
                motor1_accum = 0.0f;
                motor2_accum = 0.0f;
                motor1_target = 0;
                motor2_target = 0;
                OLED_Clear();
                break;
            case 2:  /* Key2: 双电机回零 */
                motor1_target = 0;
                motor2_target = 0;
                motor1_accum  = 0.0f;
                motor2_accum  = 0.0f;
                break;
            case 3:  /* Key3: +10° */
                motor1_target = 100;
                motor2_target = 100;
                motor1_accum  = 100.0f;
                motor2_accum  = 100.0f;
                break;
            case 4:  /* Key4: -10° */
                motor1_target = -100;
                motor2_target = -100;
                motor1_accum  = -100.0f;
                motor2_accum  = -100.0f;
                break;
            default:
                break;
            }
        }

        /* ========== 蓝牙坐标数据解析 + 电机角度控制 ========== */
        Process_Bluetooth_Data();

        /* 每 20ms 发送电机指令 + 读取编码器反馈 */
        if (tick_now - tick_last >= 20)
        {
            tick_last = tick_now;

            /* 多圈模式：直接发送目标位置（无 0~3599 限制） */
            Send_Position(MOTOR1_ID, motor1_target);
            delay_ms(3);
            Send_Position(MOTOR2_ID, motor2_target);
            delay_ms(3);

            /* 读取 F32C 多圈绝对角度 */
            Send_FeedbackRequest(MOTOR1_ID, FB_MULTI_ANGLE);
            delay_ms(3);
            Send_FeedbackRequest(MOTOR2_ID, FB_MULTI_ANGLE);
            delay_ms(3);

            /* 轮询排空 UART1 RX FIFO 并解析 F32C 反馈帧 */
            F32C_PollFeedback();
        }

        /* OLED 显示 */
        oled_cnt++;
        if (oled_cnt >= 5)
        {
            oled_cnt = 0;
            /* 多圈角度直接显示（无归一化） */
            OLED_Printf(0, 0,  8, "M1T:%+05d M1P:%+05d",
                motor1_target / 10, motor1_current_position / 10);
            OLED_Printf(0, 8,  8, "M2T:%+05d M2P:%+05d",
                motor2_target / 10, motor2_current_position / 10);
            OLED_Printf(0, 16, 8, "RX_X:%-4d  RX_Y:%-4d  ", g_rx_x, g_rx_y);
            int32_t disp_err_x = (int32_t)COORD_CENTER_X - (int32_t)g_rx_x;
            int32_t disp_err_y = (int32_t)COORD_CENTER_Y - (int32_t)g_rx_y;
            OLED_Printf(0, 24, 8, "ERR_X:%+04d   ",(int)disp_err_x);
            OLED_Printf(0, 32, 8, "ERR_Y:%+04d   ",(int)disp_err_y);
            OLED_Refresh();
        }
    }
}

#endif
