/**
 * @file    Gimbal_Test.c
 * @brief   F32C 无刷云台电机 TTL 控制测试
 * @details 蓝牙串口接收 0xAA 格式坐标数据 → 直接映射为电机角度 → UART1 发送
 *
 * 数据格式: 0xAA XH XL YH YL 0xFF
 * 映射关系: 误差=(屏幕中心-坐标), 误差±224→角度±180° (单位 0.1°)
 *
 * 按键控制:
 *   Key1: 电机1 位置 +100 度
 *   Key2: 电机1 位置 -100 度
 *   Key3: 电机2 位置 +100 度
 *   Key4: 双电机回零
 *
 * 硬件连接:
 *   F32C TTL TX -> PA9  (UART1 RX)
 *   F32C TTL RX -> PA8  (UART1 TX)
 *   F32C GND    -> GND
 *   蓝牙模块     -> PA10 (UART0 TX), PA11 (UART0 RX)
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "key.h"
#include "Timer.h"
#include "F32C.h"
#include "Serial.h"
#include "BlueSerial.h"

/* ==================== 蓝牙数据包解析 ==================== */

/**
 * @brief  从蓝牙串口(UART0)解析 0xAA 格式坐标数据包
 * @details 数据包格式: 0xAA XH XL YH YL 0xFF
 *          X = (XH<<8)|XL，映射到 motor1_target (-1800~1800)
 *          Y = (YH<<8)|YL，映射到 motor2_target (-1800~1800)
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
                /* 解析坐标并计算误差值 → 映射到电机角度 */
                uint16_t X_Data = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
                uint16_t Y_Data = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];

                /* 误差 = 坐标中心 - 实际坐标 (坐标范围 0~448, 中心 224)
                 * 误差 ±224 → 角度 ±180° (单位 0.1°) */
                int32_t err_x = (int32_t)COORD_CENTER - (int32_t)X_Data;
                int32_t err_y = (int32_t)COORD_CENTER - (int32_t)Y_Data;
                motor1_target = err_x * ANGLE_RANGE / COORD_RANGE;
                motor2_target = err_y * ANGLE_RANGE / COORD_RANGE;
            }
            else if (byte == 0xAA)
            {
                /* 帧尾错误但遇到新帧头，重新同步 */
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

int main(void)
{
    uint32_t tick_last = 0;
    uint8_t  oled_cnt = 0;
    uint8_t  key_val = 0;

    SYSCFG_DL_init();

    NVIC->ICER[0] = 0xFFFFFFFFUL;

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

    /* 电机上电初始化 */
    Send_Enable(MOTOR1_ID);
    delay_ms(5);
    Send_Enable(MOTOR2_ID);
    delay_ms(5);
    Send_Mode(MOTOR1_ID, MODE_POS);
    delay_ms(5);
    Send_Mode(MOTOR2_ID, MODE_POS);
    delay_ms(5);
    Send_Speed(MOTOR1_ID, 5000);
    delay_ms(5);
    Send_Speed(MOTOR2_ID, 5000);
    delay_ms(5);

    /* 确保 motor2 也使能成功，再发一次 */
    Send_Enable(MOTOR2_ID);
    delay_ms(2);
    Send_Mode(MOTOR2_ID, MODE_POS);
    delay_ms(2);
    Send_Speed(MOTOR2_ID, 5000);
    delay_ms(5);

    while (1)
    {
        uint32_t tick_now = (uint32_t)Count1 * 1000 + Count0;

        /* 蓝牙坐标数据解析 → 直接映射到电机角度 */
        Process_Bluetooth_Data();

        /* 按键检测 */
        key_val = Key_GetNum();
        if (key_val != 0)
        {
            switch (key_val)
            {
            case 1:
                motor1_target += POS_STEP;
                break;
            case 2:
                motor1_target -= POS_STEP;
                break;
            case 3:
                motor2_target += POS_STEP;
                break;
            case 4:
                motor1_target = 0;
                motor2_target = 0;
                break;
            default:
                break;
            }
        }

        /* 每 20ms 发送一次电机指令 */
        if (tick_now - tick_last >= 20)
        {
            tick_last = tick_now;

            Send_Position(MOTOR1_ID, motor1_target);
            delay_ms(3);
            Send_Position(MOTOR2_ID, motor2_target);
            delay_ms(3);
            Send_Feedback(MOTOR1_ID);
            delay_ms(3);
            Send_Feedback(MOTOR2_ID);
            delay_ms(3);

            Parse_Feedback();
        }

        /* OLED 显示 (每 5 帧刷新一次) */
        oled_cnt++;
        if (oled_cnt >= 5)
        {
            oled_cnt = 0;
            OLED_Printf(0, 0, 16, "M1 Tgt:%+05d", motor1_target / 10);
            OLED_Printf(0, 16, 16, "M1 Pos:%+05d", motor1_pos / 10);
            OLED_Printf(0, 32, 16, "M2 Tgt:%+05d", motor2_target / 10);
            OLED_Printf(0, 48, 16, "M2 Pos:%+05d", motor2_pos / 10);
            OLED_Refresh();
        }
    }
}
