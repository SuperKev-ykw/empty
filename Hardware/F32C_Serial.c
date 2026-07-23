/**
 * @file    F32C_Serial.c
 * @brief   F32C 电机串口帧解析实现
 * @details 从 Serial 的环形缓冲区中轮询解析 F32C 9 字节反馈帧：
 *            0x7A ADDR TYPE DATA4 BCC 0x7B
 *          ADDR=1/2 → TYPE=0x01(角度) / 0x00(速度)
 *          解析结果直接更新 F32C.h 中的全局变量。
 *
 *          原代码位于 Serial.c 的 UART1_IRQHandler 中，
 *          现提取为独立模块，通过主循环轮询方式解析。
 *
 * 依赖：
 *   - Serial.c : Serial_GetRxCount(), Serial_GetRxData()
 *   - F32C.c   : motor{1,2}_{current_position,current_speed}
 *   - F32C.h   : MOTOR1_ID, MOTOR2_ID, FB_MULTI_ANGLE, FB_SPEED
 */

#include "ti_msp_dl_config.h"
#include "Serial.h"             /* Serial_GetRxCount(), Serial_GetRxData() */
#include "F32C.h"               /* motor1/2_current_position/speed, MOTOR1/2_ID, FB_* */
#include "F32C_Serial.h"

/* ==================== F32C 帧解析调试计数 ==================== */
volatile uint32_t f32c_frame_header = 0;
volatile uint32_t f32c_frame_full   = 0;
volatile uint32_t f32c_tail_err     = 0;
volatile uint32_t f32c_bcc_err      = 0;
volatile uint32_t f32c_frame_ok     = 0;

/* ==================== 帧解析（主循环轮询） ==================== */

/**
 * @brief  从 Serial 环形缓冲区轮询解析 F32C 9 字节反馈帧
 * @note   状态机自动同步帧头，与 Serial_GetKeyFrame() 同模式
 */
uint8_t F32C_Serial_ParseFrame(void)
{
    static uint8_t rx_buf[9];
    static uint8_t rx_idx = 0;

    while (Serial_GetRxCount() > 0)
    {
        uint8_t byte = Serial_GetRxData();

        /* 等待帧头 0x7A */
        if (rx_idx == 0)
        {
            if (byte != 0x7A)
                continue;
            f32c_frame_header++;
            rx_buf[0] = byte;
            rx_idx = 1;
            continue;
        }

        /* 收集第 2~9 字节 */
        rx_buf[rx_idx++] = byte;

        if (rx_idx < 9)
            continue;

        /* ──── 已收满 9 字节，复位索引 ──── */
        rx_idx = 0;
        f32c_frame_full++;

        /* 1) 校验帧尾 */
        if (rx_buf[8] != 0x7B)
        {
            f32c_tail_err++;
            continue;
        }

        /* 2) BCC 校验：前 7 字节 XOR */
        uint8_t bcc = 0;
        for (uint8_t i = 0; i < 7; i++)
            bcc ^= rx_buf[i];
        if (bcc != rx_buf[7])
        {
            f32c_bcc_err++;
            continue;
        }

        /* 3) 提取 4 字节大端有符号整数 */
        int32_t value = (int32_t)(((uint32_t)rx_buf[3] << 24) |
                                  ((uint32_t)rx_buf[4] << 16) |
                                  ((uint32_t)rx_buf[5] << 8)  |
                                  ((uint32_t)rx_buf[6]));

        uint8_t addr = rx_buf[1];
        uint8_t type = rx_buf[2];

        /* 4) 按地址和类型更新全局变量 */
        if (addr == MOTOR1_ID)
        {
            if (type == FB_MULTI_ANGLE)
                motor1_current_position = value;
            else if (type == FB_SPEED)
                motor1_current_speed = value;
        }
        else if (addr == MOTOR2_ID)
        {
            if (type == FB_MULTI_ANGLE)
                motor2_current_position = value;
            else if (type == FB_SPEED)
                motor2_current_speed = value;
        }
        f32c_frame_ok++;

        return 1;   /* 成功解析一帧 */
    }

    return 0;       /* 缓冲区无有效帧 */
}
