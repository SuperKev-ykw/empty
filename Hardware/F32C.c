

/**
 * @file    F32C.c
 * @brief   F32C 无刷云台电机 TTL 协议驱动
 * @details 通过 UART1 (PA8/PA9, 115200) 控制 F32C 无刷电机
 *
 * 协议格式:
 *   帧头 0x7A + ID + 命令 + 数据 + BCC(XOR) + 帧尾 0x7B
 *
 * 命令列表:
 *   使能      : 0x7A ID 0x06            BCC 0x7B  (5 字节)
 *   设置模式  : 0x7A ID 0x00 ModeH ModeL BCC 0x7B  (7 字节, Mode=1=位置闭环)
 *   设置速度  : 0x7A ID 0x01 SpdH SpdL   BCC 0x7B  (7 字节)
 *   设置位置  : 0x7A ID 0x02 P3 P2 P1 P0 BCC 0x7B  (9 字节, 单位=0.1度)
 *   请求反馈  : 0x7A ID 0x0E 0x01       BCC 0x7B  (6 字节)
 *
 * 反馈格式 (9 字节):
 *   0x7A ID 0x01 P3 P2 P1 P0 BCC 0x7B  (位置反馈)
 *
 * 硬件连接：
 *   F32C TTL TX -> PA9  (UART1 RX)
 *   F32C TTL RX -> PA8  (UART1 TX)
 *   F32C GND    -> GND
 *
 * 函数清单：
 *   - BCC_Sum()             : 计算 BCC 校验和（XOR，内部）
 *   - Build_Enable()        : 构造使能命令（内部）
 *   - Build_Mode()          : 构造设置模式命令（内部）
 *   - Build_Speed()         : 构造设置速度命令（内部）
 *   - Build_Position()      : 构造设置位置命令（内部）
 *   - Build_Feedback()      : 构造请求反馈命令（内部）
 *   - Send_Enable()         : 发送使能命令
 *   - Send_Mode()           : 发送设置模式命令
 *   - Send_Speed()          : 发送设置速度命令
 *   - Send_Position()       : 发送设置位置命令
 *   - Send_Feedback()       : 发送请求反馈命令
 *   - Parse_Feedback()      : 解析电机位置反馈（状态机方式）
 *
 * 全局变量：
 *   - motor1_pos / motor2_pos     : 电机当前位置（单位 0.1度）
 *   - motor1_target / motor2_target : 电机目标位置
 *   - motor1_online / motor2_online : 电机在线标志
 *
 * 使用方式：
 *   1. 调用 Send_Enable(id) 使能电机
 *   2. 调用 Send_Mode(id, MODE_POS) 设置为位置闭环模式
 *   3. 调用 Send_Speed(id, speed) 设置运动速度
 *   4. 调用 Send_Position(id, position) 设置目标位置
 *   5. 调用 Send_Feedback(id) 请求位置反馈
 *   6. 周期性调用 Parse_Feedback() 解析反馈更新 motor*_pos
 */

#include "F32C.h"
#include "Serial.h"

/* ==================== 全局变量定义 ==================== */
volatile int32_t motor1_pos = 0;
volatile int32_t motor2_pos = 0;
int32_t motor1_target = 0;
int32_t motor2_target = 0;
uint8_t motor1_online = 0;
uint8_t motor2_online = 0;

/* ==================== 命令缓冲区 ==================== */
static uint8_t cmd_enable[5];
static uint8_t cmd_mode[7];
static uint8_t cmd_speed[7];
static uint8_t cmd_position[9];
static uint8_t cmd_feedback[6];

/* ==================== BCC 校验 ==================== */

static uint8_t BCC_Sum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
        sum ^= data[i];
    return sum;
}

/* ==================== 命令构建 (内部) ==================== */

static void Build_Enable(uint8_t id)
{
    cmd_enable[0] = 0x7A;
    cmd_enable[1] = id;
    cmd_enable[2] = 0x06;
    cmd_enable[3] = BCC_Sum(cmd_enable, 3);
    cmd_enable[4] = 0x7B;
}

static void Build_Mode(uint8_t id, uint16_t mode)
{
    cmd_mode[0] = 0x7A;
    cmd_mode[1] = id;
    cmd_mode[2] = 0x00;
    cmd_mode[3] = (uint8_t)(mode >> 8);
    cmd_mode[4] = (uint8_t)(mode);
    cmd_mode[5] = BCC_Sum(cmd_mode, 5);
    cmd_mode[6] = 0x7B;
}

static void Build_Speed(uint8_t id, int16_t speed)
{
    cmd_speed[0] = 0x7A;
    cmd_speed[1] = id;
    cmd_speed[2] = 0x01;
    cmd_speed[3] = (uint8_t)((uint16_t)speed >> 8);
    cmd_speed[4] = (uint8_t)(speed);
    cmd_speed[5] = BCC_Sum(cmd_speed, 5);
    cmd_speed[6] = 0x7B;
}

static void Build_Position(uint8_t id, int32_t position)
{
    cmd_position[0] = 0x7A;
    cmd_position[1] = id;
    cmd_position[2] = 0x02;
    cmd_position[3] = (uint8_t)((uint32_t)position >> 24);
    cmd_position[4] = (uint8_t)((uint32_t)position >> 16);
    cmd_position[5] = (uint8_t)((uint32_t)position >> 8);
    cmd_position[6] = (uint8_t)(position);
    cmd_position[7] = BCC_Sum(cmd_position, 7);
    cmd_position[8] = 0x7B;
}

static void Build_Feedback(uint8_t id)
{
    cmd_feedback[0] = 0x7A;
    cmd_feedback[1] = id;
    cmd_feedback[2] = 0x0E;
    cmd_feedback[3] = 0x01;
    cmd_feedback[4] = BCC_Sum(cmd_feedback, 4);
    cmd_feedback[5] = 0x7B;
}

/* ==================== 发送封装 (公开 API) ==================== */

void Send_Enable(uint8_t id)
{
    Build_Enable(id);
    Serial_SendArray(cmd_enable, 5);
}

void Send_Mode(uint8_t id, uint16_t mode)
{
    Build_Mode(id, mode);
    Serial_SendArray(cmd_mode, 7);
}

void Send_Speed(uint8_t id, int16_t speed)
{
    Build_Speed(id, speed);
    Serial_SendArray(cmd_speed, 7);
}

void Send_Position(uint8_t id, int32_t position)
{
    Build_Position(id, position);
    Serial_SendArray(cmd_position, 9);
}

void Send_Feedback(uint8_t id)
{
    Build_Feedback(id);
    Serial_SendArray(cmd_feedback, 6);
}

/* ==================== 反馈解析 (公开 API) ==================== */

void Parse_Feedback(void)
{
    static uint8_t rx_state = 0;
    static uint8_t rx_buf[9];
    static uint8_t rx_idx = 0;

    while (Serial_GetRxCount() > 0)
    {
        uint8_t byte = Serial_GetRxData();

        switch (rx_state)
        {
        case 0: /* 等待帧头 0x7A */
            if (byte == 0x7A)
            {
                rx_buf[0] = byte;
                rx_idx = 1;
                rx_state = 1;
            }
            break;

        case 1: /* 接收剩余 8 字节 */
            rx_buf[rx_idx++] = byte;
            if (rx_idx >= 9)
                rx_state = 2;
            break;

        case 2: /* 校验帧尾和 BCC */
            rx_state = 0;
            if (rx_buf[8] != 0x7B)
                break;
            if (rx_buf[7] != BCC_Sum(rx_buf, 7))
                break;

            /* 解析位置反馈 (类型 0x01) */
            if (rx_buf[2] == 0x01)
            {
                int32_t pos = ((int32_t)rx_buf[3] << 24) |
                              ((int32_t)rx_buf[4] << 16) |
                              ((int32_t)rx_buf[5] << 8)  |
                               (int32_t)rx_buf[6];

                if (rx_buf[1] == MOTOR1_ID)
                {
                    motor1_pos = pos;
                    motor1_online = 1;
                }
                else if (rx_buf[1] == MOTOR2_ID)
                {
                    motor2_pos = pos;
                    motor2_online = 1;
                }
            }
            break;
        }
    }
}
