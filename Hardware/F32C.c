
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
 *   - Send_Enable()         : 发送使能命令
 *   - Send_Mode()           : 发送设置模式命令
 *   - Send_Speed()          : 发送设置速度命令
 *   - Send_Position()       : 发送设置位置命令
 *
 * 全局变量：
 *   - motor1_target / motor2_target : 电机目标位置
 *
 * 使用方式：
 *   1. 调用 Send_Enable(id) 使能电机
 *   2. 调用 Send_Mode(id, MODE_POS) 设置为位置闭环模式
 *   3. 调用 Send_Speed(id, speed) 设置运动速度
 *   4. 调用 Send_Position(id, position) 设置目标位置
 */

#include "ti_msp_dl_config.h"
#include "F32C.h"
#include "Serial.h"

/* ==================== 全局变量定义 ==================== */
int32_t motor1_target = 0;
int32_t motor2_target = 0;

/* 当前编码器反馈值（角度 & 速度） */
volatile int32_t motor1_current_position = 0;
volatile int32_t motor2_current_position = 0;
volatile int32_t motor1_current_speed = 0;
volatile int32_t motor2_current_speed = 0;

/* ==================== 命令缓冲区 ==================== */
static uint8_t cmd_enable[5];
static uint8_t cmd_mode[7];
static uint8_t cmd_speed[7];
static uint8_t cmd_position[9];
static uint8_t cmd_feedback[6];     /* 反馈读取请求：0x7A ID 0x0E type BCC 0x7B */

/* ==================== BCC 校验 ==================== */

static uint8_t BCC_Sum(uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
        sum ^= data[i];
    return sum;
}

/* ==================== 命令构建 (内部) ==================== */

static void Build_Enable(uint8_t id)//使能电机
{
    cmd_enable[0] = 0x7A;
    cmd_enable[1] = id;
    cmd_enable[2] = 0x06;
    cmd_enable[3] = BCC_Sum(cmd_enable, 3);
    cmd_enable[4] = 0x7B;
}

static void Build_Mode(uint8_t id, uint16_t mode)//设置模式
{
    cmd_mode[0] = 0x7A;
    cmd_mode[1] = id;
    cmd_mode[2] = 0x00;
    cmd_mode[3] = (uint8_t)(mode >> 8);
    cmd_mode[4] = (uint8_t)(mode);
    cmd_mode[5] = BCC_Sum(cmd_mode, 5);
    cmd_mode[6] = 0x7B;
}

static void Build_Speed(uint8_t id, int16_t speed)//设置速度
{
    cmd_speed[0] = 0x7A;
    cmd_speed[1] = id;
    cmd_speed[2] = 0x01;
    cmd_speed[3] = (uint8_t)((uint16_t)speed >> 8);
    cmd_speed[4] = (uint8_t)(speed);
    cmd_speed[5] = BCC_Sum(cmd_speed, 5);
    cmd_speed[6] = 0x7B;
}

static void Build_Position(uint8_t id, int32_t position)//设置多圈绝对控制角度
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

/* ==================== 反馈请求 ==================== */

static void Build_Feedback(uint8_t id, uint8_t type)
{
    cmd_feedback[0] = 0x7A;
    cmd_feedback[1] = id;
    cmd_feedback[2] = CMD_FEEDBACK;
    cmd_feedback[3] = type;
    cmd_feedback[4] = BCC_Sum(cmd_feedback, 4);
    cmd_feedback[5] = 0x7B;
}

void Send_FeedbackRequest(uint8_t id, uint8_t type)
{
    Build_Feedback(id, type);
    Serial_SendArray(cmd_feedback, 6);
}

/* ==================== F32C 反馈帧解析器 ==================== */

/**
 * @brief  主循环轮询解析 F32C 反馈帧（不依赖中断）
 * @note   排空 UART1 RX FIFO，查找并解析 9 字节反馈帧：
 *           0x7A ADDR TYPE DATA4 BCC 0x7B
 *         解析结果更新 motor{1,2}_{current_position,current_speed}
 */
void F32C_PollFeedback(void)
{
    static uint8_t rx_buf[9];
    static uint8_t rx_idx = 0;

    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST))
    {
        uint8_t byte = DL_UART_Main_receiveData(UART_1_INST);

        /* 等待帧头 0x7A */
        if (rx_idx == 0)
        {
            if (byte != 0x7A)
                continue;
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

        /* 1) 校验帧尾 */
        if (rx_buf[8] != 0x7B)
            continue;

        /* 2) BCC 校验：前 7 字节 XOR */
        uint8_t bcc = 0;
        for (uint8_t i = 0; i < 7; i++)
            bcc ^= rx_buf[i];
        if (bcc != rx_buf[7])
            continue;

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
    }
}

/**
 * @brief  F32C 反馈帧解析（空函数，保留防链接丢失）
 */
void F32C_ParseFeedback(void)
{
    /* 已由 F32C_PollFeedback() 轮询解析 */
}
