/**
 * @file    Serial.c
 * @brief   串口通信驱动实现文件
 * @details 基于 MSPM0G3507 + SysConfig 配置的 UART 驱动
 *
 * 本驱动基于 STM32 标准库设计思路，适配 MSPM0 平台：
 *   - 外设和 GPIO 初始化由 SysConfig 完成（无需手动配置时钟和引脚）
 *   - 使用 MSPM0 DriverLib API（DL_UART_xxx）
 *   - 接收采用环形缓冲区 + 中断方式
 *
 * 硬件配置（SysConfig）：
 *   UART1：PA8(TX), PA9(RX), 115200-8N1
 *
 * 函数清单：
 *   - RingBuffer_Write()    : 环形缓冲区写入（内部）
 *   - RingBuffer_Read()     : 环形缓冲区读出（内部）
 *   - RingBuffer_GetCount() : 获取缓冲区数据量（内部）
 *   - Serial_Init()         : 初始化（使能 NVIC 中断）
 *   - Serial_SendByte()     : 发送单个字节
 *   - Serial_SendArray()    : 发送字节数组
 *   - Serial_SendString()   : 发送字符串
 *   - Serial_SendNumber()   : 按指定位数发送数字
 *   - Serial_Printf()       : 格式化输出（类似 printf）
 *   - Serial_GetRxCount()   : 获取接收缓冲区数据量
 *   - Serial_GetRxData()    : 从接收缓冲区读一个字节
 *   - UART_1_INST_IRQHandler(): UART1 接收中断服务函数
 *
 * 使用方式：
 *   1. 初始化：Serial_Init()
 *   2. 发送：Serial_Printf() / Serial_SendString() / ...
 *   3. 接收：在主循环中调用 Serial_GetRxCount() 检测，
 *           再用 Serial_GetRxData() 逐字节读取并自行解析协议
 *
 * 注意：与 BlueSerial（UART0）共用相同的 API 风格，
 *       仅硬件实例（UART_0_INST vs UART_1_INST）不同。
 */

#include "ti_msp_dl_config.h"
#include "Serial.h"
#include "F32C.h"               /* 用于 ISR 中直接解析 F32C 反馈帧 */

/* ==================== 环形缓冲区实现 ==================== */

/** @brief 环形缓冲区结构体 */
typedef struct {
    uint8_t buffer[RX_BUFFER_SIZE];
    volatile uint16_t head;   /**< 写指针（中断中写入） */
    volatile uint16_t tail;   /**< 读指针（主循环读取） */
    volatile uint16_t count;  /**< 缓冲区中数据量 */
} RingBuffer;

/** @brief 接收环形缓冲区实例 */
static RingBuffer rxBuffer = {
    .head = 0,
    .tail = 0,
    .count = 0
};

/* ---- ISR 帧解析调试计数 ---- */
volatile uint32_t uart1_isr_count = 0;      /* UART1 ISR 进入次数（供 OLED 显示） */
volatile uint32_t dbg_rx_total = 0;
volatile uint32_t dbg_frame_header = 0;
volatile uint32_t dbg_frame_full = 0;
volatile uint32_t dbg_tail_err = 0;
volatile uint32_t dbg_bcc_err = 0;
volatile uint32_t dbg_frame_ok = 0;

/**
 * @brief  向环形缓冲区写入一个字节（在中断中调用）
 * @param  data 要写入的字节
 */
static void RingBuffer_Write(RingBuffer *rb, uint8_t data)
{
    if (rb->count < RX_BUFFER_SIZE) {
        rb->buffer[rb->head] = data;
        rb->head = (rb->head + 1) % RX_BUFFER_SIZE;
        rb->count++;
    } else {
        /* 缓冲区满时覆盖最旧的数据 */
        rb->buffer[rb->head] = data;
        rb->head = (rb->head + 1) % RX_BUFFER_SIZE;
        rb->tail = (rb->tail + 1) % RX_BUFFER_SIZE;
    }
}

/**
 * @brief  从环形缓冲区读取一个字节
 * @return 读取到的字节（缓冲区为空时返回 0）
 */
static uint8_t RingBuffer_Read(RingBuffer *rb)
{
    if (rb->count > 0) {
        uint8_t data = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % RX_BUFFER_SIZE;
        rb->count--;
        return data;
    }
    return 0;
}

/**
 * @brief  获取环形缓冲区中的数据量
 */
static uint16_t RingBuffer_GetCount(RingBuffer *rb)
{
    return rb->count;
}

/* ==================== 串口 API 实现 ==================== */

/**
 * @brief  串口初始化
 * @note   SysConfig 已完成外设初始化，此函数仅使能 NVIC 中断
 */
void Serial_Init(void)
{
    /* 参考工程：SysConfig 设置优先级0，Serial_Init 只清pending + 使能NVIC，不改优先级 */
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);

    /* 设置RX FIFO阈值为1字节，确保每收到一字节都触发中断（参考工程有此设置） */
    DL_UART_setRXFIFOThreshold(UART_1_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);

    /* 不调用 NVIC_SetPriority，保持 SysConfig 设定的优先级 0（与参考工程一致） */
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/**
 * @brief  串口发送单个字节（阻塞）
 */
void Serial_SendByte(uint8_t Byte)
{
    while (DL_UART_isTXFIFOFull(UART_1_INST));
    DL_UART_transmitData(UART_1_INST, Byte);
}

/**
 * @brief  串口发送数组
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte(Array[i]);
    }
}

/**
 * @brief  串口发送字符串
 */
void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        Serial_SendByte((uint8_t)String[i]);
    }
}

/**
 * @brief  计算 10 的幂（内部函数）
 */
static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

/**
 * @brief  串口发送数字（按位发送）
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte((uint8_t)(Number / Serial_Pow(10, Length - i - 1) % 10 + '0'));
    }
}

/**
 * @brief  串口格式化输出（类似 printf）
 */
void Serial_Printf(char *format, ...)
{
    char String[128];
    va_list arg;
    va_start(arg, format);
    vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    Serial_SendString(String);
}

/**
 * @brief  获取接收缓冲区中的数据量
 */
uint16_t Serial_GetRxCount(void)
{
    return RingBuffer_GetCount(&rxBuffer);
}

/**
 * @brief  从接收缓冲区读取一个字节
 */
uint8_t Serial_GetRxData(void)
{
    return RingBuffer_Read(&rxBuffer);
}

/* ==================== 中断服务函数 ==================== */

/**
 * @brief  UART1 接收中断服务函数
 * @note   排空 RX FIFO 并解析 F32C 9 字节反馈帧（协议与参考工程一致）：
 *           0x7A ADDR TYPE DATA4 BCC 0x7B
 *         ADDR=1/2 → TYPE=0x01(角度) / 0x00(速度)
 *         解析结果直接更新 motor{1,2}_{current_position,current_speed}
 */
void UART1_IRQHandler(void)
{
    uart1_isr_count++;  /* 放在最开头，判断 ISR 是否被进入 */

    /* 确认中断来源并清除 UART 外设中断标志 */
    if (DL_UART_Main_getPendingInterrupt(UART_1_INST) != DL_UART_MAIN_IIDX_RX)
        return;

    static uint8_t rx_buf[9];
    static uint8_t rx_idx = 0;

    /* 排空 RX FIFO */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_1_INST))
    {
        uart1_isr_count++;
        uint8_t byte = DL_UART_Main_receiveData(UART_1_INST);
        dbg_rx_total++;

        /* 写入环形缓冲区（供 Serial_GetRxData() / Serial_GetRxCount() 读取） */
        RingBuffer_Write(&rxBuffer, byte);

        /* ── F32C 帧同步状态机 ── */

        /* 等待帧头 0x7A */
        if (rx_idx == 0)
        {
            if (byte != 0x7A)
                continue;
            dbg_frame_header++;
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
        dbg_frame_full++;

        /* 1) 校验帧尾 */
        if (rx_buf[8] != 0x7B)
        {
            dbg_tail_err++;
            continue;
        }

        /* 2) BCC 校验：前 7 字节 XOR */
        uint8_t bcc = 0;
        for (uint8_t i = 0; i < 7; i++)
            bcc ^= rx_buf[i];
        if (bcc != rx_buf[7])
        {
            dbg_bcc_err++;
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
        dbg_frame_ok++;
    }
}
