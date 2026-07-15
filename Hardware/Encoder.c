/**
 * @file    Encoder.c
 * @brief   编码器驱动实现
 * @details 使用 GPIO 外部中断 + 正交解码，通过 TIMG12 10ms 定时器周期
 *          读取并缓存编码器脉冲增量。
 *
 * 参考工程：STM32_CAR/NEW 8.0 基础驱动 双环循迹+频率云台 (Encoder.c)
 *
 * 解码原理：
 *   四路 GPIO 中断（每路 A/B 相的上升沿触发）→ 软件正交解码
 *   10ms 定时器中断 → 锁存增量并清零
 */

#include "ti_msp_dl_config.h"
#include "Encoder.h"

/* ==================== 编码器原始脉冲计数器 ==================== */
/* 在 GPIO 中断中实时更新 */
static volatile int16_t g_enc_left_raw = 0;     /* PB13/PB14 = 物理左轮编码器 */
static volatile int16_t g_enc_right_raw = 0;    /* PA7/PA25  = 物理右轮编码器 */

/* 10ms 定时器锁存的增量值，供 Encoder_GetCount() 读取 */
static volatile int16_t g_enc_left_delta = 0;   /* 物理左轮（取反修正后） */
static volatile int16_t g_enc_right_delta = 0;  /* 物理右轮（取反修正后） */

/* 累计总脉冲数（10ms 中断中累加，永不丢失） */
static volatile int32_t g_enc_left_total = 0;
static volatile int32_t g_enc_right_total = 0;

/* ==================== GPIO 中断触发计数（诊断用） ==================== */
volatile uint32_t g_enc_gpioa_irq_cnt = 0;   /* GPIOA 中断触发次数（右编码器，累加） */
volatile uint32_t g_enc_gpiob_irq_cnt = 0;   /* GPIOB 中断触发次数（左编码器，累加） */
static volatile int16_t g_enc_gpioa_delta = 0;   /* 10ms 窗口内的 GPIOA 中断增量 */
static volatile int16_t g_enc_gpiob_delta = 0;   /* 10ms 窗口内的 GPIOB 中断增量 */

/* ==================== 20ms 速度环累计 ==================== */
/* 每2次 10ms 中断累计一次，供 Encoder_CalcSpeed() 使用 */
static volatile int16_t g_enc_left_20ms_acc = 0;
static volatile int16_t g_enc_right_20ms_acc = 0;
static volatile uint8_t  g_enc_20ms_ready = 0;

/* ==================== 5点滑动平均滤波 ==================== */
#define SPEED_FILTER_BUF  5
static int16_t g_speed_buf_left[SPEED_FILTER_BUF]  = {0};
static int16_t g_speed_buf_right[SPEED_FILTER_BUF] = {0};
static uint8_t g_speed_buf_idx_left  = 0;
static uint8_t g_speed_buf_idx_right = 0;

/* 滤波后的速度值 */
static int16_t g_speed_left_filtered  = 0;
static int16_t g_speed_right_filtered = 0;

/* ==================== Encoder_Init ==================== */

void Encoder_Init(void)
{
    /* 使能 GPIO 中断 NVIC（在 GROUP1 中） */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);

    /* 使能 TIMG12 10ms 定时器中断 NVIC */
    NVIC_EnableIRQ(TIMER_ENCODER_INST_INT_IRQN);
}

/* ==================== Encoder_GetCount ==================== */

int16_t Encoder_GetCount(uint8_t n)
{
    int16_t val = 0;

    if (n == ENCODER_LEFT)
    {
        val = g_enc_left_delta;
        g_enc_left_delta = 0;
    }
    else
    {
        val = g_enc_right_delta;
        g_enc_right_delta = 0;
    }

    return val;
}

/* ==================== GPIO 中断 ==================== */
/**
 * GROUP1_IRQHandler — 处理 GPIOA / GPIOB 中断
 * 四路编码器信号（PA7, PA25, PB13, PB14）均配置为上升沿触发。
 *
 * 正交解码规则：
 *   A 相上升沿时读 B 相：B=1 → 正转 +1；B=0 → 反转 -1
 *   B 相上升沿时读 A 相：A=1 → 反转 -1；A=0 → 正转 +1
 */
void GROUP1_IRQHandler(void)
{
    uint32_t group1_iidx = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    /* ---------- GPIOA 中断（PA7/PA25 = 物理右轮编码器） ---------- */
    if (group1_iidx == DL_INTERRUPT_GROUP1_IIDX_GPIOA)
    {
        g_enc_gpioa_irq_cnt++;  /* 诊断：GPIOA 中断触发计数 */
        uint32_t gpioa_iidx = DL_GPIO_getPendingInterrupt(GPIOA);

        if (gpioa_iidx == DL_GPIO_IIDX_DIO7)                    /* PA7 = A 相上升沿 */
        {
            if (DL_GPIO_readPins(GPIOA, Encoders_Encoder_A2_PIN))
                g_enc_right_raw++;                              /* A↑ & B=1 → 正转 */
            else
                g_enc_right_raw--;                              /* A↑ & B=0 → 反转 */
        }
        else if (gpioa_iidx == DL_GPIO_IIDX_DIO25)              /* PA25 = B 相上升沿 */
        {
            if (DL_GPIO_readPins(GPIOA, Encoders_Encoder_A1_PIN))
                g_enc_right_raw--;                              /* B↑ & A=1 → 反转 */
            else
                g_enc_right_raw++;                              /* B↑ & A=0 → 正转 */
        }
    }

    /* ---------- GPIOB 中断（PB13/PB14 = 物理左轮编码器） ---------- */
    else if (group1_iidx == DL_INTERRUPT_GROUP1_IIDX_GPIOB)
    {
        g_enc_gpiob_irq_cnt++;  /* 诊断：GPIOB 中断触发计数 */
        uint32_t gpiob_iidx = DL_GPIO_getPendingInterrupt(GPIOB);

        if (gpiob_iidx == DL_GPIO_IIDX_DIO13)                   /* PB13 = A 相上升沿 */
        {
            if (DL_GPIO_readPins(GPIOB, Encoders_Encoder_B2_PIN))
                g_enc_left_raw++;                               /* A↑ & B=1 → 正转 */
            else
                g_enc_left_raw--;                               /* A↑ & B=0 → 反转 */
        }
        else if (gpiob_iidx == DL_GPIO_IIDX_DIO14)              /* PB14 = B 相上升沿 */
        {
            if (DL_GPIO_readPins(GPIOB, Encoders_Encoder_B1_PIN))
                g_enc_left_raw--;                               /* B↑ & A=1 → 反转 */
            else
                g_enc_left_raw++;                               /* B↑ & A=0 → 正转 */
        }
    }
}

/* ==================== TIMG12 10ms 定时器中断 ==================== */
/**
 * TIMG12_IRQHandler — 每 10ms 锁存编码器增量并清零
 * 主循环通过 Encoder_GetCount() 读取锁存值。
 * 同时累计20ms窗口供速度环PID使用。
 */
void TIMG12_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_ENCODER_INST) == DL_TIMER_IIDX_ZERO)
    {
        /* 锁存当前 10ms 增量 */
        /* 右轮编码器（PA7/PA25）A/B 相反相 → 取反使正转=正数 */
        g_enc_left_delta  = g_enc_left_raw;
        g_enc_right_delta = -g_enc_right_raw;

        /* 累加到总脉冲数 */
        g_enc_left_total  += g_enc_left_delta;
        g_enc_right_total += g_enc_right_delta;

        /* 锁存 GPIO 中断增量（诊断用） */
        g_enc_gpioa_delta = (int16_t)g_enc_gpioa_irq_cnt;
        g_enc_gpiob_delta = (int16_t)g_enc_gpiob_irq_cnt;

        /* 清零实时计数器，开始下一个 10ms 计数 */
        g_enc_left_raw     = 0;
        g_enc_right_raw    = 0;
        g_enc_gpioa_irq_cnt = 0;
        g_enc_gpiob_irq_cnt = 0;

        /* 累计到20ms窗口（速度环使用） */
        g_enc_left_20ms_acc  += g_enc_left_delta;
        g_enc_right_20ms_acc += g_enc_right_delta;

        /* 每2次中断 = 20ms，设置就绪标志 */
        static uint8_t cnt = 0;
        cnt++;
        if (cnt >= 2)
        {
            cnt = 0;
            g_enc_20ms_ready = 1;
        }
    }
}

/* ==================== Encoder_CalcSpeed（20ms 速度计算 + 5点滤波） ==================== */

/* 5点滑动平均滤波内部函数 */
static int16_t speed_low_pass_filter(int16_t raw, int16_t *buf, uint8_t *idx)
{
    buf[*idx] = raw;
    *idx = (*idx + 1) % SPEED_FILTER_BUF;
    int32_t sum = 0;
    for (uint8_t i = 0; i < SPEED_FILTER_BUF; i++)
        sum += buf[i];
    return (int16_t)(sum / SPEED_FILTER_BUF);
}

void Encoder_CalcSpeed(void)
{
    if (g_enc_20ms_ready)
    {
        g_enc_20ms_ready = 0;

        /* 左轮速度滤波 */
        g_speed_left_filtered = speed_low_pass_filter(
            g_enc_left_20ms_acc, g_speed_buf_left, &g_speed_buf_idx_left);

        /* 右轮速度滤波 */
        g_speed_right_filtered = speed_low_pass_filter(
            g_enc_right_20ms_acc, g_speed_buf_right, &g_speed_buf_idx_right);

        /* 清零20ms累计，开始下一轮 */
        g_enc_left_20ms_acc  = 0;
        g_enc_right_20ms_acc = 0;
    }
}

int16_t Encoder_GetSpeed(uint8_t n)
{
    if (n == ENCODER_LEFT)
        return g_speed_left_filtered;
    else
        return g_speed_right_filtered;
}

/* ==================== 诊断：获取 GPIO 中断 10ms 增量 ==================== */
int16_t Encoder_GetIRQCount(uint8_t port)
{
    int16_t val;
    if (port == ENCODER_LEFT)
    {
        val = g_enc_gpiob_delta;
        g_enc_gpiob_delta = 0;
    }
    else
    {
        val = g_enc_gpioa_delta;
        g_enc_gpioa_delta = 0;
    }
    return val;
}

int32_t Encoder_GetTotalCount(uint8_t n)
{
    if (n == ENCODER_LEFT)
        return g_enc_left_total;
    else
        return g_enc_right_total;
}
