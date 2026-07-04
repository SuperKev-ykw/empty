/**
  * @file    HMC5883L.c
  * @brief   HMC5883L 三轴磁力计驱动
  * @details 通过MPU6050的AUX I2C旁路(BYPASS)访问，共用 I2C1 总线
  *          参考自 STM32 标准库工程 HMC5883L.c
  *          所有 I2C 等待增加超时保护，防止总线挂死
  *          配置：
  *            - 8次平均采样 + 15Hz更新率
  *            - 增益 1090 LSB/Ga (1.3Ga量程)
  *            - 连续测量模式
  */

#include "HMC5883L.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "ti_msp_dl_config.h"

#define HMC5883L_ADDR    0x1E    /* 7位地址 */

/* HMC5883L 寄存器地址 */
#define HMC5883L_CONFIG_A    0x00
#define HMC5883L_CONFIG_B    0x01
#define HMC5883L_MODE        0x02
#define HMC5883L_DATA_X_MSB  0x03

/* I2C 超时时间 (ms) */
#define I2C_TIMEOUT_MS  20

/* 外部 1ms 定时器计数值 */
extern volatile uint16_t Count0;

/* ===================== 超时工具函数 ===================== */

static uint32_t I2C_GetTick(void)
{
    return (uint32_t)Count0;
}

static int I2C_IsTimeout(uint32_t start, uint32_t timeout_ms)
{
    uint32_t now = I2C_GetTick();
    uint32_t elapsed;
    if (now >= start) {
        elapsed = now - start;
    } else {
        elapsed = (1000 - start) + now;
    }
    return (elapsed >= timeout_ms) ? 1 : 0;
}

#define GY_I2C_WAIT_IDLE() \
    while (!(DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))

/* I2C 总线空闲等待（带超时） */
#define GY_I2C_WAIT_IDLE_TIMEOUT(start) \
    do { \
        uint32_t _start = (start); \
        while (!(DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) { \
            if (I2C_IsTimeout(_start, I2C_TIMEOUT_MS)) { \
                DL_I2C_disableController(GY_87_INST); \
                DL_I2C_resetControllerTransfer(GY_87_INST); \
                DL_I2C_enableController(GY_87_INST); \
                return; \
            } \
        } \
    } while(0)

static void HMC5883L_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    uint32_t start;

    DL_I2C_transmitControllerData(GY_87_INST, RegAddress);
    GY_I2C_WAIT_IDLE();
    DL_I2C_transmitControllerData(GY_87_INST, Data);
    DL_I2C_clearInterruptStatus(GY_87_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    DL_I2C_startControllerTransfer(GY_87_INST, HMC5883L_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    start = I2C_GetTick();
    while (!DL_I2C_getRawInterruptStatus(GY_87_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
    {
        if (I2C_IsTimeout(start, I2C_TIMEOUT_MS))
        {
            DL_I2C_clearInterruptStatus(GY_87_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
            return;
        }
    }
    GY_I2C_WAIT_IDLE_TIMEOUT(start);
}

/**
 * @brief  一次读取多个连续寄存器（burst read，带超时）
 */
static void HMC5883L_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Count)
{
    uint32_t start;
    uint8_t i;

    DL_I2C_transmitControllerData(GY_87_INST, RegAddress);
    GY_I2C_WAIT_IDLE();
    GY_87_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;

    DL_I2C_startControllerTransfer(GY_87_INST, HMC5883L_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, Count);

    for (i = 0; i < Count; i++)
    {
        start = I2C_GetTick();
        while (DL_I2C_isControllerRXFIFOEmpty(GY_87_INST))
        {
            if (I2C_IsTimeout(start, I2C_TIMEOUT_MS))
            {
                GY_87_INST->MASTER.MCTR = 0;
                DL_I2C_disableController(GY_87_INST);
                DL_I2C_resetControllerTransfer(GY_87_INST);
                DL_I2C_enableController(GY_87_INST);
                return;
            }
        }
        Data[i] = DL_I2C_receiveControllerData(GY_87_INST);
    }

    GY_87_INST->MASTER.MCTR = 0;

    start = I2C_GetTick();
    while (!DL_I2C_getRawInterruptStatus(GY_87_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE))
    {
        if (I2C_IsTimeout(start, I2C_TIMEOUT_MS))
        {
            DL_I2C_disableController(GY_87_INST);
            DL_I2C_resetControllerTransfer(GY_87_INST);
            DL_I2C_enableController(GY_87_INST);
            return;
        }
    }
    GY_I2C_WAIT_IDLE_TIMEOUT(start);
}

void HMC5883L_Init(void)
{
    /* 使能MPU6050的AUX I2C旁路，使MCU可以直接访问HMC5883L */
    MPU6050_WriteReg(MPU6050_INT_PIN_CFG, 0x02);

    /* 8次平均采样，15Hz输出速率 */
    HMC5883L_WriteReg(HMC5883L_CONFIG_A, 0x70);
    /* 增益配置：1090 LSB/Ga */
    HMC5883L_WriteReg(HMC5883L_CONFIG_B, 0xA0);
    /* 连续测量模式 */
    HMC5883L_WriteReg(HMC5883L_MODE, 0x00);
}

void HMC5883L_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ)
{
    uint8_t buf[6];

    /* 一次读取6字节，替代原来的6次单独读取 */
    HMC5883L_ReadRegs(HMC5883L_DATA_X_MSB, buf, 6);

    /* HMC5883L 数据寄存器顺序: X, Z, Y */
    *MagX = (int16_t)((buf[0] << 8) | buf[1]);
    *MagZ = (int16_t)((buf[2] << 8) | buf[3]);
    *MagY = (int16_t)((buf[4] << 8) | buf[5]);
}
