/**
  * @file    MPU6050.c
  * @brief   MPU6050 六轴陀螺仪+加速度计驱动
  * @details 通过硬件 I2C1 (GY_87_INST) 读写寄存器
  *          参考自 STM32 标准库工程 MPU6050.c
  *          配置：
  *            - 时钟源：陀螺仪X轴
  *            - 采样率：100Hz (1kHz / (1+9))
  *            - DLPF：5Hz
  *            - 陀螺仪量程：±2000dps (灵敏度16.4 LSB/dps)
  *            - 加速度计量程：±4g (灵敏度2048 LSB/g)
  *
  * 硬件连接 (SysConfig 已配置)：
  *   GY-87 SCL -> PB2 (I2C1_SCL)
  *   GY-87 SDA -> PB3 (I2C1_SDA)
  */

#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "ti_msp_dl_config.h"
#include "ti/driverlib/dl_gpio.h"

#define MPU6050_ADDR    0x68    /* 7位地址 */

/* ===================== 硬件 I2C1 初始化 =====================
 * GY_87_INST 对应 I2C1，SysConfig 已配置引脚和时钟参数。
 * 但 SYSCFG_DL_GY_87_init() 函数体未在 ti_msp_dl_config.c 中生成，
 * 故在此手动初始化。若后续 SysConfig 重新生成代码，可移除以下
 * GY_I2C_Init() 并改调用 SYSCFG_DL_GY_87_init()。
 */

/* I2C 时钟配置: BUSCLK, 不分频 */
static const DL_I2C_ClockConfig gGyI2CClkCfg = {
    .clockSel    = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

static void GY_I2C_Init(void)
{
    /* 1. 恢复 I2C1 外设并上电 */
    DL_I2C_reset(GY_87_INST);
    DL_I2C_enablePower(GY_87_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    /* 2. 配置 GPIO 引脚为 I2C1 功能 */
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_GY_87_IOMUX_SDA,
        GPIO_GY_87_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_GY_87_IOMUX_SCL,
        GPIO_GY_87_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_GY_87_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_GY_87_IOMUX_SCL);

    /* 3. 配置 I2C1 时钟 */
    DL_I2C_setClockConfig(GY_87_INST, (DL_I2C_ClockConfig *)&gGyI2CClkCfg);

    /* 4. 模拟/数字毛刺滤波 */
    DL_I2C_setAnalogGlitchFilterPulseWidth(GY_87_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(GY_87_INST);
    DL_I2C_setDigitalGlitchFilterPulseWidth(GY_87_INST,
        DL_I2C_DIGITAL_GLITCH_FILTER_WIDTH_CLOCKS_1);

    /* 5. 控制器模式配置 */
    DL_I2C_resetControllerTransfer(GY_87_INST);
    /* 400kHz: timerPeriod = (40MHz/(2*400kHz)) - 1 = 49 */
    DL_I2C_setTimerPeriod(GY_87_INST, 49);
    DL_I2C_setControllerTXFIFOThreshold(GY_87_INST,
        DL_I2C_TX_FIFO_LEVEL_BYTES_7);
    DL_I2C_setControllerRXFIFOThreshold(GY_87_INST,
        DL_I2C_RX_FIFO_LEVEL_BYTES_8);
    DL_I2C_enableControllerClockStretching(GY_87_INST);

    /* 6. 使能控制器 */
    DL_I2C_enableController(GY_87_INST);
}

/* ===================== I2C 读写操作 ===================== */

/**
 * @brief 等待 I2C 总线空闲
 */
static void GY_I2C_WaitIdle(void)
{
    while (!(DL_I2C_getControllerStatus(GY_87_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE));
}

/**
 * @brief 向 MPU6050 写一个寄存器
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    /* 将 reg 和 data 放入 TX FIFO */
    DL_I2C_transmitControllerData(GY_87_INST, RegAddress);
    GY_I2C_WaitIdle();

    DL_I2C_transmitControllerData(GY_87_INST, Data);

    /* 启动传输 2 字节 (reg + data) */
    DL_I2C_clearInterruptStatus(GY_87_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    DL_I2C_startControllerTransfer(GY_87_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    /* 等待传输完成 */
    while (!DL_I2C_getRawInterruptStatus(GY_87_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE));

    GY_I2C_WaitIdle();
}

/**
 * @brief 从 MPU6050 读一个寄存器
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    /* 发送寄存器地址 */
    DL_I2C_transmitControllerData(GY_87_INST, RegAddress);
    GY_I2C_WaitIdle();

    /* 使能发送空后自动转为读取 (repeated start) */
    GY_87_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_clearInterruptStatus(GY_87_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    /* 启动接收 1 字节 */
    DL_I2C_startControllerTransfer(GY_87_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    /* 等待接收完成 */
    while (!DL_I2C_getRawInterruptStatus(GY_87_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE));

    Data = DL_I2C_receiveControllerData(GY_87_INST);
    GY_87_INST->MASTER.MCTR = 0;   /* 恢复 MCTR */

    GY_I2C_WaitIdle();

    return Data;
}

/**
 * @brief 从 MPU6050 连续读多个寄存器
 */
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    uint8_t i;

    /* 发送寄存器地址 */
    DL_I2C_transmitControllerData(GY_87_INST, RegAddress);
    GY_I2C_WaitIdle();

    /* 使能 repeated start */
    GY_87_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;

    DL_I2C_startControllerTransfer(GY_87_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, Count);

    /* 逐个读取 RX FIFO */
    for (i = 0; i < Count; i++)
    {
        while (DL_I2C_isControllerRXFIFOEmpty(GY_87_INST));
        DataArray[i] = DL_I2C_receiveControllerData(GY_87_INST);
    }

    GY_87_INST->MASTER.MCTR = 0;

    /* 等待传输完成 */
    while (!DL_I2C_getRawInterruptStatus(GY_87_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE));

    GY_I2C_WaitIdle();
}

/* ===================== MPU6050 API ===================== */

void MPU6050_Init(void)
{
    /* 初始化硬件 I2C1 (GY_87 总线) */
    GY_I2C_Init();

    /* 配置 MPU6050 寄存器 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);      /* 时钟源=陀螺仪X轴 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);      /* 所有轴使能 */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);      /* 采样率=100Hz */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);           /* DLPF=5Hz */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);      /* ±2000dps */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);     /* ±4g */
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];

    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);

    *AccX = (int16_t)((Data[0]  << 8) | Data[1]);
    *AccY = (int16_t)((Data[2]  << 8) | Data[3]);
    *AccZ = (int16_t)((Data[4]  << 8) | Data[5]);

    *GyroX = (int16_t)((Data[8]  << 8) | Data[9]);
    *GyroY = (int16_t)((Data[10] << 8) | Data[11]);
    *GyroZ = (int16_t)((Data[12] << 8) | Data[13]);
}
