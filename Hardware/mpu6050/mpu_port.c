/**
 * @file    mpu_port.c
 * @brief   MPU6050 DMP 移植层实现
 * @details 实现 inv_mpu.c 所需的 I2C 读写函数，并封装 DMP_Init / DMP_Read_Data
 *
 * 硬件：I2C1 (GY_87_INST)  PB2(SCL) / PB3(SDA)
 *
 * 函数清单：
 *   - mget_ms()       : 获取当前毫秒计数
 *   - MPU_Tick()      : 1ms 定时器 ISR 调用，递增 sys_tick_ms
 *   - MPU_Write_Len() : DMP 库调用，写 MPU6050 寄存器（带超时）
 *   - MPU_Read_Len()  : DMP 库调用，读 MPU6050 寄存器（带超时）
 *   - inv_row_2_scale(): DMP 库辅助函数，方向矩阵行→标量
 *   - inv_orientation_matrix_to_scalar(): DMP 库辅助函数，方向矩阵→标量
 *   - DMP_Init()      : 初始化 MPU6050 DMP 引擎
 *   - DMP_Read_Data() : 读取 DMP 解算的 Pitch/Roll/Yaw 角度（度）
 *
 * 使用方式：
 *   1. 在 TIMER_0_INST_IRQHandler 中每 1ms 调用一次 MPU_Tick()
 *   2. 启动时调用 DMP_Init() 初始化 DMP
 *   3. 循环调用 DMP_Read_Data() 获取姿态角
 */

#include "mpu_port.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#include "ti_msp_dl_config.h"
#include "delay.h"             /* 使用工程自带的 delay_ms */

static volatile uint32_t sys_tick_ms = 0;

void mget_ms(uint32_t *time) {
    if (time) {
        *time = sys_tick_ms;
    }
}

/* 由 SysTick 中断每 1ms 调用一次，供 mget_ms 使用 */
void MPU_Tick(void) {
    sys_tick_ms++;
}


int MPU_Write_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf) {
    volatile uint32_t timeout = 100000;
    while (!(DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return -1;
    }
    
    DL_I2C_transmitControllerData(GY_87_INST, reg);
    DL_I2C_startControllerTransfer(GY_87_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, len + 1);
    
    for (uint16_t i = 0; i < len; i++) {
        timeout = 100000;
        while (DL_I2C_isControllerTXFIFOFull(GY_87_INST)) {
            if (--timeout == 0) return -2;
        }
        DL_I2C_transmitControllerData(GY_87_INST, buf[i]);
    }
    
    timeout = 100000;
    while (DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--timeout == 0) return -3;
    }
    timeout = 100000;
    while (!(DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return -4;
    }
    return 0;
}


int MPU_Read_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf) {
    volatile uint32_t timeout; 
    
    timeout = 100000;
    while (!(DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return -1;
    }
    
    DL_I2C_transmitControllerData(GY_87_INST, reg);
    DL_I2C_startControllerTransfer(GY_87_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    
    timeout = 100000;
    while (DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--timeout == 0) return -2;
    }
    timeout = 100000;
    while (!(DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return -3;
    }

    DL_I2C_startControllerTransfer(GY_87_INST, addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);
    
    for (uint16_t i = 0; i < len; i++) {
        timeout = 100000;
        while (DL_I2C_isControllerRXFIFOEmpty(GY_87_INST)) {
            
            if (DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) return -4;
            
            if (--timeout == 0) return -5; 
        }
        buf[i] = DL_I2C_receiveControllerData(GY_87_INST);
    }
    
    timeout = 100000;
    while (DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--timeout == 0) return -6;
    }
    timeout = 100000;
    while (!(DL_I2C_getControllerStatus(GY_87_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return -7;
    }
    return 0;
}





static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1 };
unsigned short inv_row_2_scale(const signed char *row) {
    unsigned short b;
    if (row[0] > 0) b = 0;
    else if (row[0] < 0) b = 4;
    else if (row[1] > 0) b = 1;
    else if (row[1] < 0) b = 5;
    else if (row[2] > 0) b = 2;
    else if (row[2] < 0) b = 6;
    else b = 7;
    return b;
}
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) {
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}


int DMP_Init(void) {
    int res;
    /* 使用工程的 TIMER_0 提供 1ms 计时，不配置 SysTick */
    delay_cycles(CPUCLK_FREQ / 10);
    res = mpu_init();
    if (res) return res; 
    
    
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(100); 
    
    
    res = dmp_load_motion_driver_firmware();
    if (res) return res; 
    
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    
    
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | 
                       DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | 
                       DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
                       
    dmp_set_fifo_rate(100); 
    res = mpu_set_dmp_state(1); 
    
    return res;
}


#define q30  1073741824.0f 
int DMP_Read_Data(float *pitch, float *roll, float *yaw) {
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    
    
    if (dmp_read_fifo(gyro, accel, quat, NULL, &sensors, &more) == 0) {
        if (sensors & INV_WXYZ_QUAT) {
            float q0 = quat[0] / q30;
            float q1 = quat[1] / q30;
            float q2 = quat[2] / q30;
            float q3 = quat[3] / q30;
            
            
            *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f;
            *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f;
            *yaw   = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f;
            return 0; 
        }
    }
    return -1; 
}
