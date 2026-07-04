/**
  * @file    AHRS.c
  * @brief   姿态解算 (加速度计+磁力计+陀螺仪互补融合)
  * @details Roll/Pitch 来自加速度计静态解算
  *          Yaw 通过互补滤波融合陀螺仪和磁力计：
  *            yaw = 0.98 * (yaw + gyroZ * dt) + 0.02 * mag_yaw
  *          陀螺仪提供短期稳定性，磁力计抑制长期零漂
  *          参考自 STM32 标准库工程 AHRS.c，增加陀螺仪融合
  */

#include "AHRS.h"
#include <math.h>

#define PI          3.14159265f
#define RAD_TO_DEG  57.29578f

/* 传感器灵敏度 */
#define ACCEL_SCALE     2048.0f     /* ±4g => 2048 LSB/g */
#define GYRO_SCALE      16.4f       /* ±2000dps => 16.4 LSB/dps */

/* 互补滤波系数 (alpha越大，陀螺仪权重越大) */
#define COMP_ALPHA      0.98f

static float Yaw = 0.0f;
static float Pitch = 0.0f;
static float Roll = 0.0f;

/* 磁力计硬铁偏移 */
static float Mag_OffsetX = 0.0f;
static float Mag_OffsetY = 0.0f;
static float Mag_OffsetZ = 0.0f;

void AHRS_CalibrateMag(int16_t MagX, int16_t MagY, int16_t MagZ)
{
    static int16_t MagX_Min = 32767, MagX_Max = -32768;
    static int16_t MagY_Min = 32767, MagY_Max = -32768;
    static int16_t MagZ_Min = 32767, MagZ_Max = -32768;
    static uint8_t Calib_Count = 0;

    if (MagX < MagX_Min) MagX_Min = MagX;
    if (MagX > MagX_Max) MagX_Max = MagX;
    if (MagY < MagY_Min) MagY_Min = MagY;
    if (MagY > MagY_Max) MagY_Max = MagY;
    if (MagZ < MagZ_Min) MagZ_Min = MagZ;
    if (MagZ > MagZ_Max) MagZ_Max = MagZ;

    Calib_Count++;

    if (Calib_Count >= 100)
    {
        Mag_OffsetX = (float)(MagX_Max + MagX_Min) / 2.0f;
        Mag_OffsetY = (float)(MagY_Max + MagY_Min) / 2.0f;
        Mag_OffsetZ = (float)(MagZ_Max + MagZ_Min) / 2.0f;
        Calib_Count = 0;
        MagX_Min = MagZ_Min = MagY_Min = 32767;
        MagX_Max = MagY_Max = MagZ_Max = -32768;
    }
}

void AHRS_SetMagOffset(float OffsetX, float OffsetY, float OffsetZ)
{
    Mag_OffsetX = OffsetX;
    Mag_OffsetY = OffsetY;
    Mag_OffsetZ = OffsetZ;
}

void AHRS_GetMagOffset(float *OffsetX, float *OffsetY, float *OffsetZ)
{
    if (OffsetX) *OffsetX = Mag_OffsetX;
    if (OffsetY) *OffsetY = Mag_OffsetY;
    if (OffsetZ) *OffsetZ = Mag_OffsetZ;
}

void AHRS_Update(int16_t AccX, int16_t AccY, int16_t AccZ,
                 int16_t GyroZ,
                 int16_t MagX, int16_t MagY, int16_t MagZ,
                 float dt)
{
    float ax, ay, az;
    float gz;
    float mx, my, mz;
    float roll, pitch;
    float cos_roll, sin_roll, cos_pitch, sin_pitch;
    float mx_comp, my_comp, mag_yaw;

    /* ======== 加速度计转g ======== */
    ax = (float)AccX / ACCEL_SCALE;
    ay = (float)AccY / ACCEL_SCALE;
    az = (float)AccZ / ACCEL_SCALE;

    /* ======== Roll/Pitch 来自加速度计 ======== */
    roll = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    Roll = roll * RAD_TO_DEG;
    Pitch = pitch * RAD_TO_DEG;

    /* ======== 磁力计硬铁校准 ======== */
    mx = (float)MagX - Mag_OffsetX;
    my = (float)MagY - Mag_OffsetY;
    mz = (float)MagZ - Mag_OffsetZ;

    /* ======== 磁力计倾斜补偿 ======== */
    cos_roll = cosf(roll);
    sin_roll = sinf(roll);
    cos_pitch = cosf(pitch);
    sin_pitch = sinf(pitch);

    mx_comp = mx * cos_pitch + mz * sin_pitch;
    my_comp = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;

    /* 磁力计Yaw (0-360度) */
    mag_yaw = atan2f(-my_comp, mx_comp) * RAD_TO_DEG;
    if (mag_yaw < 0) mag_yaw += 360.0f;

    /* ======== 陀螺仪转dps ======== */
    gz = (float)GyroZ / GYRO_SCALE;   /* dps */

    /* ======== 互补滤波：陀螺仪 + 磁力计融合 ========
     *  yaw = alpha * (yaw + gyroZ * dt) + (1-alpha) * mag_yaw
     *  陀螺仪提供短期动态响应，磁力计抑制长期零漂
     *  alpha=0.98, dt=0.01s时，时间常数约0.5s
     *
     *  处理360度环绕：确保融合时角度差在[-180, 180]范围内
     */
    {
        float gyro_yaw = Yaw + gz * dt;     /* 陀螺仪积分 */
        float diff = mag_yaw - gyro_yaw;      /* 磁力计修正量 */

        /* 处理360度环绕 */
        if (diff > 180.0f)  diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;

        /* 互补融合 */
        Yaw = gyro_yaw + (1.0f - COMP_ALPHA) * diff;

        /* 归一化到0-360 */
        if (Yaw < 0.0f)   Yaw += 360.0f;
        if (Yaw >= 360.0f) Yaw -= 360.0f;
    }
}

float AHRS_GetYaw(void)   { return Yaw; }
float AHRS_GetPitch(void) { return Pitch; }
float AHRS_GetRoll(void)  { return Roll; }
