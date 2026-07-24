/**
 * @file    Grayscale.h
 * @brief   8 路灰度传感器驱动接口
 * @details 定义灰度传感器引脚、电平宏、加权表和函数声明
 *
 * 硬件引脚：Gray_1=GPIOB.6（最右）, ..., Gray_8=GPIOB.18（最左）
 *
 * 加权偏差算法：
 *   偏差 = 加权和 / 检测到黑线的数量
 *   权重：Gray_8=-4, Gray_7=-3, Gray_6=-2, Gray_5=-1,
 *         Gray_4=+1, Gray_3=+2, Gray_2=+3, Gray_1=+4
 *
 * 函数清单：
 *   - Gray_Sensor_Init()      : 初始化 GPIO（SysConfig 已完成，本函数保留接口）
 *   - Gray_Sensor_Read()      : 读取 8 路灰度传感器状态到全局变量
 *   - Grayscale_GetDeviation(): 计算灰度加权偏差值
 *
 * 全局变量：Gray_1 ~ Gray_8（每个为 0=白底 或 1=黑线）
 */

#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ===================== 电平定义 ===================== */
#define GRAY_BLACK       1            // 检测到黑线（高电平）
#define GRAY_WHITE       0            // 检测到白底（低电平）

/* ===================== 传感器权重表 =====================
  * 传感器位置（从左到右）：Gray_8  Gray_7  Gray_6  Gray_5  Gray_4  Gray_3  Gray_2  Gray_1
  * 对应权重：              -4      -3      -2      -1      +1      +2      +3      +4
  * 偏差 = 加权和 / 检测到黑线的数量
  * 参考自 STM32 标准库工程 5路权重：-2, -1, 0, +1, +2
  * ==================================================== */
#define GRAY_WEIGHT_8    -4
#define GRAY_WEIGHT_7    -3
#define GRAY_WEIGHT_6    -2
#define GRAY_WEIGHT_5    -1
#define GRAY_WEIGHT_4     1
#define GRAY_WEIGHT_3     2
#define GRAY_WEIGHT_2     3
#define GRAY_WEIGHT_1     4

/* ===================== 全局变量 ===================== */
extern uint8_t Gray_1;   // 最右边灰度
extern uint8_t Gray_2;
extern uint8_t Gray_3;
extern uint8_t Gray_4;
extern uint8_t Gray_5;
extern uint8_t Gray_6;
extern uint8_t Gray_7;
extern uint8_t Gray_8;   // 最左边灰度

/* ===================== 循迹控制全局变量 ===================== */
extern uint8_t RunFlag;              // 循迹运行标志
extern int16_t PWML, PWMR;          // 左右电机当前PWM值
extern uint8_t Turn_Mode;            // 1=三段式拐弯(默认), 2=纯PD循迹(8路全参与)
extern uint8_t Gray_LineFound;       // 0=寻线中直行, 1=已捕获黑线进入循迹

/* ===================== 拐弯状态机（模式1） ===================== */
#define TURN_IDLE     0       // 正常循迹，PD微调
#define TURN_DECEL    1       // 外侧触发，减速直行
#define TURN_ACTIVE   2       // 差速拐弯

extern uint8_t Turn_State;           // 模式1状态
extern uint8_t Turn_Direction;       // 1=左弯，2=右弯
extern uint32_t Turn_Start_Tick;     // 拐弯触发时刻

/* ===================== 通用蓝牙可调参数（两模式共用） ===================== */
extern uint16_t BaseSpeed;           // BSp  直行基础速度
extern uint16_t SoftStartTime;       // SST  起步/出弯软加速时间（ms）

/* ===================== 模式1蓝牙参数（三段式拐弯） ===================== */
extern uint16_t TurnDecelTime;       // TdT  减速平滑时间（ms）
extern uint16_t TurnPower;           // TdP  差速幅值
extern uint16_t DecelSpeed;          // TdS  减速目标速度
extern uint16_t JoyTurn;             // JTurn 摇杆转向倍率

/* ===================== 模式2状态 ===================== */
#define M2_IDLE     0       /* 直行：仅中间灰度见线，全速PD */
#define M2_TURN     1       /* 转弯：外侧灰度触发，降速PD */

extern uint8_t M2_State;             /* 模式2状态 */

/* ===================== 模式2蓝牙参数（纯PD循迹） ===================== */
extern float TurnSlowRatio;          // TSlo 过弯降速比例，默认0.15
extern float Gray_WeightScale;       // WSc  外侧权重倍率，默认4.1

/* ===================== 通用速度斜坡结构体 ===================== */
typedef struct {
    int16_t  start_val;      /* 起始速度 */
    int16_t  target_val;     /* 目标速度 */
    uint32_t start_tick;     /* 开始时刻(System_Tick_Count) */
    uint32_t duration;       /* 过渡时长(ms) */
    uint8_t  active;         /* 1=正在过渡中 */
} SpeedRamp_t;

void SpeedRamp_Start(SpeedRamp_t *ramp, int16_t from, int16_t to, uint32_t duration_ms);
int16_t SpeedRamp_Update(SpeedRamp_t *ramp);   /* 返回当前速度，完成后返回目标值 */
uint8_t SpeedRamp_IsActive(SpeedRamp_t *ramp); /* 查询是否仍在过渡中 */
void SpeedRamp_Stop(SpeedRamp_t *ramp);         /* 强制结束斜坡 */

/* ===================== 原始函数声明 ===================== */
void Gray_Sensor_Init(void);        // 初始化8路灰度传感器GPIO
void Gray_Sensor_Read(void);        // 读取8路灰度传感器数据
float Grayscale_GetDeviation(void); // 获取加权偏差值（全部8路）

/* ===================== 循迹控制函数声明 ===================== */
void Gray_Track_Control(void);          // 灰度循迹控制主函数（三段式拐弯+PD差速）
float Grayscale_GetDeviation_Track(void); // 获取循迹偏差（仅用中间6路传感器）
void Gray_SoftStart_Reset(void);        // 重置软启动状态（兼容旧接口）

#endif
