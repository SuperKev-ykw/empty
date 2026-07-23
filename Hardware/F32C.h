/**
 * @file    F32C.h
 * @brief   F32C 无刷云台电机 TTL 协议驱动头文件
 * @details 电机 ID、模式宏定义及公开 API 声明
 *
 * 硬件：UART1 (PA8=TX, PA9=RX, 115200)
 *
 * 函数清单：
 *   - Send_Enable()    : 发送使能命令
 *   - Send_Mode()      : 发送设置模式命令
 *   - Send_Speed()     : 发送设置速度命令
 *   - Send_Position()  : 发送设置位置命令（位置单位 0.1度）
 *
 * 常用宏：
 *   - MOTOR1_ID / MOTOR2_ID : 电机 ID
 *   - MODE_POS              : 位置闭环模式
 *   - COORD_CENTER (224)    : 屏幕中心坐标
 *   - COORD_RANGE  (448)    : 屏幕坐标范围
 *   - ANGLE_RANGE  (3600)   : 角度范围 ±1800 (0.1°)
 *
 * 全局变量：
 *   - motor1_target / motor2_target : 目标位置（0.1度）
 */

#ifndef __F32C_H
#define __F32C_H

#include <stdint.h>

/* ==================== 电机 ID ==================== */
#define MOTOR1_ID       0x01
#define MOTOR2_ID       0x02

/* ==================== 模式定义 ==================== */
#define MODE_SPEED          0   /* 速度闭环模式，上电默认 */
#define MODE_POS            1   /* 多圈 T 型位置闭环模式（云台推荐） */
#define MODE_POS_SINGLE     2   /* 单圈 T 型位置模式（0~359.9°，就近旋转） */

/* ==================== 角度常量 ==================== */
#define POS_10          (10 * 10)   /* 10 度，单位 0.1 度 */
/* ==================== 坐标映射常量 ==================== */
#define COORD_CENTER_X  320         /* X 轴中心坐标 (640/2) */
#define COORD_CENTER_Y  240         /* Y 轴中心坐标 (480/2) */
#define COORD_RANGE_X   640         /* X 轴坐标范围（0~640） */
#define COORD_RANGE_Y   480         /* Y 轴坐标范围（0~480） */
#define ANGLE_RANGE     50         /* 角度范围 ±25 (0.1°)，即 ±2.5° */

/* ==================== 反馈命令定义 ==================== */
#define CMD_FEEDBACK        0x0E     /* 反馈读取命令 */
#define FB_SPEED            0x00     /* 速度反馈 (单位 RPM) */
#define FB_MULTI_ANGLE      0x01     /* 多圈绝对角度 (单位 0.1°) */
#define FB_SINGLE_ANGLE     0x02     /* 单圈机械角度 (单位 0.1°, 0~3599) */

/* ==================== 全局变量声明 ==================== */
extern int32_t motor1_target;
extern int32_t motor2_target;

/* ---- 编码器反馈（当前角度/速度） ---- */
extern volatile int32_t motor1_current_position;    /* 电机1当前角度 (0.1°) */
extern volatile int32_t motor2_current_position;    /* 电机2当前角度 (0.1°) */
extern volatile int32_t motor1_current_speed;       /* 电机1当前转速 (RPM) */
extern volatile int32_t motor2_current_speed;       /* 电机2当前转速 (RPM) */

/* ==================== 发送 API ==================== */
void Send_Enable(uint8_t id);
void Send_Disable(uint8_t id);                              /* 0x05: 失能电机 */
void Send_Mode(uint8_t id, uint16_t mode);
void Send_Speed(uint8_t id, int16_t speed);
void Send_Position(uint8_t id, int32_t position);
void Send_Position_Single(uint8_t id, int16_t position);    /* 单圈位置（0.1°, 0~3599） */
void Send_SetZero(uint8_t id);                              /* 0x0A: 当前位置设为单圈 0° */
void Send_MultiClear(uint8_t id);                           /* 0x09: 清空多圈累计值 */
void Send_Save(uint8_t id);                                 /* 0x08: 保存参数到 Flash */
void Send_Acc(uint8_t id, uint16_t acc);                    /* 0x07: 设置加速度（转/s²） */

/**
 * @brief 发送反馈读取请求
 * @param id   电机 ID (MOTOR1_ID / MOTOR2_ID)
 * @param type 反馈类型 (FB_MULTI_ANGLE / FB_SPEED)
 */
void Send_FeedbackRequest(uint8_t id, uint8_t type);

/**
 * @brief 解析 F32C 反馈帧（从 Serial 环形缓冲区读取）
 * @details 在主循环中周期调用，自动提取当前角度/速度到全局变量
 */
void F32C_ParseFeedback(void);

/**
 * @brief 轮询排空 UART1 RX FIFO 并解析 F32C 反馈帧（不依赖中断）
 * @details 在主循环中调用，直接读取硬件 FIFO 并解析 9 字节帧
 */
void F32C_PollFeedback(void);

#endif /* __F32C_H */
