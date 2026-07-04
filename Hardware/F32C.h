/**
 * @file    F32C.h
 * @brief   F32C 无刷云台电机 TTL 协议驱动头文件
 * @details 电机 ID、模式宏定义及公开 API 声明
 */

#ifndef __F32C_H
#define __F32C_H

#include <stdint.h>

/* ==================== 电机 ID ==================== */
#define MOTOR1_ID       0x01
#define MOTOR2_ID       0x02

/* ==================== 模式定义 ==================== */
#define MODE_POS        1       /* 位置闭环模式 */

/* ==================== 角度常量 ==================== */
#define POS_10          (10 * 10)   /* 10 度，单位 0.1 度 */
#define POS_STEP        1000        /* 按键单次步进：100 度 */

/* ==================== 坐标映射常量 ==================== */
#define COORD_CENTER    224         /* 屏幕中心坐标 (448/2) */
#define COORD_RANGE     448         /* 屏幕坐标范围（0~448，摄像头分辨率） */
#define ANGLE_RANGE     3600        /* 角度范围 ±1800 (0.1°) */

/* ==================== 全局变量声明 ==================== */
extern volatile int32_t motor1_pos;
extern volatile int32_t motor2_pos;
extern int32_t motor1_target;
extern int32_t motor2_target;
extern uint8_t motor1_online;
extern uint8_t motor2_online;

/* ==================== 发送 API ==================== */
void Send_Enable(uint8_t id);
void Send_Mode(uint8_t id, uint16_t mode);
void Send_Speed(uint8_t id, int16_t speed);
void Send_Position(uint8_t id, int32_t position);
void Send_Feedback(uint8_t id);

/* ==================== 接收解析 API ==================== */
void Parse_Feedback(void);

#endif /* __F32C_H */
