/**
  * @file    Grayscale.c
  * @brief   8路灰度传感器驱动
  * @details 实现灰度传感器的数据读取
  *          参考自 STM32 标准库工程 Grayscale.c
  *          引脚映射（GPIO 由 SysConfig 初始化）：
  *            Gray_1（最右）：GPIOB.6
  *            Gray_2：       GPIOB.7
  *            Gray_3：       GPIOB.8
  *            Gray_4：       GPIOB.9
  *            Gray_5：       GPIOB.10
  *            Gray_6：       GPIOB.11
  *            Gray_7：       GPIOB.12
  *            Gray_8（最左）：GPIOB.18
  */

#include "Grayscale.h"

/* ===================== 全局变量 ===================== */
uint8_t Gray_1 = 0;   // 最右边灰度
uint8_t Gray_2 = 0;
uint8_t Gray_3 = 0;
uint8_t Gray_4 = 0;
uint8_t Gray_5 = 0;
uint8_t Gray_6 = 0;
uint8_t Gray_7 = 0;
uint8_t Gray_8 = 0;   // 最左边灰度

/**
  * @brief  初始化8路灰度传感器GPIO
  * @details GPIO 已在 SYSCFG_DL_init() 中通过 SysConfig 配置为上拉数字输入
  *          本函数仅作为接口预留，硬件初始化由 SysConfig 自动完成
  *          参考 STM32 标准库 Gray_Sensor_Init() 风格
  */
void Gray_Sensor_Init(void)
{
    /* GPIO 初始化已由 SysConfig 在 SYSCFG_DL_init() 中完成：
     *   DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM23~29, PINCM44,
     *       DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
     *       DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
     * 此处保留函数声明以保持与 STM32 参考工程接口一致
     */
}

/**
  * @brief  读取8路灰度传感器数据
  * @details 保存到全局变量 Gray_1 ~ Gray_8
  *           Gray_1 对应最右边传感器，Gray_8 对应最左边传感器
  */
void Gray_Sensor_Read(void)
{
    Gray_1 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_1_PIN) ? 1 : 0;
    Gray_2 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_2_PIN) ? 1 : 0;
    Gray_3 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_3_PIN) ? 1 : 0;
    Gray_4 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_4_PIN) ? 1 : 0;
    Gray_5 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_5_PIN) ? 1 : 0;
    Gray_6 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_6_PIN) ? 1 : 0;
    Gray_7 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_7_PIN) ? 1 : 0;
    Gray_8 = DL_GPIO_readPins(Gray_PORT, Gray_Gray_8_PIN) ? 1 : 0;
}

/**
  * @brief  计算灰度传感器加权偏差值
  * @retval 偏差值（加权平均）
  *         >0 表示线偏右侧（车头偏左，需左转）
  *         <0 表示线偏左侧（车头偏右，需右转）
  *         0 表示在中间 或 全白离线
  * @note   参考 STM32 标准库工程 Grayscale_GetDeviation() 实现
  *         偏差 = 加权和 / 检测到黑线的数量
  */
float Grayscale_GetDeviation(void)
{
    Gray_Sensor_Read();

    float weighted_sum = 0;
    uint8_t active_count = 0;

    if (Gray_8 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_8; active_count++; }
    if (Gray_7 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_7; active_count++; }
    if (Gray_6 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_6; active_count++; }
    if (Gray_5 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_5; active_count++; }
    if (Gray_4 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_4; active_count++; }
    if (Gray_3 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_3; active_count++; }
    if (Gray_2 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_2; active_count++; }
    if (Gray_1 == GRAY_BLACK) { weighted_sum += GRAY_WEIGHT_1; active_count++; }

    if (active_count == 0) return 0;   // 全白离线，偏差返回0

    return weighted_sum / active_count;
}
