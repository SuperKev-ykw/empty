/**
 * @file    HMC5883L.h
 * @brief   HMC5883L 三轴磁力计驱动头文件
 */

#ifndef __HMC5883L_H
#define __HMC5883L_H

#include <stdint.h>

void HMC5883L_Init(void);
void HMC5883L_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ);

#endif /* __HMC5883L_H */
