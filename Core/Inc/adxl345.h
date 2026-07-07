#ifndef __ADXL345_H
#define __ADXL345_H
#include "stm32g4xx_hal.h"

#define ADXL345_ADDR        (0x53U << 1)
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATAX0      0x32

HAL_StatusTypeDef ADXL345_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ADXL345_ReadAccel(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az);

#endif
