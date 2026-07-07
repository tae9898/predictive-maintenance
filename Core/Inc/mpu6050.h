#ifndef __MPU6050_H
#define __MPU6050_H
#include "stm32g4xx_hal.h"

#define MPU6050_ADDR        (0x68U << 1)  /* 8-bit I2C addr */
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az);

#endif
