#include "mpu6050.h"

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    /* Wake up (PWR_MGMT_1 = 0) */
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1,
                          I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);
    /* Accel config: +/-2g (0x00) */
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG,
                       I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    /* Sample rate divider = 9 (100Hz internal, but we poll anyway) */
    data = 0x09;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV,
                       I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az) {
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, buf, 6, 100) != HAL_OK)
        return HAL_ERROR;
    int16_t rx = (buf[0] << 8) | buf[1];
    int16_t ry = (buf[2] << 8) | buf[3];
    int16_t rz = (buf[4] << 8) | buf[5];
    /* +/-2g range: sensitivity = 16384 LSB/g */
    *ax = (float)rx / 16384.0f;
    *ay = (float)ry / 16384.0f;
    *az = (float)rz / 16384.0f;
    return HAL_OK;
}
