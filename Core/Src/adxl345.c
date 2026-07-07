#include "adxl345.h"

HAL_StatusTypeDef ADXL345_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    /* DATA_FORMAT: +/-16g, full-res, right-justify */
    data = 0x0B;
    HAL_I2C_Mem_Write(hi2c, ADXL345_ADDR, ADXL345_REG_DATA_FORMAT,
                       I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    /* BW_RATE: 3200Hz 출력 (0x0F) — 1kHz 샘플링이 센서 내부 레이트에 안 막히게 */
    data = 0x0F;
    HAL_I2C_Mem_Write(hi2c, ADXL345_ADDR, ADXL345_REG_BW_RATE,
                       I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    /* POWER_CTL: measure mode (bit 3 = 1) */
    data = 0x08;
    HAL_I2C_Mem_Write(hi2c, ADXL345_ADDR, ADXL345_REG_POWER_CTL,
                       I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    HAL_Delay(10);
    return HAL_OK;
}

HAL_StatusTypeDef ADXL345_ReadAccel(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az) {
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(hi2c, ADXL345_ADDR, ADXL345_REG_DATAX0,
                         I2C_MEMADD_SIZE_8BIT, buf, 6, 100) != HAL_OK)
        return HAL_ERROR;
    int16_t rx = (buf[1] << 8) | buf[0];  /* ADXL345: little-endian */
    int16_t ry = (buf[3] << 8) | buf[2];
    int16_t rz = (buf[5] << 8) | buf[4];
    /* Full-res mode: 4mg/LSB = 0.0039g/LSB */
    *ax = (float)rx * 0.0039f;
    *ay = (float)ry * 0.0039f;
    *az = (float)rz * 0.0039f;
    return HAL_OK;
}
