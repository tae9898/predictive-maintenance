#ifndef __DS18B20_H
#define __DS18B20_H
#include "stm32g4xx_hal.h"
#include "main.h"

void    DS18B20_Init(void);
uint8_t DS18B20_ReadTemp(float *temp);  /* returns 1=ok, 0=fail */

#endif
