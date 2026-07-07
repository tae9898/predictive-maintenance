#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include "stm32g4xx_hal.h"

HAL_StatusTypeDef UART_DebugInit(UART_HandleTypeDef *huart);
void Debug_Print(const char *fmt, ...);

#endif /* __UART_DEBUG_H */
