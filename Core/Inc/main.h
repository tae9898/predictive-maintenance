#ifndef __MAIN_H
#define __MAIN_H

#include "stm32g4xx_hal.h"

/* Handles */
extern I2C_HandleTypeDef hi2c1;   /* MPU6050 */
extern I2C_HandleTypeDef hi2c2;   /* ADXL345 */
extern UART_HandleTypeDef huart2; /* Debug (ST-LINK VCP) */
extern TIM_HandleTypeDef htim6;   /* 1kHz sampling timer */

/* Pin defs */
#define LED_PIN       GPIO_PIN_5
#define LED_PORT      GPIOA
#define LED_ON()      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_OFF()     HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_TOGGLE()  HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

/* 1-Wire pin (DS18B20) */
#define ONEWIRE_PORT   GPIOB
#define ONEWIRE_PIN    GPIO_PIN_3

/* System clock (HSI 16MHz, PLL 미사용 — HAL RCC 행 회피) */
#define SYSCLK_FREQ    16000000U

void Error_Handler(void);

#endif /* __MAIN_H */
