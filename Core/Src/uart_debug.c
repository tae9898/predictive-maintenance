#include "uart_debug.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>

HAL_StatusTypeDef UART_DebugInit(UART_HandleTypeDef *huart) {
    huart->Instance = USART2;
    huart->Init.BaudRate = 115200U;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    setvbuf(stdout, NULL, _IONBF, 0);
    return HAL_UART_Init(huart);
}

void Debug_Print(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uint16_t len = 0;
    while (buf[len] && len < sizeof(buf)) len++;
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}
