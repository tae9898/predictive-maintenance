/**
 * @file    stm32g4xx_it.c
 * @brief   Interrupt handlers - TIM6 + FreeRTOS SysTick
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern TIM_HandleTypeDef htim6;
extern void xPortSysTickHandler(void);

/**
 * @brief  TIM6/DAC interrupt handler
 */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

/**
 * @brief  SysTick handler (HAL + FreeRTOS shared)
 * @note   Order: HAL_IncTick first (fast), xPortSysTickHandler last (may context switch)
 *         PendSV_Handler and SVC_Handler are defined by FreeRTOS port.c.
 */
void SysTick_Handler(void)
{
    HAL_IncTick();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}
