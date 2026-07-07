/**
 * @file    main.c
 * @brief   Predictive Maintenance - Sensor Data Collection (깔끔한 재작성)
 * @note    I2C1: PB8(SCL) PB9(SDA) = Arduino CN5
 *          MPU6050 @ 0x68
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_debug.h"
#include "mpu6050.h"
#include "adxl345.h"
#include "ds18b20.h"
#include <stdio.h>

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim6;

extern DMA_HandleTypeDef hdma_usart2_tx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);

static void vSensorTask(void *pvParameters) {
    float mx=0,my=0,mz=0, ax=0,ay=0,az=0, temp=0;
    int n = 0;
    for(;;) {
        HAL_StatusTypeDef mst = MPU6050_ReadAccel(&hi2c1, &mx,&my,&mz);
        HAL_StatusTypeDef ast = ADXL345_ReadAccel(&hi2c1, &ax,&ay,&az);
        if ((++n % 25) == 0) {   /* 온도는 ~5s마다 (DS18B20 변환 750ms 블록 수반) */
            uint8_t tok = DS18B20_ReadTemp(&temp);
            Debug_Print("[S] MPU%d x=%d y=%d z=%d | ADXL%d X=%d Y=%d Z=%d | T%d=%.2fC\r\n",
                (int)mst, (int)(mx*1000),(int)(my*1000),(int)(mz*1000),
                (int)ast, (int)(ax*1000),(int)(ay*1000),(int)(az*1000),
                (int)tok, (double)temp);
        } else {
            Debug_Print("[S] MPU%d x=%d y=%d z=%d | ADXL%d X=%d Y=%d Z=%d\r\n",
                (int)mst, (int)(mx*1000),(int)(my*1000),(int)(mz*1000),
                (int)ast, (int)(ax*1000),(int)(ay*1000),(int)(az*1000));
        }
        LED_TOGGLE();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* 임시 진단: I2C 버스 스캔 (ACK 디바이스 주소 나열) */
static void i2c_scan(I2C_HandleTypeDef *hi2c, const char *name) {
    Debug_Print("[SCAN] %s:", name);
    uint8_t found = 0;
    for (uint8_t a = 1; a < 128; a++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(a << 1), 1, 10) == HAL_OK) {
            Debug_Print(" 0x%02X", a); found++;
        }
    }
    Debug_Print(found ? " (%u found)\r\n" : " (none)\r\n", (unsigned)found);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM6_Init();

    HAL_Delay(2000);
    Debug_Print("[B] Sensor Test v2\r\n");

    MX_I2C1_Init();
    MPU6050_Init(&hi2c1);
    ADXL345_Init(&hi2c1);   /* I2C1 공유: MPU6050(0x68) + ADXL345(0x53) 주소 충돌 없음 */
    Debug_Print("[B] I2C1(PB8/PB9) MPU6050+ADXL345 init done\r\n");
    i2c_scan(&hi2c1, "I2C1");   /* 정상: 0x68 + 0x53 */

    DS18B20_Init();
    Debug_Print("[B] DS18B20(1-Wire PB3) init done\r\n");

    xTaskCreate(vSensorTask, "Sensor", 1536, NULL, 2, NULL);
    vTaskStartScheduler();
    while(1);
}

void SystemClock_Config(void) {
    /* 리셋 기본 클럭(HSI 16MHz, 프리스케일러 /1) 그대로 사용 — 클럭 설정 생략.
     * 이유: HAL_RCC_OscConfig()가 HSI(현 SYSCLK 소스)를 재설정하며 클럭을 잃어 행함(이 보드 검증됨).
     * SystemCoreClock=16M는 system_stm32g4xx.c에서, configCPU_CLOCK_HZ=16M는 FreeRTOSConfig.h에서 일치시킴.
     * → HAL UART BRR·HAL_Delay·FreeRTOS 틱 모두 16MHz 기준으로 정확. */
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gi = {0};
    gi.Pin = LED_PIN; gi.Mode = GPIO_MODE_OUTPUT_PP; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gi);
    /* PB3 1-Wire (DS18B20) */
    gi.Pin = ONEWIRE_PIN; gi.Mode = GPIO_MODE_INPUT; gi.Pull = GPIO_PULLUP; gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ONEWIRE_PORT, &gi);
}

static void MX_I2C1_Init(void) {
    /* GPIO: PB8=SCL, PB9=SDA, AF4 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gi.Mode = GPIO_MODE_AF_OD;
    gi.Pull = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gi);

    /* DMA for USART2 TX (선택사항) */
    /* I2C init */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x30100F12UL;  /* ~100kHz @ HSI 16MHz 커널 (fortest 검증값, bare-metal I2C gotcha와 무관 — HAL Mem_Read) */
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) while(1);
}

static void MX_I2C2_Init(void) {
    /* GPIO: PB10=SCL, PB11=SDA, AF4 (ADXL345). HAL_I2C_MspInit에서도 세팅(중복 무해) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gi.Mode = GPIO_MODE_AF_OD;
    gi.Pull = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gi);

    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x30100F12UL;  /* ~100kHz @ HSI 16MHz */
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) while(1);
}

static void MX_USART2_UART_Init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gi.Mode = GPIO_MODE_AF_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gi);
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart2);
    setvbuf(stdout, NULL, _IONBF, 0);
}

static void MX_TIM6_Init(void) {
    __HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 42500U - 1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1000U - 1;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask; while(1);
}
void vApplicationMallocFailedHook(void) { while(1); }
void Error_Handler(void) { while(1) { LED_TOGGLE(); HAL_Delay(200); } }
