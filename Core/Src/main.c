/**
 * @file    main.c
 * @brief   Predictive Maintenance — Phase 2a: 진동 분석 파이프라인
 * @note    ADXL345(z축) 1kHz 샘플링(TIM6 ISR 동기) → 256-pt FFT → 특징 추출
 *          MPU6050(I2C1), DS18B20(1-Wire PB3). HSI 16MHz.
 *          I2C1은 vVibTask(ADXL)만 사용 — vBgTask는 DS18B20(1-Wire)만 담당(I2C 경합 회피).
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_debug.h"
#include "mpu6050.h"
#include "adxl345.h"
#include "ds18b20.h"
#include "dsp.h"
#include "anomaly.h"
#include <stdio.h>

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim6;

extern DMA_HandleTypeDef hdma_usart2_tx;

static TaskHandle_t xVibTaskHandle = NULL;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);

/* ADXL345 z축 ~1kHz 샘플링(256샘플) → FFT → 특징 → 이상탐지 → 시리얼 출력 (~1Hz)
 * TIM6 ISR 경로는 이 보드에서 폴트/플러드 → vTaskDelayUntil 페이싱 사용(정밀화 TODO).
 * Phase 2b: 부팅 후 20윈도우 베이스라인 학습 → 이후 매 윈도우 임계 초과 시 ANOMALY. */
static void vVibTask(void *pv) {
    (void)pv;
    static float samples[DSP_N];        /* 1KB (.bss) — 스택 아님 */
    dsp_features_t f;
    char reason[24];
    anom_state_t prev = ANOM_CALIB;
    TickType_t last = xTaskGetTickCount();
    anomaly_reset();
    for (;;) {
        for (int i = 0; i < DSP_N; i++) {
            vTaskDelayUntil(&last, pdMS_TO_TICKS(1));   /* 평균 1ms 주기 (RTOS 틱) */
            float x, y, z;
            ADXL345_ReadAccel(&hi2c1, &x, &y, &z);
            samples[i] = z * 1000.0f;   /* mg 단위 저장 */
        }
        /* UART 'r'/'c' 입력 → 베이스라인 재학습 (echo r > /dev/ttyACM0) */
        if (USART2->ISR & USART_ISR_RXNE) {
            char c = (char)USART2->RDR;
            while (USART2->ISR & USART_ISR_RXNE) (void)USART2->RDR;
            if (c == 'r' || c == 'c') { anomaly_reset(); Debug_Print("[CALIB] re-calibrating...\r\n"); prev = ANOM_CALIB; }
        }
        dsp_extract(samples, DSP_N, 1000.0f, &f);
        anom_state_t st = anomaly_eval(&f, reason, sizeof reason);
        if (st == ANOM_CALIB) {
            Debug_Print("[CALIB] %d/%d  rms=%.0f kurt=%.2f  (keep still)\r\n",
                anomaly_count(), ANOM_BASELINE_N, (double)f.rms, (double)f.kurtosis);
        } else {
            const char *tag = (st == ANOM_NORMAL) ? "NORMAL" : "**ANOMALY**";
            Debug_Print("[VIB] rms=%.0f peak=%.0f kurt=%.2f crest=%.2f f0=%.1f | %s%s%s\r\n",
                (double)f.rms, (double)f.peak, (double)f.kurtosis, (double)f.crest, (double)f.f0,
                tag, reason[0] ? " " : "", reason);
            if (st == ANOM_ALERT && prev != ANOM_ALERT) {   /* rising edge → 명시적 알림 */
                float tr, tk; anomaly_thresholds(&tr, &tk);
                Debug_Print("[ALERT] *** ANOMALY *** rms=%.0f(thr %.0f) kurt=%.2f(thr %.2f)\r\n",
                    (double)f.rms, (double)tr, (double)f.kurtosis, (double)tk);
            }
        }
        if (st == ANOM_ALERT) LED_ON();   /* 이상: LED 켜짐 유지(경고) / 정상: 토글 */
        else LED_TOGGLE();
        prev = st;
    }
}

/* 백그라운드: DS18B20 온도만 (1-Wire → I2C 경합 없음) */
static void vBgTask(void *pv) {
    (void)pv;
    float temp = 0.0f;
    for (;;) {
        uint8_t ok = DS18B20_ReadTemp(&temp);   /* 변환 750ms 포함 */
        Debug_Print("[BG] T%d=%.2fC\r\n", (int)ok, (double)temp);
        vTaskDelay(pdMS_TO_TICKS(4000));        /* ~5s 주기 */
    }
}

/* I2C 버스 스캔 (ACK 디바이스 주소 나열) — 부팅 진단 */
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
    Debug_Print("[B] Vibration pipeline (Phase 2a)\r\n");

    dsp_init();                         /* twiddle/비트반전 테이블 */

    MX_I2C1_Init();
    MPU6050_Init(&hi2c1);               /* MPU6050은 초기화만 (진동 데모에선 정지) */
    ADXL345_Init(&hi2c1);               /* I2C1: MPU6050(0x68) + ADXL345(0x53) 공유 */
    Debug_Print("[B] I2C1 MPU6050+ADXL345 init done\r\n");
    i2c_scan(&hi2c1, "I2C1");           /* 정상: 0x68 + 0x53 */

    DS18B20_Init();
    Debug_Print("[B] DS18B20(1-Wire PB3) init done\r\n");

    xTaskCreate(vVibTask, "Vib", 1536, NULL, 2, &xVibTaskHandle);
    xTaskCreate(vBgTask,  "Bg",  512,  NULL, 1, NULL);
    vTaskStartScheduler();
    while(1);
}

void SystemClock_Config(void) {
    /* 리셋 기본 클럭(HSI 16MHz, 프리스케일러 /1) 그대로 사용 — 클럭 설정 생략.
     * 이유: HAL_RCC_OscConfig()가 HSI(현 SYSCLK 소스) 재설정 중 행함(이 보드 검증됨).
     * SystemCoreClock=16M(system_stm32g4xx.c)·configCPU_CLOCK_HZ=16M(FreeRTOSConfig.h) 일치. */
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gi = {0};
    gi.Pin = LED_PIN; gi.Mode = GPIO_MODE_OUTPUT_PP; gi.Pull = GPIO_NOPULL; gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gi);
    /* PB3 1-Wire (DS18B20) — DS18B20_Init에서 open-drain으로 재설정 */
    gi.Pin = ONEWIRE_PIN; gi.Mode = GPIO_MODE_INPUT; gi.Pull = GPIO_PULLUP; gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ONEWIRE_PORT, &gi);
}

static void MX_I2C1_Init(void) {
    /* I2C 버스 락업 복구: 슬레이브가 SDA를 잡고 있으면 SCL 9클럭으로 stuck 바이트를
     * 밀어내고 STOP 생성. (이전 400kHz 글리치 등으로 락업 시 MCU 리셋만으로는 안 풀림) */
    GPIO_InitTypeDef gr = {0};
    gr.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gr.Mode = GPIO_MODE_OUTPUT_OD; gr.Pull = GPIO_PULLUP; gr.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gr);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   /* SDA high */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        for (volatile int d = 0; d < 2000; d++);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 2000; d++);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) break;  /* SDA 풀림 */
    }
    /* STOP: SCL high에서 SDA low→high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 2000; d++);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    for (volatile int d = 0; d < 2000; d++);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    GPIO_InitTypeDef gi = {0};
    gi.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gi.Mode = GPIO_MODE_AF_OD;
    gi.Pull = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gi);

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x30100F12UL;   /* 100kHz @ HSI 16MHz (검증값). 400kHz는 풀업 2.35k 병렬에 플레키 → 안정 100kHz.
                                          * vTaskDelayUntil(1ms) 페이싱이면 100kHz(읽기≈0.8ms)도 1ms 슬롯 내 → ~1kHz 샘플링 */
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) while(1);
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
    htim6.Init.Prescaler = 16U - 1;       /* 16MHz / 16 = 1MHz 타이머 클럭 */
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1000U - 1;        /* 1MHz / 1000 = 1kHz 샘플링 */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim6);            /* MspInit에서 NVIC 우선순위 5 설정 (≤ configMAX_SYSCALL=5) */
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask; (void)pcTaskName; while(1);
}
void vApplicationMallocFailedHook(void) { while(1); }
void Error_Handler(void) { while(1) { LED_TOGGLE(); HAL_Delay(200); } }
