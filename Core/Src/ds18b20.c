#include "ds18b20.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

/* DWT cycle counter for microsecond delays */
/* HSI 16MHz: 1us = 16 cycles (170→16, PLL 미사용에 맞춤) */
#define US_CYCLES  16U

static inline void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * US_CYCLES;
    while ((DWT->CYCCNT - start) < cycles);
}

/* GPIO helpers for 1-Wire bit-bang on PB3 — open-drain + 직접 레지스터 (빠름, 모드 전환 없음).
 * 외부 10kΩ 풀업이 HIGH를 당김. OUTPUT_OD: ODR=0 → LOW 구동, ODR=1 → release(high-Z).
 * IDR은 출력 모드에서도 핀 실제 레벨 반영 → 읽기 가능. (예전 HAL_GPIO_Ini per-bit는 느려 타이밍 붕괴) */
static void ow_pin_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = ONEWIRE_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;            /* 외부 10kΩ 풀업 사용 */
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ONEWIRE_PORT, &g);
    ONEWIRE_PORT->BSRR = (uint32_t)ONEWIRE_PIN;   /* release */
}
static inline void ow_pin_low(void)    { ONEWIRE_PORT->BSRR = (uint32_t)ONEWIRE_PIN << 16; } /* RESET → LOW */
static inline void ow_pin_release(void){ ONEWIRE_PORT->BSRR = (uint32_t)ONEWIRE_PIN; }        /* SET → release */
static inline uint8_t ow_pin_read(void){ return (ONEWIRE_PORT->IDR & ONEWIRE_PIN) ? 1 : 0; }

/* 1-Wire reset */
static uint8_t ow_reset(void) {
    ow_pin_low();
    delay_us(480);
    ow_pin_release();
    delay_us(70);
    uint8_t presence = ow_pin_read();  /* 0 = device present */
    delay_us(410);
    return presence ? 0 : 1;  /* return 1 if present */
}

/* Write one bit */
static void ow_write_bit(uint8_t bit) {
    ow_pin_low();
    delay_us(6);
    if (bit) {
        ow_pin_release();
    }
    delay_us(64);
    ow_pin_release();
    delay_us(10);
}

/* Read one bit — 마스터 low를 짧게(2us) 하고 약 10us에서 샘플링.
 * DS18B20 데이터 유효구간 1~15us 안에서 마진 확보 (예전 15us 샘플링은 끝이라 0xFF 읽음). */
static uint8_t ow_read_bit(void) {
    ow_pin_low();
    delay_us(2);
    ow_pin_release();
    delay_us(8);
    uint8_t b = ow_pin_read();
    delay_us(55);
    return b;
}

/* Write one byte LSB first */
static void ow_write_byte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        ow_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

/* Read one byte LSB first */
static uint8_t ow_read_byte(void) {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (ow_read_bit()) {
            byte |= (1 << i);
        }
    }
    return byte;
}

/* Public: initialize PB3 (open-drain) + DWT */
void DS18B20_Init(void) {
    ow_pin_init();   /* PB3 OUTPUT_OD + 외부 풀업, 한 번만 설정 (이후 모드 전환 없음) */
    dwt_init();
}

/* Public: read temperature from DS18B20 */
uint8_t DS18B20_ReadTemp(float *temp) {
    if (!ow_reset()) return 0;

    ow_write_byte(0xCC);  /* Skip ROM */
    ow_write_byte(0x44);  /* Convert T */
    vTaskDelay(pdMS_TO_TICKS(750));  /* Wait for conversion */

    if (!ow_reset()) return 0;

    ow_write_byte(0xCC);  /* Skip ROM */
    ow_write_byte(0xBE);  /* Read scratchpad */

    uint8_t lsb = ow_read_byte();
    uint8_t msb = ow_read_byte();
    int16_t raw = (int16_t)((msb << 8) | lsb);

    /* 12-bit default: 1 LSB = 0.0625 degC */
    *temp = (float)raw * 0.0625f;
    return 1;
}
