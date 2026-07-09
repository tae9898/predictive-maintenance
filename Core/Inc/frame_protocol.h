#ifndef __FRAME_PROTOCOL_H
#define __FRAME_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>

/* STM32(predictive-maintenance) ↔ RPi3(sdv-gateway) UART 프레임 프로토콜 — Project 4 Phase 0.
 * 프레임: [MAGIC 0xA5][TYPE 1B][LEN 2B LE][PAYLOAD nB][CRC16 2B LE]
 *   - CRC16-CCITT (poly 0x1021, init 0xFFFF), CRC 범위 = TYPE|LEN|PAYLOAD (MAGIC 제외)
 *   - 모든 다중바이트 필드는 리틀엔디언 (STM32G4 = LE)
 *   - PAYLOAD 구조체는 #pragma pack(1) — 패딩 없음.
 * 문서: docs/uart-protocol.md
 */

#define FRAME_MAGIC      0xA5u
#define FRAME_HDR_LEN    4u        /* MAGIC + TYPE + LEN(2) */
#define FRAME_CRC_LEN    2u
#define FRAME_MAX_PAYLOAD 64u

typedef enum {
    FT_SENSOR_RAW     = 0x01,
    FT_FFT_RESULT     = 0x02,
    FT_FEATURE_VECTOR = 0x03,   /* 진동 특징 (매 윈도우) */
    FT_ANOMALY_ALERT  = 0x04,   /* 이상 발생 (rising edge) */
} frame_type_t;

#pragma pack(push, 1)
/* FT_FEATURE_VECTOR payload — 진동 특징 (현재 z축 기준). 정수 고정소수점. */
typedef struct {
    uint32_t ts_ms;          /* 부팅 후 ms (HAL_GetTick) */
    int16_t  rms;            /* mg (×1) */
    int16_t  peak;           /* mg */
    int16_t  peak2peak;      /* mg */
    int16_t  kurtosis_x100;  /* 첨도 ×100 (예: 1.5 → 150) */
    uint16_t crest_x100;     /* crest factor ×100 */
    uint16_t f0_x10;         /* 지배주파수 Hz ×10 */
    uint32_t band_low;       /* 0-50Hz 에너지 */
    uint32_t band_mid;       /* 50-200Hz */
    uint32_t band_high;      /* 200-500Hz */
    uint8_t  anomaly;        /* 0=정상, 1=이상 */
} frame_feature_t;           /* sizeof = 29 */

/* FT_ANOMALY_ALERT payload — 이상 발생 순간 */
typedef struct {
    uint32_t ts_ms;
    int16_t  rms;            /* mg */
    int16_t  kurtosis_x100;
    int16_t  thr_rms;        /* mg 임계 */
    int16_t  thr_kurt_x100;
    uint8_t  reason;         /* 0=rms 초과, 1=kurtosis 초과 */
} frame_anomaly_t;           /* sizeof = 13 */
#pragma pack(pop)

/* CRC16-CCITT (0x1021, init 0xFFFF) */
uint16_t frame_crc16(const uint8_t *data, size_t len);

/* 프레임 인코딩. buf에 [MAGIC|TYPE|LEN|PAYLOAD|CRC] 기록. 반환=전체 프레임 길이(0=버퍼 부족). */
size_t frame_encode(uint8_t *buf, size_t bufsize, frame_type_t type,
                    const void *payload, uint16_t payload_len);

#endif
