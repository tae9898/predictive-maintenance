#ifndef __DSP_H
#define __DSP_H
#include <stdint.h>

/* 진동 분석 DSP — 256-point radix-2 FFT + 진단 특징 추출.
 * 의존성 없음(CMSIS-DSP 미사용). HSI 16MHz + -O2에서 수 ms. */

#define DSP_N 256          /* FFT 크기 (2의 거듭제곱) */

typedef struct {
    /* 시간영역 (DC 제거 후, 단위 = 입력 신호 단위) */
    float rms;        /* 진동 심도 */
    float peak;       /* |최대| */
    float peak2peak;  /* 최대-최소 */
    float kurtosis;   /* 초과 첨도(excess) — Gaussian≈0, 충격성↑(베어링 조기고장 지표) */
    float crest;      /* peak/RMS (크레스트 팩터) */
    /* 주파수영역 (FFT magnitude² = 에너지) */
    float f0;         /* 지배 주파수 Hz (DC 제외) */
    float band_low;   /* 0~50 Hz 대역 에너지 */
    float band_mid;   /* 50~200 Hz */
    float band_high;  /* 200~500 Hz */
} dsp_features_t;

/* twiddle/비트반전 테이블 초기화 — 부팅 시 1회 */
void dsp_init(void);

/* Hann 창 인플레이스 적용 */
void dsp_apply_hann(float *x, int n);

/* 256-point radix-2 FFT (인플레이스, DIT). re/im 길이 DSP_N. */
void dsp_fft256(float *re, float *im);

/* 신호에서 특징 추출. sig[] 길이 n(=DSP_N 권장), fs 샘플링 주파수 Hz.
 * 내부적으로 DC 제거 → Hann → FFT → 특징. */
void dsp_extract(const float *sig, int n, float fs, dsp_features_t *out);

#endif /* __DSP_H */
