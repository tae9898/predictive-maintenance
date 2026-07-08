#ifndef __ANOMALY_H
#define __ANOMALY_H
#include "dsp.h"

/* 이상탐지: 정상(베이스라인) 학습 → 임계 초과 시 경고.
 * Phase 2b. vVibTask가 매 윈도우(FFT 특징)마다 호출. */

#define ANOM_BASELINE_N 20   /* 학습 윈도우 수 (~5s @1Hz) */

typedef enum { ANOM_CALIB = 0, ANOM_NORMAL, ANOM_ALERT } anom_state_t;

/* 베이스라인 학습을 처음부터 다시(부팅 시 / UART 'r' 입력 시 호출) */
void anomaly_reset(void);

/* 학습 진행 횟수(0..ANOM_BASELINE_N). CALIB 단계 표시용 */
int  anomaly_count(void);

/* 매 윈도우 평가. CALIB 중이면 학습 누적 후 CALIB 반환;
 * 모니터링 중이면 임계 초과 시 ALERT(reason에 "rms .." / "kurt .." 기입), 아니면 NORMAL */
anom_state_t anomaly_eval(const dsp_features_t *f, char *reason, int reason_len);

/* 현재 임계값 조회(출력용). 학습 완료 전엔 0 */
void anomaly_thresholds(float *rms_thr, float *kurt_thr);

#endif
