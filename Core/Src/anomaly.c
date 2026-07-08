/*
 * anomaly.c — 진동 이상탐지 (Phase 2b)
 * 부팅 후 ANOM_BASELINE_N 윈도우 동안 정상 RMS/kurtosis의 mean+std 학습 →
 * 임계(mean+3σ, 최소 마진 clamp). 이후 매 윈도우 초과 시 ALERT.
 */
#include "anomaly.h"
#include <math.h>
#include <stdio.h>

static int    g_n;                  /* 학습된 윈도우 수 */
static double g_sum_rms, g_sq_rms;  /* RMS 합/제곱합 */
static double g_sum_k,  g_sq_k;     /* kurtosis 합/제곱합 */
static float  g_thr_rms, g_thr_kurt;

void anomaly_reset(void) {
    g_n = 0;
    g_sum_rms = g_sq_rms = g_sum_k = g_sq_k = 0;
    g_thr_rms = g_thr_kurt = 0.0f;
}

int anomaly_count(void) { return g_n; }

void anomaly_thresholds(float *rms_thr, float *kurt_thr) {
    if (rms_thr)  *rms_thr  = g_thr_rms;
    if (kurt_thr) *kurt_thr = g_thr_kurt;
}

anom_state_t anomaly_eval(const dsp_features_t *f, char *reason, int rl) {
    if (reason && rl > 0) reason[0] = '\0';

    /* 1) 베이스라인 학습 */
    if (g_n < ANOM_BASELINE_N) {
        g_n++;
        g_sum_rms += f->rms;        g_sq_rms += (double)f->rms * f->rms;
        g_sum_k   += f->kurtosis;   g_sq_k   += (double)f->kurtosis * f->kurtosis;
        if (g_n == ANOM_BASELINE_N) {
            float mr = (float)(g_sum_rms / ANOM_BASELINE_N);
            float sr = (float)sqrt(g_sq_rms / ANOM_BASELINE_N - (double)mr * mr);
            float mk = (float)(g_sum_k   / ANOM_BASELINE_N);
            float sk = (float)sqrt(g_sq_k   / ANOM_BASELINE_N - (double)mk * mk);
            /* 임계 = mean + 3σ, 단 너무 타이트해지지게 최소 마진 clamp */
            g_thr_rms  = mr + 3.0f * sr;  if (g_thr_rms  < mr * 1.3f) g_thr_rms  = mr * 1.3f;  /* RMS: 최소 +30% */
            g_thr_kurt = mk + 3.0f * sk;  if (g_thr_kurt < mk + 1.5f) g_thr_kurt = mk + 1.5f;  /* 첨도: 최소 +1.5 */
        }
        return ANOM_CALIB;
    }

    /* 2) 모니터링: 임계 초과 시 ALERT */
    if (f->rms > g_thr_rms) {
        if (reason) snprintf(reason, rl, "rms %.0f>%.0f", (double)f->rms, (double)g_thr_rms);
        return ANOM_ALERT;
    }
    if (f->kurtosis > g_thr_kurt) {
        if (reason) snprintf(reason, rl, "kurt %.1f>%.1f", (double)f->kurtosis, (double)g_thr_kurt);
        return ANOM_ALERT;
    }
    return ANOM_NORMAL;
}
