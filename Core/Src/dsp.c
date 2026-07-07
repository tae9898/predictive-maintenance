/*
 * dsp.c — 진동 분석 DSP (256-point radix-2 FFT + 특징 추출)
 * 의존성: libm (sinf/cosf/sqrtf/fabsf/fmaxf). CMSIS-DSP 불필요.
 */
#include "dsp.h"
#include <math.h>

#ifndef DSP_PI
#define DSP_PI 3.14159265358979323846f
#endif

/* 정적 테이블 (.bss). twiddle 128 복소수(cos/sin) + 비트반전 256 int ≈ 2KB */
static float tw_cos[DSP_N / 2];
static float tw_sin[DSP_N / 2];
static int   br[DSP_N];

void dsp_init(void)
{
    /* twiddle: W[k] = exp(-j*2π*k/N), k=0..N/2-1 */
    for (int i = 0; i < DSP_N / 2; i++) {
        float a = -2.0f * DSP_PI * (float)i / (float)DSP_N;
        tw_cos[i] = cosf(a);
        tw_sin[i] = sinf(a);
    }
    /* 비트반전 순서 (N=256 → 8비트) */
    for (int i = 0; i < DSP_N; i++) {
        int j = 0, x = i;
        for (int b = DSP_N; b > 1; b >>= 1) { j = (j << 1) | (x & 1); x >>= 1; }
        br[i] = j;
    }
}

void dsp_apply_hann(float *x, int n)
{
    for (int i = 0; i < n; i++)
        x[i] *= 0.5f * (1.0f - cosf(2.0f * DSP_PI * (float)i / (float)(n - 1)));
}

/* 인플레이스 radix-2 DIT FFT. 입력: re[] (실수), im[]=0 권장. */
void dsp_fft256(float *re, float *im)
{
    /* 1) 비트반전 재정렬 */
    for (int i = 0; i < DSP_N; i++) {
        int j = br[i];
        if (j > i) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
    /* 2) 버터플라이 스테이지: len = 2,4,...,N */
    for (int len = 2; len <= DSP_N; len <<= 1) {
        int half = len >> 1;
        int step = DSP_N / len;          /* twiddle 인덱스 증분 */
        for (int i = 0; i < DSP_N; i += len) {
            for (int k = 0; k < half; k++) {
                float wr = tw_cos[k * step];
                float wi = tw_sin[k * step];
                float xr = re[i + k + half];
                float xi = im[i + k + half];
                float tr = wr * xr - wi * xi;
                float ti = wr * xi + wi * xr;
                re[i + k + half] = re[i + k] - tr;
                im[i + k + half] = im[i + k] - ti;
                re[i + k] += tr;
                im[i + k] += ti;
            }
        }
    }
}

void dsp_extract(const float *sig, int n, float fs, dsp_features_t *o)
{
    /* --- DC(중력 등) 제거 + 시간영역 통계 --- */
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += sig[i];
    float mean = sum / (float)n;

    float s2 = 0.0f, s4 = 0.0f, mx = -1e30f, mn = 1e30f;
    for (int i = 0; i < n; i++) {
        float v = sig[i] - mean;
        float v2 = v * v;
        s2 += v2;
        s4 += v2 * v2;
        if (v > mx) mx = v;
        if (v < mn) mn = v;
    }
    float rms = sqrtf(s2 / (float)n);
    o->rms       = rms;
    o->peak      = fmaxf(fabsf(mx), fabsf(mn));
    o->peak2peak = mx - mn;
    /* 초과 첨도 = (E[X^4]/σ^4) − 3  (Gaussian ≈ 0) */
    if (rms > 1e-9f) {
        float k = (s4 / (float)n) / (rms * rms * rms * rms);
        o->kurtosis = k - 3.0f;
        o->crest    = o->peak / rms;
    } else {
        o->kurtosis = 0.0f;
        o->crest    = 0.0f;
    }

    /* --- 주파수영역: Hann → FFT → magnitude² --- */
    static float re[DSP_N], im[DSP_N];
    for (int i = 0; i < n; i++) { re[i] = sig[i] - mean; im[i] = 0.0f; }
    dsp_apply_hann(re, n);
    dsp_fft256(re, im);

    float binw = fs / (float)n;     /* Hz/bin (fs=1000, n=256 → 3.9Hz) */
    int   maxbin = 1;
    float maxmag = -1.0f;
    o->band_low = o->band_mid = o->band_high = 0.0f;
    for (int k = 1; k < n / 2; k++) {            /* DC(k=0) 제외 */
        float mag = re[k] * re[k] + im[k] * im[k]; /* 에너지 (sqrt 생략) */
        float f = k * binw;
        if (f <= 50.0f)       o->band_low  += mag;
        else if (f <= 200.0f) o->band_mid  += mag;
        else if (f <= 500.0f) o->band_high += mag;
        if (mag > maxmag) { maxmag = mag; maxbin = k; }
    }
    o->f0 = (float)maxbin * binw;
}
