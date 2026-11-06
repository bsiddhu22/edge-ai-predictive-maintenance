#include "dsp.h"
#include "accel.h"

#include <math.h>

#define HALF (PDM_N / 2u)
static const float PI_F = 3.14159265358979f;
static const float EPS = 1e-6f;

static float    s_hann[PDM_N];
static float    s_tw_re[HALF], s_tw_im[HALF];
static uint16_t s_bitrev[PDM_N];
static float    s_win_power;           /* sum(w^2) */

static float    s_re[PDM_N], s_im[PDM_N];
static float    s_psd[HALF];

void dsp_init(void)
{
    s_win_power = 0.0f;
    for (unsigned i = 0; i < PDM_N; i++) {
        s_hann[i] = 0.5f - 0.5f * cosf(2.0f * PI_F * (float)i / (float)PDM_N);   /* periodic Hann */
        s_win_power += s_hann[i] * s_hann[i];
    }
    for (unsigned k = 0; k < HALF; k++) {
        s_tw_re[k] = cosf(2.0f * PI_F * (float)k / (float)PDM_N);
        s_tw_im[k] = -sinf(2.0f * PI_F * (float)k / (float)PDM_N);
    }
    for (unsigned i = 0; i < PDM_N; i++) {
        unsigned r = 0;
        for (unsigned b = 0; b < PDM_LOG2N; b++) {
            if (i & (1u << b)) r |= 1u << (PDM_LOG2N - 1u - b);
        }
        s_bitrev[i] = (uint16_t)r;
    }
}

void dsp_fft(float *re, float *im)
{
    for (unsigned i = 0; i < PDM_N; i++) {
        const unsigned j = s_bitrev[i];
        if (j > i) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (unsigned len = 2; len <= PDM_N; len <<= 1) {
        const unsigned half = len >> 1;
        const unsigned step = PDM_N / len;
        for (unsigned start = 0; start < PDM_N; start += len) {
            for (unsigned k = 0; k < half; k++) {
                const float wr = s_tw_re[k * step], wi = s_tw_im[k * step];
                const unsigned a = start + k, b = a + half;
                const float xr = re[b] * wr - im[b] * wi;
                const float xi = re[b] * wi + im[b] * wr;
                re[b] = re[a] - xr;  im[b] = im[a] - xi;
                re[a] += xr;         im[a] += xi;
            }
        }
    }
}

static float band_energy(float f_lo, float f_hi)
{
    const float df = (float)PDM_FS_HZ / (float)PDM_N;
    float e = 0.0f;
    for (unsigned k = 1; k < HALF; k++) {
        const float f = (float)k * df;
        if (f >= f_lo && f < f_hi) e += s_psd[k];
    }
    return e;
}

void features_axis(const int16_t *frames, unsigned axis, float out[PDM_FEAT_PER_AXIS])
{
    const float scale = 1.0f / ACCEL_LSB_PER_G;

    /* De-interleave straight from the DMA half-buffer (zero copy) */
    float mean = 0.0f;
    for (unsigned i = 0; i < PDM_N; i++) {
        s_re[i] = (float)frames[i * PDM_AXES + axis] * scale;
        mean += s_re[i];
    }
    mean /= (float)PDM_N;

    /* Time-domain statistics on the DC-removed signal */
    float m2 = 0.0f, m4 = 0.0f, peak = 0.0f;
    for (unsigned i = 0; i < PDM_N; i++) {
        const float x = s_re[i] - mean;
        const float x2 = x * x;
        m2 += x2;
        m4 += x2 * x2;
        const float ax = fabsf(x);
        if (ax > peak) peak = ax;
        s_re[i] = x * s_hann[i];            /* window in place for the FFT */
        s_im[i] = 0.0f;
    }
    m2 /= (float)PDM_N;
    m4 /= (float)PDM_N;
    const float rms = sqrtf(m2);

    dsp_fft(s_re, s_im);

    const float norm = 2.0f / ((float)PDM_N * s_win_power);
    unsigned kmax = 1;
    s_psd[0] = 0.0f;
    for (unsigned k = 1; k < HALF; k++) {
        s_psd[k] = (s_re[k] * s_re[k] + s_im[k] * s_im[k]) * norm;
        if (s_psd[k] > s_psd[kmax]) kmax = k;
    }

    out[0] = log10f(rms + EPS);
    out[1] = peak / (rms + EPS);
    out[2] = m4 / (m2 * m2 + EPS);
    out[3] = log10f(band_energy(BAND_1X_LO, BAND_1X_HI) + EPS);
    out[4] = log10f(band_energy(BAND_2X_LO, BAND_2X_HI) + EPS);
    out[5] = log10f(band_energy(BAND_HARM_LO, BAND_HARM_HI) + EPS);
    out[6] = log10f(band_energy(BAND_HF_LO, BAND_HF_HI) + EPS);
    out[7] = (float)kmax * ((float)PDM_FS_HZ / (float)PDM_N) / 100.0f;
}

const float *dsp_last_spectrum(void)
{
    return s_psd;
}
