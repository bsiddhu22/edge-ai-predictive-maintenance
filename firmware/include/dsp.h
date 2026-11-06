/**
 * @file dsp.h
 * @brief Fixed-size signal processing: Hann window, in-place radix-2 FFT and
 *        per-axis vibration features. All buffers are static; no heap.
 *
 * Feature vector per axis (8 values), identical to ml/features.py:
 *   0 log10(rms)          time domain, DC removed, in g
 *   1 crest factor        peak / rms
 *   2 kurtosis            impulsiveness (bearing defects)
 *   3 log10(E 1x band)    imbalance
 *   4 log10(E 2x band)    misalignment
 *   5 log10(E 3x-6x band) harmonics
 *   6 log10(E 0.5-1.5k)   high-frequency resonance (bearing impacts)
 *   7 dominant freq / 100 Hz
 * Spectrum P[k] = |X[k]|^2 * 2 / (N * sum(w^2)), k = 1..N/2-1.
 */
#ifndef DSP_H
#define DSP_H

#include "pdm_config.h"
#include <stdint.h>

void dsp_init(void);

/** In-place complex FFT of length PDM_N (re/im arrays). */
void dsp_fft(float *re, float *im);

/** Features for one axis of an interleaved x,y,z frame buffer. */
void features_axis(const int16_t *frames, unsigned axis, float out[PDM_FEAT_PER_AXIS]);

/** Last computed power spectrum (N/2 bins), for diagnostics/plots. */
const float *dsp_last_spectrum(void);

#endif /* DSP_H */
