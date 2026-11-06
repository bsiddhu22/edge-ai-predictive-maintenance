/**
 * @file pdm_config.h
 * @brief Build-time configuration of the predictive-maintenance node.
 *
 * Target concept: STM32F4-class Cortex-M4F @ 168 MHz, 3-axis accelerometer
 * (ADXL345-compatible register map) on SPI, sampled at 3.2 kHz by a
 * timer-triggered DMA stream into a circular double buffer.
 */
#ifndef PDM_CONFIG_H
#define PDM_CONFIG_H

#define PDM_CPU_HZ        168000000u
#define PDM_FS_HZ         3200u          /* sample rate per axis            */
#define PDM_N             512u           /* samples per axis per window     */
#define PDM_LOG2N         9u
#define PDM_AXES          3u
#define PDM_FEAT_PER_AXIS 8u
#define PDM_N_FEATURES    (PDM_FEAT_PER_AXIS * PDM_AXES)

/* Window period = N / fs = 160 ms. Processing of one half-buffer must finish
 * before DMA comes back around to it: that is the real-time deadline. */
#define PDM_WINDOW_CYCLES ((PDM_CPU_HZ / PDM_FS_HZ) * PDM_N)

/* Spectral bands [Hz] for a ~1500 rpm (25 Hz) motor */
#define BAND_1X_LO    18.0f
#define BAND_1X_HI    32.0f
#define BAND_2X_LO    43.0f
#define BAND_2X_HI    57.0f
#define BAND_HARM_LO  70.0f
#define BAND_HARM_HI 160.0f
#define BAND_HF_LO   500.0f
#define BAND_HF_HI  1500.0f

/* Health state machine */
#define HEALTH_HISTORY     8u     /* windows considered                 */
#define HEALTH_SUSPECT_K   3u     /* abnormal windows to enter SUSPECT  */
#define HEALTH_ALARM_K     6u     /* abnormal windows to enter ALARM    */
#define HEALTH_CLEAR_K     1u     /* at most this many to drop back     */
#define HEALTH_MIN_CONF    0.60f  /* min class confidence to count      */

#define TELEMETRY_EVERY_WINDOWS 6u   /* ~1 s */

#endif /* PDM_CONFIG_H */
