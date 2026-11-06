/**
 * @file app.h
 * @brief Bare-metal application: init, then a super-loop that sleeps (WFI)
 *        until the DMA interrupts hand it a window, and processes it:
 *
 *   take half-buffer -> [optional raw dump] -> features (x,y,z)
 *   -> deadline check -> int8 inference -> health state machine -> telemetry
 */
#ifndef APP_H
#define APP_H

#include "accel.h"
#include "health.h"
#include "pdm_config.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     debug_dump_raw;     /* stream every raw window over UART (data collection) */
    uint32_t uart_baud;
    bool     validate_deadline;  /* drop windows DMA overwrote during processing        */
} app_cfg_t;

typedef struct {
    uint32_t seq;
    bool     valid;
    int      cls;
    float    conf;
    float    probs[4];
    health_state_t health;
    const float *features;
} app_window_report_t;

typedef struct {
    accel_status_t accel;
    health_t  health;
    uint32_t  valid_windows;
    uint32_t  dropped_windows;
    uint32_t  abnormal_windows;
    uint32_t  class_counts[4];
    uint32_t  telemetry_sent;
    int       last_cls;
    float     last_conf;
} app_status_t;

typedef void (*app_window_hook_t)(const app_window_report_t *r);

void app_default_cfg(app_cfg_t *cfg);
void app_init(const app_cfg_t *cfg);
void app_loop_once(void);
void app_set_window_hook(app_window_hook_t hook);
const app_status_t *app_status(void);

/* Estimated Cortex-M4F cycle costs used by the SIL timing model. */
#define COST_FEATURES_AXIS  ((PDM_N * 16u) + ((PDM_N / 2u) * PDM_LOG2N * 12u) + (PDM_N * 10u))
#define COST_INFER_FIXED    1200u
#define COST_HEALTH         300u
#define COST_TELEMETRY_FMT  4000u

#endif /* APP_H */
