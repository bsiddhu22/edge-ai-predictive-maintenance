/**
 * @file perf.h
 * @brief Per-stage execution-time accounting from the cycle counter, plus
 *        idle time for CPU load. On target the counter is DWT->CYCCNT.
 */
#ifndef PERF_H
#define PERF_H

#include <stdint.h>

typedef enum {
    PERF_FEATURES = 0,
    PERF_INFER,
    PERF_HEALTH,
    PERF_TELEMETRY,
    PERF_DEBUG_DUMP,
    PERF_WINDOW_TOTAL,
    PERF_N_STAGES
} perf_stage_t;

typedef struct {
    uint32_t last, max;
    uint64_t sum;
    uint32_t count;
} perf_stat_t;

typedef struct {
    perf_stat_t stage[PERF_N_STAGES];
    uint64_t    idle_cycles;
    uint64_t    busy_cycles;
} perf_t;

void     perf_init(void);
uint32_t perf_begin(void);
void     perf_end(perf_stage_t s, uint32_t start);
void     perf_add_idle(uint32_t cycles);
void     perf_add_busy(uint32_t cycles);
const perf_t *perf_get(void);
uint32_t perf_load_permille(void);
const char *perf_stage_name(perf_stage_t s);

#endif /* PERF_H */
