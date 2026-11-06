#include "perf.h"
#include "hal.h"

#include <string.h>

static perf_t s_perf;

void perf_init(void)
{
    memset(&s_perf, 0, sizeof(s_perf));
}

uint32_t perf_begin(void)
{
    return hal_cycles();
}

void perf_end(perf_stage_t s, uint32_t start)
{
    const uint32_t dt = hal_cycles() - start;      /* wrap-safe unsigned subtraction */
    perf_stat_t *p = &s_perf.stage[s];
    p->last = dt;
    if (dt > p->max) p->max = dt;
    p->sum += dt;
    p->count++;
}

void perf_add_idle(uint32_t cycles) { s_perf.idle_cycles += cycles; }
void perf_add_busy(uint32_t cycles) { s_perf.busy_cycles += cycles; }

const perf_t *perf_get(void)
{
    return &s_perf;
}

uint32_t perf_load_permille(void)
{
    const uint64_t total = s_perf.idle_cycles + s_perf.busy_cycles;
    return total ? (uint32_t)((s_perf.busy_cycles * 1000u) / total) : 0u;
}

const char *perf_stage_name(perf_stage_t s)
{
    static const char *const n[PERF_N_STAGES] = {
        "features", "inference", "health", "telemetry", "debug_dump", "window_total"
    };
    return ((unsigned)s < PERF_N_STAGES) ? n[s] : "?";
}
