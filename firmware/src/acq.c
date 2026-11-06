#include "acq.h"
#include "hal.h"
#include "pdm_config.h"

#include <string.h>

static int16_t s_buf[2u * PDM_N * PDM_AXES];

static volatile uint8_t  s_ready;          /* bit h set = half h ready */
static volatile uint8_t  s_newest;         /* last half completed      */
static volatile uint32_t s_completions;
static volatile uint32_t s_overruns;
static uint32_t s_taken;

/* ---- interrupt context: keep it to a few instructions ---- */
static void complete(uint8_t h)
{
    s_completions++;
    if (s_ready & (1u << h)) {
        s_overruns++;                       /* previous data in this half never consumed */
    }
    s_ready = (uint8_t)(s_ready | (1u << h));
    s_newest = h;
}

void acq_isr_half(void) { complete(0); }
void acq_isr_full(void) { complete(1); }

/* ---- thread (main loop) context ---- */
void acq_start(void)
{
    memset(s_buf, 0, sizeof(s_buf));
    s_ready = 0;
    s_newest = 1;
    s_completions = 0;
    s_overruns = 0;
    s_taken = 0;
    hal_acq_dma_start(s_buf, 2u * PDM_N, acq_isr_half, acq_isr_full);
}

void acq_stop(void)
{
    hal_acq_dma_stop();
}

bool acq_take(acq_window_t *w)
{
    hal_irq_disable();
    const uint8_t ready = s_ready;
    if (ready == 0u) {
        hal_irq_enable();
        return false;
    }
    /* If both halves are ready we already overran; the older half is the one
     * DMA is writing now, so only the newest is safe to use. */
    const uint8_t h = s_newest;
    if (ready == 0x03u) {
        s_overruns++;                        /* older half lost unprocessed */
    }
    s_ready = 0;
    w->completions_at_take = s_completions;
    hal_irq_enable();

    w->half = h;
    w->frames = &s_buf[(uint32_t)h * PDM_N * PDM_AXES];
    w->seq = w->completions_at_take;
    s_taken++;
    return true;
}

bool acq_still_valid(const acq_window_t *w)
{
    /* When a half is taken, DMA is filling the other half. Once that one
     * completes, DMA starts overwriting ours. So the window is intact only
     * if no further half has completed since we took it. */
    return s_completions == w->completions_at_take;
}

acq_stats_t acq_stats(void)
{
    acq_stats_t s;
    s.completions = s_completions;
    s.overruns = s_overruns;
    s.taken = s_taken;
    return s;
}

const int16_t *acq_buffer(void)
{
    return s_buf;
}
