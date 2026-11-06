/**
 * @file acq.h
 * @brief Double-buffered (ping-pong) acquisition on top of a circular DMA.
 *
 * The DMA half-transfer and transfer-complete interrupts only mark a half as
 * ready. The main loop takes the newest ready half and processes it in place
 * (zero copy) while DMA fills the other half. Two failure modes are detected:
 *  - overrun: a half completes again before the main loop consumed it
 *  - torn window: DMA wrapped back into the half while it was being processed
 *    (checked with acq_still_valid() after processing; the window is dropped)
 */
#ifndef ACQ_H
#define ACQ_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    const int16_t *frames;        /* PDM_N interleaved x,y,z samples */
    uint8_t        half;
    uint32_t       completions_at_take;
    uint32_t       seq;           /* window sequence number          */
} acq_window_t;

typedef struct {
    uint32_t completions;   /* half-buffers completed by DMA               */
    uint32_t overruns;      /* half completed while still unconsumed       */
    uint32_t taken;         /* windows handed to the main loop             */
} acq_stats_t;

void acq_start(void);
void acq_stop(void);
bool acq_take(acq_window_t *w);
bool acq_still_valid(const acq_window_t *w);
acq_stats_t acq_stats(void);

/* Exposed for unit tests (normally only called from the DMA ISRs). */
void acq_isr_half(void);
void acq_isr_full(void);
const int16_t *acq_buffer(void);

#endif /* ACQ_H */
