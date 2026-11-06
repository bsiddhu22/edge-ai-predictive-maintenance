/* HAL backed by a virtual clock, an emulated accelerometer and a motor
 * model. Time only moves when firmware sleeps (WFI), declares CPU work
 * (hal_cpu_cost) or blocks on the UART; DMA samples and their interrupts are
 * delivered as virtual time passes, so ISRs "preempt" the main loop at those
 * points exactly as they would on target. Fully deterministic. */
#include "hal.h"
#include "pdm_config.h"
#include "sim_hal.h"

#include <string.h>

#define TICK_CYCLES ((uint64_t)(PDM_CPU_HZ / PDM_FS_HZ))

static sim_t *S;

static struct {
    int16_t     *buf;
    uint32_t     frames_total, idx;
    hal_isr_cb_t half_cb, full_cb;
    int          running;
} dma;

static int s_irq_fired;
static char s_line[256];

void sim_attach(sim_t *s, motor_t *m, adxl_emu_t *e)
{
    memset(s, 0, sizeof(*s));
    s->motor = m;
    s->emu = e;
    s->baud = 115200;
    memset(&dma, 0, sizeof(dma));
    S = s;
}

double sim_time_s(void)
{
    return (double)S->now / (double)PDM_CPU_HZ;
}

/* One sample period: motor -> sensor registers -> DMA SPI read -> RAM */
static void sample_tick(void)
{
    double g[3];
    const double t = (double)S->now / (double)PDM_CPU_HZ;
    if (S->tick_hook) S->tick_hook(t);
    motor_sample(S->motor, 1.0 / PDM_FS_HZ, g);
    adxl_emu_set_accel(S->emu, g);
    if (!dma.running) return;

    uint8_t tx[7] = { 0xC0u | 0x32u, 0, 0, 0, 0, 0, 0 }, rx[7];
    adxl_emu_spi(S->emu, tx, rx, 7);
    int16_t *f = &dma.buf[dma.idx * 3u];
    for (int a = 0; a < 3; a++) {
        f[a] = (int16_t)((uint16_t)rx[1 + 2 * a] | ((uint16_t)rx[2 + 2 * a] << 8));
    }
    dma.idx++;
    if (dma.idx == dma.frames_total / 2u) {
        s_irq_fired = 1;
        dma.half_cb();
    } else if (dma.idx == dma.frames_total) {
        dma.idx = 0;
        s_irq_fired = 1;
        dma.full_cb();
    }
}

static void advance(uint64_t cycles)
{
    const uint64_t target = S->now + cycles;
    while (S->next_tick <= target) {
        S->now = S->next_tick;
        sample_tick();
        S->next_tick += TICK_CYCLES;
    }
    S->now = target;
}

/* ------------------------------ HAL ------------------------------------ */
void hal_spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    adxl_emu_spi(S->emu, tx, rx, len);
    advance(8u * 16u * len);                       /* ~SPI at 10.5 MHz */
}

void hal_acq_dma_start(int16_t *buf, uint32_t frames_total, hal_isr_cb_t half_cb, hal_isr_cb_t full_cb)
{
    dma.buf = buf;
    dma.frames_total = frames_total;
    dma.idx = 0;
    dma.half_cb = half_cb;
    dma.full_cb = full_cb;
    dma.running = 1;
}

void hal_acq_dma_stop(void)       { dma.running = 0; }
uint32_t hal_cycles(void)         { return (uint32_t)S->now; }
void hal_cpu_cost(uint32_t c)     { advance(c); }
void hal_irq_disable(void)        { /* ISRs only run inside advance() */ }
void hal_irq_enable(void)         { }
void hal_watchdog_kick(void)      { S->wdg_kicks++; }
void hal_uart_init(uint32_t baud) { S->baud = baud; }

void hal_wfi(void)
{
    if (!dma.running) {
        advance(PDM_CPU_HZ / 1000u);               /* nothing to wake us: 1 ms */
        return;
    }
    s_irq_fired = 0;
    while (!s_irq_fired) {
        advance(S->next_tick - S->now);
    }
}

void hal_uart_write(const uint8_t *data, size_t len)
{
    /* Telemetry sentences are written in one call starting with '$';
     * anything else (raw window dumps) is binary and only counted. */
    if (len > 0 && data[0] == '$' && len < sizeof(s_line)) {
        memcpy(s_line, data, len);
        s_line[len] = '\0';
        if (S->uart_log) fprintf(S->uart_log, "%.3f %s", sim_time_s(), s_line);
        S->uart_lines++;
    }
    S->uart_bytes += len;
    /* blocking transmit: 10 bits per byte (8N1) */
    advance((uint64_t)len * 10u * PDM_CPU_HZ / S->baud);
}
