/* Minimal HAL for unit tests: SPI routed to the emulated accelerometer,
 * DMA callbacks captured so tests can fire "interrupts" by hand. */
#include "hal.h"
#include "adxl_emu.h"

adxl_emu_t   g_test_emu;
hal_isr_cb_t g_test_half_cb, g_test_full_cb;
int16_t     *g_test_dma_buf;
static uint32_t s_cycles;

void hal_spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len) { adxl_emu_spi(&g_test_emu, tx, rx, len); }

void hal_acq_dma_start(int16_t *buf, uint32_t frames_total, hal_isr_cb_t half_cb, hal_isr_cb_t full_cb)
{
    (void)frames_total;
    g_test_dma_buf = buf;
    g_test_half_cb = half_cb;
    g_test_full_cb = full_cb;
}

void hal_acq_dma_stop(void) {}
uint32_t hal_cycles(void) { return s_cycles; }
void hal_cpu_cost(uint32_t c) { s_cycles += c; }
void hal_wfi(void) { s_cycles += 1000u; }
void hal_irq_disable(void) {}
void hal_irq_enable(void) {}
void hal_uart_init(uint32_t baud) { (void)baud; }
void hal_uart_write(const uint8_t *d, size_t n) { (void)d; (void)n; }
void hal_watchdog_kick(void) {}
