/**
 * @file hal.h
 * @brief Hardware abstraction layer. The only way firmware touches hardware.
 *
 * Implemented by sim/sim_hal.c for software-in-the-loop runs, and by
 * tests/unit/test_hal.c for unit tests. On an STM32 these map to SPI, TIM+DMA,
 * USART, IWDG and the DWT cycle counter.
 */
#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- SPI to the accelerometer (chip select held for the whole transfer) ---- */
void hal_spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len);

/* ---- Acquisition DMA ----
 * Starts a timer-triggered DMA stream that reads one 3-axis sample (6 bytes)
 * from the sensor every 1/fs and stores it as 3 int16 in @p buf, circularly.
 * @p half_cb fires when the first half is full, @p full_cb when the second is.
 * Both run in interrupt context. */
typedef void (*hal_isr_cb_t)(void);
void hal_acq_dma_start(int16_t *buf, uint32_t frames_total,
                       hal_isr_cb_t half_cb, hal_isr_cb_t full_cb);
void hal_acq_dma_stop(void);

/* ---- Time ---- */
uint32_t hal_cycles(void);              /* free-running CPU cycle counter (DWT->CYCCNT) */

/**
 * SIL timing model: declares that the code just executed would take
 * @p cycles on the target. The simulator advances virtual time (and lets
 * DMA/ISRs run). Compiles to nothing on real hardware, where time passes
 * by itself and the same perf hooks read the real cycle counter.
 */
void hal_cpu_cost(uint32_t cycles);

/** Sleep until the next interrupt (WFI). */
void hal_wfi(void);

/* ---- Critical sections ---- */
void hal_irq_disable(void);
void hal_irq_enable(void);

/* ---- Debug/telemetry UART (blocking) ---- */
void hal_uart_init(uint32_t baud);
void hal_uart_write(const uint8_t *data, size_t len);

/* ---- Watchdog ---- */
void hal_watchdog_kick(void);

#endif /* HAL_H */
