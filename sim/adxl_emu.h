/**
 * @file adxl_emu.h
 * @brief Register-level emulation of an ADXL345-compatible accelerometer on
 *        SPI: register file, read/write and multi-byte protocol, 13-bit
 *        full-resolution data conversion with saturation.
 */
#ifndef ADXL_EMU_H
#define ADXL_EMU_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t reg[64];
    uint32_t transactions;
} adxl_emu_t;

void    adxl_emu_init(adxl_emu_t *e, uint8_t devid);
void    adxl_emu_set_accel(adxl_emu_t *e, const double g[3]);
void    adxl_emu_spi(adxl_emu_t *e, const uint8_t *tx, uint8_t *rx, size_t len);
int16_t adxl_emu_g_to_lsb(double g);

#endif /* ADXL_EMU_H */
