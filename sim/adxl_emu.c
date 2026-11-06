#include "adxl_emu.h"

#include <math.h>
#include <string.h>

void adxl_emu_init(adxl_emu_t *e, uint8_t devid)
{
    memset(e, 0, sizeof(*e));
    e->reg[0x00] = devid;
    e->reg[0x2C] = 0x0A;   /* reset default: 100 Hz */
    e->reg[0x2D] = 0x00;   /* standby */
    e->reg[0x31] = 0x00;
}

int16_t adxl_emu_g_to_lsb(double g)
{
    double v = floor(g * 256.0 + 0.5);       /* full-res: 3.9 mg/LSB */
    if (v > 4095.0) v = 4095.0;              /* 13-bit signed, +/-16 g */
    if (v < -4096.0) v = -4096.0;
    return (int16_t)v;
}

void adxl_emu_set_accel(adxl_emu_t *e, const double g[3])
{
    for (int a = 0; a < 3; a++) {
        int16_t v = (e->reg[0x2D] & 0x08u) ? adxl_emu_g_to_lsb(g[a]) : 0;   /* zeros in standby */
        e->reg[0x32 + 2 * a] = (uint8_t)((uint16_t)v & 0xFFu);
        e->reg[0x33 + 2 * a] = (uint8_t)((uint16_t)v >> 8);
    }
}

void adxl_emu_spi(adxl_emu_t *e, const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (len == 0) return;
    e->transactions++;
    const int read = (tx[0] & 0x80u) != 0;
    const int multi = (tx[0] & 0x40u) != 0;
    uint8_t addr = tx[0] & 0x3Fu;
    rx[0] = 0xFF;
    for (size_t i = 1; i < len; i++) {
        if (read) {
            rx[i] = e->reg[addr];
        } else {
            rx[i] = 0xFF;
            if (addr != 0x00 && !(addr >= 0x32 && addr <= 0x37)) {   /* read-only regs */
                e->reg[addr] = tx[i];
            }
        }
        if (multi) addr = (uint8_t)((addr + 1u) & 0x3Fu);
    }
}
