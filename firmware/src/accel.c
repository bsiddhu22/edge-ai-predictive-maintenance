#include "accel.h"
#include "hal.h"

static uint8_t s_devid;

uint8_t accel_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(ADXL_SPI_READ | (reg & 0x3Fu)), 0x00u };
    uint8_t rx[2] = { 0, 0 };
    hal_spi_xfer(tx, rx, 2);
    return rx[1];
}

void accel_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x3Fu), value };
    uint8_t rx[2];
    hal_spi_xfer(tx, rx, 2);
}

void accel_read_xyz(int16_t out[3])
{
    uint8_t tx[7] = { (uint8_t)(ADXL_SPI_READ | ADXL_SPI_MULTIBYTE | ADXL_REG_DATAX0), 0, 0, 0, 0, 0, 0 };
    uint8_t rx[7] = { 0 };
    hal_spi_xfer(tx, rx, 7);
    for (int a = 0; a < 3; a++) {
        out[a] = (int16_t)((uint16_t)rx[1 + 2 * a] | ((uint16_t)rx[2 + 2 * a] << 8));
    }
}

accel_status_t accel_init(void)
{
    /* Verify a known-constant register before trusting anything else:
     * a wrong SPI mode or a missing part shows up here, at boot. */
    s_devid = accel_read_reg(ADXL_REG_DEVID);
    if (s_devid != ADXL_DEVID_VALUE) {
        return ACCEL_ERR_ID;
    }

    accel_write_reg(ADXL_REG_POWER_CTL, 0x00u);              /* standby while configuring */
    accel_write_reg(ADXL_REG_BW_RATE, ADXL_RATE_3200HZ);
    accel_write_reg(ADXL_REG_DATA_FORMAT, ADXL_FMT_FULLRES_16G);
    accel_write_reg(ADXL_REG_POWER_CTL, ADXL_POWER_MEASURE);

    if (accel_read_reg(ADXL_REG_BW_RATE) != ADXL_RATE_3200HZ ||
        accel_read_reg(ADXL_REG_DATA_FORMAT) != ADXL_FMT_FULLRES_16G ||
        accel_read_reg(ADXL_REG_POWER_CTL) != ADXL_POWER_MEASURE) {
        return ACCEL_ERR_VERIFY;
    }
    return ACCEL_OK;
}

uint8_t accel_last_devid(void)
{
    return s_devid;
}
