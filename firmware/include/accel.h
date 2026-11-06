/**
 * @file accel.h
 * @brief Register-level driver for an ADXL345-compatible 3-axis accelerometer
 *        on 4-wire SPI (mode 3). Full-resolution, +/-16 g, 3.2 kHz output rate.
 */
#ifndef ACCEL_H
#define ACCEL_H

#include <stdint.h>

#define ADXL_REG_DEVID        0x00u
#define ADXL_REG_BW_RATE      0x2Cu
#define ADXL_REG_POWER_CTL    0x2Du
#define ADXL_REG_DATA_FORMAT  0x31u
#define ADXL_REG_DATAX0       0x32u

#define ADXL_DEVID_VALUE      0xE5u
#define ADXL_RATE_3200HZ      0x0Fu
#define ADXL_POWER_MEASURE    0x08u
#define ADXL_FMT_FULLRES_16G  0x0Bu   /* FULL_RES | range = +/-16 g */

#define ADXL_SPI_READ         0x80u
#define ADXL_SPI_MULTIBYTE    0x40u

#define ACCEL_LSB_PER_G       256.0f  /* 3.9 mg/LSB in full-resolution mode */

typedef enum {
    ACCEL_OK = 0,
    ACCEL_ERR_ID,       /* identity register mismatch: wiring, SPI mode, part */
    ACCEL_ERR_VERIFY    /* configuration did not read back                    */
} accel_status_t;

accel_status_t accel_init(void);
uint8_t        accel_read_reg(uint8_t reg);
void           accel_write_reg(uint8_t reg, uint8_t value);
void           accel_read_xyz(int16_t out[3]);
uint8_t        accel_last_devid(void);

#endif /* ACCEL_H */
