/**
 * @file telemetry.h
 * @brief NMEA-style status sentences over UART, with XOR checksum:
 *   $PDM,<seq>,<health>,<class>,<conf%>,<fault>,<valid>,<dropped>,<overruns>,<load%%>*CS\r\n
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t seq;
    const char *health;
    const char *cls;
    uint32_t conf_pct;
    const char *fault;
    uint32_t valid_windows;
    uint32_t dropped_windows;
    uint32_t overruns;
    uint32_t load_permille;
} telemetry_t;

/** Format a sentence into @p buf. @return length (excluding NUL). */
size_t   telemetry_format(const telemetry_t *t, char *buf, size_t cap);
uint8_t  telemetry_checksum(const char *s, size_t len);

#endif /* TELEMETRY_H */
