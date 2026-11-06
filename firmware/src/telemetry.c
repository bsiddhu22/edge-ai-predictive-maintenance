#include "telemetry.h"

#include <stdio.h>

uint8_t telemetry_checksum(const char *s, size_t len)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= (uint8_t)s[i];
    return cs;
}

size_t telemetry_format(const telemetry_t *t, char *buf, size_t cap)
{
    int n = snprintf(buf, cap, "$PDM,%lu,%s,%s,%lu,%s,%lu,%lu,%lu,%lu.%lu",
                     (unsigned long)t->seq, t->health, t->cls, (unsigned long)t->conf_pct,
                     t->fault, (unsigned long)t->valid_windows, (unsigned long)t->dropped_windows,
                     (unsigned long)t->overruns, (unsigned long)(t->load_permille / 10u),
                     (unsigned long)(t->load_permille % 10u));
    if (n < 0 || (size_t)n + 6u > cap) {
        return 0;
    }
    const uint8_t cs = telemetry_checksum(buf + 1, (size_t)n - 1u);   /* between '$' and '*' */
    n += snprintf(buf + n, cap - (size_t)n, "*%02X\r\n", cs);
    return (size_t)n;
}
