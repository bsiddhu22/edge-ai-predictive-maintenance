#ifndef SIM_HAL_H
#define SIM_HAL_H

#include "adxl_emu.h"
#include "motor_model.h"

#include <stdint.h>
#include <stdio.h>

typedef void (*sim_tick_hook_t)(double t_s);

typedef struct {
    motor_t    *motor;
    adxl_emu_t *emu;
    uint64_t    now;              /* virtual CPU cycles */
    uint64_t    next_tick;
    uint32_t    baud;
    FILE       *uart_log;         /* telemetry sentences */
    uint64_t    uart_bytes;
    uint32_t    uart_lines;
    uint32_t    wdg_kicks;
    sim_tick_hook_t tick_hook;    /* scenario script, called every sample */
} sim_t;

void   sim_attach(sim_t *s, motor_t *m, adxl_emu_t *e);
double sim_time_s(void);

#endif /* SIM_HAL_H */
