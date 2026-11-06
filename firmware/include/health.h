/**
 * @file health.h
 * @brief Machine-health state machine with k-of-n debounce and hysteresis,
 *        so one odd window can't raise an alarm and one good window can't
 *        clear it. Invalid (dropped) windows carry no evidence either way.
 *
 *   OK --(>=3 of last 8 abnormal)--> SUSPECT --(>=6 of 8)--> ALARM
 *   SUSPECT --(<=1 of 8)--> OK        ALARM --(0 of 8)--> OK
 */
#ifndef HEALTH_H
#define HEALTH_H

#include "pdm_config.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HEALTH_OK = 0,
    HEALTH_SUSPECT,
    HEALTH_ALARM,
    HEALTH_SENSOR_FAULT
} health_state_t;

typedef struct {
    health_state_t state;
    uint8_t  hist_abn[HEALTH_HISTORY];   /* 1 = abnormal */
    uint8_t  hist_cls[HEALTH_HISTORY];
    uint8_t  head, filled;
    uint8_t  abnormal_count;
    uint8_t  fault_class;                /* dominant abnormal class in history */
    uint32_t transitions;
} health_t;

void health_init(health_t *h);
/** @return true if the state changed */
bool health_update(health_t *h, int cls, float confidence);
void health_set_sensor_fault(health_t *h);
const char *health_name(health_state_t s);

#endif /* HEALTH_H */
