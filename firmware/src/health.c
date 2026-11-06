#include "health.h"
#include "nn.h"

#include <string.h>

void health_init(health_t *h)
{
    memset(h, 0, sizeof(*h));
    h->state = HEALTH_OK;
}

static void recount(health_t *h)
{
    uint8_t votes[4] = { 0, 0, 0, 0 };
    h->abnormal_count = 0;
    for (unsigned i = 0; i < h->filled; i++) {
        if (h->hist_abn[i]) {
            h->abnormal_count++;
            if (h->hist_cls[i] < 4u) votes[h->hist_cls[i]]++;
        }
    }
    uint8_t best = NN_CLASS_NORMAL;
    for (uint8_t c = 1; c < 4u; c++) {
        if (votes[c] > votes[best]) best = c;
    }
    h->fault_class = h->abnormal_count ? best : (uint8_t)NN_CLASS_NORMAL;
}

bool health_update(health_t *h, int cls, float confidence)
{
    if (h->state == HEALTH_SENSOR_FAULT) {
        return false;
    }
    const bool abnormal = (cls != NN_CLASS_NORMAL) && (confidence >= HEALTH_MIN_CONF);

    h->hist_abn[h->head] = abnormal ? 1u : 0u;
    h->hist_cls[h->head] = (uint8_t)cls;
    h->head = (uint8_t)((h->head + 1u) % HEALTH_HISTORY);
    if (h->filled < HEALTH_HISTORY) h->filled++;
    recount(h);

    const health_state_t prev = h->state;
    const uint8_t k = h->abnormal_count;
    switch (h->state) {
    case HEALTH_OK:
        if (k >= HEALTH_ALARM_K)        h->state = HEALTH_ALARM;
        else if (k >= HEALTH_SUSPECT_K) h->state = HEALTH_SUSPECT;
        break;
    case HEALTH_SUSPECT:
        if (k >= HEALTH_ALARM_K)        h->state = HEALTH_ALARM;
        else if (k <= HEALTH_CLEAR_K)   h->state = HEALTH_OK;
        break;
    case HEALTH_ALARM:
        if (k == 0u && h->filled == HEALTH_HISTORY) h->state = HEALTH_OK;   /* hysteresis */
        break;
    default:
        break;
    }
    if (h->state != prev) {
        h->transitions++;
        return true;
    }
    return false;
}

void health_set_sensor_fault(health_t *h)
{
    h->state = HEALTH_SENSOR_FAULT;
    h->transitions++;
}

const char *health_name(health_state_t s)
{
    switch (s) {
    case HEALTH_OK:           return "OK";
    case HEALTH_SUSPECT:      return "SUSPECT";
    case HEALTH_ALARM:        return "ALARM";
    case HEALTH_SENSOR_FAULT: return "SENSOR_FAULT";
    default:                  return "?";
    }
}
