#include "app.h"
#include "acq.h"
#include "dsp.h"
#include "hal.h"
#include "nn.h"
#include "perf.h"
#include "telemetry.h"

#include <string.h>

static app_cfg_t         s_cfg;
static app_status_t      s_st;
static app_window_hook_t s_hook;
static uint32_t          s_last_fault_tlm;
static float             s_features[PDM_N_FEATURES];
static uint32_t          s_windows_seen;

void app_default_cfg(app_cfg_t *cfg)
{
    cfg->debug_dump_raw = false;
    cfg->uart_baud = 115200u;
    cfg->validate_deadline = true;
}

void app_set_window_hook(app_window_hook_t hook)
{
    s_hook = hook;
}

const app_status_t *app_status(void)
{
    return &s_st;
}

static void send_telemetry(uint32_t seq)
{
    const uint32_t t0 = perf_begin();
    const acq_stats_t a = acq_stats();
    telemetry_t t;
    t.seq = seq;
    t.health = health_name(s_st.health.state);
    t.cls = nn_class_name(s_st.last_cls);
    t.conf_pct = (uint32_t)(s_st.last_conf * 100.0f + 0.5f);
    t.fault = (s_st.health.state == HEALTH_SENSOR_FAULT) ? "SENSOR" : nn_class_name(s_st.health.fault_class);
    t.valid_windows = s_st.valid_windows;
    t.dropped_windows = s_st.dropped_windows;
    t.overruns = a.overruns;
    t.load_permille = perf_load_permille();

    char line[128];
    const size_t n = telemetry_format(&t, line, sizeof line);
    hal_cpu_cost(COST_TELEMETRY_FMT);
    hal_uart_write((const uint8_t *)line, n);
    s_st.telemetry_sent++;
    perf_end(PERF_TELEMETRY, t0);
}

void app_init(const app_cfg_t *cfg)
{
    s_cfg = *cfg;
    memset(&s_st, 0, sizeof(s_st));
    s_windows_seen = 0;
    health_init(&s_st.health);
    perf_init();
    dsp_init();
    hal_uart_init(s_cfg.uart_baud);

    s_st.accel = accel_init();
    if (s_st.accel != ACCEL_OK) {
        health_set_sensor_fault(&s_st.health);   /* no acquisition on a sensor we can't trust */
        s_last_fault_tlm = hal_cycles();
        send_telemetry(0);
        return;
    }
    acq_start();
}

static void maybe_send_telemetry(uint32_t seq)
{
    /* Driven by windows seen, valid or not, so the node keeps reporting
     * (dropped/overrun counters) even when every window is being rejected. */
    if ((++s_windows_seen % TELEMETRY_EVERY_WINDOWS) == 0u) {
        send_telemetry(seq);
    }
}

static void process_window(const acq_window_t *w)
{
    const uint32_t t_total = perf_begin();

    if (s_cfg.debug_dump_raw) {
        /* Raw-window logging used to collect training data. Blocking UART:
         * 3 KB at 115200 baud takes ~270 ms, longer than the 160 ms deadline. */
        const uint32_t t0 = perf_begin();
        static const uint8_t hdr[4] = { 'R', 'A', 'W', 0x00 };
        hal_uart_write(hdr, sizeof hdr);
        hal_uart_write((const uint8_t *)w->frames, PDM_N * PDM_AXES * sizeof(int16_t));
        perf_end(PERF_DEBUG_DUMP, t0);
    }

    /* Features are computed in place from the DMA half-buffer, axis by axis. */
    uint32_t t0 = perf_begin();
    for (unsigned a = 0; a < PDM_AXES; a++) {
        features_axis(w->frames, a, &s_features[a * PDM_FEAT_PER_AXIS]);
        hal_cpu_cost(COST_FEATURES_AXIS);
    }
    perf_end(PERF_FEATURES, t0);

    app_window_report_t r;
    memset(&r, 0, sizeof r);
    r.seq = w->seq;
    r.features = s_features;

    /* Deadline check: if DMA wrapped into this half while we were reading it,
     * the features describe two different moments stitched together. */
    r.valid = acq_still_valid(w);
    if (!r.valid && s_cfg.validate_deadline) {
        s_st.dropped_windows++;
        r.cls = -1;
        r.health = s_st.health.state;
        if (s_hook) s_hook(&r);
        maybe_send_telemetry(w->seq);
        perf_end(PERF_WINDOW_TOTAL, t_total);
        return;
    }

    t0 = perf_begin();
    int8_t q[PDM_N_FEATURES], logits[4];
    nn_quantize_input(s_features, q);
    nn_infer(q, logits);
    nn_softmax(logits, r.probs);
    r.cls = nn_argmax(logits, 4);
    r.conf = r.probs[r.cls];
    hal_cpu_cost(COST_INFER_FIXED + 3u * nn_macs());
    perf_end(PERF_INFER, t0);

    t0 = perf_begin();
    health_update(&s_st.health, r.cls, r.conf);
    hal_cpu_cost(COST_HEALTH);
    perf_end(PERF_HEALTH, t0);

    s_st.valid_windows++;
    s_st.class_counts[r.cls]++;
    if (r.cls != NN_CLASS_NORMAL && r.conf >= HEALTH_MIN_CONF) s_st.abnormal_windows++;
    s_st.last_cls = r.cls;
    s_st.last_conf = r.conf;
    r.health = s_st.health.state;
    if (s_hook) s_hook(&r);

    maybe_send_telemetry(w->seq);
    perf_end(PERF_WINDOW_TOTAL, t_total);
}

void app_loop_once(void)
{
    hal_watchdog_kick();

    if (s_st.health.state == HEALTH_SENSOR_FAULT) {
        const uint32_t t0 = hal_cycles();
        hal_wfi();
        perf_add_idle(hal_cycles() - t0);
        if ((uint32_t)(hal_cycles() - s_last_fault_tlm) >= PDM_CPU_HZ) {   /* ~1 s */
            s_last_fault_tlm = hal_cycles();
            send_telemetry(0);
        }
        return;
    }

    acq_window_t w;
    if (!acq_take(&w)) {
        const uint32_t t0 = hal_cycles();
        hal_wfi();
        perf_add_idle(hal_cycles() - t0);
        return;
    }

    const uint32_t t0 = hal_cycles();
    process_window(&w);
    perf_add_busy(hal_cycles() - t0);
}
