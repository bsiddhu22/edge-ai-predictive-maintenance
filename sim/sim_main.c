/**
 * @file sim_main.c
 * @brief SIL runner: firmware + emulated sensor + motor model on a virtual
 *        clock, with scripted machine conditions and fault injection.
 *
 *   usage: pdm_sim <scenario> [out_dir]      pdm_sim --list
 *
 * Writes <out>/<scenario>_windows.csv (one row per processed window),
 * <scenario>_telemetry.log (UART sentences) and <scenario>_summary.json.
 */
#include "acq.h"
#include "adxl_emu.h"
#include "app.h"
#include "model_data.h"
#include "motor_model.h"
#include "nn.h"
#include "perf.h"
#include "sim_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { double t; motor_cond_t cond; double severity; } step_t;

typedef struct {
    const char *name;
    const char *desc;
    double      duration_s;
    step_t      steps[10];       /* condition schedule, terminated by t < 0 */
    int         dump_raw;
    uint32_t    baud;
    int         validate;
    uint8_t     devid;
    double      residual_unb_g;  /* 0 = installation default */
} scenario_t;

#define END { -1.0, COND_NORMAL, 0.0 }

static const scenario_t SC[] = {
    { "normal", "Healthy motor for 20 s: no alarms, no overruns", 20.0,
      { {0, COND_NORMAL, 0}, END }, 0, 115200, 1, 0xE5, 0.0 },
    { "imbalance", "Imbalance develops at 5 s", 15.0,
      { {0, COND_NORMAL, 0}, {5, COND_IMBALANCE, 1.0}, END }, 0, 115200, 1, 0xE5, 0.0 },
    { "misalignment", "Misalignment develops at 5 s", 15.0,
      { {0, COND_NORMAL, 0}, {5, COND_MISALIGNMENT, 1.0}, END }, 0, 115200, 1, 0xE5, 0.0 },
    { "bearing", "Outer-race bearing defect appears at 5 s", 15.0,
      { {0, COND_NORMAL, 0}, {5, COND_BEARING, 1.0}, END }, 0, 115200, 1, 0xE5, 0.0 },
    { "normal_marginal", "Healthy but rough installation near the decision boundary", 30.0,
      { {0, COND_NORMAL, 0}, END }, 0, 115200, 1, 0xE5, 0.115 },
    { "intermittent", "0.3 s imbalance bursts every 4 s: debounce must hold", 20.0,
      { {0, COND_NORMAL, 0}, {4.0, COND_IMBALANCE, 1.0}, {4.3, COND_NORMAL, 0},
        {8.0, COND_IMBALANCE, 1.0}, {8.3, COND_NORMAL, 0}, {12.0, COND_IMBALANCE, 1.0},
        {12.3, COND_NORMAL, 0}, {16.0, COND_IMBALANCE, 1.0}, {16.3, COND_NORMAL, 0}, END },
      0, 115200, 1, 0xE5, 0.0 },
    { "recovery", "Imbalanced until 8 s, then repaired: alarm must clear", 16.0,
      { {0, COND_IMBALANCE, 1.0}, {8, COND_NORMAL, 0}, END }, 0, 115200, 1, 0xE5, 0.0 },
    { "debug_dump_unprotected", "Raw dump at 115200 baud, deadline check OFF (the original bug)", 12.0,
      { {0, COND_NORMAL, 0}, END }, 1, 115200, 0, 0xE5, 0.0 },
    { "debug_dump", "Raw dump at 115200 baud, deadline check ON: torn windows dropped", 12.0,
      { {0, COND_NORMAL, 0}, END }, 1, 115200, 1, 0xE5, 0.0 },
    { "debug_dump_fast", "Raw dump at 921600 baud: fits the deadline", 12.0,
      { {0, COND_NORMAL, 0}, END }, 1, 921600, 1, 0xE5, 0.0 },
    { "sensor_fault", "Accelerometer not responding (DEVID reads 0x00)", 3.0,
      { {0, COND_NORMAL, 0}, END }, 0, 115200, 1, 0x00, 0.0 },
};
#define N_SC (sizeof(SC) / sizeof(SC[0]))

static const scenario_t *g_sc;
static motor_t g_motor;
static int g_step;
static FILE *g_wcsv;
static motor_cond_t g_truth;

static void script(double t)
{
    while (g_sc->steps[g_step + 1].t >= 0.0 && t >= g_sc->steps[g_step + 1].t) {
        g_step++;
    }
    const step_t *s = &g_sc->steps[g_step];
    if (g_motor.cond != s->cond || g_motor.severity != (s->cond == COND_NORMAL ? 0.0 : s->severity)) {
        motor_set_condition(&g_motor, s->cond, s->severity);
    }
    g_truth = s->cond;
}

static void on_window(const app_window_report_t *r)
{
    fprintf(g_wcsv, "%.4f,%u,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%s,%s",
            sim_time_s(), (unsigned)r->seq, r->valid ? 1 : 0, r->cls, (double)r->conf,
            (double)r->probs[0], (double)r->probs[1], (double)r->probs[2], (double)r->probs[3],
            health_name(r->health), motor_cond_name(g_truth));
    for (unsigned i = 0; i < PDM_N_FEATURES; i++) fprintf(g_wcsv, ",%.5f", (double)r->features[i]);
    fputc('\n', g_wcsv);
}

static void perf_json(FILE *f)
{
    const perf_t *p = perf_get();
    fprintf(f, "  \"timing_us\": {\n");
    for (int s = 0; s < PERF_N_STAGES; s++) {
        const perf_stat_t *st = &p->stage[s];
        const double avg = st->count ? (double)st->sum / st->count : 0.0;
        fprintf(f, "    \"%s\": {\"avg\": %.1f, \"max\": %.1f, \"count\": %u}%s\n", perf_stage_name((perf_stage_t)s),
                avg * 1e6 / PDM_CPU_HZ, (double)st->max * 1e6 / PDM_CPU_HZ, st->count,
                s == PERF_N_STAGES - 1 ? "" : ",");
    }
    fprintf(f, "  },\n");
    fprintf(f, "  \"deadline_us\": %.1f,\n", (double)PDM_WINDOW_CYCLES * 1e6 / PDM_CPU_HZ);
    fprintf(f, "  \"cpu_load_pct\": %.2f,\n", perf_load_permille() / 10.0);
}

int main(int argc, char **argv)
{
    if (argc < 2 || !strcmp(argv[1], "--list")) {
        printf("usage: pdm_sim <scenario> [out_dir]\n\nscenarios:\n");
        for (size_t i = 0; i < N_SC; i++) printf("  %-24s %s\n", SC[i].name, SC[i].desc);
        return argc < 2 ? 1 : 0;
    }
    for (size_t i = 0; i < N_SC; i++) if (!strcmp(SC[i].name, argv[1])) g_sc = &SC[i];
    if (!g_sc) { fprintf(stderr, "unknown scenario '%s'\n", argv[1]); return 2; }
    const char *out = argc > 2 ? argv[2] : ".";

    char path[512];
    snprintf(path, sizeof path, "%s/%s_windows.csv", out, g_sc->name);
    g_wcsv = fopen(path, "wb");
    snprintf(path, sizeof path, "%s/%s_telemetry.log", out, g_sc->name);
    FILE *tlm = fopen(path, "wb");
    if (!g_wcsv || !tlm) { perror("open output"); return 3; }

    fprintf(g_wcsv, "t_s,seq,valid,cls,conf,p_normal,p_imbalance,p_misalignment,p_bearing,health,truth");
    for (int a = 0; a < 3; a++) {
        static const char *fn[8] = { "logrms", "crest", "kurt", "e1x", "e2x", "eharm", "ehf", "fpk" };
        for (int k = 0; k < 8; k++) fprintf(g_wcsv, ",%c_%s", "xyz"[a], fn[k]);
    }
    fputc('\n', g_wcsv);

    /* Scenario installation: fixed seed, outside the training seed range. */
    motor_init_session(&g_motor, 900001u);
    if (g_sc->residual_unb_g > 0.0) g_motor.residual_unb_g = g_sc->residual_unb_g;
    adxl_emu_t emu;
    adxl_emu_init(&emu, g_sc->devid);
    sim_t sim;
    sim_attach(&sim, &g_motor, &emu);
    sim.uart_log = tlm;
    sim.tick_hook = script;
    g_step = 0;
    script(0.0);

    app_cfg_t cfg;
    app_default_cfg(&cfg);
    cfg.debug_dump_raw = g_sc->dump_raw != 0;
    cfg.uart_baud = g_sc->baud;
    cfg.validate_deadline = g_sc->validate != 0;
    app_set_window_hook(on_window);
    app_init(&cfg);

    const uint64_t end = (uint64_t)(g_sc->duration_s * PDM_CPU_HZ);
    while (sim.now < end) app_loop_once();

    fclose(g_wcsv);
    fclose(tlm);

    const app_status_t *st = app_status();
    const acq_stats_t aq = acq_stats();
    snprintf(path, sizeof path, "%s/%s_summary.json", out, g_sc->name);
    FILE *js = fopen(path, "wb");
    if (!js) { perror(path); return 3; }
    fprintf(js, "{\n  \"scenario\": \"%s\",\n", g_sc->name);
    fprintf(js, "  \"accel_status\": %d,\n  \"devid\": \"0x%02X\",\n", (int)st->accel, accel_last_devid());
    fprintf(js, "  \"final_health\": \"%s\",\n  \"health_transitions\": %u,\n",
            health_name(st->health.state), st->health.transitions);
    fprintf(js, "  \"fault_class\": \"%s\",\n", nn_class_name(st->health.fault_class));
    fprintf(js, "  \"valid_windows\": %u,\n  \"dropped_windows\": %u,\n  \"overruns\": %u,\n",
            st->valid_windows, st->dropped_windows, aq.overruns);
    fprintf(js, "  \"abnormal_windows\": %u,\n", st->abnormal_windows);
    fprintf(js, "  \"class_counts\": [%u, %u, %u, %u],\n", st->class_counts[0], st->class_counts[1],
            st->class_counts[2], st->class_counts[3]);
    fprintf(js, "  \"telemetry_lines\": %u,\n  \"uart_bytes\": %llu,\n", sim.uart_lines,
            (unsigned long long)sim.uart_bytes);
    perf_json(js);
    fprintf(js, "  \"memory_bytes\": {\"dma_buffer\": %u, \"dsp_static\": %u, \"model_flash\": %u},\n",
            (unsigned)(2u * PDM_N * PDM_AXES * sizeof(int16_t)),
            (unsigned)(PDM_N * 4u * 4u + (PDM_N / 2u) * 4u * 3u + PDM_N * 2u),
            (unsigned)(sizeof(nn_w1) + sizeof(nn_b1) + sizeof(nn_w2) + sizeof(nn_b2) +
                       sizeof(nn_feat_mean) + sizeof(nn_feat_std) + 6u * sizeof(int32_t) + 2u * sizeof(float)));
    fprintf(js, "  \"watchdog_kicks\": %u\n}\n", sim.wdg_kicks);
    fclose(js);

    printf("%-24s health=%-12s fault=%-12s valid=%-4u dropped=%-3u overruns=%-3u abnormal=%-3u load=%.2f%%\n",
           g_sc->name, health_name(st->health.state), nn_class_name(st->health.fault_class),
           st->valid_windows, st->dropped_windows, aq.overruns, st->abnormal_windows,
           perf_load_permille() / 10.0);
    return 0;
}
