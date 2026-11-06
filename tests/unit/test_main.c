/* Unit tests: driver, acquisition logic, DSP, int8 kernels, health state
 * machine, telemetry, and golden-vector parity with the Python pipeline. */
#include "accel.h"
#include "acq.h"
#include "adxl_emu.h"
#include "dsp.h"
#include "hal.h"
#include "health.h"
#include "model_data.h"
#include "nn.h"
#include "telemetry.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern adxl_emu_t   g_test_emu;
extern hal_isr_cb_t g_test_half_cb, g_test_full_cb;

static int g_fail, g_pass;
#define CHECK(c) do { if (c) g_pass++; else { g_fail++; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)
#define CHECK_NEAR(a, b, t) CHECK(fabs((double)(a) - (double)(b)) <= (double)(t))
#define RUN(fn) do { printf("%s\n", #fn); fn(); } while (0)

static const double PI = 3.14159265358979323846;

/* ------------------------------------------------------------- driver */
static void test_accel_init_ok_and_configures_sensor(void)
{
    adxl_emu_init(&g_test_emu, ADXL_DEVID_VALUE);
    CHECK(accel_init() == ACCEL_OK);
    CHECK(g_test_emu.reg[ADXL_REG_BW_RATE] == ADXL_RATE_3200HZ);
    CHECK(g_test_emu.reg[ADXL_REG_DATA_FORMAT] == ADXL_FMT_FULLRES_16G);
    CHECK(g_test_emu.reg[ADXL_REG_POWER_CTL] == ADXL_POWER_MEASURE);
}

static void test_accel_rejects_wrong_identity(void)
{
    adxl_emu_init(&g_test_emu, 0x00);               /* MISO stuck low / no part */
    CHECK(accel_init() == ACCEL_ERR_ID);
    CHECK(g_test_emu.reg[ADXL_REG_POWER_CTL] == 0x00);   /* never configured */
}

static void test_accel_multibyte_read(void)
{
    adxl_emu_init(&g_test_emu, ADXL_DEVID_VALUE);
    CHECK(accel_init() == ACCEL_OK);
    const double g[3] = { 0.5, -0.25, 1.0 };
    adxl_emu_set_accel(&g_test_emu, g);
    int16_t xyz[3];
    accel_read_xyz(xyz);
    CHECK(xyz[0] == 128 && xyz[1] == -64 && xyz[2] == 256);
}

/* -------------------------------------------------------- acquisition */
static void test_acq_take_newest_and_validity(void)
{
    acq_start();
    acq_window_t w;
    CHECK(!acq_take(&w));                           /* nothing yet */

    g_test_half_cb();                               /* half 0 full */
    CHECK(acq_take(&w) && w.half == 0);
    CHECK(acq_still_valid(&w));                     /* DMA is in half 1 */
    g_test_full_cb();                               /* half 1 full: DMA now overwriting half 0 */
    CHECK(!acq_still_valid(&w));
    CHECK(acq_stats().overruns == 0);
}

static void test_acq_overrun_detected(void)
{
    acq_start();
    acq_window_t w;
    g_test_half_cb();
    g_test_full_cb();
    g_test_half_cb();                               /* half 0 again, never consumed */
    CHECK(acq_stats().overruns == 1);
    CHECK(acq_take(&w) && w.half == 0);             /* newest only */
    CHECK(!acq_take(&w));
}

/* ---------------------------------------------------------------- DSP */
static void test_fft_matches_naive_dft(void)
{
    dsp_init();
    static float re[PDM_N], im[PDM_N];
    static double xr[PDM_N];
    srand(7);
    for (unsigned i = 0; i < PDM_N; i++) {
        xr[i] = (double)rand() / RAND_MAX - 0.5;
        re[i] = (float)xr[i];
        im[i] = 0.0f;
    }
    dsp_fft(re, im);
    double max_err = 0.0;
    for (unsigned k = 0; k < PDM_N; k += 37) {
        double sr = 0, si = 0;
        for (unsigned n = 0; n < PDM_N; n++) {
            sr += xr[n] * cos(2 * PI * k * n / PDM_N);
            si -= xr[n] * sin(2 * PI * k * n / PDM_N);
        }
        max_err = fmax(max_err, fabs(sr - re[k]) + fabs(si - im[k]));
    }
    CHECK(max_err < 1e-3);
}

static void test_features_on_pure_tone(void)
{
    dsp_init();
    static int16_t frames[PDM_N * PDM_AXES];
    for (unsigned i = 0; i < PDM_N; i++) {
        const double t = (double)i / PDM_FS_HZ;
        frames[i * 3 + 0] = (int16_t)lround(256.0 * 0.5 * sin(2 * PI * 25.0 * t));     /* 1x tone */
        frames[i * 3 + 1] = (int16_t)lround(256.0 * 0.5 * sin(2 * PI * 50.0 * t));     /* 2x tone */
        frames[i * 3 + 2] = 256;                                                     /* gravity only */
    }
    float fx[8], fy[8];
    features_axis(frames, 0, fx);
    features_axis(frames, 1, fy);
    CHECK_NEAR(powf(10.0f, fx[0]), 0.5 / sqrt(2.0), 0.01);       /* rms of 0.5 g sine */
    CHECK_NEAR(fx[1], sqrt(2.0), 0.05);                          /* crest of a sine */
    CHECK_NEAR(fx[2], 1.5, 0.05);                                /* kurtosis of a sine */
    CHECK(fx[3] > fx[4] + 2.0f);                                 /* energy in 1x, not 2x */
    CHECK(fy[4] > fy[3] + 2.0f);
    CHECK_NEAR(fx[7] * 100.0f, 25.0, 3.2);                       /* within one bin */
}

/* ---------------------------------------------------------- int8 math */
static void test_requantize_reference_cases(void)
{
    /* multiplier 0.5 in Q31 = 2^30, shift 0 -> halves with round-half-away */
    CHECK(nn_requantize(100, 1 << 30, 0) == 50);
    CHECK(nn_requantize(101, 1 << 30, 0) == 51);
    CHECK(nn_requantize(-101, 1 << 30, 0) == -50);   /* gemmlowp: -50.5 -> -50 (tie toward +inf) */
    /* 0.5 * 2^-2 = 0.125 */
    CHECK(nn_requantize(1000, 1 << 30, -2) == 125);
    CHECK(nn_requantize(-1000, 1 << 30, -2) == -125);
}

/* ------------------------------------------------------ health machine */
static void feed(health_t *h, int n, int cls)
{
    for (int i = 0; i < n; i++) health_update(h, cls, 0.9f);
}

static void test_health_single_window_cannot_alarm(void)
{
    health_t h; health_init(&h);
    feed(&h, 5, NN_CLASS_NORMAL);
    feed(&h, 2, NN_CLASS_IMBALANCE);
    CHECK(h.state == HEALTH_OK);
    feed(&h, 1, NN_CLASS_IMBALANCE);                /* 3 of 8 */
    CHECK(h.state == HEALTH_SUSPECT);
    feed(&h, 3, NN_CLASS_IMBALANCE);                /* 6 of 8 */
    CHECK(h.state == HEALTH_ALARM);
    CHECK(h.fault_class == NN_CLASS_IMBALANCE);
}

static void test_health_alarm_hysteresis(void)
{
    health_t h; health_init(&h);
    feed(&h, 8, NN_CLASS_BEARING);
    CHECK(h.state == HEALTH_ALARM);
    feed(&h, 7, NN_CLASS_NORMAL);                   /* 1 abnormal still in history */
    CHECK(h.state == HEALTH_ALARM);
    feed(&h, 1, NN_CLASS_NORMAL);                   /* 8 clean windows */
    CHECK(h.state == HEALTH_OK);
}

static void test_health_low_confidence_ignored(void)
{
    health_t h; health_init(&h);
    for (int i = 0; i < 8; i++) health_update(&h, NN_CLASS_BEARING, 0.4f);
    CHECK(h.state == HEALTH_OK);
}

/* ---------------------------------------------------------- telemetry */
static void test_telemetry_checksum(void)
{
    telemetry_t t = { 42, "OK", "NORMAL", 97, "NORMAL", 100, 0, 0, 7 };
    char buf[128];
    const size_t n = telemetry_format(&t, buf, sizeof buf);
    CHECK(n > 0 && buf[0] == '$' && buf[n - 2] == '\r' && buf[n - 1] == '\n');
    const char *star = strchr(buf, '*');
    CHECK(star != NULL);
    const unsigned cs = (unsigned)strtoul(star + 1, NULL, 16);
    CHECK(cs == telemetry_checksum(buf + 1, (size_t)(star - buf - 1)));
}

/* ------------------------------------------- golden-vector parity test */
static void test_golden_parity_with_python(void)
{
    FILE *f = fopen(GOLDEN_PATH, "rb");
    CHECK(f != NULL);
    if (!f) return;
    char magic[4];
    int32_t hdr[4];
    CHECK(fread(magic, 1, 4, f) == 4 && memcmp(magic, "PDMG", 4) == 0);
    CHECK(fread(hdr, 4, 4, f) == 4);
    CHECK(hdr[1] == (int32_t)PDM_N && hdr[2] == NN_IN && hdr[3] == NN_OUT);

    dsp_init();
    static int16_t frames[PDM_N * PDM_AXES];
    float py_feat[NN_IN], c_feat[NN_IN];
    int8_t py_q[NN_IN], c_q[NN_IN], py_logits[NN_OUT], c_logits[NN_OUT];
    double max_feat_err = 0.0;
    int q_off_by_one = 0, q_worse = 0, logits_mismatch = 0;

    for (int w = 0; w < hdr[0]; w++) {
        if (fread(frames, sizeof frames, 1, f) != 1 || fread(py_feat, 4, NN_IN, f) != NN_IN ||
            fread(py_q, 1, NN_IN, f) != NN_IN || fread(py_logits, 1, NN_OUT, f) != NN_OUT) {
            CHECK(0);
            break;
        }
        for (unsigned a = 0; a < PDM_AXES; a++) features_axis(frames, a, &c_feat[a * PDM_FEAT_PER_AXIS]);
        for (int i = 0; i < NN_IN; i++) {
            const double tol_scale = fmax(1.0, fabs((double)py_feat[i]));
            max_feat_err = fmax(max_feat_err, fabs((double)c_feat[i] - (double)py_feat[i]) / tol_scale);
        }
        nn_quantize_input(c_feat, c_q);
        for (int i = 0; i < NN_IN; i++) {
            const int d = abs((int)c_q[i] - (int)py_q[i]);
            if (d == 1) q_off_by_one++;
            if (d > 1) q_worse++;
        }
        nn_infer(py_q, c_logits);                       /* integer kernels must be bit-exact */
        if (memcmp(c_logits, py_logits, NN_OUT) != 0) logits_mismatch++;
    }
    fclose(f);
    printf("  golden: windows=%d  max rel feature err=%.2e  q off-by-one=%d  logits mismatches=%d\n",
           hdr[0], max_feat_err, q_off_by_one, logits_mismatch);
    CHECK(max_feat_err < 1e-3);        /* float32 firmware vs float64 host */
    CHECK(q_worse == 0);
    CHECK(logits_mismatch == 0);
}

int main(void)
{
    RUN(test_accel_init_ok_and_configures_sensor);
    RUN(test_accel_rejects_wrong_identity);
    RUN(test_accel_multibyte_read);
    RUN(test_acq_take_newest_and_validity);
    RUN(test_acq_overrun_detected);
    RUN(test_fft_matches_naive_dft);
    RUN(test_features_on_pure_tone);
    RUN(test_requantize_reference_cases);
    RUN(test_health_single_window_cannot_alarm);
    RUN(test_health_alarm_hysteresis);
    RUN(test_health_low_confidence_ignored);
    RUN(test_telemetry_checksum);
    RUN(test_golden_parity_with_python);
    printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
