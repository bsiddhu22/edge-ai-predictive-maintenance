#include "nn.h"
#include "model_data.h"

#include <math.h>

/* ---- gemmlowp / TFLite reference fixed-point primitives ---- */
static int32_t sat_round_doubling_high_mul(int32_t a, int32_t b)
{
    if (a == b && a == INT32_MIN) {
        return INT32_MAX;
    }
    const int64_t ab = (int64_t)a * (int64_t)b;
    const int64_t nudge = (ab >= 0) ? (1LL << 30) : (1LL - (1LL << 30));
    return (int32_t)((ab + nudge) / (1LL << 31));      /* C '/' truncates toward zero */
}

static int32_t rounding_divide_by_pot(int32_t x, int exponent)
{
    const int32_t mask = (int32_t)((1LL << exponent) - 1);
    const int32_t remainder = x & mask;
    const int32_t threshold = (mask >> 1) + ((x < 0) ? 1 : 0);
    return (x >> exponent) + ((remainder > threshold) ? 1 : 0);
}

int32_t nn_requantize(int32_t acc, int32_t multiplier, int32_t shift)
{
    const int left = (shift > 0) ? (int)shift : 0;
    const int right = (shift > 0) ? 0 : (int)-shift;
    return rounding_divide_by_pot(sat_round_doubling_high_mul(acc * (1 << left), multiplier), right);
}

static int8_t clamp_i8(int32_t v, int32_t lo)
{
    if (v < lo) v = lo;
    if (v > 127) v = 127;
    return (int8_t)v;
}

static float round_half_away(float x)
{
    return (x >= 0.0f) ? floorf(x + 0.5f) : -floorf(-x + 0.5f);
}

void nn_quantize_input(const float *x, int8_t *q)
{
    for (int i = 0; i < NN_IN; i++) {
        const float z = (x[i] - nn_feat_mean[i]) / nn_feat_std[i];
        const float v = round_half_away(z / nn_in_scale) + (float)nn_in_zp;
        q[i] = clamp_i8((int32_t)fmaxf(fminf(v, 127.0f), -128.0f), -128);
    }
}

void nn_infer(const int8_t *in, int8_t *logits)
{
    int8_t h[NN_H];

    for (int j = 0; j < NN_H; j++) {
        int32_t acc = nn_b1[j];
        for (int i = 0; i < NN_IN; i++) {
            acc += ((int32_t)in[i] - nn_in_zp) * (int32_t)nn_w1[j][i];
        }
        const int32_t v = nn_requantize(acc, nn_m1, nn_s1) + nn_h_zp;
        h[j] = clamp_i8(v, nn_h_zp);            /* fused ReLU: clamp at real 0 */
    }

    for (int k = 0; k < NN_OUT; k++) {
        int32_t acc = nn_b2[k];
        for (int j = 0; j < NN_H; j++) {
            acc += ((int32_t)h[j] - nn_h_zp) * (int32_t)nn_w2[k][j];
        }
        logits[k] = clamp_i8(nn_requantize(acc, nn_m2, nn_s2) + nn_out_zp, -128);
    }
}

int nn_argmax(const int8_t *v, int n)
{
    int best = 0;
    for (int i = 1; i < n; i++) {
        if (v[i] > v[best]) best = i;
    }
    return best;
}

void nn_softmax(const int8_t *logits, float *p)
{
    float mx = -1e30f, sum = 0.0f;
    float r[NN_OUT];
    for (int k = 0; k < NN_OUT; k++) {
        r[k] = nn_out_scale * (float)((int32_t)logits[k] - nn_out_zp);
        if (r[k] > mx) mx = r[k];
    }
    for (int k = 0; k < NN_OUT; k++) {
        p[k] = expf(r[k] - mx);
        sum += p[k];
    }
    for (int k = 0; k < NN_OUT; k++) {
        p[k] /= sum;
    }
}

const char *nn_class_name(int c)
{
    switch (c) {
    case NN_CLASS_NORMAL:       return "NORMAL";
    case NN_CLASS_IMBALANCE:    return "IMBALANCE";
    case NN_CLASS_MISALIGNMENT: return "MISALIGNMENT";
    case NN_CLASS_BEARING:      return "BEARING";
    default:                    return "?";
    }
}

unsigned nn_macs(void)
{
    return (unsigned)(NN_IN * NN_H + NN_H * NN_OUT);
}
