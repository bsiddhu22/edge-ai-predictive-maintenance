#include "motor_model.h"

#include <math.h>
#include <string.h>

#define TWO_PI 6.283185307179586
#define BPFO_ORDER 3.58
#define F_RES_HZ 1150.0
#define TAU_RES_S 0.0012

static uint64_t xorshift(uint64_t *s)
{
    uint64_t x = *s;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    *s = x;
    return x;
}

double motor_uniform(motor_t *m, double lo, double hi)
{
    const double u = (double)(xorshift(&m->rng) >> 11) * (1.0 / 9007199254740992.0);
    return lo + (hi - lo) * u;
}

static double gauss(motor_t *m)
{
    double u1 = motor_uniform(m, 1e-12, 1.0), u2 = motor_uniform(m, 0.0, 1.0);
    return sqrt(-2.0 * log(u1)) * cos(TWO_PI * u2);
}

void motor_init_session(motor_t *m, uint64_t seed)
{
    memset(m, 0, sizeof(*m));
    m->rng = seed * 0x9E3779B97F4A7C15ull + 0x1234567ull;
    for (int i = 0; i < 8; i++) (void)xorshift(&m->rng);

    m->fr_hz          = motor_uniform(m, 23.5, 26.5);
    m->residual_unb_g = motor_uniform(m, 0.03, 0.12);
    m->h2_base_g      = motor_uniform(m, 0.01, 0.05);
    m->noise_g        = motor_uniform(m, 0.015, 0.05);
    for (int a = 0; a < 3; a++) m->gain[a] = motor_uniform(m, 0.7, 1.3);
    m->phase2         = motor_uniform(m, 0.0, TWO_PI);
    m->theta          = motor_uniform(m, 0.0, TWO_PI);
    m->cond = COND_NORMAL;
    m->severity = 0.0;
    m->next_impact = 0.0;
}

void motor_set_condition(motor_t *m, motor_cond_t c, double severity)
{
    m->cond = c;
    m->severity = (c == COND_NORMAL) ? 0.0 : severity;
}

void motor_sample(motor_t *m, double dt, double g[3])
{
    /* slow speed wander (bounded random walk, about +/-1 %) */
    m->wander = 0.999 * m->wander + 0.02 * gauss(m) * sqrt(dt);
    const double fr = m->fr_hz * (1.0 + 0.01 * m->wander);
    m->theta += TWO_PI * fr * dt;
    if (m->theta > TWO_PI) m->theta -= TWO_PI;
    m->t += dt;

    const double th = m->theta, t = m->t, s = m->severity;
    double a1 = m->residual_unb_g;       /* 1x radial */
    double a2 = m->h2_base_g;            /* 2x radial */
    double ax = 0.02;                    /* axial 1x  */
    double ax2 = 0.0;                    /* axial 2x  */
    double bear = 0.0;

    switch (m->cond) {
    case COND_IMBALANCE:
        a1 += 0.25 * s;
        break;
    case COND_MISALIGNMENT:
        a2 += 0.20 * s;
        ax += 0.25 * s;
        ax2 += 0.25 * s;
        break;
    case COND_BEARING: {
        if (t >= m->next_impact) {
            const double bpfo = BPFO_ORDER * fr;
            m->impact_t = t;
            m->impact_amp = 0.8 * s * (1.0 + 0.3 * cos(th)) * motor_uniform(m, 0.8, 1.2);
            m->next_impact = t + (1.0 / bpfo) * motor_uniform(m, 0.99, 1.01);
        }
        const double tau = t - m->impact_t;
        bear = m->impact_amp * exp(-tau / TAU_RES_S) * sin(TWO_PI * F_RES_HZ * tau);
        break;
    }
    default:
        break;
    }

    const double line = 0.01 * cos(TWO_PI * 120.0 * t);
    g[0] = m->gain[0] * (a1 * cos(th) + a2 * cos(2.0 * th + m->phase2) + line + bear + m->noise_g * gauss(m));
    g[1] = m->gain[1] * (a1 * sin(th) + a2 * sin(2.0 * th + m->phase2) + 0.7 * bear + m->noise_g * gauss(m));
    g[2] = 1.0 + m->gain[2] * (ax * cos(th) + ax2 * cos(2.0 * th) + 0.2 * bear + m->noise_g * gauss(m));
}

const char *motor_cond_name(motor_cond_t c)
{
    switch (c) {
    case COND_NORMAL:       return "NORMAL";
    case COND_IMBALANCE:    return "IMBALANCE";
    case COND_MISALIGNMENT: return "MISALIGNMENT";
    case COND_BEARING:      return "BEARING";
    default:                return "?";
    }
}
