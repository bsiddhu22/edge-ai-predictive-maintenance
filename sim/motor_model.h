/**
 * @file motor_model.h
 * @brief Synthetic 3-axis vibration of a ~1500 rpm induction motor, in g.
 *
 * Physics-inspired signatures (simplified):
 *  - residual unbalance: small 1x radial (x, y in quadrature)
 *  - IMBALANCE: large 1x radial
 *  - MISALIGNMENT: 2x radial plus strong 1x/2x axial (z)
 *  - BEARING (outer race): impacts at BPFO = 3.58 x shaft speed, each
 *    ringing a ~1.15 kHz structural resonance, amplitude-modulated by load zone
 *  - always: 120 Hz electrical component, 1 g gravity on z, sensor noise,
 *    slow speed wander, per-session mounting gain differences
 */
#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include <stdint.h>

typedef enum {
    COND_NORMAL = 0,
    COND_IMBALANCE,
    COND_MISALIGNMENT,
    COND_BEARING,
    COND_COUNT
} motor_cond_t;

typedef struct {
    /* session (installation) parameters */
    double fr_hz;
    double residual_unb_g;
    double h2_base_g;
    double noise_g;
    double gain[3];
    double phase2;
    /* condition */
    motor_cond_t cond;
    double severity;
    /* state */
    double t, theta, wander;
    double impact_t, impact_amp, next_impact;
    uint64_t rng;
} motor_t;

/** Randomize installation parameters for a new "session" from @p seed. */
void motor_init_session(motor_t *m, uint64_t seed);
void motor_set_condition(motor_t *m, motor_cond_t c, double severity);
void motor_sample(motor_t *m, double dt, double out_g[3]);
double motor_uniform(motor_t *m, double lo, double hi);
const char *motor_cond_name(motor_cond_t c);

#endif /* MOTOR_MODEL_H */
