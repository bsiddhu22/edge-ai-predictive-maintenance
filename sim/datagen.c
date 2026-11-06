/**
 * @file datagen.c
 * @brief Training-data generator: records labeled raw windows through the
 *        same motor model and sensor conversion the SIL runs use.
 *
 *   usage: pdm_datagen <out.bin> <sessions_per_class> <windows_per_session> <seed_base>
 *
 * Each "session" is a new installation (speed, mounting gains, noise,
 * residual unbalance) with one condition at a random severity. Windows from
 * one session are consecutive, so train/test must be split by session.
 *
 * File: "PDMD" | int32 n_windows | int32 N | int32 axes |
 *       n x { int32 label | int32 session | float32 severity | int16[N*axes] }
 */
#include "adxl_emu.h"
#include "motor_model.h"
#include "pdm_config.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    if (argc < 5) {
        fprintf(stderr, "usage: %s out.bin sessions_per_class windows_per_session seed_base\n", argv[0]);
        return 1;
    }
    const int spc = atoi(argv[2]), wps = atoi(argv[3]);
    const unsigned long base = strtoul(argv[4], NULL, 10);
    FILE *f = fopen(argv[1], "wb");
    if (!f) { perror(argv[1]); return 2; }

    const int32_t n = COND_COUNT * spc * wps, N = PDM_N, axes = PDM_AXES;
    fwrite("PDMD", 1, 4, f);
    fwrite(&n, 4, 1, f); fwrite(&N, 4, 1, f); fwrite(&axes, 4, 1, f);

    static int16_t frames[PDM_N * PDM_AXES];
    const double dt = 1.0 / PDM_FS_HZ;
    int32_t session = 0;

    for (int c = 0; c < COND_COUNT; c++) {
        for (int s = 0; s < spc; s++, session++) {
            motor_t m;
            motor_init_session(&m, base + (unsigned long)session);
            const float sev = (c == COND_NORMAL) ? 0.0f : (float)motor_uniform(&m, 0.1, 1.5);
            motor_set_condition(&m, (motor_cond_t)c, sev);
            double g[3];
            for (int i = 0; i < (int)(0.3 * PDM_FS_HZ); i++) motor_sample(&m, dt, g);   /* settle */

            for (int w = 0; w < wps; w++) {
                for (unsigned i = 0; i < PDM_N; i++) {
                    motor_sample(&m, dt, g);
                    for (unsigned a = 0; a < PDM_AXES; a++) frames[i * PDM_AXES + a] = adxl_emu_g_to_lsb(g[a]);
                }
                const int32_t label = c;
                fwrite(&label, 4, 1, f);
                fwrite(&session, 4, 1, f);
                fwrite(&sev, 4, 1, f);
                fwrite(frames, sizeof(frames), 1, f);
            }
        }
    }
    fclose(f);
    printf("wrote %d windows (%d sessions) to %s\n", n, session, argv[1]);
    return 0;
}
