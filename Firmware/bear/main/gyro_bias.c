#include "gyro_bias.h"
#include <math.h>
#include <string.h>
#include "config.h"

void gyro_bias_init(gyro_bias_t *g, const float b0[3]) {
    memset(g, 0, sizeof(*g));
    for (int i = 0; i < 3; i++) {
        g->b[i] = b0[i];
        g->m[i] = b0[i];
    }
    g->acc_m = 1.0f;
}

bool gyro_bias_update(gyro_bias_t *g, const float raw[3], float acc_norm, float dt) {
    float a = dt / GB_WINDOW_S;
    if (a > 1.0f) a = 1.0f;
    bool still = true;
    for (int i = 0; i < 3; i++) {
        float d = raw[i] - g->m[i];
        g->m[i] += a * d;
        g->v[i] += a * (d * d - g->v[i]);
        if (g->v[i] > GB_STILL_STD_RADS * GB_STILL_STD_RADS || fabsf(g->m[i]) > GB_MAX_BIAS_RADS) still = false;
    }
    float da = acc_norm - g->acc_m;
    g->acc_m += a * da;
    g->acc_v += a * (da * da - g->acc_v);
    if (g->acc_v > GB_STILL_ACC_STD_G * GB_STILL_ACC_STD_G) still = false;

    g->seen_s += dt;
    if (g->seen_s < 2.0f * GB_WINDOW_S) still = false;
    g->still_ms = still ? g->still_ms + dt * 1000.0f : 0.0f;
    if (g->still_ms < GB_STILL_MS) return false;

    float k = dt / GB_TAU_S;
    for (int i = 0; i < 3; i++) g->b[i] += k * (g->m[i] - g->b[i]);
    return true;
}
