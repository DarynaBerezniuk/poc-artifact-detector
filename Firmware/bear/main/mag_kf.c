#include "mag_kf.h"
#include <string.h>

void mag_kf_init(mag_kf_t *k, float q, float r, float gate, int max_rejects) {
    memset(k, 0, sizeof(*k));
    k->q = q;
    k->r = r;
    k->gate = gate;
    k->max_rejects = max_rejects;
}

void mag_kf_predict(mag_kf_t *k, const float gyr[3], float dt) {
    if (!k->inited) return;
    float mx = k->m[0], my = k->m[1], mz = k->m[2];
    k->m[0] = mx + (my * gyr[2] - mz * gyr[1]) * dt;
    k->m[1] = my + (mz * gyr[0] - mx * gyr[2]) * dt;
    k->m[2] = mz + (mx * gyr[1] - my * gyr[0]) * dt;
    k->p += k->q * dt;
}

static void reset_to(mag_kf_t *k, const float z[3]) {
    for (int i = 0; i < 3; i++) k->m[i] = z[i];
    k->p = k->r;
    k->rejects = 0;
    k->inited = true;
}

bool mag_kf_update(mag_kf_t *k, const float z[3]) {
    if (!k->inited) {
        reset_to(k, z);
        return true;
    }
    float y[3], y2 = 0.0f;
    for (int i = 0; i < 3; i++) {
        y[i] = z[i] - k->m[i];
        y2 += y[i] * y[i];
    }
    float s = k->p + k->r;
    if (y2 > k->gate * k->gate * s) {
        if (++k->rejects >= k->max_rejects) {
            reset_to(k, z);
            return true;
        }
        return false;
    }
    k->rejects = 0;
    float g = k->p / s;
    for (int i = 0; i < 3; i++) k->m[i] += g * y[i];
    k->p *= 1.0f - g;
    return true;
}
