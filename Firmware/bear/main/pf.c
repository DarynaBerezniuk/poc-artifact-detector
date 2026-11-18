#include "pf.h"
#include <math.h>
#include <string.h>

#define JITTER_M          0.07f
#define CLUSTER_MIN_W     0.2f
#define CLUSTER_MIN_SEP   1.0f
#define CLUSTER_DOMINANT  0.85f

static uint32_t rnd(pf_t *pf) {
    uint32_t x = pf->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    pf->rng = x;
    return x;
}

static float rnd_unit(pf_t *pf) {
    return (float)(rnd(pf) >> 8) * (1.0f / 16777216.0f);
}

static float rnd_gauss(pf_t *pf) {
    float u1 = rnd_unit(pf);
    float u2 = rnd_unit(pf);
    if (u1 < 1e-7f) u1 = 1e-7f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

void pf_create(pf_t *pf, uint32_t seed) {
    memset(pf, 0, sizeof(*pf));
    pf->rng = seed ? seed : 2463534242u;
}

void pf_clear(pf_t *pf) {
    pf->inited = false;
}

void pf_init_ring(pf_t *pf, float px, float py, float r, float path) {
    for (int i = 0; i < PF_N; i++) {
        float a = 2.0f * (float)M_PI * rnd_unit(pf);
        pf->x[i] = px + r * sinf(a);
        pf->y[i] = py + r * cosf(a);
        pf->w[i] = 1.0f / PF_N;
    }
    pf->inited = true;
    pf->path_at_init = path;
    pf->init_x = px;
    pf->init_y = py;
}

void pf_reseed_around(pf_t *pf, float x, float y, float spread, float path) {
    for (int i = 0; i < PF_N; i++) {
        pf->x[i] = x + spread * rnd_gauss(pf);
        pf->y[i] = y + spread * rnd_gauss(pf);
        pf->w[i] = 1.0f / PF_N;
    }
    pf->inited = true;
    pf->path_at_init = path;
}

static void resample(pf_t *pf) {
    float nx[PF_N], ny[PF_N];
    float step = 1.0f / PF_N;
    float u = rnd_unit(pf) * step;
    float c = pf->w[0];
    int j = 0;
    for (int i = 0; i < PF_N; i++) {
        float t = u + i * step;
        while (t > c && j < PF_N - 1) {
            j++;
            c += pf->w[j];
        }
        nx[i] = pf->x[j];
        ny[i] = pf->y[j];
    }
    for (int i = 0; i < PF_N; i++) {
        pf->x[i] = nx[i] + JITTER_M * rnd_gauss(pf);
        pf->y[i] = ny[i] + JITTER_M * rnd_gauss(pf);
        pf->w[i] = 1.0f / PF_N;
    }
}

void pf_update(pf_t *pf, float px, float py, float r, float sigma) {
    if (!pf->inited) return;
    float inv2s2 = 1.0f / (2.0f * sigma * sigma);
    float sum = 0.0f;
    for (int i = 0; i < PF_N; i++) {
        float dx = pf->x[i] - px;
        float dy = pf->y[i] - py;
        float d = sqrtf(dx * dx + dy * dy);
        float e = d - r;
        pf->w[i] *= expf(-e * e * inv2s2);
        sum += pf->w[i];
    }
    if (sum < 1e-30f) {
        for (int i = 0; i < PF_N; i++) pf->w[i] = 1.0f / PF_N;
        return;
    }
    float inv = 1.0f / sum;
    float sq = 0.0f;
    for (int i = 0; i < PF_N; i++) {
        pf->w[i] *= inv;
        sq += pf->w[i] * pf->w[i];
    }
    if (1.0f / sq < 0.5f * PF_N) resample(pf);
}

void pf_estimate(const pf_t *pf, pf_est_t *est) {
    memset(est, 0, sizeof(*est));
    float mx = 0.0f, my = 0.0f;
    for (int i = 0; i < PF_N; i++) {
        mx += pf->w[i] * pf->x[i];
        my += pf->w[i] * pf->y[i];
    }
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < PF_N; i++) {
        float dx = pf->x[i] - mx, dy = pf->y[i] - my;
        vx += pf->w[i] * dx * dx;
        vy += pf->w[i] * dy * dy;
    }
    est->x = mx;
    est->y = my;
    est->spread = sqrtf(vx + vy);
    est->cx[0] = mx; est->cy[0] = my; est->cw[0] = 1.0f;
    est->dominant = 0;

    int a = 0;
    float best = -1.0f;
    for (int i = 0; i < PF_N; i++) {
        float dx = pf->x[i] - mx, dy = pf->y[i] - my;
        float d = dx * dx + dy * dy;
        if (d > best) { best = d; a = i; }
    }
    int b = 0;
    best = -1.0f;
    for (int i = 0; i < PF_N; i++) {
        float dx = pf->x[i] - pf->x[a], dy = pf->y[i] - pf->y[a];
        float d = dx * dx + dy * dy;
        if (d > best) { best = d; b = i; }
    }
    float c0x = pf->x[a], c0y = pf->y[a], c1x = pf->x[b], c1y = pf->y[b];
    float w0 = 0.0f, w1 = 0.0f;
    float s0x = 0, s0y = 0, s1x = 0, s1y = 0;
    float v0 = 0.0f, v1 = 0.0f;
    for (int it = 0; it < 6; it++) {
        s0x = s0y = s1x = s1y = 0.0f;
        w0 = w1 = 0.0f;
        for (int i = 0; i < PF_N; i++) {
            float d0 = (pf->x[i] - c0x) * (pf->x[i] - c0x) + (pf->y[i] - c0y) * (pf->y[i] - c0y);
            float d1 = (pf->x[i] - c1x) * (pf->x[i] - c1x) + (pf->y[i] - c1y) * (pf->y[i] - c1y);
            if (d0 <= d1) {
                w0 += pf->w[i]; s0x += pf->w[i] * pf->x[i]; s0y += pf->w[i] * pf->y[i];
            } else {
                w1 += pf->w[i]; s1x += pf->w[i] * pf->x[i]; s1y += pf->w[i] * pf->y[i];
            }
        }
        if (w0 > 1e-6f) { c0x = s0x / w0; c0y = s0y / w0; }
        if (w1 > 1e-6f) { c1x = s1x / w1; c1y = s1y / w1; }
    }
    v0 = v1 = 0.0f;
    for (int i = 0; i < PF_N; i++) {
        float d0 = (pf->x[i] - c0x) * (pf->x[i] - c0x) + (pf->y[i] - c0y) * (pf->y[i] - c0y);
        float d1 = (pf->x[i] - c1x) * (pf->x[i] - c1x) + (pf->y[i] - c1y) * (pf->y[i] - c1y);
        if (d0 <= d1) v0 += pf->w[i] * d0; else v1 += pf->w[i] * d1;
    }
    float sd0 = w0 > 1e-6f ? sqrtf(v0 / w0) : 0.0f;
    float sd1 = w1 > 1e-6f ? sqrtf(v1 / w1) : 0.0f;
    float sep = sqrtf((c0x - c1x) * (c0x - c1x) + (c0y - c1y) * (c0y - c1y));
    float sd_max = sd0 > sd1 ? sd0 : sd1;
    float sep_need = 2.0f * sd_max;
    if (sep_need < CLUSTER_MIN_SEP) sep_need = CLUSTER_MIN_SEP;

    est->cx[0] = c0x; est->cy[0] = c0y; est->cw[0] = w0;
    est->cx[1] = c1x; est->cy[1] = c1y; est->cw[1] = w1;
    est->dominant = w0 >= w1 ? 0 : 1;

    bool bimodal = w0 > CLUSTER_MIN_W && w1 > CLUSTER_MIN_W && sep > sep_need;
    if (bimodal) {
        est->two_clusters = true;
    } else {
        int d = est->dominant;
        if (est->cw[d] > CLUSTER_DOMINANT || !(w0 > CLUSTER_MIN_W && w1 > CLUSTER_MIN_W)) {
            est->x = est->cx[d];
            est->y = est->cy[d];
            est->spread = d == 0 ? sd0 : sd1;
        }
    }
}
