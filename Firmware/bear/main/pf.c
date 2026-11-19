#include "pf.h"
#include <math.h>
#include <string.h>

#define JITTER_M          0.15f
#define CLUSTER_MIN_W     0.2f
#define CLUSTER_MIN_SEP   1.0f
#define CLUSTER_DOMINANT  0.85f
#define SCALE_MIN         0.6f
#define SCALE_MAX         1.5f
#define SCALE_INIT_W      0.35f
#define SCALE_HINT_W      0.15f
#define SCALE_JITTER      0.01f
#define SCALE_RESEED_SD   0.05f

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

static float clamp_scale(float s) {
    if (s < SCALE_MIN) return SCALE_MIN;
    if (s > SCALE_MAX) return SCALE_MAX;
    return s;
}

static float mean_scale(const pf_t *pf) {
    float s = 0.0f;
    for (int i = 0; i < PF_N; i++) s += pf->w[i] * pf->s[i];
    return s;
}

static void user_pos_i(const pf_t *pf, int i, float px, float py, float *ux, float *uy) {
    *ux = pf->ox + pf->s[i] * (px - pf->ox);
    *uy = pf->oy + pf->s[i] * (py - pf->oy);
}

void pf_create(pf_t *pf, uint32_t seed) {
    memset(pf, 0, sizeof(*pf));
    pf->rng = seed ? seed : 2463534242u;
    pf->s_hint = 1.0f;
    pf->s_hint_w = SCALE_INIT_W;
    for (int i = 0; i < PF_N; i++) pf->s[i] = 1.0f;
}

void pf_remember_scale(pf_t *pf, float scale) {
    pf->s_hint = clamp_scale(scale);
    pf->s_hint_w = SCALE_HINT_W;
}

void pf_user_pos(const pf_t *pf, float scale, float px, float py, float *ux, float *uy) {
    *ux = pf->ox + scale * (px - pf->ox);
    *uy = pf->oy + scale * (py - pf->oy);
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
        pf->s[i] = pf->learn_scale ? clamp_scale(pf->s_hint + pf->s_hint_w * (2.0f * rnd_unit(pf) - 1.0f)) : 1.0f;
    }
    pf->ox = px;
    pf->oy = py;
    pf->inited = true;
    pf->path_at_init = path;
    pf->init_x = px;
    pf->init_y = py;
    pf->stale = false;
}

void pf_reseed_around(pf_t *pf, float px, float py, float x, float y, float spread, float path) {
    float sc = mean_scale(pf);
    float ux, uy;
    pf_user_pos(pf, sc, px, py, &ux, &uy);
    float sx = px - ux, sy = py - uy;
    for (int i = 0; i < PF_N; i++) {
        pf->x[i] = x + sx + spread * rnd_gauss(pf);
        pf->y[i] = y + sy + spread * rnd_gauss(pf);
        pf->w[i] = 1.0f / PF_N;
        pf->s[i] = pf->learn_scale ? clamp_scale(sc + SCALE_RESEED_SD * rnd_gauss(pf)) : 1.0f;
    }
    pf->ox = px;
    pf->oy = py;
    pf->inited = true;
    pf->path_at_init = path;
    pf->stale = false;
}

static void resample(pf_t *pf) {
    static float nx[PF_N], ny[PF_N], ns[PF_N];
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
        ns[i] = pf->s[j];
    }
    for (int i = 0; i < PF_N; i++) {
        pf->x[i] = nx[i] + JITTER_M * rnd_gauss(pf);
        pf->y[i] = ny[i] + JITTER_M * rnd_gauss(pf);
        pf->s[i] = pf->learn_scale ? clamp_scale(ns[i] + SCALE_JITTER * rnd_gauss(pf)) : 1.0f;
        pf->w[i] = 1.0f / PF_N;
    }
}

void pf_update(pf_t *pf, float px, float py, float r, float sigma) {
    if (!pf->inited) return;
    float inv2s2 = 1.0f / (2.0f * sigma * sigma);
    float sum = 0.0f;
    for (int i = 0; i < PF_N; i++) {
        float ux, uy;
        user_pos_i(pf, i, px, py, &ux, &uy);
        float dx = pf->x[i] - ux;
        float dy = pf->y[i] - uy;
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
    est->scale = mean_scale(pf);
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
    float sep_need = 3.0f * sd_max;
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

float pf_residual(const pf_t *pf, float px, float py, float r) {
    float res = 0.0f;
    for (int i = 0; i < PF_N; i++) {
        float ux, uy;
        user_pos_i(pf, i, px, py, &ux, &uy);
        float d = hypotf(pf->x[i] - ux, pf->y[i] - uy);
        res += pf->w[i] * fabsf(d - r);
    }
    return res;
}

bool pf_stale(pf_t *pf, float px, float py, float r, float thr, uint32_t t_ms, uint32_t hold_ms) {
    if (!pf->inited) return false;
    if (pf_residual(pf, px, py, r) <= thr) {
        pf->stale = false;
        return false;
    }
    if (!pf->stale) {
        pf->stale = true;
        pf->stale_since_ms = t_ms;
        return false;
    }
    return (t_ms - pf->stale_since_ms) >= hold_ms;
}
