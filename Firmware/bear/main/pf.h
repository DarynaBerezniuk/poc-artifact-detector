#pragma once
#include <stdbool.h>
#include <stdint.h>

#define PF_N 300

typedef struct {
    float x[PF_N];
    float y[PF_N];
    float w[PF_N];
    uint32_t rng;
    bool inited;
    float path_at_init;
    float init_x, init_y;
    bool stale;
    uint32_t stale_since_ms;
} pf_t;

typedef struct {
    float x, y;
    float spread;
    bool two_clusters;
    float cx[2], cy[2], cw[2];
    int dominant;
} pf_est_t;

void pf_create(pf_t *pf, uint32_t seed);
void pf_clear(pf_t *pf);
void pf_init_ring(pf_t *pf, float px, float py, float r, float path);
void pf_reseed_around(pf_t *pf, float x, float y, float spread, float path);
void pf_update(pf_t *pf, float px, float py, float r, float sigma);
void pf_estimate(const pf_t *pf, pf_est_t *est);
float pf_residual(const pf_t *pf, float px, float py, float r);
bool pf_stale(pf_t *pf, float px, float py, float r, float thr, uint32_t t_ms, uint32_t hold_ms);
