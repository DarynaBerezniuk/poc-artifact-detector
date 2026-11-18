#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float g_est;
    float hp;
    bool armed;
    uint32_t last_step_ms;
    float lp_alpha;
    float g_alpha;
    float thr_hi;
    float thr_lo;
    uint32_t min_gap_ms;
} stepdet_t;

void stepdet_init(stepdet_t *s, float rate_hz, float threshold_g, uint32_t min_gap_ms);
bool stepdet_update(stepdet_t *s, float acc_mag_g, uint32_t t_ms);

typedef struct {
    float x, y;
    float path;
    uint32_t steps;
    float step_len;
} pdr_t;

void pdr_init(pdr_t *p, float step_len);
void pdr_reset(pdr_t *p);
void pdr_step(pdr_t *p, float heading_rad);
