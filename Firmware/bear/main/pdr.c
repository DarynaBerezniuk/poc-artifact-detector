#include "pdr.h"
#include <math.h>

void stepdet_init(stepdet_t *s, float rate_hz, float threshold_g, uint32_t min_gap_ms) {
    float dt = 1.0f / rate_hz;
    float rc_lp = 1.0f / (2.0f * (float)M_PI * 5.0f);
    float rc_g = 1.0f;
    s->lp_alpha = dt / (rc_lp + dt);
    s->g_alpha = dt / (rc_g + dt);
    s->g_est = 1.0f;
    s->hp = 0.0f;
    s->armed = true;
    s->last_step_ms = 0;
    s->thr_hi = threshold_g;
    s->thr_lo = threshold_g * 0.5f;
    s->min_gap_ms = min_gap_ms;
}

bool stepdet_update(stepdet_t *s, float acc_mag_g, uint32_t t_ms) {
    s->g_est += s->g_alpha * (acc_mag_g - s->g_est);
    float raw = acc_mag_g - s->g_est;
    s->hp += s->lp_alpha * (raw - s->hp);
    if (s->armed) {
        if (s->hp > s->thr_hi && (t_ms - s->last_step_ms) >= s->min_gap_ms) {
            s->armed = false;
            s->last_step_ms = t_ms;
            return true;
        }
    } else if (s->hp < s->thr_lo) {
        s->armed = true;
    }
    return false;
}

void pdr_init(pdr_t *p, float step_len) {
    p->step_len = step_len;
    pdr_reset(p);
}

void pdr_reset(pdr_t *p) {
    p->x = 0.0f;
    p->y = 0.0f;
    p->path = 0.0f;
    p->steps = 0;
}

void pdr_step(pdr_t *p, float heading_rad) {
    p->x += p->step_len * sinf(heading_rad);
    p->y += p->step_len * cosf(heading_rad);
    p->path += p->step_len;
    p->steps++;
}
