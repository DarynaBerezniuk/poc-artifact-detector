#include "range_filter.h"
#include <math.h>

#define NLOS_OK_DB      6.0f
#define NLOS_SCALE_MAX  4.0f
#define MAX_REJECTS     3

void rf_init(range_filter_t *r, float max_jump_m, float ema_alpha) {
    r->max_jump_m = max_jump_m;
    r->ema_alpha = ema_alpha;
    rf_reset(r);
}

void rf_reset(range_filter_t *r) {
    r->count = 0;
    r->head = 0;
    r->have_last = false;
    r->rejects = 0;
    r->have_smooth = false;
    r->smooth = 0.0f;
    r->last_ok = 0.0f;
}

static float median(const float *v, int n) {
    float t[RF_WIN];
    for (int i = 0; i < n; i++) t[i] = v[i];
    for (int i = 1; i < n; i++) {
        float k = t[i];
        int j = i - 1;
        while (j >= 0 && t[j] > k) {
            t[j + 1] = t[j];
            j--;
        }
        t[j + 1] = k;
    }
    return (n & 1) ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
}

bool rf_push(range_filter_t *r, float raw_m, float nlos_db, float *out_m, float *sigma_scale) {
    r->win[r->head] = raw_m;
    r->head = (r->head + 1) % RF_WIN;
    if (r->count < RF_WIN) r->count++;
    if (r->count < 3) return false;

    float m = median(r->win, r->count);

    if (r->have_last && fabsf(m - r->last_ok) > r->max_jump_m) {
        if (++r->rejects < MAX_REJECTS) return false;
    }
    r->rejects = 0;
    r->last_ok = m;
    r->have_last = true;

    if (!r->have_smooth) {
        r->smooth = m;
        r->have_smooth = true;
    } else {
        r->smooth = r->ema_alpha * m + (1.0f - r->ema_alpha) * r->smooth;
    }

    float s = 1.0f;
    if (nlos_db > NLOS_OK_DB) {
        s = 1.0f + 0.5f * (nlos_db - NLOS_OK_DB);
        if (s > NLOS_SCALE_MAX) s = NLOS_SCALE_MAX;
    }
    *out_m = m;
    *sigma_scale = s;
    return true;
}

float rf_smoothed(const range_filter_t *r) {
    return r->smooth;
}

bool rf_has_value(const range_filter_t *r) {
    return r->have_smooth;
}
