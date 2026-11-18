#pragma once
#include <stdbool.h>

#define RF_WIN 5

typedef struct {
    float win[RF_WIN];
    int count;
    int head;
    float last_ok;
    bool have_last;
    int rejects;
    float smooth;
    bool have_smooth;
    float max_jump_m;
    float ema_alpha;
} range_filter_t;

void rf_init(range_filter_t *r, float max_jump_m, float ema_alpha);
void rf_reset(range_filter_t *r);
bool rf_push(range_filter_t *r, float raw_m, float nlos_db, float *out_m, float *sigma_scale);
float rf_smoothed(const range_filter_t *r);
bool rf_has_value(const range_filter_t *r);
