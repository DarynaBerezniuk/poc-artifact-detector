#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    NAV_SEARCH,
    NAV_TRACK,
    NAV_CLOSE,
    NAV_LOST
} nav_state_t;

typedef struct {
    nav_state_t state;
    bool have_range;
    float range_m;
    bool have_arrow;
    float arrow_rad;
    bool two_clusters;
    int trend;
    float spread_m;
    float north_rad;
    float psi_rad;
    bool mag_ok;
    uint32_t pulse_period_ms;
    uint32_t steps;
} nav_view_t;

void nav_start(void);
void nav_view_update(nav_view_t *v);
