#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "pf.h"
#include "range_filter.h"

#define NAVC_HIST_LEN 32

typedef enum {
    NAV_SEARCH,
    NAV_TRACK,
    NAV_CLOSE,
    NAV_LOST
} nav_state_t;

typedef struct {
    float x, y;
    float path;
    float psi;
    bool mag_ok;
    uint32_t steps;
} nav_pose_t;

typedef struct {
    bool have_pf;
    float ax, ay, spread, walked;
    bool two;
    float range;
    bool have_range;
    uint32_t last_range_ms;
    int trend;
} nav_snap_t;

typedef struct {
    pf_t pf;
    range_filter_t rf;
    float ring_path;
    float upd_x, upd_y;
    uint32_t upd_ms;
    uint32_t hist_t[NAVC_HIST_LEN];
    float hist_r[NAVC_HIST_LEN];
    int hist_head;
    int hist_count;
} nav_core_t;

typedef struct {
    float r;
    float scale;
    float residual;
    pf_est_t est;
} nav_step_info_t;

typedef enum {
    NAVC_SKIPPED,
    NAVC_UPDATED,
    NAVC_RESTARTED
} navc_result_t;

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

typedef struct {
    nav_state_t prev;
    bool two_prev;
    uint32_t two_since;
    float sx, sy;
    bool arrow_init;
} nav_view_state_t;

void navc_init(nav_core_t *c, uint32_t seed);
void navc_reset(nav_core_t *c);
navc_result_t navc_on_range(nav_core_t *c, uint32_t t, const nav_pose_t *p, float raw_m, float nlos_db,
                            nav_snap_t *snap, nav_step_info_t *info);
void navc_view_init(nav_view_state_t *vs);
void navc_view(nav_view_state_t *vs, const nav_snap_t *s, const nav_pose_t *p, uint32_t t, nav_view_t *v);
