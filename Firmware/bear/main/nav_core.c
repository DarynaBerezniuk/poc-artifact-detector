#include "nav_core.h"
#include <math.h>
#include <string.h>
#include "config.h"

static void hist_push(nav_core_t *c, uint32_t t, float r) {
    c->hist_t[c->hist_head] = t;
    c->hist_r[c->hist_head] = r;
    c->hist_head = (c->hist_head + 1) % NAVC_HIST_LEN;
    if (c->hist_count < NAVC_HIST_LEN) c->hist_count++;
}

static int compute_trend(const nav_core_t *c, uint32_t t, float r) {
    for (int k = 0; k < c->hist_count; k++) {
        int idx = (c->hist_head - 1 - k + NAVC_HIST_LEN) % NAVC_HIST_LEN;
        if (t - c->hist_t[idx] >= RANGE_TREND_WINDOW_MS) {
            float d = r - c->hist_r[idx];
            if (d < -RANGE_TREND_EPS_M) return -1;
            if (d > RANGE_TREND_EPS_M) return 1;
            return 0;
        }
    }
    return 0;
}

static void start_ring(nav_core_t *c, uint32_t t, const nav_pose_t *p, float r) {
    pf_init_ring(&c->pf, p->x, p->y, r, p->path);
    c->ring_path = p->path;
    c->upd_x = p->x;
    c->upd_y = p->y;
    c->upd_ms = t;
}

void navc_init(nav_core_t *c, uint32_t seed) {
    memset(c, 0, sizeof(*c));
    pf_create(&c->pf, seed);
    rf_init(&c->rf, RANGE_MAX_JUMP_M, RANGE_EMA_ALPHA);
}

void navc_reset(nav_core_t *c) {
    pf_clear(&c->pf);
    rf_reset(&c->rf);
    c->hist_count = 0;
    c->hist_head = 0;
}

navc_result_t navc_on_range(nav_core_t *c, uint32_t t, const nav_pose_t *p, float raw_m, float nlos_db,
                            nav_snap_t *snap, nav_step_info_t *info) {
    snap->last_range_ms = t;
    float r, scale;
    if (!rf_push(&c->rf, raw_m, nlos_db, &r, &scale)) return NAVC_SKIPPED;

    navc_result_t res = NAVC_UPDATED;
    pf_est_t est;
    bool fresh_ring = false;
    if (!c->pf.inited) {
        start_ring(c, t, p, r);
        fresh_ring = true;
    } else if (p->path - c->pf.path_at_init > PF_WINDOW_M) {
        pf_estimate(&c->pf, &est);
        if (est.two_clusters) {
            start_ring(c, t, p, r);
            fresh_ring = true;
        } else {
            pf_reseed_around(&c->pf, est.x, est.y, PF_RESEED_SPREAD_M, p->path);
        }
    }

    float sig = hypotf(PF_SIGMA_BASE_M, PF_SIGMA_WALK_COEF * (p->path - c->pf.path_at_init));
    if (!p->mag_ok) sig *= PF_SIGMA_NOMAG_FACTOR;
    if (sig > PF_SIGMA_MAX_M) sig = PF_SIGMA_MAX_M;
    float moved = hypotf(p->x - c->upd_x, p->y - c->upd_y);
    if (!fresh_ring && (moved >= PF_MIN_MOVE_M || (t - c->upd_ms) >= PF_MAX_IDLE_MS)) {
        pf_update(&c->pf, p->x, p->y, r, sig * scale);
        c->upd_x = p->x;
        c->upd_y = p->y;
        c->upd_ms = t;
    }
    float residual = pf_residual(&c->pf, p->x, p->y, r);
    if (!fresh_ring && pf_stale(&c->pf, p->x, p->y, r, PF_STALE_ERR_M + 2.0f * sig, t, PF_STALE_MS)) {
        start_ring(c, t, p, r);
        res = NAVC_RESTARTED;
    }
    pf_estimate(&c->pf, &est);
    est.spread = hypotf(est.spread, PF_SPREAD_DRIFT_COEF * (p->path - c->pf.path_at_init));

    float sm = rf_smoothed(&c->rf);
    hist_push(c, t, sm);

    snap->have_pf = true;
    snap->ax = est.x;
    snap->ay = est.y;
    snap->spread = est.spread;
    snap->walked = p->path - c->ring_path;
    snap->two = est.two_clusters;
    snap->range = sm;
    snap->have_range = true;
    snap->trend = compute_trend(c, t, sm);

    if (info) {
        info->r = r;
        info->scale = scale;
        info->residual = residual;
        info->est = est;
    }
    return res;
}

static float wrap_pi(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void navc_view_init(nav_view_state_t *vs) {
    memset(vs, 0, sizeof(*vs));
    vs->prev = NAV_LOST;
    vs->sy = 1.0f;
}

void navc_view(nav_view_state_t *vs, const nav_snap_t *s, const nav_pose_t *p, uint32_t t, nav_view_t *v) {
    memset(v, 0, sizeof(*v));
    v->psi_rad = p->psi;
    v->north_rad = -p->psi;
    v->mag_ok = p->mag_ok;
    v->steps = p->steps;
    v->pulse_period_ms = 1200;

    bool lost = !s->have_range || (t - s->last_range_ms) > RANGE_LOST_MS;
    if (lost) {
        v->state = NAV_LOST;
        vs->prev = NAV_LOST;
        vs->arrow_init = false;
        vs->two_prev = false;
        vs->near = false;
        return;
    }

    v->have_range = true;
    v->range_m = s->range;
    if (s->two && !vs->two_prev) vs->two_since = t;
    vs->two_prev = s->two;
    v->two_clusters = s->two && (t - vs->two_since) >= NAV_TWO_HOLD_MS;
    v->spread_m = s->spread;
    v->trend = s->trend;

    float ratio = (NAV_FAR_M - s->range) / (NAV_FAR_M - NAV_NEAR_M);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    v->pulse_period_ms = (uint32_t)(1200.0f - ratio * 1160.0f);

    nav_state_t st;
    if (s->range < NAV_CLOSE_ENTER_M || (vs->prev == NAV_CLOSE && s->range < NAV_CLOSE_EXIT_M)) {
        st = NAV_CLOSE;
    } else if (s->two || s->spread > NAV_SEARCH_ENTER_SPREAD || !s->have_pf) {
        st = NAV_SEARCH;
    } else if (s->spread < NAV_TRACK_ENTER_SPREAD && s->walked >= NAV_TRACK_MIN_WALK_M) {
        st = NAV_TRACK;
    } else {
        st = (vs->prev == NAV_TRACK) ? NAV_TRACK : NAV_SEARCH;
    }
    vs->prev = st;
    v->state = st;

    if (s->range < NAV_ARROW_MIN_M) vs->near = true;
    else if (s->range > NAV_ARROW_MIN_M + NAV_ARROW_HYST_M) vs->near = false;
    bool arrow_ok = s->have_pf && !s->two && s->spread < NAV_SEARCH_ENTER_SPREAD && s->walked >= NAV_TRACK_MIN_WALK_M;
    if (st == NAV_TRACK || (st == NAV_CLOSE && arrow_ok && !vs->near)) {
        float theta = atan2f(s->ax - p->x, s->ay - p->y);
        float rel = wrap_pi(theta - p->psi);
        if (!vs->arrow_init) {
            vs->sx = sinf(rel);
            vs->sy = cosf(rel);
            vs->arrow_init = true;
        } else {
            vs->sx += NAV_ARROW_ALPHA * (sinf(rel) - vs->sx);
            vs->sy += NAV_ARROW_ALPHA * (cosf(rel) - vs->sy);
        }
        v->arrow_rad = atan2f(vs->sx, vs->sy);
        if (NAV_ARROW_INVERT) v->arrow_rad = wrap_pi(v->arrow_rad + (float)M_PI);
        v->have_arrow = true;
    } else {
        vs->arrow_init = false;
    }
}
