#include "nav.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "motion.h"
#include "uwb_range.h"
#include "range_filter.h"
#include "pf.h"

#define TAG "NAV"
#define HIST_LEN 32

typedef struct {
    bool have_pf;
    float ax, ay, spread;
    bool two;
    float range;
    bool have_range;
    uint32_t last_range_ms;
    int trend;
} nav_shared_t;

typedef struct {
    uint32_t t;
    float r;
} hist_t;

static portMUX_TYPE shared_lock = portMUX_INITIALIZER_UNLOCKED;
static nav_shared_t shared;
static pf_t pf;
static range_filter_t rf;
static hist_t hist[HIST_LEN];
static int hist_head = 0;
static int hist_count = 0;

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void hist_push(uint32_t t, float r) {
    hist[hist_head].t = t;
    hist[hist_head].r = r;
    hist_head = (hist_head + 1) % HIST_LEN;
    if (hist_count < HIST_LEN) hist_count++;
}

static int compute_trend(uint32_t t, float r) {
    for (int k = 0; k < hist_count; k++) {
        int idx = (hist_head - 1 - k + HIST_LEN) % HIST_LEN;
        if (t - hist[idx].t >= RANGE_TREND_WINDOW_MS) {
            float d = r - hist[idx].r;
            if (d < -RANGE_TREND_EPS_M) return -1;
            if (d > RANGE_TREND_EPS_M) return 1;
            return 0;
        }
    }
    return 0;
}

static void drop_lock(void) {
    pf_clear(&pf);
    rf_reset(&rf);
    hist_count = 0;
    hist_head = 0;
    portENTER_CRITICAL(&shared_lock);
    shared.have_pf = false;
    shared.have_range = false;
    shared.trend = 0;
    portEXIT_CRITICAL(&shared_lock);
}

static void nav_task(void *arg) {
    pf_create(&pf, esp_random());
    rf_init(&rf, RANGE_MAX_JUMP_M, RANGE_EMA_ALPHA);
    uint8_t seq = 0;
    bool locked = false;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(UWB_PERIOD_MS);

    while (1) {
        vTaskDelayUntil(&last, period);
        uwb_sample_t s;
        uint32_t t = now_ms();
        if (uwb_measure(seq++, &s)) {
            portENTER_CRITICAL(&shared_lock);
            shared.last_range_ms = t;
            portEXIT_CRITICAL(&shared_lock);
            locked = true;

            pose_t p;
            motion_get_pose(&p);
            float r, scale;
            if (rf_push(&rf, s.dist_m, s.nlos_db, &r, &scale)) {
                pf_est_t est;
                if (!pf.inited) {
                    pf_init_ring(&pf, p.x, p.y, r, p.path);
                } else if (p.path - pf.path_at_init > PF_WINDOW_M) {
                    pf_estimate(&pf, &est);
                    if (est.two_clusters) pf_init_ring(&pf, p.x, p.y, r, p.path);
                    else pf_reseed_around(&pf, est.x, est.y, PF_RESEED_SPREAD_M, p.path);
                }
                float walked = p.path - pf.path_at_init;
                float sig = hypotf(PF_SIGMA_BASE_M, PF_SIGMA_WALK_COEF * walked);
                if (sig > PF_SIGMA_MAX_M) sig = PF_SIGMA_MAX_M;
                pf_update(&pf, p.x, p.y, r, sig * scale);
                pf_estimate(&pf, &est);

                float sm = rf_smoothed(&rf);
                hist_push(t, sm);
                int trend = compute_trend(t, sm);

                portENTER_CRITICAL(&shared_lock);
                shared.have_pf = true;
                shared.ax = est.x;
                shared.ay = est.y;
                shared.spread = est.spread;
                shared.two = est.two_clusters;
                shared.range = sm;
                shared.have_range = true;
                shared.trend = trend;
                portEXIT_CRITICAL(&shared_lock);

#if LOG_CSV
                printf("CSV,%lu,%.3f,%.3f,%.3f,%.2f,%.1f\n", (unsigned long)t, p.x, p.y, r,
                       s.nlos_db, p.psi * 180.0f / (float)M_PI);
#endif
            }
        } else if (locked && (t - shared.last_range_ms) > RANGE_LOST_MS) {
            locked = false;
            drop_lock();
            ESP_LOGW(TAG, "artifact lost");
        }
    }
}

void nav_start(void) {
    memset(&shared, 0, sizeof(shared));
    xTaskCreatePinnedToCore(nav_task, "nav", 6144, NULL, 5, NULL, 0);
}

static float wrap_pi(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void nav_view_update(nav_view_t *v) {
    static nav_state_t prev = NAV_LOST;
    static float sx = 0.0f, sy = 1.0f;
    static bool arrow_init = false;

    pose_t p;
    motion_get_pose(&p);
    nav_shared_t s;
    portENTER_CRITICAL(&shared_lock);
    s = shared;
    portEXIT_CRITICAL(&shared_lock);

    uint32_t t = now_ms();
    memset(v, 0, sizeof(*v));
    v->psi_rad = p.psi;
    v->north_rad = -p.psi;
    v->mag_ok = p.mag_ok;
    v->steps = p.steps;
    v->pulse_period_ms = 1200;

    bool lost = !s.have_range || (t - s.last_range_ms) > RANGE_LOST_MS;
    if (lost) {
        v->state = NAV_LOST;
        prev = NAV_LOST;
        arrow_init = false;
        return;
    }

    v->have_range = true;
    float shown = s.range;
    if (s.have_pf && !s.two && s.spread < NAV_TRACK_ENTER_SPREAD) {
        float pf_d = hypotf(s.ax - p.x, s.ay - p.y);
        float g = (4.0f - s.range) / 2.0f;
        if (g < 0.0f) g = 0.0f;
        if (g > 1.0f) g = 1.0f;
        shown = g * s.range + (1.0f - g) * pf_d;
    }
    v->range_m = shown;
    v->two_clusters = s.two;
    v->spread_m = s.spread;
    v->trend = s.trend;

    float ratio = (NAV_FAR_M - s.range) / (NAV_FAR_M - NAV_NEAR_M);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    v->pulse_period_ms = (uint32_t)(1200.0f - ratio * 1160.0f);

    nav_state_t st;
    if (s.range < NAV_CLOSE_ENTER_M || (prev == NAV_CLOSE && s.range < NAV_CLOSE_EXIT_M)) {
        st = NAV_CLOSE;
    } else if (s.two || s.spread > NAV_SEARCH_ENTER_SPREAD || !s.have_pf) {
        st = NAV_SEARCH;
    } else if (s.spread < NAV_TRACK_ENTER_SPREAD) {
        st = NAV_TRACK;
    } else {
        st = (prev == NAV_TRACK) ? NAV_TRACK : NAV_SEARCH;
    }
    prev = st;
    v->state = st;

    if (st == NAV_TRACK && s.have_pf) {
        float theta = atan2f(s.ax - p.x, s.ay - p.y);
        float rel = wrap_pi(theta - p.psi);
        if (!arrow_init) {
            sx = sinf(rel);
            sy = cosf(rel);
            arrow_init = true;
        } else {
            sx += NAV_ARROW_ALPHA * (sinf(rel) - sx);
            sy += NAV_ARROW_ALPHA * (cosf(rel) - sy);
        }
        v->arrow_rad = atan2f(sx, sy);
        v->have_arrow = true;
    } else {
        arrow_init = false;
    }
}
