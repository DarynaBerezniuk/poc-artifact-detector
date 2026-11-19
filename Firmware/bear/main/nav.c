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

#define TAG "NAV"

static portMUX_TYPE shared_lock = portMUX_INITIALIZER_UNLOCKED;
static nav_snap_t shared;
static nav_core_t core;

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void to_nav_pose(const pose_t *in, nav_pose_t *out) {
    out->x = in->x;
    out->y = in->y;
    out->path = in->path;
    out->psi = in->psi;
    out->mag_ok = in->mag_ok;
    out->steps = in->steps;
}

static void nav_task(void *arg) {
    navc_init(&core, esp_random());
    nav_snap_t snap;
    memset(&snap, 0, sizeof(snap));
    uint8_t seq = 0;
    bool locked = false;
    uint32_t n_ok = 0, n_fail = 0, stat_ms = 0;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(UWB_PERIOD_MS);

    while (1) {
        vTaskDelayUntil(&last, period);
        uwb_sample_t s;
        uint32_t t = now_ms();
        if (uwb_measure(seq++, &s)) {
            n_ok++;
            locked = true;
            pose_t mp;
            motion_get_pose(&mp);
            nav_pose_t p;
            to_nav_pose(&mp, &p);
            nav_step_info_t info;
            navc_result_t res = navc_on_range(&core, t, &p, s.dist_m, s.nlos_db, &snap, &info);
            portENTER_CRITICAL(&shared_lock);
            shared = snap;
            portEXIT_CRITICAL(&shared_lock);

            if (res == NAVC_RESTARTED) {
                ESP_LOGW(TAG, "estimate does not fit the range any more (off by %.1f m), starting over", info.residual);
            }
#if LOG_CSV
            if (res != NAVC_SKIPPED) {
                printf("CSV,%lu,%.3f,%.3f,%.3f,%.2f,%.1f\n", (unsigned long)t, p.x, p.y, info.r,
                       s.nlos_db, p.psi * 180.0f / (float)M_PI);
#if LOG_UWB_RAW
                printf("RNG,%lu,%.3f,%.3f,%.2f,%.3f,%.2f,%ld\n", (unsigned long)t, s.dist_m, info.r, info.scale,
                       s.clock_ppm, s.nlos_db, (long)s.clock_raw);
#endif
                printf("EST,%lu,%.2f,%.2f,%.2f,%d,%.2f\n", (unsigned long)t, info.est.x, info.est.y, info.est.spread,
                       info.est.two_clusters, info.est.scale * STEP_LENGTH_M);
            }
#endif
        } else {
            n_fail++;
            if (locked && (t - snap.last_range_ms) > RANGE_LOST_MS) {
                locked = false;
                navc_reset(&core);
                snap.have_pf = false;
                snap.have_range = false;
                snap.trend = 0;
                portENTER_CRITICAL(&shared_lock);
                shared = snap;
                portEXIT_CRITICAL(&shared_lock);
                ESP_LOGW(TAG, "artifact lost");
            }
        }
#if LOG_UWB_RAW
        if (t - stat_ms >= 1000) {
            stat_ms = t;
            ESP_LOGI(TAG, "UWB ok %lu fail %lu (last second) locked %d", (unsigned long)n_ok, (unsigned long)n_fail, locked);
            n_ok = 0;
            n_fail = 0;
        }
#endif
    }
}

void nav_start(void) {
    memset(&shared, 0, sizeof(shared));
    xTaskCreatePinnedToCore(nav_task, "nav", 6144, NULL, 5, NULL, 0);
}

void nav_view_update(nav_view_t *v) {
    static nav_view_state_t vs;
    static bool vs_init = false;
    if (!vs_init) {
        navc_view_init(&vs);
        vs_init = true;
    }

    pose_t mp;
    motion_get_pose(&mp);
    nav_pose_t p;
    to_nav_pose(&mp, &p);
    nav_snap_t s;
    portENTER_CRITICAL(&shared_lock);
    s = shared;
    portEXIT_CRITICAL(&shared_lock);

    uint32_t t = now_ms();
    navc_view(&vs, &s, &p, t, v);

#if LOG_NAV
    static uint32_t nav_log_ms = 0;
    if (v->state != NAV_LOST && t - nav_log_ms >= 500) {
        nav_log_ms = t;
        static const char *names[] = {"SEARCH", "TRACK", "CLOSE", "LOST"};
        float ux, uy;
        navc_user_pos(&s, &p, &ux, &uy);
        float bearing = s.have_pf ? atan2f(s.ax - ux, s.ay - uy) * 180.0f / (float)M_PI : 0.0f;
        printf("NAV,%lu,%s,shown=%.2f,range=%.2f,trend=%d,spread=%.2f,two=%d,pf=%d,art=%.2f,%.2f,bearing=%.0f,psi=%.0f,arrow=%.0f,magok=%d,steps=%lu\n",
               (unsigned long)t, names[v->state], v->range_m, s.range, s.trend, s.spread, s.two, s.have_pf,
               s.ax, s.ay, bearing, p.psi * 180.0f / (float)M_PI,
               v->have_arrow ? v->arrow_rad * 180.0f / (float)M_PI : 0.0f, p.mag_ok, (unsigned long)p.steps);
    }
#endif
}
