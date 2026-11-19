#include <stdio.h>
#include <math.h>
#include "../../main/config.h"
#include "../../main/pf.h"

int main(void) {
    char line[256];
    pf_t pf;
    pf_create(&pf, 99);
    pf_est_t est;
    float last_x = 0, last_y = 0, path = 0;
    float upd_x = 0, upd_y = 0;
    unsigned long upd_t = 0, last_t = 0;
    int n = 0, resets = 0;
    while (fgets(line, sizeof(line), stdin)) {
        unsigned long t;
        float x, y, r, nlos, psi;
        if (sscanf(line, "CSV,%lu,%f,%f,%f,%f,%f", &t, &x, &y, &r, &nlos, &psi) != 6) continue;
        if (t < last_t) {
            printf("--- new run ---\n");
            pf_clear(&pf);
            path = 0;
            last_x = last_y = 0;
        }
        last_t = t;
        path += hypotf(x - last_x, y - last_y);
        last_x = x;
        last_y = y;

        int fresh = 0;
        if (!pf.inited) {
            pf_init_ring(&pf, x, y, r, path);
            fresh = 1;
        } else if (path - pf.path_at_init > PF_WINDOW_M) {
            pf_estimate(&pf, &est);
            if (est.two_clusters) {
                pf_init_ring(&pf, x, y, r, path);
                fresh = 1;
            } else {
                pf_reseed_around(&pf, est.x, est.y, PF_RESEED_SPREAD_M, path);
            }
        }
        if (fresh) {
            upd_x = x;
            upd_y = y;
            upd_t = t;
        }
        float sig = hypotf(PF_SIGMA_BASE_M, PF_SIGMA_WALK_COEF * (path - pf.path_at_init));
        if (sig > PF_SIGMA_MAX_M) sig = PF_SIGMA_MAX_M;
        float scale = nlos > 6.0f ? 1.0f + 0.5f * (nlos - 6.0f) : 1.0f;
        if (scale > 4.0f) scale = 4.0f;
        if (!fresh && (hypotf(x - upd_x, y - upd_y) >= PF_MIN_MOVE_M || t - upd_t >= PF_MAX_IDLE_MS)) {
            pf_update(&pf, x, y, r, sig * scale);
            upd_x = x;
            upd_y = y;
            upd_t = t;
        }
        if (!fresh && pf_stale(&pf, x, y, r, PF_STALE_ERR_M + 2.0f * sig, (uint32_t)t, PF_STALE_MS)) {
            printf("t=%lu reset: estimate off by %.1f m\n", t, pf_residual(&pf, x, y, r));
            pf_init_ring(&pf, x, y, r, path);
            upd_x = x;
            upd_y = y;
            upd_t = t;
            resets++;
        }
        pf_estimate(&pf, &est);
        if (++n % 50 == 0) {
            printf("t=%lu pos=(%.1f,%.1f) r=%.2f est=(%.2f,%.2f) pred=%.2f spread=%.2f two=%d\n",
                   t, x, y, r, est.x, est.y, hypotf(est.x - x, est.y - y), est.spread, est.two_clusters);
        }
    }
    printf("resets: %d\n", resets);
    return 0;
}
