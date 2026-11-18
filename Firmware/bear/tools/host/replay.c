#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../../main/pf.h"
#include "../../main/range_filter.h"

int main(void) {
    char line[256];
    pf_t pf;
    pf_create(&pf, 99);
    pf_est_t est;
    float last_x = 0, last_y = 0, path = 0;
    int n = 0;
    while (fgets(line, sizeof(line), stdin)) {
        unsigned long t;
        float x, y, r, nlos, psi;
        if (sscanf(line, "CSV,%lu,%f,%f,%f,%f,%f", &t, &x, &y, &r, &nlos, &psi) != 6) continue;
        path += hypotf(x - last_x, y - last_y);
        last_x = x;
        last_y = y;
        if (!pf.inited) pf_init_ring(&pf, x, y, r, path);
        float walked = path - pf.path_at_init;
        float sig = hypotf(0.4f, 0.07f * walked);
        if (sig > 1.5f) sig = 1.5f;
        float scale = nlos > 6.0f ? 1.0f + 0.5f * (nlos - 6.0f) : 1.0f;
        if (scale > 4.0f) scale = 4.0f;
        pf_update(&pf, x, y, r, sig * scale);
        pf_estimate(&pf, &est);
        if (++n % 10 == 0) {
            printf("t=%lu pos=(%.1f,%.1f) r=%.2f est=(%.2f,%.2f) spread=%.2f two=%d\n",
                   t, x, y, r, est.x, est.y, est.spread, est.two_clusters);
        }
    }
    return 0;
}
