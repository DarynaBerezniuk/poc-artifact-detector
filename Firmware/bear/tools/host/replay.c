#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../main/config.h"
#include "../../main/nav_core.h"

typedef struct {
    double t_state[4];
    double t_sideways;
    double t_walk_around;
    double t_arrow;
    double t_two_raw;
    double t_total;
    int restarts;
    int samples;
} stats_t;

typedef struct {
    double t_state[4];
    double t_two;
    double t_total;
} dev_stats_t;

#define MAX_ANCHORS 4096
#define ANCHOR_R_M 0.7f
#define ANCHOR_WINDOW_MS 20000
#define ANCHOR_MIN_DIST_M 1.5f

typedef struct {
    int run;
    unsigned long t;
    float x, y;
} anchor_t;

static anchor_t anchors[MAX_ANCHORS];
static int n_anchors = 0;
static double arrow_checks = 0, arrow_good = 0, arrow_err_sum = 0;
static double jit_sum[3], jit_time[3], flips[3];
#define HIST_N 64
#define WALK_CHECK_MS 2000
#define WALK_MIN_M 1.0f
static struct {
    unsigned long t;
    float x, y, r, arrow_world;
    int arrow;
} hist[HIST_N];
static int hist_head = 0, hist_count = 0;
static double walk_checks = 0, walk_agree = 0, walk_err_sum = 0;
static int prev_arrow_valid = 0;
static float prev_arrow = 0;

static const char *names[] = {"SEARCH", "TRACK", "CLOSE", "LOST"};

static nav_core_t core;
static nav_snap_t snap;
static nav_view_state_t vs;
static stats_t st;
static dev_stats_t dev;
static int verbose = 0;
static unsigned seed = 99;
static int passes = 10;
static float step_scale = 1.0f;
static int pass = 0;
static int run = 0;
static int has_rng = 0;

static struct {
    int valid;
    unsigned long t;
    float x, y, r, nlos, psi;
} pending;

static float path = 0, last_x = 0, last_y = 0;
static int mag_ok = 1;
static unsigned long last_sample_t = 0, last_view_t = 0, last_print_t = 0;
static unsigned long dev_last_t = 0;
static int dev_have = 0;

static void start_run(void) {
    run++;
    navc_init(&core, seed + 1000u * pass + run);
    memset(&snap, 0, sizeof(snap));
    navc_view_init(&vs);
    path = last_x = last_y = 0;
    last_sample_t = last_view_t = last_print_t = 0;
    pending.valid = 0;
    dev_have = 0;
    hist_count = 0;
    hist_head = 0;
    if (verbose && pass == 0) printf("--- run %d ---\n", run);
}

static void process(unsigned long t, float x, float y, float raw, float nlos, float psi_deg) {
    x *= step_scale;
    y *= step_scale;
    if (last_sample_t && t - last_sample_t > RANGE_LOST_MS) {
        navc_reset(&core);
        snap.have_pf = false;
        snap.have_range = false;
    }
    path += hypotf(x - last_x, y - last_y);
    last_x = x;
    last_y = y;
    nav_pose_t p = {x, y, path, psi_deg * (float)M_PI / 180.0f, mag_ok != 0, 0};
    nav_step_info_t info;
    navc_result_t res = navc_on_range(&core, (uint32_t)t, &p, raw, nlos, &snap, &info);
    if (res == NAVC_RESTARTED) {
        st.restarts++;
        if (verbose && pass == 0) printf("t=%lu restart: estimate off by %.1f m\n", t, info.residual);
    }
    st.samples++;

    nav_view_t v;
    navc_view(&vs, &snap, &p, (uint32_t)t, &v);
    float world_bearing = v.arrow_rad + p.psi;
    if (v.have_arrow && snap.have_pf) {
        const anchor_t *best = NULL;
        long best_dt = ANCHOR_WINDOW_MS + 1;
        for (int i = 0; i < n_anchors; i++) {
            if (anchors[i].run != run) continue;
            long d = labs((long)anchors[i].t - (long)t);
            if (d < best_dt) {
                best_dt = d;
                best = &anchors[i];
            }
        }
        if (best && hypotf(best->x - x, best->y - y) > ANCHOR_MIN_DIST_M) {
            float b_est = world_bearing;
            float b_true = atan2f(best->x - x, best->y - y);
            float e = b_est - b_true;
            while (e > (float)M_PI) e -= 2.0f * (float)M_PI;
            while (e < -(float)M_PI) e += 2.0f * (float)M_PI;
            e = fabsf(e) * 180.0f / (float)M_PI;
            arrow_checks++;
            arrow_err_sum += e;
            if (e <= 45.0f) arrow_good++;
        }
    }
    if (v.have_arrow && prev_arrow_valid && last_view_t) {
        float d = world_bearing - prev_arrow;
        while (d > (float)M_PI) d -= 2.0f * (float)M_PI;
        while (d < -(float)M_PI) d += 2.0f * (float)M_PI;
        d = fabsf(d) * 180.0f / (float)M_PI;
        int band = snap.range < 2.0f ? 0 : (snap.range < 4.0f ? 1 : 2);
        double dt = (t - last_view_t) / 1000.0;
        if (dt > 0 && dt <= 0.5) {
            jit_sum[band] += d;
            jit_time[band] += dt;
            if (d > 45.0f) flips[band]++;
        }
    }
    prev_arrow_valid = v.have_arrow;
    prev_arrow = world_bearing;

    if (snap.have_range) {
        for (int k = 0; k < hist_count; k++) {
            int i = (hist_head - 1 - k + HIST_N) % HIST_N;
            if (t - hist[i].t < WALK_CHECK_MS) continue;
            float dx = x - hist[i].x, dy = y - hist[i].y, dm = hypotf(dx, dy);
            if (hist[i].arrow && dm >= WALK_MIN_M && t - hist[i].t < 2 * WALK_CHECK_MS) {
                float c_true = -(snap.range - hist[i].r) / dm;
                if (c_true > 1.0f) c_true = 1.0f;
                if (c_true < -1.0f) c_true = -1.0f;
                float c_pred = cosf(atan2f(dx, dy) - hist[i].arrow_world);
                float a_true = acosf(c_true), a_pred = acosf(c_pred);
                walk_checks++;
                walk_err_sum += fabsf(a_true - a_pred) * 180.0f / (float)M_PI;
                if ((c_true > 0) == (c_pred > 0)) walk_agree++;
            }
            break;
        }
        hist[hist_head].t = t;
        hist[hist_head].x = x;
        hist[hist_head].y = y;
        hist[hist_head].r = snap.range;
        hist[hist_head].arrow_world = world_bearing;
        hist[hist_head].arrow = v.have_arrow;
        hist_head = (hist_head + 1) % HIST_N;
        if (hist_count < HIST_N) hist_count++;
    }
    if (last_view_t) {
        double dt = (t - last_view_t) / 1000.0;
        if (dt > 0.5) dt = 0.5;
        st.t_total += dt;
        st.t_state[v.state] += dt;
        if (v.state == NAV_SEARCH) {
            if (v.two_clusters) st.t_sideways += dt;
            else st.t_walk_around += dt;
            if (snap.two) st.t_two_raw += dt;
        }
        if (v.have_arrow) st.t_arrow += dt;
    }
    last_view_t = t;
    last_sample_t = t;

    if (verbose && pass == 0 && t - last_print_t >= 1000) {
        last_print_t = t;
        const char *text = "";
        if (v.state == NAV_SEARCH) text = v.two_clusters ? "MOVE SIDEWAYS" : "WALK AROUND";
        printf("t=%6.1f %-6s %-13s r=%5.2f pos=(%6.1f,%6.1f) art=(%6.1f,%6.1f) spread=%5.2f two=%d walked=%5.1f arrow=%s\n",
               t / 1000.0, names[v.state], text, snap.range, x, y, snap.ax, snap.ay, snap.spread, snap.two,
               snap.walked, v.have_arrow ? "yes" : "no");
    }
}

static void flush_pending(void) {
    if (!pending.valid) return;
    pending.valid = 0;
    process(pending.t, pending.x, pending.y, pending.r, pending.nlos, pending.psi);
}

static void device_nav_line(const char *line) {
    unsigned long t;
    char state[16];
    if (sscanf(line, "NAV,%lu,%15[^,]", &t, state) != 2) return;
    int two = strstr(line, ",two=1,") != NULL;
    int idx = 0;
    for (int i = 0; i < 4; i++) if (!strcmp(state, names[i])) idx = i;
    if (dev_have && t > dev_last_t) {
        double dt = (t - dev_last_t) / 1000.0;
        if (dt > 1.0) dt = 1.0;
        dev.t_total += dt;
        dev.t_state[idx] += dt;
        if (idx == NAV_SEARCH && two) dev.t_two += dt;
    }
    dev_last_t = t;
    dev_have = 1;
}

int main(int argc, char **argv) {
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-v")) verbose = 1;
        else if (!strcmp(argv[i], "-s") && i + 1 < argc) seed = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "-n") && i + 1 < argc) passes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-k") && i + 1 < argc) step_scale = (float)atof(argv[++i]);
    }

    size_t cap = 1 << 20, len = 0;
    char *buf = malloc(cap);
    size_t n;
    while ((n = fread(buf + len, 1, cap - len, stdin)) > 0) {
        len += n;
        if (len == cap) buf = realloc(buf, cap *= 2);
    }
    buf = realloc(buf, len + 1);
    buf[len] = 0;
    has_rng = strstr(buf, "RNG,") != NULL;
    if (passes < 1) passes = 1;
    char *work = malloc(len + 1);

    {
        memcpy(work, buf, len + 1);
        int arun = 1, nsamp = 0;
        unsigned long at = 0, prev_t = 0;
        float ax_ = 0, ay_ = 0, ar = 99;
        int have = 0;
        char *save = NULL;
        for (char *line = strtok_r(work, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
            unsigned long t;
            float x, y, r, nlos, psi, raw, filt;
            if (strstr(line, "MPU6050: WHO_AM_I")) {
                if (nsamp) arun++;
                nsamp = 0;
                prev_t = 0;
                continue;
            }
            if (sscanf(line, "CSV,%lu,%f,%f,%f,%f,%f", &t, &x, &y, &r, &nlos, &psi) == 6) {
                if (prev_t && t < prev_t) {
                    arun++;
                    nsamp = 0;
                }
                prev_t = t;
                nsamp++;
                at = t;
                ax_ = x;
                ay_ = y;
                ar = r;
                have = 1;
                if (!has_rng && ar < ANCHOR_R_M && n_anchors < MAX_ANCHORS)
                    anchors[n_anchors++] = (anchor_t){arun, at, ax_, ay_};
                continue;
            }
            if (has_rng && have && sscanf(line, "RNG,%lu,%f,%f", &t, &raw, &filt) == 3 && t == at) {
                if (raw < ANCHOR_R_M && n_anchors < MAX_ANCHORS) anchors[n_anchors++] = (anchor_t){arun, at, ax_, ay_};
                have = 0;
            }
        }
        (void)ar;
    }

    for (pass = 0; pass < passes; pass++) {
    memcpy(work, buf, len + 1);
    run = 0;
    last_sample_t = 0;
    int samples_before = st.samples;
    start_run();
    char *save = NULL;
    for (char *line = strtok_r(work, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
        unsigned long t;
        float x, y, r, nlos, psi, raw, filt, scale, ppm;
        char *m;
        if (strstr(line, "MPU6050: WHO_AM_I")) {
            flush_pending();
            if (st.samples > samples_before) start_run();
            continue;
        }
        if ((m = strstr(line, "MOTION: psi")) != NULL) {
            char *k = strstr(m, " mag ");
            if (k) mag_ok = atoi(k + 5);
            continue;
        }
        if (!strncmp(line, "NAV,", 4)) {
            if (pass == 0) device_nav_line(line);
            continue;
        }
        if (sscanf(line, "CSV,%lu,%f,%f,%f,%f,%f", &t, &x, &y, &r, &nlos, &psi) == 6) {
            flush_pending();
            if (last_sample_t && t < last_sample_t) start_run();
            pending.valid = 1;
            pending.t = t;
            pending.x = x;
            pending.y = y;
            pending.r = r;
            pending.nlos = nlos;
            pending.psi = psi;
            if (!has_rng) flush_pending();
            continue;
        }
        if (sscanf(line, "RNG,%lu,%f,%f,%f,%f", &t, &raw, &filt, &scale, &ppm) == 5) {
            if (pending.valid && pending.t == t) {
                pending.r = raw;
                flush_pending();
            }
            continue;
        }
    }
    flush_pending();
    }

    double T = st.t_total > 0 ? st.t_total : 1;
    printf("replay: %d range samples, %d run(s), %.0f s, ranges from %s, averaged over %d filter seeds\n",
           st.samples / passes, run, st.t_total / passes, has_rng ? "raw RNG lines" : "filtered CSV lines", passes);
    printf("  SEARCH %4.0f%%  (two candidates %4.0f%%; text MOVE SIDEWAYS %4.0f%%, WALK AROUND %4.0f%%)\n",
           100 * st.t_state[NAV_SEARCH] / T, 100 * st.t_two_raw / T, 100 * st.t_sideways / T, 100 * st.t_walk_around / T);
    printf("  TRACK  %4.0f%%   CLOSE %4.0f%%   arrow on screen %4.0f%%   restarts %.1f\n",
           100 * st.t_state[NAV_TRACK] / T, 100 * st.t_state[NAV_CLOSE] / T, 100 * st.t_arrow / T,
           (double)st.restarts / passes);
    if (arrow_checks > 0)
        printf("  arrow check against %d near-artifact spots: within 45 deg %4.0f%%, mean error %.0f deg (%d checks)\n",
               n_anchors, 100 * arrow_good / arrow_checks, arrow_err_sum / arrow_checks, (int)(arrow_checks / passes));
    else
        printf("  arrow check: no near-artifact spots in the log to compare with\n");
    if (walk_checks > 0)
        printf("  walking check (range change vs arrow, whole log): right side %4.0f%%, angle error %.0f deg (%d checks)\n",
               100 * walk_agree / walk_checks, walk_err_sum / walk_checks, (int)(walk_checks / passes));
    {
        static const char *bands[] = {"< 2 m", "2-4 m", "> 4 m"};
        printf("  shown arrow direction noise (turning of the device excluded):");
        for (int b = 0; b < 3; b++)
            if (jit_time[b] > 0)
                printf("  %s %.0f deg/s (%.1f jumps >45 deg per min)", bands[b], jit_sum[b] / jit_time[b],
                       60.0 * flips[b] / jit_time[b]);
        printf("\n");
    }
    if (dev.t_total > 0) {
        double D = dev.t_total;
        printf("device (firmware that wrote the log), %.0f s:\n", dev.t_total);
        printf("  SEARCH %4.0f%%  (two candidates %4.0f%%)\n  TRACK  %4.0f%%   CLOSE %4.0f%%\n",
               100 * dev.t_state[NAV_SEARCH] / D, 100 * dev.t_two / D, 100 * dev.t_state[NAV_TRACK] / D,
               100 * dev.t_state[NAV_CLOSE] / D);
    }
    free(work);
    free(buf);
    return 0;
}
