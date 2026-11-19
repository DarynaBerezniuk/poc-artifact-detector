#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "../../main/config.h"
#include "../../main/madgwick.h"
#include "../../main/mag_kf.h"
#include "../../main/gyro_bias.h"
#include "../../main/pdr.h"
#include "../../main/pf.h"
#include "../../main/range_filter.h"

static float frand(void) { return rand() / (float)RAND_MAX; }
static float gauss(void) {
    float u1 = frand() + 1e-7f, u2 = frand();
    return sqrtf(-2 * logf(u1)) * cosf(2 * M_PI * u2);
}

static void quat_from_yaw(float yaw, float q[4]) {
    q[0] = cosf(yaw / 2); q[1] = 0; q[2] = 0; q[3] = sinf(yaw / 2);
}

static void rotate_inv(const float q[4], const float v[3], float o[3]) {
    float q0 = q[0], q1 = -q[1], q2 = -q[2], q3 = -q[3];
    float t[3] = {
        2 * (q2 * v[2] - q3 * v[1]),
        2 * (q3 * v[0] - q1 * v[2]),
        2 * (q1 * v[1] - q2 * v[0])};
    o[0] = v[0] + q0 * t[0] + (q2 * t[2] - q3 * t[1]);
    o[1] = v[1] + q0 * t[1] + (q3 * t[0] - q1 * t[2]);
    o[2] = v[2] + q0 * t[2] + (q1 * t[1] - q2 * t[0]);
}


static int test_heading(void) {
    madgwick_t f;
    madgwick_init(&f, 2.0f);
    float incl = 60.0f * M_PI / 180.0f;
    float earth_m[3] = {cosf(incl), 0, -sinf(incl)};
    float earth_g[3] = {0, 0, 1};
    float dt = 0.01f;
    int bad = 0;
    float true_head_deg[] = {0, 90, 180, 270, 45, 0};
    float yaw_ccw = 0;
    for (int seg = 0; seg < 6; seg++) {
        float target = -true_head_deg[seg] * M_PI / 180.0f;
        float diff = atan2f(sinf(target - yaw_ccw), cosf(target - yaw_ccw));
        int turn_steps = seg == 0 ? 0 : 150;
        float start = yaw_ccw;
        for (int s = 0; s < 400 + turn_steps; s++) {
            float prev = yaw_ccw;
            if (s < turn_steps) yaw_ccw = start + diff * (s + 1) / turn_steps;
            float q[4];
            quat_from_yaw(yaw_ccw, q);
            float a[3], m[3];
            rotate_inv(q, earth_g, a);
            rotate_inv(q, earth_m, m);
            float wz = (yaw_ccw - prev) / dt;
            float gyr[3] = {0.002f, -0.001f, wz};
            for (int k = 0; k < 3; k++) { a[k] += 0.01f * gauss(); m[k] += 0.01f * gauss(); }
            if (s == 100) f.beta = 0.05f;
            if (seg == 0 && s == 0) f.beta = 2.0f;
            madgwick_update(&f, gyr, a, m, true, dt);
        }
        float h = madgwick_heading_rad(&f) * 180.0f / M_PI;
        float e = h - true_head_deg[seg];
        while (e > 180) e -= 360;
        while (e < -180) e += 360;
        printf("heading seg %d: true %.0f est %.1f err %.1f\n", seg, true_head_deg[seg], h, e);
        if (fabsf(e) > 4.0f) bad++;
    }
    return bad;
}

static int test_pf(int curved, int move_at, float *err_out) {
    pf_t pf;
    pf_create(&pf, 12345);
    range_filter_t rf;
    rf_init(&rf, 1.5f, 0.35f);
    float ax = 6.0f, ay = 4.0f;
    float px = 0, py = 0;
    float pdr_x = 0, pdr_y = 0;
    float drift_bias = 0.03f;
    float path = 0;
    float t = 0;
    int n = 0;
    pf_est_t est;
    float step = 0.7f;
    float heading = 0.0f;
    uint32_t t_ms = 0;
    int resets = 0;
    for (int i = 0; i < 200; i++) {
        float h;
        if (!curved) h = 90.0f * M_PI / 180.0f;
        else if (move_at < 0) h = (i < 25 ? 90.0f : (i < 50 ? 0.0f : 135.0f)) * M_PI / 180.0f;
        else h = (i < 25 ? 90.0f : (i < 50 ? 0.0f : (i < 100 ? 135.0f : (i < 130 ? 45.0f : 315.0f)))) * M_PI / 180.0f;
        if (i == move_at) {
            ax = px - 4.0f;
            ay = py + 6.0f;
        }
        heading = h;
        px += step * sinf(h);
        py += step * cosf(h);
        path += step;
        float hn = h + drift_bias * gauss();
        pdr_x += step * sinf(hn) * (1.0f + 0.03f * gauss());
        pdr_y += step * cosf(hn) * (1.0f + 0.03f * gauss());
        for (int k = 0; k < 2; k++) {
            float tr = hypotf(ax - px, ay - py);
            float r = tr + 0.1f * gauss();
            if (frand() < 0.03f) r += 3.0f;
            float out, sc;
            t_ms += 250;
            if (rf_push(&rf, r, 0.0f, &out, &sc)) {
                if (!pf.inited) pf_init_ring(&pf, pdr_x, pdr_y, out, path);
                float wd = path - pf.path_at_init;
                float sig = sqrtf(0.6f * 0.6f + (0.07f * wd) * (0.07f * wd));
                if (sig > 1.5f) sig = 1.5f;
                pf_update(&pf, pdr_x, pdr_y, out, sig * sc);
                if (pf_stale(&pf, pdr_x, pdr_y, out, PF_STALE_ERR_M + 2.0f * sig, t_ms, PF_STALE_MS)) {
                    pf_init_ring(&pf, pdr_x, pdr_y, out, path);
                    resets++;
                }
            }
        }
        (void)heading; (void)t; (void)n;
        pf_estimate(&pf, &est);
        if (i % 40 == 39) {
            printf("  step %3d pos(%.1f,%.1f) est(%.2f,%.2f) spread %.2f two=%d w=(%.2f,%.2f) c0=(%.1f,%.1f) c1=(%.1f,%.1f)\n",
                   i, pdr_x, pdr_y, est.x, est.y, est.spread, est.two_clusters, est.cw[0], est.cw[1],
                   est.cx[0], est.cy[0], est.cx[1], est.cy[1]);
        }
    }
    float e = hypotf(est.x - ax, est.y - ay);
    *err_out = e;
    printf("final err %.2f m (two=%d) resets %d\n", e, est.two_clusters, resets);
    return resets;
}

static int count_steps(float thr, float amp, float noise, float shake_amp, float shake_hz) {
    stepdet_t sd;
    stepdet_init(&sd, IMU_RATE_HZ, thr, STEP_MIN_GAP_MS);
    int steps = 0;
    for (int i = 0; i < 2000; i++) {
        float t = i / 100.0f;
        float a = 1.0f + amp * fmaxf(0, sinf(2 * M_PI * 1.8f * t)) + noise * gauss()
                  + shake_amp * sinf(2 * M_PI * shake_hz * t);
        if (stepdet_update(&sd, a, (uint32_t)(i * 10))) steps++;
    }
    return steps;
}

static int test_mag_kf(void) {
    mag_kf_t k;
    mag_kf_init(&k, MAG_KF_Q, MAG_KF_R, MAG_KF_GATE, MAG_KF_MAX_REJECTS);
    const float wx = 0.20f, wy = 0.0f, wz = -0.45f;
    const float noise = 0.01f;
    float yaw = 0.0f;
    double se_raw = 0, se_kf = 0, se_spike = 0;
    int n = 0, n_spike = 0, accepted_spikes = 0;
    for (int i = 0; i < 1200; i++) {
        float t = i / 100.0f;
        float w = (t > 2.0f && t < 6.0f) ? 1.57f : 0.0f;
        yaw += w * 0.01f;
        float g[3] = {0.0f, 0.0f, w + 0.01f};
        mag_kf_predict(&k, g, 0.01f);
        float c = cosf(yaw), s = sinf(yaw);
        float truth[3] = {c * wx + s * wy, -s * wx + c * wy, wz};
        if (i % 4 == 3) continue;
        float z[3];
        for (int j = 0; j < 3; j++) z[j] = truth[j] + noise * gauss();
        bool spike = (i % 150 == 100);
        if (spike) z[0] += 1.0f;
        bool acc = mag_kf_update(&k, z);
        if (i < 50) continue;
        float e_raw = 0, e_kf = 0;
        for (int j = 0; j < 3; j++) {
            e_raw += (z[j] - truth[j]) * (z[j] - truth[j]);
            e_kf += (k.m[j] - truth[j]) * (k.m[j] - truth[j]);
        }
        if (spike) {
            n_spike++;
            se_spike += e_kf;
            if (acc) accepted_spikes++;
            continue;
        }
        se_raw += e_raw;
        se_kf += e_kf;
        n++;
    }
    float rms_raw = sqrtf(se_raw / n), rms_kf = sqrtf(se_kf / n), rms_spike = sqrtf(se_spike / n_spike);
    printf("mag kalman: raw error %.4f G, filtered %.4f G (incl. 90 deg/s turns), at spikes %.4f G, spikes accepted %d of %d\n",
           rms_raw, rms_kf, rms_spike, accepted_spikes, n_spike);
    return !(rms_kf < 0.7f * rms_raw && rms_spike < 0.05f && accepted_spikes == 0);
}

static int test_straight_no_guess(void) {
    int two = 0, right = 0, wrong = 0;
    for (int seed = 1; seed <= 200; seed++) {
        srand(seed);
        pf_t pf;
        pf_create(&pf, seed * 7 + 1);
        range_filter_t rf;
        rf_init(&rf, RANGE_MAX_JUMP_M, RANGE_EMA_ALPHA);
        float ax = 6, ay = 4, px = 0, py = 0, dx = 0, dy = 0, path = 0;
        pf_est_t est;
        for (int i = 0; i < 60; i++) {
            float h = M_PI / 2;
            px += 0.7f * sinf(h);
            py += 0.7f * cosf(h);
            path += 0.7f;
            float hn = h + 0.03f * gauss();
            dx += 0.7f * sinf(hn) * (1 + 0.03f * gauss());
            dy += 0.7f * cosf(hn) * (1 + 0.03f * gauss());
            for (int k = 0; k < 2; k++) {
                float r = hypotf(ax - px, ay - py) + 0.1f * gauss(), out, sc;
                if (rf_push(&rf, r, 0, &out, &sc)) {
                    if (!pf.inited) pf_init_ring(&pf, dx, dy, out, path);
                    float sig = hypotf(PF_SIGMA_BASE_M, PF_SIGMA_WALK_COEF * (path - pf.path_at_init));
                    if (sig > PF_SIGMA_MAX_M) sig = PF_SIGMA_MAX_M;
                    pf_update(&pf, dx, dy, out, sig * sc);
                }
            }
        }
        pf_estimate(&pf, &est);
        if (est.two_clusters) two++;
        else if (hypotf(est.x - ax, est.y - ay) < 2.0f) right++;
        else wrong++;
    }
    printf("straight walk x200: two candidates %d, guessed right %d, guessed WRONG %d (must stay <= 10)\n", two, right, wrong);
    srand(7);
    return wrong > 10;
}

static float run_gyro_bias(int mode, float secs, int *updates) {
    const float truth[3] = {0.03f, -0.02f, 0.03f};
    const float start[3] = {0.0f, 0.0f, -0.098f};
    gyro_bias_t g;
    gyro_bias_init(&g, start);
    *updates = 0;
    for (int i = 0; i < (int)(secs * 100); i++) {
        float t = i / 100.0f;
        float raw[3], an;
        for (int j = 0; j < 3; j++) raw[j] = truth[j] + 0.003f * gauss();
        an = 1.0f + 0.002f * gauss();
        if (mode == 1) {
            for (int j = 0; j < 3; j++) raw[j] += 0.04f * gauss();
            an += 0.02f * gauss();
        } else if (mode == 2) {
            raw[2] += 0.5f * sinf(2 * M_PI * 0.9f * t);
            raw[0] += 0.3f * sinf(2 * M_PI * 1.8f * t);
            an += 0.2f * fmaxf(0, sinf(2 * M_PI * 1.8f * t));
        }
        if (gyro_bias_update(&g, raw, an, 0.01f)) (*updates)++;
    }
    float e = 0;
    for (int j = 0; j < 3; j++) e = fmaxf(e, fabsf(g.b[j] - truth[j]));
    return e;
}

static int test_gyro_bias(void) {
    int u0, u1, u2;
    float e0 = run_gyro_bias(0, 15.0f, &u0);
    float e1 = run_gyro_bias(1, 15.0f, &u1);
    float e2 = run_gyro_bias(2, 15.0f, &u2);
    printf("gyro bias, start 0.128 rad/s off: on a table error %.4f rad/s; in a shaking hand %d updates; walking %d updates (expect 0)\n",
           e0, u1, u2);
    (void)e1;
    (void)e2;
    return !(e0 < 0.005f && u1 == 0 && u2 == 0);
}

static void run_scale_case(int learn, float true_step, float *mean_err, float *good, float *scale_out) {
    double err_sum = 0, good_n = 0, n = 0, sc_sum = 0;
    for (int seed = 1; seed <= 20; seed++) {
        srand(100 + seed);
        pf_t pf;
        pf_create(&pf, seed * 13 + 5);
        pf.learn_scale = learn;
        range_filter_t rf;
        rf_init(&rf, RANGE_MAX_JUMP_M, RANGE_EMA_ALPHA);
        float ax = 5.0f, ay = 7.0f, px = 0, py = 0, dx = 0, dy = 0, path = 0;
        pf_est_t est;
        for (int i = 0; i < 160; i++) {
            float hd = (i < 20 ? 90.0f : (i < 45 ? 0.0f : (i < 70 ? 225.0f : (i < 100 ? 120.0f : (i < 130 ? 330.0f : 60.0f)))));
            float h = hd * (float)M_PI / 180.0f;
            px += true_step * sinf(h);
            py += true_step * cosf(h);
            path += STEP_LENGTH_M;
            float hn = h + 0.03f * gauss();
            dx += STEP_LENGTH_M * sinf(hn);
            dy += STEP_LENGTH_M * cosf(hn);
            for (int k = 0; k < 2; k++) {
                float r = hypotf(ax - px, ay - py) + 0.1f * gauss(), out, sc;
                if (rf_push(&rf, r, 0, &out, &sc)) {
                    if (!pf.inited) pf_init_ring(&pf, dx, dy, out, path);
                    float sig = hypotf(PF_SIGMA_BASE_M, PF_SIGMA_WALK_COEF * (path - pf.path_at_init));
                    if (sig > PF_SIGMA_MAX_M) sig = PF_SIGMA_MAX_M;
                    pf_update(&pf, dx, dy, out, sig * sc);
                }
            }
            if (i >= 60 && pf.inited) {
                pf_estimate(&pf, &est);
                if (est.two_clusters) continue;
                float ux, uy;
                pf_user_pos(&pf, est.scale, dx, dy, &ux, &uy);
                float e = atan2f(est.x - ux, est.y - uy) - atan2f(ax - px, ay - py);
                while (e > (float)M_PI) e -= 2.0f * (float)M_PI;
                while (e < -(float)M_PI) e += 2.0f * (float)M_PI;
                e = fabsf(e) * 180.0f / (float)M_PI;
                err_sum += e;
                if (e <= 45.0f) good_n++;
                n++;
            }
        }
        pf_estimate(&pf, &est);
        sc_sum += est.scale * STEP_LENGTH_M;
    }
    *mean_err = n ? err_sum / n : 180.0f;
    *good = n ? 100.0 * good_n / n : 0.0f;
    *scale_out = sc_sum / 20.0;
}

static int test_step_learning(void) {
    float e0, g0, s0, e1, g1, s1, e2, g2, s2;
    run_scale_case(0, 0.40f, &e0, &g0, &s0);
    run_scale_case(1, 0.40f, &e1, &g1, &s1);
    run_scale_case(1, STEP_LENGTH_M, &e2, &g2, &s2);
    printf("step learning, real step 0.40 m vs assumed %.2f m: fixed step %.0f deg (%.0f%% within 45), learned %.0f deg (%.0f%%), learned step %.2f m\n",
           STEP_LENGTH_M, e0, g0, e1, g1, s1);
    printf("step learning, real step = assumed: learned %.0f deg (%.0f%% within 45), learned step %.2f m\n", e2, g2, s2);
    srand(7);
    return !(e1 < e0 && fabsf(s1 - 0.40f) < 0.08f && e2 < 25.0f);
}

static int test_steps(void) {
    int normal = count_steps(STEP_THRESHOLD_G, 0.35f, 0.03f, 0.0f, 0.0f);
    int gentle = count_steps(STEP_THRESHOLD_G, 0.12f, 0.01f, 0.0f, 0.0f);
    int rest = count_steps(STEP_THRESHOLD_G, 0.0f, 0.015f, 0.0f, 0.0f);
    int shake = count_steps(STEP_THRESHOLD_G, 0.0f, 0.015f, 0.02f, 6.0f);
    printf("steps in 20 s at 1.8 Hz: normal %d, gentle %d (expect ~36); at rest %d, hand tremor %d (expect 0)\n",
           normal, gentle, rest, shake);
    return (normal < 32 || normal > 40) || (gentle < 32 || gentle > 40) || rest > 0 || shake > 0;
}

int main(void) {
    srand(7);
    int bad = 0;
    bad += test_heading();
    bad += test_steps();
    bad += test_mag_kf();
    bad += test_gyro_bias();
    bad += test_straight_no_guess();
    bad += test_step_learning();
    float e1, e2, e3;
    printf("straight walk:\n");
    int r1 = test_pf(0, -1, &e1);
    printf("curved walk:\n");
    int r2 = test_pf(1, -1, &e2);
    if (e2 > 1.5f || r2 != 0) bad++;
    printf("curved walk, artifact moved at step 100:\n");
    int r3 = test_pf(1, 100, &e3);
    if (e3 > 1.5f || r3 < 1) bad++;
    (void)r1;
    printf("%s\n", bad ? "FAIL" : "PASS");
    return bad != 0;
}
