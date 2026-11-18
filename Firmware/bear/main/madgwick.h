#pragma once
#include <stdbool.h>

typedef struct {
    float q0, q1, q2, q3;
    float beta;
} madgwick_t;

void madgwick_init(madgwick_t *f, float beta);
void madgwick_update(madgwick_t *f, const float gyr[3], const float acc[3], const float mag[3], bool use_mag, float dt);
float madgwick_heading_rad(const madgwick_t *f);
void madgwick_tilt_rad(const madgwick_t *f, float *roll, float *pitch);
