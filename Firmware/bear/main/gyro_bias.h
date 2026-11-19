#pragma once
#include <stdbool.h>

typedef struct {
    float b[3];
    float m[3];
    float v[3];
    float acc_m;
    float acc_v;
    float still_ms;
    float seen_s;
} gyro_bias_t;

void gyro_bias_init(gyro_bias_t *g, const float b0[3]);
bool gyro_bias_update(gyro_bias_t *g, const float raw[3], float acc_norm, float dt);
