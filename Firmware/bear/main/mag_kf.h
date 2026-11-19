#pragma once
#include <stdbool.h>

typedef struct {
    float m[3];
    float p;
    float q;
    float r;
    float gate;
    int rejects;
    int max_rejects;
    bool inited;
} mag_kf_t;

void mag_kf_init(mag_kf_t *k, float q, float r, float gate, int max_rejects);
void mag_kf_predict(mag_kf_t *k, const float gyr[3], float dt);
bool mag_kf_update(mag_kf_t *k, const float z[3]);
