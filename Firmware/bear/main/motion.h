#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef void (*progress_cb_t)(const char *line1, const char *line2, float progress);

typedef struct {
    float x, y;
    float psi;
    float path;
    uint32_t steps;
    bool mag_ok;
    float roll, pitch;
    uint32_t t_ms;
} pose_t;

esp_err_t motion_init(void);
void motion_calibrate(progress_cb_t cb);
void motion_start(void);
void motion_get_pose(pose_t *out);
