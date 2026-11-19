#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float dist_m;
    float nlos_db;
    float clock_ppm;
} uwb_sample_t;

esp_err_t uwb_init(void);
bool uwb_measure(uint8_t seq, uwb_sample_t *out);
