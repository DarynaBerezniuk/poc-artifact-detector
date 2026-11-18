#pragma once
#include "esp_err.h"

esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read(float acc_g[3], float gyr_rads[3]);
