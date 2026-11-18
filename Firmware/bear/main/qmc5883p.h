#pragma once
#include <stdbool.h>
#include "esp_err.h"

esp_err_t qmc5883p_init(void);
esp_err_t qmc5883p_read(float mag_gauss[3], bool *fresh);
