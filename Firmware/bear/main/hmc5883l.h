#pragma once
#include <stdbool.h>
#include "esp_err.h"

esp_err_t hmc5883l_init(void);
esp_err_t hmc5883l_read(float mag_gauss[3], bool *fresh);
