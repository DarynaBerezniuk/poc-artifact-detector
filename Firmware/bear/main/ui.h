#pragma once
#include "esp_err.h"

esp_err_t ui_init(void);
void ui_boot_progress(const char *line1, const char *line2, float progress);
void ui_error(const char *line1, const char *line2);
void ui_start(void);
