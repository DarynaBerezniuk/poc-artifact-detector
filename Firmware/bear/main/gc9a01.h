#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define LCD_W 240
#define LCD_H 240

esp_err_t gc9a01_init(void);
void gc9a01_flush(const uint16_t *fb);
void gc9a01_backlight(bool on);
