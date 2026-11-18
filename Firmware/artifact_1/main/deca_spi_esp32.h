#pragma once
#include <stdint.h>
#include "esp_err.h"

// NOTE: decamutexon(), decamutexoff(), writetospi(), readfromspi() and
// deca_sleep() are declared by deca_device_api.h. Only declare the
// ESP32-specific helpers here.

esp_err_t dw1000_spi_init(int mosi_pin, int miso_pin, int sclk_pin, int cs_pin, int freq_hz);
void dw1000_spi_set_speed(int freq_hz);
void dw1000_hard_reset(int rst_pin);
void dw1000_spi_deinit(void);
void dw1000_spi_set_mode(int mode, int freq_hz);