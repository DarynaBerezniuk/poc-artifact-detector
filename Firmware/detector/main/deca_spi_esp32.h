#pragma once
#include <stdint.h>
#include "esp_err.h"

esp_err_t dw1000_spi_init(int mosi_pin, int miso_pin, int sclk_pin, int cs_pin);
void dw1000_spi_set_speed(int freq_hz);
void dw1000_hard_reset(int rst_pin);
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer);
int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer);
void deca_sleep(unsigned int time_ms);
void decamutexon(void);
void decamutexoff(void);