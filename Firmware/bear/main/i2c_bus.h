#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t i2c_bus_init(void);
esp_err_t i2c_bus_probe(uint8_t addr);
i2c_master_dev_handle_t i2c_bus_add(uint8_t addr);
esp_err_t i2c_reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val);
esp_err_t i2c_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len);
void i2c_bus_scan(void);
