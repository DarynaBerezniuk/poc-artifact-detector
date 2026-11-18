#include "i2c_bus.h"
#include "esp_log.h"
#include "pins.h"

#define TAG "I2C"
#define I2C_TIMEOUT_MS  20
#define I2C_FREQ_HZ     400000

static i2c_master_bus_handle_t bus = NULL;

esp_err_t i2c_bus_init(void) {
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_PIN_SDA,
        .scl_io_num = I2C_PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, &bus);
}

esp_err_t i2c_bus_probe(uint8_t addr) {
    return i2c_master_probe(bus, addr, I2C_TIMEOUT_MS);
}

i2c_master_dev_handle_t i2c_bus_add(uint8_t addr) {
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK) return NULL;
    return dev;
}

esp_err_t i2c_reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, 2, I2C_TIMEOUT_MS);
}

esp_err_t i2c_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

void i2c_bus_scan(void) {
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_bus_probe(a) == ESP_OK) {
            ESP_LOGI(TAG, "device at 0x%02X", a);
            found++;
        }
    }
    if (!found) ESP_LOGW(TAG, "no I2C devices found, check SDA 21 / SCL 22");
}
