#include "qmc5883p.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"

#define TAG "QMC5883P"
#define QMC_ADDR        0x2C
#define REG_CHIP_ID     0x00
#define REG_DATA        0x01
#define REG_STATUS      0x09
#define REG_CTRL1       0x0A
#define REG_CTRL2       0x0B
#define CHIP_ID_EXPECT  0x80
#define LSB_PER_GAUSS   3750.0f

static i2c_master_dev_handle_t dev = NULL;

esp_err_t qmc5883p_init(void) {
    dev = i2c_bus_add(QMC_ADDR);
    if (!dev) return ESP_FAIL;

    uint8_t id = 0;
    esp_err_t e = i2c_reg_read(dev, REG_CHIP_ID, &id, 1);
    if (e != ESP_OK) return e;
    ESP_LOGI(TAG, "chip id = 0x%02X", id);
    if (id != CHIP_ID_EXPECT) ESP_LOGW(TAG, "unexpected chip id, expected 0x%02X", CHIP_ID_EXPECT);

    i2c_reg_write(dev, REG_CTRL2, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_reg_write(dev, REG_CTRL2, 0x08);
    i2c_reg_write(dev, REG_CTRL1, 0xCF);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t qmc5883p_read(float mag_gauss[3], bool *fresh) {
    uint8_t st = 0;
    esp_err_t e = i2c_reg_read(dev, REG_STATUS, &st, 1);
    if (e != ESP_OK) return e;
    *fresh = (st & 0x01) != 0;
    if (!*fresh) return ESP_OK;
    uint8_t b[6];
    e = i2c_reg_read(dev, REG_DATA, b, sizeof(b));
    if (e != ESP_OK) return e;
    for (int i = 0; i < 3; i++) {
        int16_t v = (int16_t)((b[2 * i + 1] << 8) | b[2 * i]);
        mag_gauss[i] = v / LSB_PER_GAUSS;
    }
    return ESP_OK;
}
