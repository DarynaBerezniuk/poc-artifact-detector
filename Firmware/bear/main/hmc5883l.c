#include "hmc5883l.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"

#define TAG "HMC5883L"
#define HMC_ADDR        0x1E
#define REG_CRA         0x00
#define REG_CRB         0x01
#define REG_MODE        0x02
#define REG_DATA        0x03
#define REG_STATUS      0x09
#define REG_ID_A        0x0A
#define LSB_PER_GAUSS   1090.0f

static i2c_master_dev_handle_t dev = NULL;

esp_err_t hmc5883l_init(void) {
    dev = i2c_bus_add(HMC_ADDR);
    if (!dev) return ESP_FAIL;

    uint8_t id[3] = {0};
    esp_err_t e = i2c_reg_read(dev, REG_ID_A, id, sizeof(id));
    if (e != ESP_OK) return e;
    ESP_LOGI(TAG, "id = %c%c%c", id[0], id[1], id[2]);
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3') ESP_LOGW(TAG, "unexpected id, expected H43");

    i2c_reg_write(dev, REG_CRA, 0x78);
    i2c_reg_write(dev, REG_CRB, 0x20);
    i2c_reg_write(dev, REG_MODE, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

esp_err_t hmc5883l_read(float mag_gauss[3], bool *fresh) {
    uint8_t st = 0;
    esp_err_t e = i2c_reg_read(dev, REG_STATUS, &st, 1);
    if (e != ESP_OK) return e;
    *fresh = (st & 0x01) != 0;
    if (!*fresh) return ESP_OK;
    uint8_t b[6];
    e = i2c_reg_read(dev, REG_DATA, b, sizeof(b));
    if (e != ESP_OK) return e;
    int16_t x = (int16_t)((b[0] << 8) | b[1]);
    int16_t z = (int16_t)((b[2] << 8) | b[3]);
    int16_t y = (int16_t)((b[4] << 8) | b[5]);
    if (x == -4096 || y == -4096 || z == -4096) {
        *fresh = false;
        return ESP_OK;
    }
    mag_gauss[0] = x / LSB_PER_GAUSS;
    mag_gauss[1] = y / LSB_PER_GAUSS;
    mag_gauss[2] = z / LSB_PER_GAUSS;
    return ESP_OK;
}
