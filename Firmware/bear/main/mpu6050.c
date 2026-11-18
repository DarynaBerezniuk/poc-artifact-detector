#include "mpu6050.h"
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"

#define TAG "MPU6050"
#define MPU_ADDR        0x68
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_OUT   0x3B
#define REG_PWR_MGMT_1  0x6B
#define REG_WHO_AM_I    0x75

#define ACC_LSB_PER_G   8192.0f
#define GYR_LSB_PER_DPS 65.5f

static i2c_master_dev_handle_t dev = NULL;

esp_err_t mpu6050_init(void) {
    dev = i2c_bus_add(MPU_ADDR);
    if (!dev) return ESP_FAIL;

    uint8_t who = 0;
    esp_err_t e = i2c_reg_read(dev, REG_WHO_AM_I, &who, 1);
    if (e != ESP_OK) return e;
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who);

    e = i2c_reg_write(dev, REG_PWR_MGMT_1, 0x80);
    if (e != ESP_OK) return e;
    vTaskDelay(pdMS_TO_TICKS(100));
    i2c_reg_write(dev, REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));
    i2c_reg_write(dev, REG_SMPLRT_DIV, 9);
    i2c_reg_write(dev, REG_CONFIG, 0x03);
    i2c_reg_write(dev, REG_GYRO_CFG, 0x08);
    i2c_reg_write(dev, REG_ACCEL_CFG, 0x08);
    return ESP_OK;
}

esp_err_t mpu6050_read(float acc_g[3], float gyr_rads[3]) {
    uint8_t b[14];
    esp_err_t e = i2c_reg_read(dev, REG_ACCEL_OUT, b, sizeof(b));
    if (e != ESP_OK) return e;
    int16_t r[7];
    for (int i = 0; i < 7; i++) r[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    for (int i = 0; i < 3; i++) {
        acc_g[i] = r[i] / ACC_LSB_PER_G;
        gyr_rads[i] = r[4 + i] / GYR_LSB_PER_DPS * ((float)M_PI / 180.0f);
    }
    return ESP_OK;
}
