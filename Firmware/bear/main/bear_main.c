#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "i2c_bus.h"
#include "motion.h"
#include "nav.h"
#include "uwb_range.h"
#include "ui.h"

#define TAG "BEAR"

static void halt(const char *l1, const char *l2) {
    ui_error(l1, l2);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    if (ui_init() != ESP_OK) {
        ESP_LOGE(TAG, "display init failed");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ui_boot_progress("STARTING", "", 0.0f);

    if (i2c_bus_init() != ESP_OK) halt("I2C ERROR", "CHECK WIRING");
    i2c_bus_scan();
    if (motion_init() != ESP_OK) halt("IMU ERROR", "MPU OR MAG");
    if (uwb_init() != ESP_OK) halt("UWB ERROR", "CHECK BU01");

    motion_calibrate(ui_boot_progress);
    motion_start();
    nav_start();
    ui_start();
}
