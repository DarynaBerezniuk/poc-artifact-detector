#include "deca_spi_esp32.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static spi_device_handle_t spi_dev = NULL;
static int saved_cs_pin = -1;

esp_err_t dw1000_spi_init(int mosi_pin, int miso_pin, int sclk_pin, int cs_pin) {
    saved_cs_pin = cs_pin;
    spi_bus_config_t buscfg = {
        .miso_io_num = miso_pin,
        .mosi_io_num = mosi_pin,
        .sclk_io_num = sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    dw1000_spi_set_speed(2000000);
    return ESP_OK;
}

void dw1000_spi_set_speed(int freq_hz) {
    if (spi_dev != NULL) {
        spi_bus_remove_device(spi_dev);
        spi_dev = NULL;
    }
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = freq_hz,
        .mode = 0,
        .spics_io_num = saved_cs_pin,
        .cs_ena_pretrans = 4,
        .cs_ena_posttrans = 2,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev));
}

void dw1000_hard_reset(int rst_pin) {
    gpio_reset_pin(rst_pin);
    gpio_set_direction(rst_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_direction(rst_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(rst_pin, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(50));
}

int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer) {
    uint8_t tx_buf[headerLength + bodylength];
    memcpy(tx_buf, headerBuffer, headerLength);
    if (bodylength > 0 && bodyBuffer != NULL) {
        memcpy(tx_buf + headerLength, bodyBuffer, bodylength);
    }
    spi_transaction_t t = {
        .length = (headerLength + bodylength) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = NULL
    };
    spi_device_polling_transmit(spi_dev, &t);
    return 0;
}

int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer) {
    uint8_t tx_buf[headerLength + readlength];
    uint8_t rx_buf[headerLength + readlength];
    memset(tx_buf, 0, sizeof(tx_buf));
    memcpy(tx_buf, headerBuffer, headerLength);

    spi_transaction_t t = {
        .length = (headerLength + readlength) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf
    };
    spi_device_polling_transmit(spi_dev, &t);
    memcpy(readBuffer, rx_buf + headerLength, readlength);
    return 0;
}

void deca_sleep(unsigned int time_ms) { vTaskDelay(pdMS_TO_TICKS(time_ms)); }
void decamutexon(void) {}
void decamutexoff(void) {}