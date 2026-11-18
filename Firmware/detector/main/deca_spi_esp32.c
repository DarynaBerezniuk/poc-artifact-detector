#include "deca_spi_esp32.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "deca_device_api.h"

#define SPI_MAX_XFER 1024

static spi_device_handle_t spi_dev = NULL;
static int saved_cs_pin = -1;
static int spi_mode_sel = 0;

static uint8_t spi_tx_buf[SPI_MAX_XFER];
static uint8_t spi_rx_buf[SPI_MAX_XFER];

esp_err_t dw1000_spi_init(int mosi_pin, int miso_pin, int sclk_pin, int cs_pin, int freq_hz) {
    saved_cs_pin = cs_pin;
    spi_bus_config_t buscfg = {
        .miso_io_num = miso_pin,
        .mosi_io_num = mosi_pin,
        .sclk_io_num = sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MAX_XFER
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    dw1000_spi_set_speed(500000);
    return ESP_OK;
}

void dw1000_spi_set_speed(int freq_hz) {
    if (spi_dev != NULL) {
        spi_bus_remove_device(spi_dev);
        spi_dev = NULL;
    }
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = freq_hz,
        .mode = spi_mode_sel,
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = 2,
        .spics_io_num = saved_cs_pin,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev));
}

void dw1000_hard_reset(int rst_pin) {
    gpio_reset_pin(rst_pin);
    gpio_set_direction(rst_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_pull_mode(rst_pin, GPIO_FLOATING);
    gpio_set_direction(rst_pin, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(50));
}

int writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
               uint32_t bodylength, const uint8_t *bodyBuffer) {
    uint32_t total = (uint32_t)headerLength + bodylength;
    if (spi_dev == NULL || total > SPI_MAX_XFER) {
        return DWT_ERROR;
    }

    memcpy(spi_tx_buf, headerBuffer, headerLength);
    if (bodylength > 0 && bodyBuffer != NULL) {
        memcpy(spi_tx_buf + headerLength, bodyBuffer, bodylength);
    }

    spi_transaction_t t = {
        .length = total * 8,
        .tx_buffer = spi_tx_buf,
        .rx_buffer = NULL
    };
    return (spi_device_polling_transmit(spi_dev, &t) == ESP_OK) ? DWT_SUCCESS : DWT_ERROR;
}

int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer,
                uint32_t readlength, uint8_t *readBuffer) {
    uint32_t total = (uint32_t)headerLength + readlength;
    if (spi_dev == NULL || total > SPI_MAX_XFER) {
        return DWT_ERROR;
    }

    memset(spi_tx_buf, 0, total);
    memcpy(spi_tx_buf, headerBuffer, headerLength);

    spi_transaction_t t = {
        .length = total * 8,
        .tx_buffer = spi_tx_buf,
        .rx_buffer = spi_rx_buf
    };
    if (spi_device_polling_transmit(spi_dev, &t) != ESP_OK) {
        return DWT_ERROR;
    }
    memcpy(readBuffer, spi_rx_buf + headerLength, readlength);
    return DWT_SUCCESS;
}

void deca_sleep(unsigned int time_ms) {
    vTaskDelay(pdMS_TO_TICKS(time_ms));
}

decaIrqStatus_t decamutexon(void) {
    return 0;
}

void decamutexoff(decaIrqStatus_t s) {
    (void)s;
}

void dw1000_spi_deinit(void) {
    if (spi_dev != NULL) {
        spi_bus_remove_device(spi_dev);
        spi_dev = NULL;
    }
    spi_bus_free(SPI2_HOST);
}

void dw1000_spi_set_mode(int mode, int freq_hz) {
    spi_mode_sel = mode;
    dw1000_spi_set_speed(freq_hz);
}