#include "gc9a01.h"
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pins.h"

#define LCD_HOST        SPI3_HOST
#define LCD_CLOCK_HZ    40000000
#define CHUNK_LINES     16
#define CHUNK_BYTES     (LCD_W * CHUNK_LINES * 2)
#define MADCTL_VALUE    0x08

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t data[12];
} lcd_cmd_t;

static const lcd_cmd_t init_seq[] = {
    {0xEF, 0, {0}},
    {0xEB, 1, {0x14}},
    {0xFE, 0, {0}},
    {0xEF, 0, {0}},
    {0xEB, 1, {0x14}},
    {0x84, 1, {0x40}},
    {0x85, 1, {0xFF}},
    {0x86, 1, {0xFF}},
    {0x87, 1, {0xFF}},
    {0x88, 1, {0x0A}},
    {0x89, 1, {0x21}},
    {0x8A, 1, {0x00}},
    {0x8B, 1, {0x80}},
    {0x8C, 1, {0x01}},
    {0x8D, 1, {0x01}},
    {0x8E, 1, {0xFF}},
    {0x8F, 1, {0xFF}},
    {0xB6, 2, {0x00, 0x00}},
    {0x36, 1, {MADCTL_VALUE}},
    {0x3A, 1, {0x05}},
    {0x90, 4, {0x08, 0x08, 0x08, 0x08}},
    {0xBD, 1, {0x06}},
    {0xBC, 1, {0x00}},
    {0xFF, 3, {0x60, 0x01, 0x04}},
    {0xC3, 1, {0x13}},
    {0xC4, 1, {0x13}},
    {0xC9, 1, {0x22}},
    {0xBE, 1, {0x11}},
    {0xE1, 2, {0x10, 0x0E}},
    {0xDF, 3, {0x21, 0x0C, 0x02}},
    {0xF0, 6, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}},
    {0xF1, 6, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}},
    {0xF2, 6, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A}},
    {0xF3, 6, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F}},
    {0xED, 2, {0x1B, 0x0B}},
    {0xAE, 1, {0x77}},
    {0xCD, 1, {0x63}},
    {0x70, 9, {0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03}},
    {0xE8, 1, {0x34}},
    {0x62, 12, {0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70}},
    {0x63, 12, {0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70}},
    {0x64, 7, {0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07}},
    {0x66, 10, {0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00}},
    {0x67, 10, {0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98}},
    {0x74, 7, {0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00}},
    {0x98, 2, {0x3E, 0x07}},
    {0x35, 0, {0}},
    {0x21, 0, {0}},
};

static spi_device_handle_t lcd = NULL;
static uint8_t *dma_buf = NULL;

static void IRAM_ATTR pre_cb(spi_transaction_t *t) {
    gpio_set_level(LCD_PIN_DC, (int)(intptr_t)t->user);
}

static void lcd_cmd(uint8_t cmd) {
    spi_transaction_t t = {
        .length = 8,
        .tx_data = {cmd},
        .flags = SPI_TRANS_USE_TXDATA,
        .user = (void *)0,
    };
    spi_device_transmit(lcd, &t);
}

static void lcd_data(const uint8_t *d, size_t len) {
    if (!len) return;
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = d,
        .user = (void *)1,
    };
    spi_device_transmit(lcd, &t);
}

static void lcd_window(int x0, int y0, int x1, int y1) {
    uint8_t c[4] = {x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF};
    uint8_t r[4] = {y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF};
    lcd_cmd(0x2A);
    lcd_data(c, 4);
    lcd_cmd(0x2B);
    lcd_data(r, 4);
    lcd_cmd(0x2C);
}

esp_err_t gc9a01_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LCD_PIN_DC) | (1ULL << LCD_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    spi_bus_config_t bus = {
        .mosi_io_num = LCD_PIN_SDA,
        .miso_io_num = -1,
        .sclk_io_num = LCD_PIN_SCL,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = CHUNK_BYTES,
    };
    esp_err_t e = spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO);
    if (e != ESP_OK) return e;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = LCD_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = LCD_PIN_CS,
        .queue_size = 2,
        .pre_cb = pre_cb,
    };
    e = spi_bus_add_device(LCD_HOST, &dev, &lcd);
    if (e != ESP_OK) return e;

    dma_buf = heap_caps_malloc(CHUNK_BYTES, MALLOC_CAP_DMA);
    if (!dma_buf) return ESP_ERR_NO_MEM;

    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    for (size_t i = 0; i < sizeof(init_seq) / sizeof(init_seq[0]); i++) {
        lcd_cmd(init_seq[i].cmd);
        lcd_data(init_seq[i].data, init_seq[i].len);
    }
    lcd_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x29);
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

void gc9a01_flush(const uint16_t *fb) {
    lcd_window(0, 0, LCD_W - 1, LCD_H - 1);
    const uint8_t *src = (const uint8_t *)fb;
    for (int line = 0; line < LCD_H; line += CHUNK_LINES) {
        memcpy(dma_buf, src + (size_t)line * LCD_W * 2, CHUNK_BYTES);
        lcd_data(dma_buf, CHUNK_BYTES);
    }
}

void gc9a01_backlight(bool on) {
    (void)on;
}
