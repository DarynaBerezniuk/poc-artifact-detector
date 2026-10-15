#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi_esp32.h"

#define TAG "ARTIFACT_C3"

#define UWB_PIN_CS          GPIO_NUM_5   // Pin 1 (GPIO5) -> R12 (100R) -> BU01 Pin 17 (CSn)
#define UWB_PIN_MOSI        GPIO_NUM_6   // Pin 2 (GPIO6) -> R11 (100R) -> BU01 Pin 18 (MOSI)
#define UWB_PIN_MISO        GPIO_NUM_7   // Pin 3 (GPIO7) -> R10 (100R) -> BU01 Pin 19 (MISO)
#define UWB_PIN_CLK         GPIO_NUM_4   // Pin 13 (GPIO4) -> R13 (100R) -> BU01 Pin 20 (CLK)
#define UWB_PIN_IRQ         GPIO_NUM_3   // Pin 12 (GPIO3) -> BU01 Pin 22 (IRQ)
#define UWB_PIN_RST         GPIO_NUM_2   // Pin 11 (GPIO2) -> BU01 Pin 3 (RSTn) + R9 (1k)

#define LED_BREATH_PIN      GPIO_NUM_0   // Pin 9 (GPIO0)

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_12_BIT
#define LEDC_FREQUENCY      5000

#define UUS_TO_DWT_TIME     65536
#define POLL_RX_TO_RESP_TX_DLY_UUS  2800

static dwt_config_t config_structure = {
    .chan = 2,
    .prf = DWT_PRF_64M,
    .txPreambLength = DWT_PLEN_128,
    .rxPAC = DWT_PAC8,
    .txCode = 9,
    .rxCode = 9,
    .nsSFD = 0,
    .dataRate = DWT_BR_6M8,
    .phrMode = DWT_PHRMODE_STD,
    .sfdTO = (129 + 8 - 8)
};

static uint8_t rx_buffer[128];
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', 'T', 'A', 'R', 0xE1, 0,0,0,0, 0,0,0,0, 0,0};

static uint64_t get_rx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    dwt_readrxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) {
        ts = (ts << 8) | ts_tab[i];
    }
    return ts;
}

void led_breathing_task(void *pvParameters) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_num      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LED_BREATH_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    float angle = 0.0f;
    const uint32_t safe_max_duty = 1400;

    while (1) {
        float normalized = (sinf(angle) + 1.0f) / 2.0f;
        uint32_t duty = (uint32_t)(powf(normalized, 2.2f) * safe_max_duty);

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        angle += 0.05f;
        if (angle > 2.0f * M_PI) angle -= 2.0f * M_PI;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void uwb_responder_task(void *pvParameters) {
    gpio_set_direction(UWB_PIN_IRQ, GPIO_MODE_INPUT);
    dw1000_hard_reset(UWB_PIN_RST);
    dw1000_spi_init(UWB_PIN_MOSI, UWB_PIN_MISO, UWB_PIN_CLK, UWB_PIN_CS);

    if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "DW1000 init FAILED! Check SPI pins 4,5,6,7 and RST pin 2.");
        vTaskDelete(NULL);
    }

    dw1000_spi_set_speed(8000000);
    dwt_configure(&config_structure);
    dwt_setleds(DWT_LEDS_ENABLE_ALL);

    ESP_LOGI(TAG, "Artifact UWB Responder started successfully!");

    while (1) {
        dwt_setrxaftertxdelay(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while (!((dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG) ||
                 (dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)))) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

        if (status_reg & SYS_STATUS_RXFCG) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

            if (frame_len <= sizeof(rx_buffer)) {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                if (rx_buffer[9] == 0xE0) {
                    uint64_t poll_rx_ts = get_rx_timestamp_u64();
                    uint32_t resp_tx_time = (poll_rx_ts + ((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);

                    uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + 16436;

                    for (int i = 0; i < 4; i++) {
                        tx_resp_msg[10 + i] = (uint8_t)(poll_rx_ts >> (i * 8));
                        tx_resp_msg[14 + i] = (uint8_t)(resp_tx_ts >> (i * 8));
                    }

                    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

                    if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {};
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
                    }
                }
            }
        } else {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
            dwt_rxreset();
        }
    }
}

void app_main(void) {
    xTaskCreate(led_breathing_task, "breathing_led", 2048, NULL, 5, NULL);
    xTaskCreate(uwb_responder_task, "uwb_responder", 4096, NULL, 5, NULL);
}