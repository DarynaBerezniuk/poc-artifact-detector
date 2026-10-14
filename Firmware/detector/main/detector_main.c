#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi_esp32.h"

#define TAG "DETECTOR_WROOM"

#define PIN_BUZZER          GPIO_NUM_14  // Pin 11 (D14/T6) -> R5 (220R) -> BUZZER U2
#define PIN_LED             GPIO_NUM_12  // Pin 12 (D12/T5) -> LED D2 -> R4 (1k)

#define UWB_PIN_CS          GPIO_NUM_5   // Pin 23 (D5)     -> R2 (10k) -> BU01 Pin 17 (CSn)
#define UWB_PIN_MOSI        GPIO_NUM_23  // Pin 30 (D23)    -> BU01 Pin 18 (MOSI)
#define UWB_PIN_MISO        GPIO_NUM_19  // Pin 25 (D19)    -> BU01 Pin 19 (MISO)
#define UWB_PIN_CLK         GPIO_NUM_18  // Pin 24 (D18)    -> BU01 Pin 20 (CLK)
#define UWB_PIN_IRQ         GPIO_NUM_4   // Pin 20 (D4/T0)  -> BU01 Pin 22 (IRQ)
#define UWB_PIN_RST         GPIO_NUM_2   // Pin 19 (D2/T2)  -> R3 (10k) -> BU01 Pin 3 (RSTn)

#define SPEED_OF_LIGHT      299792458.0f
#define DIST_MAX_M          12.0f
#define DIST_MIN_M          0.4f

static volatile float current_filtered_distance = -1.0f;
static volatile TickType_t last_packet_time = 0;

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

static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'E', 'T', 'C', 0xE0, 0, 0};
static uint8_t rx_buffer[128];

static uint64_t get_tx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    dwt_readtxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) ts = (ts << 8) | ts_tab[i];
    return ts;
}

static uint64_t get_rx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    dwt_readrxtimestamp(ts_tab);
    for (int i = 4; i >= 0; i--) ts = (ts << 8) | ts_tab[i];
    return ts;
}

void geiger_feedback_task(void *pvParameters) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_BUZZER) | (1ULL << PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_BUZZER, 0);
    gpio_set_level(PIN_LED, 0);

    while (1) {
        TickType_t now = xTaskGetTickCount();

        if ((now - last_packet_time) > pdMS_TO_TICKS(2500) ||
            current_filtered_distance < 0.0f ||
            current_filtered_distance > DIST_MAX_M) {
            current_filtered_distance = -1.0f;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        float clamped_dist = current_filtered_distance;
        if (clamped_dist < DIST_MIN_M) clamped_dist = DIST_MIN_M;
        if (clamped_dist > DIST_MAX_M) clamped_dist = DIST_MAX_M;

        float ratio = (DIST_MAX_M - clamped_dist) / (DIST_MAX_M - DIST_MIN_M);
        uint32_t delay_ms = (uint32_t)(1200 - (ratio * 1160));

        gpio_set_level(PIN_BUZZER, 1);
        gpio_set_level(PIN_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(15));
        gpio_set_level(PIN_BUZZER, 0);
        gpio_set_level(PIN_LED, 0);

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void uwb_initiator_task(void *pvParameters) {
    gpio_set_direction(UWB_PIN_IRQ, GPIO_MODE_INPUT);
    dw1000_hard_reset(UWB_PIN_RST);
    dw1000_spi_init(UWB_PIN_MOSI, UWB_PIN_MISO, UWB_PIN_CLK, UWB_PIN_CS);

    if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "DW1000 init FAILED! Check SPI pins 5,18,19,23 and RST pin 2.");
        vTaskDelete(NULL);
    }

    dw1000_spi_set_speed(8000000);
    dwt_configure(&config_structure);

    dwt_setleds(DWT_LEDS_ENABLE_ALL);

    ESP_LOGI(TAG, "Detector UWB Initiator started!");

    while (1) {
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {};
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        TickType_t start_wait = xTaskGetTickCount();
        bool received = false;

        while ((xTaskGetTickCount() - start_wait) < pdMS_TO_TICKS(20)) {
            uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
            if (status_reg & SYS_STATUS_RXFCG) {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
                received = true;
                break;
            }
            if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (received) {
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= sizeof(rx_buffer)) {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                if (rx_buffer[9] == 0xE1) {
                    uint32_t poll_tx_ts = (uint32_t)get_tx_timestamp_u64();
                    uint32_t resp_rx_ts = (uint32_t)get_rx_timestamp_u64();

                    uint32_t poll_rx_ts = 0, resp_tx_ts = 0;
                    for (int i = 0; i < 4; i++) {
                        poll_rx_ts |= ((uint32_t)rx_buffer[10 + i]) << (i * 8);
                        resp_tx_ts |= ((uint32_t)rx_buffer[14 + i]) << (i * 8);
                    }

                    int64_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
                    int64_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);
                    double tof = ((rtd_init - rtd_resp) / 2.0) * DWT_TIME_UNITS;
                    float dist_m = (float)(tof * SPEED_OF_LIGHT);

                    if (dist_m >= 0.0f && dist_m < 30.0f) {
                        if (current_filtered_distance < 0.0f) {
                            current_filtered_distance = dist_m;
                        } else {
                            current_filtered_distance = 0.35f * dist_m + 0.65f * current_filtered_distance;
                        }
                        last_packet_time = xTaskGetTickCount();
                    }
                }
            }
        } else {
            dwt_rxreset();
        }

        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void app_main(void) {
    xTaskCreate(geiger_feedback_task, "geiger_task", 2048, NULL, 6, NULL);
    xTaskCreate(uwb_initiator_task, "uwb_initiator", 4096, NULL, 5, NULL);
}