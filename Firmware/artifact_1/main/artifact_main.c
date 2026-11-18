#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi_esp32.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TAG "ARTIFACT_C3"

#define UWB_PIN_CS          GPIO_NUM_7   // BU01 CSn   (pin 17)
#define UWB_PIN_MOSI        GPIO_NUM_6   // BU01 MOSI  (pin 18)
#define UWB_PIN_MISO        GPIO_NUM_5   // BU01 MISO  (pin 19)
#define UWB_PIN_CLK         GPIO_NUM_4   // BU01 CLK   (pin 20)
#define UWB_PIN_IRQ         GPIO_NUM_3   // BU01 IRQ   (pin 22)
#define UWB_PIN_RST         GPIO_NUM_1   // BU01 RSTn  (pin 3)

#define UWB_SPI_MODE        2            // SPI mode (CPOL/CPHA)
#define SPI_SLOW_HZ         500000       // used until DW1000 is initialised (max 3 MHz)
#define SPI_FAST_HZ         4000000      // after init

#define BEACON_MODE         0
#define BEACON_PERIOD_MS    200          // nominal blink interval
#define BEACON_JITTER_MS    20           // random 0..N ms added so beacons don't stay in lock-step

#define LED_BREATH_PIN      GPIO_NUM_0   // Pin 9 (GPIO0)

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_12_BIT
#define LEDC_FREQUENCY      5000

#define ANT_DELAY_DEFAULT   16436        // calibrate per board pair for accurate distance
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
        .timer_num        = LEDC_TIMER,      // timer config uses timer_num
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,        // channel config uses timer_sel
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

// Tries every assignment of GPIO1..7 to CLK/MOSI/MISO/CS and reports the one
// that reads the DW1000 ID.
#define PIN_SCAN_ON_FAIL 0   // set to 1 to search for the SPI wiring
static void uwb_pin_scan(void) {
    const int pins[] = {1, 2, 3, 4, 5, 6, 7};
    const int n = sizeof(pins) / sizeof(pins[0]);
    ESP_LOGW(TAG, "Scanning GPIO1..7 for the DW1000 SPI wiring...");
    for (int a = 0; a < n; a++)          // CLK
    for (int b = 0; b < n; b++)          // MOSI
    for (int m = 0; m < n; m++)          // MISO
    for (int d = 0; d < n; d++) {        // CS
        if (a==b || a==m || a==d || b==m || b==d || m==d) continue;
        dw1000_spi_init(pins[b], pins[m], pins[a], pins[d], SPI_SLOW_HZ);
        uint32_t id = dwt_readdevid();
        dw1000_spi_deinit();
        if (id == 0xDECA0130) {
            ESP_LOGW(TAG, "FOUND: CLK=GPIO%d MOSI=GPIO%d MISO=GPIO%d CS=GPIO%d",
                     pins[a], pins[b], pins[m], pins[d]);
            return;
        }
        if ((id != 0x00000000 && id != 0xFFFFFFFF)) {
            ESP_LOGI(TAG, "partial: CLK=%d MOSI=%d MISO=%d CS=%d -> 0x%08lX",
                     pins[a], pins[b], pins[m], pins[d], (unsigned long)id);
        }
    }
    ESP_LOGE(TAG, "Scan finished: no valid ID. Likely power/reset/solder problem, not pin mapping.");
}

static bool uwb_bringup(void) {
    gpio_set_direction(UWB_PIN_IRQ, GPIO_MODE_INPUT);
    dw1000_hard_reset(UWB_PIN_RST);
    dw1000_spi_init(UWB_PIN_MOSI, UWB_PIN_MISO, UWB_PIN_CLK, UWB_PIN_CS, SPI_SLOW_HZ);

    // This board only reads correctly in SPI mode 2 (found by testing modes 0-3)
    dw1000_spi_set_mode(UWB_SPI_MODE, SPI_SLOW_HZ);

    uint32_t id = 0;
    for (int i = 0; i < 5; i++) {
        id = dwt_readdevid();
        ESP_LOGI(TAG, "DEV_ID = 0x%08lX (expect 0xDECA0130)", (unsigned long)id);
        if (id == 0xDECA0130) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (id != 0xDECA0130) {
        ESP_LOGE(TAG, "DW1000 not responding. Check CLK=GPIO4, MOSI=GPIO6, MISO=GPIO5, CS=GPIO7, 3V3 and GND.");
        dw1000_spi_deinit();
#if PIN_SCAN_ON_FAIL
        uwb_pin_scan();
#endif
        return false;
    }
    if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_initialise() failed");
        return false;
    }
    dw1000_spi_set_speed(SPI_FAST_HZ);
    dwt_configure(&config_structure);

    static dwt_txconfig_t txconfig = { .PGdly = 0xC2, .power = 0x07274767 };
    dwt_configuretxrf(&txconfig);

    dwt_setrxantennadelay(ANT_DELAY_DEFAULT);
    dwt_settxantennadelay(ANT_DELAY_DEFAULT);

    dwt_setleds(DWT_LEDS_ENABLE);
    return true;
}

#if BEACON_MODE
// Blink frame (IEEE 802.15.4 / ISO 24730-62 style):
//   [0]    0xC5  frame control (blink)
//   [1]    sequence number
//   [2-9]  8-byte beacon ID (ESP32 MAC, zero padded)
//   [10-11] FCS (filled by hardware)
void uwb_responder_task(void *pvParameters) {
    if (!uwb_bringup()) {
        vTaskDelete(NULL);
    }

    uint8_t frame[12] = {0xC5, 0};
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    memcpy(&frame[2], mac, 6);          // bytes 8-9 stay 0
    uint8_t seq = 0;

    ESP_LOGI(TAG, "Beacon started. ID=%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    while (1) {
        frame[1] = seq++;
        dwt_forcetrxoff();
        dwt_writetxdata(sizeof(frame), frame, 0);
        dwt_writetxfctrl(sizeof(frame), 0, 0);

        if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS) {
            uint32_t t0 = xTaskGetTickCount();
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
                if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(20)) {
                    ESP_LOGW(TAG, "TX timeout");
                    break;
                }
                vTaskDelay(1);
            }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        } else {
            ESP_LOGW(TAG, "starttx failed");
        }
        vTaskDelay(pdMS_TO_TICKS(BEACON_PERIOD_MS + (esp_random() % (BEACON_JITTER_MS + 1))));
    }
}
#else
void uwb_responder_task(void *pvParameters) {
    if (!uwb_bringup()) {
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Artifact UWB Responder started successfully!");

    uint32_t idle_spins = 0;
    uint32_t n_polls = 0, n_replies = 0, n_late = 0, n_rxerr = 0;
    uint32_t rep_polls = 0, rep_replies = 0, rep_late = 0, rep_rxerr = 0;
    int64_t last_report_us = esp_timer_get_time();
    float last_rx_dbm = 0.0f;
    while (1) {
        dwt_setrxaftertxdelay(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while (!((dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG) ||
                 (dwt_read32bitreg(SYS_STATUS_ID) & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)))) {

            esp_rom_delay_us(200);

            if (++idle_spins >= 5000) { idle_spins = 0; vTaskDelay(1); }

            int64_t tnow = esp_timer_get_time();
            if (tnow - last_report_us >= 1000000) {
                last_report_us = tnow;
                uint32_t dp = n_polls - rep_polls, dr = n_replies - rep_replies;
                uint32_t dl = n_late - rep_late,   de = n_rxerr - rep_rxerr;
                rep_polls = n_polls; rep_replies = n_replies; rep_late = n_late; rep_rxerr = n_rxerr;
                if (dp == 0) {
                    ESP_LOGW(TAG, "NO DETECTOR: 0 polls in the last second (rx errors: %lu)", (unsigned long)de);
                } else {
                    ESP_LOGI(TAG, "DETECTOR LINKED: %lu polls/s, %lu replies/s, late %lu, errors %lu, signal %.1f dBm",
                             (unsigned long)dp, (unsigned long)dr, (unsigned long)dl, (unsigned long)de, last_rx_dbm);
                }
            }
        }

        uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

        if (status_reg & SYS_STATUS_RXFCG) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD);
            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

            if (frame_len <= sizeof(rx_buffer)) {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                if (rx_buffer[9] == 0xE0) {
                    n_polls++;
                    dwt_rxdiag_t diag;
                    dwt_readdiagnostics(&diag);   // must be read before the next rxenable
                    if (diag.rxPreamCount > 0 && diag.maxGrowthCIR > 0) {
                        float n = (float)diag.rxPreamCount;
                        last_rx_dbm = 10.0f * log10f((float)diag.maxGrowthCIR * 131072.0f / (n * n)) - 121.74f;
                    }
                    uint64_t poll_rx_ts = get_rx_timestamp_u64();
                    uint32_t resp_tx_time = (poll_rx_ts + ((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);

                    uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + ANT_DELAY_DEFAULT;

                    for (int i = 0; i < 4; i++) {
                        tx_resp_msg[10 + i] = (uint8_t)(poll_rx_ts >> (i * 8));
                        tx_resp_msg[14 + i] = (uint8_t)(resp_tx_ts >> (i * 8));
                    }

                    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

                    if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {};
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
                        n_replies++;
                    } else {
                        n_late++;
                        dwt_forcetrxoff();
                    }
                }
            }
        } else {
            n_rxerr++;
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
            dwt_rxreset();
        }
    }
}

#endif

void app_main(void) {
    xTaskCreate(led_breathing_task, "breathing_led", 2048, NULL, 5, NULL);
    xTaskCreate(uwb_responder_task, "uwb_responder", 4096, NULL, 5, NULL);
}