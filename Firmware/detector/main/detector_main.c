#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
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
#define SPEED_OF_LIGHT          299792458.0f
#define DIST_MAX_M              12.0f
#define DIST_MIN_M              0.4f
#define DW1000_DEVICE_ID        0xDECA0130UL
#define ANT_DELAY_DEFAULT       16436
#define NO_SIGNAL_TIMEOUT_MS    2500         // no valid reply -> artifact lost
#define NO_ARTIFACT_LOG_MS      2000         // repeat "Artifact not found" at this interval
#define POLL_PERIOD_MS          40
#define TX_DONE_TIMEOUT_US      5000         // max wait for poll TX to complete
#define RESP_WAIT_TIMEOUT_US    25000        // max wait for response from the other BU01
#define STATUS_POLL_STEP_US     200          // status register polling step
#define MSG_FUNC_POLL           0xE0
#define MSG_FUNC_RESP           0xE1
#define MSG_MIN_RESP_LEN        18           // 10 hdr + 4 poll_rx_ts + 4 resp_tx_ts (CRC excluded)


static volatile float current_filtered_distance = -1.0f;
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
    .sfdTO = (128 + 1 + 8 - 8)
};

// [0..1] frame ctrl, [2] seq, [3..4] PAN, [5..6] dest, [7..8] src, [9] function, [10..11] CRC (auto)

static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'E', 'T', 'C', MSG_FUNC_POLL, 0, 0};
static uint8_t rx_buffer[128];

static inline TickType_t ms_to_ticks(uint32_t ms) {
    TickType_t t = pdMS_TO_TICKS(ms);
    return t ? t : 1;
}

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
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
    gpio_set_level(PIN_BUZZER, 0);
    gpio_set_level(PIN_LED, 0);

    while (1) {
        float dist = current_filtered_distance;   // take one snapshot

        if (dist < 0.0f || dist > DIST_MAX_M) {
            vTaskDelay(ms_to_ticks(200));
            continue;
        }

        if (dist < DIST_MIN_M) dist = DIST_MIN_M;

        float ratio = (DIST_MAX_M - dist) / (DIST_MAX_M - DIST_MIN_M);
        uint32_t delay_ms = (uint32_t)(1200.0f - (ratio * 1160.0f));

        gpio_set_level(PIN_BUZZER, 1);
        gpio_set_level(PIN_LED, 1);

        vTaskDelay(ms_to_ticks(15));

        gpio_set_level(PIN_BUZZER, 0);
        gpio_set_level(PIN_LED, 0);

        vTaskDelay(ms_to_ticks(delay_ms));
    }
}

// failure path returns false

static bool uwb_do_ranging(uint8_t seq, float *dist_out) {
    const uint32_t clear_mask = SYS_STATUS_TXFRS | SYS_STATUS_ALL_RX_GOOD |
                                SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR;
    
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, clear_mask);
    
    tx_poll_msg[2] = seq;
    
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
    
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        ESP_LOGW(TAG, "starttx failed");
        return false;
    }

    int64_t t0 = esp_timer_get_time();
    bool tx_done = false;

    while ((esp_timer_get_time() - t0) < TX_DONE_TIMEOUT_US) {
        if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS) {
            tx_done = true;
            break;
        }
        
        esp_rom_delay_us(STATUS_POLL_STEP_US);
    }

    if (!tx_done) {
        ESP_LOGW(TAG, "TX timeout, status=0x%08lx",
                 (unsigned long)dwt_read32bitreg(SYS_STATUS_ID));
        
        dwt_forcetrxoff();
        dwt_rxreset();
        
        return false;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

    // wait for response

    int64_t t1 = esp_timer_get_time();
    bool received = false;

    while ((esp_timer_get_time() - t1) < RESP_WAIT_TIMEOUT_US) {
        uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        
        if (status_reg & SYS_STATUS_RXFCG) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD);
            received = true;
            break;
        }

        if (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            break;
        }

        esp_rom_delay_us(STATUS_POLL_STEP_US);
    }

    if (!received) {
        dwt_forcetrxoff();
        dwt_rxreset();

        return false;
    }

    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

    if (frame_len < (MSG_MIN_RESP_LEN + 2) || frame_len > sizeof(rx_buffer)) {
        return false;
    }

    dwt_readrxdata(rx_buffer, frame_len, 0);

    if (rx_buffer[9] != MSG_FUNC_RESP) {
        return false;
    }

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

    if (dist_m < -0.5f || dist_m >= 30.0f) {
        return false;
    }

    if (dist_m < 0.0f) dist_m = 0.0f;
    *dist_out = dist_m;

    return true;
}


void uwb_initiator_task(void *pvParameters) {
    gpio_set_direction(UWB_PIN_IRQ, GPIO_MODE_INPUT);
    dw1000_hard_reset(UWB_PIN_RST);

    dw1000_spi_init(UWB_PIN_MOSI, UWB_PIN_MISO, UWB_PIN_CLK, UWB_PIN_CS);

    vTaskDelay(ms_to_ticks(10));

    uint32_t dev_id = dwt_readdevid();

    ESP_LOGI(TAG, "DW1000 device ID: 0x%08lX", (unsigned long)dev_id);

    if (dev_id != DW1000_DEVICE_ID) {
        ESP_LOGE(TAG, "Wrong/no device ID! Check SPI pins 5,18,19,23 and RST pin 2.");
        vTaskDelete(NULL);
        return;
    }

    if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "DW1000 init FAILED! Check SPI pins 5,18,19,23 and RST pin 2.");
        vTaskDelete(NULL);
        return;

    }
    dw1000_spi_set_speed(8000000);

    dwt_configure(&config_structure);

    dwt_setleds(DWT_LEDS_ENABLE);
    dwt_setrxantennadelay(ANT_DELAY_DEFAULT);
    dwt_settxantennadelay(ANT_DELAY_DEFAULT);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(15000);

    ESP_LOGI(TAG, "Detector UWB Initiator started!");

    uint8_t seq = 0;
    bool artifact_present = false;
    
    TickType_t last_valid_tick = 0;
    TickType_t last_nodet_log = xTaskGetTickCount() - pdMS_TO_TICKS(NO_ARTIFACT_LOG_MS);
    
    while (1) {
        float dist_m = 0.0f;
        bool got_valid = uwb_do_ranging(seq++, &dist_m);
    
        if (got_valid) {
            if (current_filtered_distance < 0.0f) {
                current_filtered_distance = dist_m;
            } else {
                current_filtered_distance =
                    0.35f * dist_m + 0.65f * current_filtered_distance;
            }
        }
    
        TickType_t now = xTaskGetTickCount();
    
        if (got_valid) {
            last_valid_tick = now;
            if (!artifact_present) {
                artifact_present = true;
                ESP_LOGI(TAG, "Artifact detected! Distance: %.2f m", current_filtered_distance);
            }
        } else {
            // No valid reply from the other BU01
            if (artifact_present && (now - last_valid_tick) > pdMS_TO_TICKS(NO_SIGNAL_TIMEOUT_MS)) {
                artifact_present = false;
                current_filtered_distance = -1.0f;   // silences buzzer/LED
                ESP_LOGW(TAG, "Artifact not found (signal lost)");
                last_nodet_log = now;
            }

            if (!artifact_present && (now - last_nodet_log) >= pdMS_TO_TICKS(NO_ARTIFACT_LOG_MS)) {
                ESP_LOGW(TAG, "Artifact not found");
                last_nodet_log = now;
            }
        }

        vTaskDelay(ms_to_ticks(POLL_PERIOD_MS));
    }
}


void app_main(void) {
    xTaskCreate(geiger_feedback_task, "geiger_task", 2048, NULL, 6, NULL);
    xTaskCreate(uwb_initiator_task, "uwb_initiator", 6144, NULL, 5, NULL);
}