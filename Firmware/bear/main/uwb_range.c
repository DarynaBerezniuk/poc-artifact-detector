#include "uwb_range.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi_esp32.h"
#include "config.h"
#include "pins.h"

#define TAG "UWB"
#define SPEED_OF_LIGHT          299792458.0f
#define DW1000_DEVICE_ID        0xDECA0130UL
#define TX_DONE_TIMEOUT_US      5000
#define RESP_WAIT_TIMEOUT_US    25000
#define STATUS_POLL_STEP_US     200
#define MSG_FUNC_POLL           0xE0
#define MSG_FUNC_RESP           0xE1
#define MSG_MIN_RESP_LEN        18
#define RX_POWER_CONST_A        121.74f

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

static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'E', 'T', 'C', MSG_FUNC_POLL, 0, 0};
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

static float nlos_indicator_db(void) {
    dwt_rxdiag_t d;
    dwt_readdiagnostics(&d);
    float n = (float)d.rxPreamCount;
    float c = (float)d.maxGrowthCIR;
    float f1 = (float)d.firstPathAmp1, f2 = (float)d.firstPathAmp2, f3 = (float)d.firstPathAmp3;
    float fp = f1 * f1 + f2 * f2 + f3 * f3;
    if (n < 1.0f || c < 1.0f || fp < 1.0f) return 0.0f;
    float rx_db = 10.0f * log10f(c * 131072.0f / (n * n)) - RX_POWER_CONST_A;
    float fp_db = 10.0f * log10f(fp / (n * n)) - RX_POWER_CONST_A;
    float diff = rx_db - fp_db;
    return diff < 0.0f ? 0.0f : diff;
}

esp_err_t uwb_init(void) {
    gpio_set_direction(UWB_PIN_IRQ, GPIO_MODE_INPUT);
#if UWB_PIN_RST >= 0
    dw1000_hard_reset(UWB_PIN_RST);
#endif
    dw1000_spi_init(UWB_PIN_MOSI, UWB_PIN_MISO, UWB_PIN_CLK, UWB_PIN_CS, UWB_SPI_SLOW_HZ);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t dev_id = 0;
    int mode_found = -1;
    for (int k = 0; k < 4 && mode_found < 0; k++) {
        int mode = (UWB_SPI_MODE + k) & 3;
        dw1000_spi_set_mode(mode, UWB_SPI_SLOW_HZ);
        for (int i = 0; i < 3; i++) {
            dev_id = dwt_readdevid();
            if (dev_id == DW1000_DEVICE_ID) { mode_found = mode; break; }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    ESP_LOGI(TAG, "DW1000 device ID: 0x%08lX, SPI mode %d", (unsigned long)dev_id, mode_found);
    if (mode_found < 0) {
        ESP_LOGE(TAG, "Wrong/no device ID! Check SPI pins 5,18,19,23, 3V3 and GND.");
        return ESP_FAIL;
    }
    if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "DW1000 init FAILED!");
        return ESP_FAIL;
    }
    dw1000_spi_set_speed(UWB_SPI_FAST_HZ);
    dwt_configure(&config_structure);
    static dwt_txconfig_t txconfig = { .PGdly = 0xC2, .power = 0x07274767 };
    dwt_configuretxrf(&txconfig);
    dwt_setleds(DWT_LEDS_ENABLE);
    dwt_setrxantennadelay(UWB_ANT_DELAY);
    dwt_settxantennadelay(UWB_ANT_DELAY);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(15000);
    return ESP_OK;
}

bool uwb_measure(uint8_t seq, uwb_sample_t *out) {
    const uint32_t clear_mask = SYS_STATUS_TXFRS | SYS_STATUS_ALL_RX_GOOD |
                                SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR;
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, clear_mask);
    tx_poll_msg[2] = seq;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        return false;
    }

    int64_t t0 = esp_timer_get_time();
    bool tx_done = false;
    while ((esp_timer_get_time() - t0) < TX_DONE_TIMEOUT_US) {
        if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS) { tx_done = true; break; }
        esp_rom_delay_us(STATUS_POLL_STEP_US);
    }
    if (!tx_done) {
        dwt_forcetrxoff();
        dwt_rxreset();
        return false;
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

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
        vTaskDelay(1);
    }
    if (!received) {
        dwt_forcetrxoff();
        dwt_rxreset();
        return false;
    }

    float clock_ratio = 0.0f;
#if UWB_CLOCK_CORRECT
    clock_ratio = (float)dwt_readcarrierintegrator() * (float)(FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_2 / 1.0e6);
#endif
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
    double tof = ((rtd_init - rtd_resp * (1.0 - (double)clock_ratio)) / 2.0) * DWT_TIME_UNITS;
    float dist_m = (float)(tof * SPEED_OF_LIGHT) + RANGE_BIAS_M;
    if (dist_m < -0.5f || dist_m >= RANGE_MAX_M) {
        return false;
    }
    if (dist_m < 0.0f) dist_m = 0.0f;

    out->dist_m = dist_m;
    out->nlos_db = nlos_indicator_db();
    return true;
}
