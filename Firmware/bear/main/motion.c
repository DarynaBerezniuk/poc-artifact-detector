#include "motion.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "config.h"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "qmc5883p.h"
#include "madgwick.h"
#include "pdr.h"

#define TAG "MOTION"
#define NVS_NS          "bear"
#define NVS_KEY_MAGCAL  "magcal"
#define MAG_MIN_HALF_G  0.10f
#define MAG_FRESH_MS    100

typedef struct {
    float off[3];
    float scl[3];
    float ref;
} mag_cal_t;

static const int imu_idx[3] = IMU_AXIS_IDX;
static const int imu_sgn[3] = IMU_AXIS_SIGN;
static const int mag_idx[3] = MAG_AXIS_IDX;
static const int mag_sgn[3] = MAG_AXIS_SIGN;

static madgwick_t filt;
static stepdet_t sdet;
static pdr_t pdr;
static mag_cal_t mcal = {{0, 0, 0}, {1, 1, 1}, 0.0f};
static bool mcal_valid = false;
static float gyro_bias[3];
static float mag_last[3];
static uint32_t mag_last_ms = 0;
static portMUX_TYPE pose_lock = portMUX_INITIALIZER_UNLOCKED;
static pose_t pose;

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void remap(const float in[3], const int idx[3], const int sgn[3], float out[3]) {
    for (int i = 0; i < 3; i++) out[i] = in[idx[i]] * sgn[i];
}

static bool read_imu(float acc[3], float gyr[3]) {
    float a[3], g[3];
    if (mpu6050_read(a, g) != ESP_OK) return false;
    remap(a, imu_idx, imu_sgn, acc);
    remap(g, imu_idx, imu_sgn, gyr);
    return true;
}

static bool read_mag_raw(float m[3]) {
    float r[3];
    bool fresh = false;
    if (qmc5883p_read(r, &fresh) != ESP_OK || !fresh) return false;
    remap(r, mag_idx, mag_sgn, m);
    return true;
}

static bool cal_load(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = sizeof(mcal);
    mag_cal_t tmp;
    esp_err_t e = nvs_get_blob(h, NVS_KEY_MAGCAL, &tmp, &len);
    nvs_close(h);
    if (e != ESP_OK || len != sizeof(tmp) || tmp.ref < MAG_MIN_HALF_G) return false;
    mcal = tmp;
    return true;
}

static void cal_save(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY_MAGCAL, &mcal, sizeof(mcal));
    nvs_commit(h);
    nvs_close(h);
}

static bool mag_calibrate(progress_cb_t cb) {
    float mn[3] = {1e9f, 1e9f, 1e9f}, mx[3] = {-1e9f, -1e9f, -1e9f};
    uint32_t t0 = now_ms();
    while (now_ms() - t0 < MAG_CAL_MS) {
        float m[3];
        if (read_mag_raw(m)) {
            for (int i = 0; i < 3; i++) {
                if (m[i] < mn[i]) mn[i] = m[i];
                if (m[i] > mx[i]) mx[i] = m[i];
            }
        }
        if (cb) cb("ROTATE", "ALL WAYS", (float)(now_ms() - t0) / MAG_CAL_MS);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    float half[3], avg = 0.0f;
    for (int i = 0; i < 3; i++) {
        half[i] = 0.5f * (mx[i] - mn[i]);
        if (half[i] < MAG_MIN_HALF_G) return false;
        avg += half[i];
    }
    avg /= 3.0f;
    for (int i = 0; i < 3; i++) {
        mcal.off[i] = 0.5f * (mx[i] + mn[i]);
        mcal.scl[i] = avg / half[i];
    }
    mcal.ref = avg;
    return true;
}

static void mag_apply(const float raw[3], float out[3]) {
    for (int i = 0; i < 3; i++) out[i] = (raw[i] - mcal.off[i]) * mcal.scl[i];
}

static void publish(void) {
    pose_t p;
    p.x = pdr.x;
    p.y = pdr.y;
    p.path = pdr.path;
    p.steps = pdr.steps;
    float h = madgwick_heading_rad(&filt) + SCREEN_UP_OFFSET_DEG * (float)M_PI / 180.0f;
    h = fmodf(h, 2.0f * (float)M_PI);
    if (h < 0.0f) h += 2.0f * (float)M_PI;
    p.psi = h;
    madgwick_tilt_rad(&filt, &p.roll, &p.pitch);
    p.t_ms = now_ms();
    p.mag_ok = false;
    portENTER_CRITICAL(&pose_lock);
    p.mag_ok = pose.mag_ok;
    pose = p;
    portEXIT_CRITICAL(&pose_lock);
}

static void imu_step(float dt, bool count_steps) {
    float acc[3], gyr[3];
    if (!read_imu(acc, gyr)) return;
    for (int i = 0; i < 3; i++) gyr[i] -= gyro_bias[i];

    float raw[3];
    if (read_mag_raw(raw)) {
        mag_apply(raw, mag_last);
        mag_last_ms = now_ms();
    }

    float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    bool acc_ok = an > ACC_VALID_MIN_G && an < ACC_VALID_MAX_G;

    bool mag_ok = false;
    if (mcal_valid && (now_ms() - mag_last_ms) < MAG_FRESH_MS) {
        float mn = sqrtf(mag_last[0] * mag_last[0] + mag_last[1] * mag_last[1] + mag_last[2] * mag_last[2]);
        mag_ok = fabsf(mn - mcal.ref) < MAG_DISTURB_FRAC * mcal.ref;
    }

    float zero[3] = {0, 0, 0};
    madgwick_update(&filt, gyr, acc_ok ? acc : zero, mag_last, mag_ok && acc_ok, dt);

    portENTER_CRITICAL(&pose_lock);
    pose.mag_ok = mag_ok;
    portEXIT_CRITICAL(&pose_lock);

    if (count_steps && stepdet_update(&sdet, an, now_ms())) {
        pdr_step(&pdr, madgwick_heading_rad(&filt) + SCREEN_UP_OFFSET_DEG * (float)M_PI / 180.0f);
    }
}

esp_err_t motion_init(void) {
    esp_err_t e = mpu6050_init();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "MPU-6050 not responding at 0x68");
        i2c_bus_scan();
        return e;
    }
    e = qmc5883p_init();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "QMC5883P not responding at 0x2C");
        i2c_bus_scan();
        return e;
    }
    stepdet_init(&sdet, IMU_RATE_HZ, STEP_THRESHOLD_G, STEP_MIN_GAP_MS);
    pdr_init(&pdr, STEP_LENGTH_M);
    memset(&pose, 0, sizeof(pose));
    return ESP_OK;
}

void motion_calibrate(progress_cb_t cb) {
    memset(gyro_bias, 0, sizeof(gyro_bias));
    float sum[3] = {0, 0, 0};
    int n = 0;
    float acc_z = 0.0f;
    uint32_t t0 = now_ms();
    while (now_ms() - t0 < GYRO_CAL_MS) {
        float a[3], g[3];
        if (read_imu(a, g)) {
            for (int i = 0; i < 3; i++) sum[i] += g[i];
            acc_z += a[2];
            n++;
        }
        if (cb) cb("HOLD STILL", "GYRO", (float)(now_ms() - t0) / GYRO_CAL_MS);
        vTaskDelay(pdMS_TO_TICKS(1000 / IMU_RATE_HZ));
    }
    if (n > 0) {
        for (int i = 0; i < 3; i++) gyro_bias[i] = sum[i] / n;
        acc_z /= n;
    }
    ESP_LOGI(TAG, "gyro bias %.4f %.4f %.4f rad/s", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    bool force = acc_z < UPSIDE_DOWN_RECAL_G;
    mcal_valid = !force && cal_load();
    if (!mcal_valid) {
        if (force) ESP_LOGI(TAG, "upside-down boot, forcing magnetometer calibration");
        if (mag_calibrate(cb)) {
            mcal_valid = true;
            cal_save();
            ESP_LOGI(TAG, "mag cal off %.3f %.3f %.3f scl %.3f %.3f %.3f ref %.3f G",
                     mcal.off[0], mcal.off[1], mcal.off[2], mcal.scl[0], mcal.scl[1], mcal.scl[2], mcal.ref);
        } else {
            ESP_LOGW(TAG, "magnetometer calibration failed, heading from gyro only");
            if (cb) cb("MAG CAL", "FAILED", 1.0f);
            vTaskDelay(pdMS_TO_TICKS(1500));
        }
    }

    madgwick_init(&filt, IMU_BETA_ALIGN);
    uint32_t ta = now_ms();
    float dt = 1.0f / IMU_RATE_HZ;
    while (now_ms() - ta < IMU_ALIGN_MS) {
        imu_step(dt, false);
        if (cb) cb("HOLD STILL", "ALIGN", (float)(now_ms() - ta) / IMU_ALIGN_MS);
        vTaskDelay(pdMS_TO_TICKS(1000 / IMU_RATE_HZ));
    }
    filt.beta = IMU_BETA_RUN;
    pdr_reset(&pdr);
    publish();
}

static void motion_task(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(1000 / IMU_RATE_HZ);
    TickType_t last = xTaskGetTickCount();
    float dt = 1.0f / IMU_RATE_HZ;
    uint32_t log_ms = 0;
    while (1) {
        vTaskDelayUntil(&last, period);
        imu_step(dt, true);
        publish();
        uint32_t t = now_ms();
        if (t - log_ms >= 1000) {
            log_ms = t;
            pose_t p;
            motion_get_pose(&p);
            ESP_LOGI(TAG, "psi %.0f roll %.0f pitch %.0f steps %lu pos %.1f,%.1f mag %d",
                     p.psi * 180.0f / (float)M_PI, p.roll * 180.0f / (float)M_PI, p.pitch * 180.0f / (float)M_PI,
                     (unsigned long)p.steps, p.x, p.y, p.mag_ok);
        }
    }
}

void motion_start(void) {
    xTaskCreatePinnedToCore(motion_task, "motion", 4096, NULL, 6, NULL, 1);
}

void motion_get_pose(pose_t *out) {
    portENTER_CRITICAL(&pose_lock);
    *out = pose;
    portEXIT_CRITICAL(&pose_lock);
}
