#include "ui.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "gc9a01.h"
#include "gfx.h"
#include "nav.h"

#define CX 120
#define CY 120
#define PI_F ((float)M_PI)

#define COL_BG      RGB565(0, 14, 4)
#define COL_DIM     RGB565(0, 70, 26)
#define COL_MID     RGB565(0, 130, 48)
#define COL_BRIGHT  RGB565(90, 255, 110)
#define COL_AMBER   RGB565(255, 180, 0)
#define COL_RED     RGB565(255, 60, 40)

static void polar(float r, float a, int *x, int *y) {
    *x = CX + (int)lroundf(r * sinf(a));
    *y = CY - (int)lroundf(r * cosf(a));
}

esp_err_t ui_init(void) {
    esp_err_t e = gfx_init();
    if (e != ESP_OK) return e;
    e = gc9a01_init();
    if (e != ESP_OK) return e;
    gfx_clear(COL_BG);
    gfx_present();
    gc9a01_backlight(true);
    return ESP_OK;
}

void ui_boot_progress(const char *line1, const char *line2, float progress) {
    gfx_clear(COL_BG);
    gfx_ring(CX, CY, 112, 114, COL_DIM);
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;
    gfx_arc(CX, CY, 104, 110, 0.0f, 2.0f * PI_F * progress, COL_BRIGHT);
    gfx_text_center(CX, 70, "BEAR", 5, COL_BRIGHT);
    gfx_text_center(CX, 125, line1, 3, COL_AMBER);
    gfx_text_center(CX, 155, line2, 2, COL_MID);
    gfx_present();
}

void ui_error(const char *line1, const char *line2) {
    gfx_clear(COL_BG);
    gfx_ring(CX, CY, 112, 114, COL_RED);
    gfx_text_center(CX, 90, line1, 3, COL_RED);
    gfx_text_center(CX, 130, line2, 2, COL_AMBER);
    gfx_present();
}

static void draw_base(const nav_view_t *v, bool lost) {
    uint16_t dim = lost ? RGB565(0, 40, 16) : COL_DIM;
    uint16_t mid = lost ? RGB565(0, 60, 24) : COL_MID;
    gfx_clear(COL_BG);
    gfx_circle(CX, CY, 30, dim);
    gfx_circle(CX, CY, 60, dim);
    gfx_circle(CX, CY, 90, dim);
    gfx_ring(CX, CY, 111, 113, mid);
    gfx_line(CX - 112, CY, CX + 112, CY, dim);
    gfx_line(CX, CY - 112, CX, CY + 112, dim);
    for (int d = 0; d < 360; d += 15) {
        float a = d * PI_F / 180.0f;
        int x0, y0, x1, y1;
        float r0 = (d % 90 == 0) ? 102.0f : 106.0f;
        polar(r0, a, &x0, &y0);
        polar(118.0f, a, &x1, &y1);
        gfx_line(x0, y0, x1, y1, mid);
    }
    if (!lost) {
        int nx, ny, ax, ay, bx, by;
        polar(112.0f, v->north_rad, &nx, &ny);
        polar(98.0f, v->north_rad - 0.12f, &ax, &ay);
        polar(98.0f, v->north_rad + 0.12f, &bx, &by);
        gfx_fill_tri(nx, ny, ax, ay, bx, by, COL_AMBER);
    }
}

static void draw_pulse(const nav_view_t *v, uint32_t t) {
    uint32_t period = v->pulse_period_ms < 60 ? 60 : v->pulse_period_ms;
    float phase = (float)(t % period) / (float)period;
    int r = 12 + (int)(phase * 96.0f);
    int g = (int)((1.0f - phase) * 200.0f);
    gfx_ring(CX, CY, r, r + 2, RGB565(0, g, g / 3));
}

static void draw_arrow(float a) {
    float ux = sinf(a), uy = -cosf(a);
    float px = cosf(a), py = sinf(a);
    int tipx = CX + (int)(88.0f * ux), tipy = CY + (int)(88.0f * uy);
    int lx = CX + (int)(-24.0f * ux + 26.0f * px), ly = CY + (int)(-24.0f * uy + 26.0f * py);
    int rx = CX + (int)(-24.0f * ux - 26.0f * px), ry = CY + (int)(-24.0f * uy - 26.0f * py);
    int nx = CX + (int)(-6.0f * ux), ny = CY + (int)(-6.0f * uy);
    gfx_fill_tri(tipx, tipy, lx, ly, nx, ny, COL_BRIGHT);
    gfx_fill_tri(tipx, tipy, rx, ry, nx, ny, RGB565(40, 200, 70));
    gfx_line(tipx, tipy, lx, ly, COL_DIM);
    gfx_line(tipx, tipy, rx, ry, COL_DIM);
}

static void fmt_range(char *buf, size_t n, float r) {
    if (r < 10.0f) snprintf(buf, n, "%.2f", r);
    else snprintf(buf, n, "%.1f", r);
}

static void draw_frame(const nav_view_t *v, uint32_t t) {
    char buf[24];
    bool lost = v->state == NAV_LOST;
    draw_base(v, lost);

    if (lost) {
        gfx_text_center(CX, 100, "NO SIGNAL", 3, COL_RED);
        gfx_text_center(CX, 135, "SEARCHING", 2, COL_MID);
    } else {
        draw_pulse(v, t);
        fmt_range(buf, sizeof(buf), v->range_m);
        switch (v->state) {
        case NAV_CLOSE:
            if (v->have_arrow) {
                draw_arrow(v->arrow_rad);
                gfx_text_center(CX, 188, buf, 3, COL_AMBER);
            } else {
                gfx_text_center(CX, 88, buf, 6, COL_BRIGHT);
                gfx_text_center(CX, 140, "M", 3, COL_MID);
                gfx_text_center(CX, 168, "VERY CLOSE", 2, COL_AMBER);
            }
            break;
        case NAV_TRACK:
            if (v->have_arrow) draw_arrow(v->arrow_rad);
            gfx_text_center(CX, 188, buf, 3, COL_BRIGHT);
            break;
        default:
            gfx_text_center(CX, 66, buf, 5, COL_BRIGHT);
            gfx_text_center(CX, 108, "M", 2, COL_MID);
            if (v->trend < 0) gfx_text_center(CX, 134, "WARMER", 3, COL_BRIGHT);
            else if (v->trend > 0) gfx_text_center(CX, 134, "COLDER", 3, COL_AMBER);
            else gfx_text_center(CX, 134, "STEADY", 3, COL_MID);
            gfx_text_center(CX, 170, v->two_clusters ? "MOVE SIDEWAYS" : "WALK AROUND", 2, COL_AMBER);
            break;
        }
    }
    if (!v->mag_ok) gfx_text_center(CX, 214, "MAG!", 2, COL_AMBER);
}

static void ui_task(void *arg) {
    nav_view_t v;
    const TickType_t period = pdMS_TO_TICKS(1000 / UI_FPS);
    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, period);
        nav_view_update(&v);
        draw_frame(&v, (uint32_t)(esp_timer_get_time() / 1000));
        gfx_present();
    }
}

void ui_start(void) {
    xTaskCreatePinnedToCore(ui_task, "ui", 6144, NULL, 3, NULL, 1);
}
