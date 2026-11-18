#include "gfx.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "gc9a01.h"
#include "font5x7.h"

static uint16_t *fb = NULL;

esp_err_t gfx_init(void) {
    fb = heap_caps_malloc(LCD_W * LCD_H * sizeof(uint16_t), MALLOC_CAP_8BIT);
    if (!fb) return ESP_ERR_NO_MEM;
    memset(fb, 0, LCD_W * LCD_H * sizeof(uint16_t));
    return ESP_OK;
}

uint16_t *gfx_fb(void) {
    return fb;
}

void gfx_clear(uint16_t c) {
    for (int i = 0; i < LCD_W * LCD_H; i++) fb[i] = c;
}

void gfx_pixel(int x, int y, uint16_t c) {
    if ((unsigned)x < LCD_W && (unsigned)y < LCD_H) fb[y * LCD_W + x] = c;
}

static void hspan(int x0, int x1, int y, uint16_t c) {
    if ((unsigned)y >= LCD_H) return;
    if (x0 < 0) x0 = 0;
    if (x1 >= LCD_W) x1 = LCD_W - 1;
    for (int x = x0; x <= x1; x++) fb[y * LCD_W + x] = c;
}

void gfx_line(int x0, int y0, int x1, int y1, uint16_t c) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (1) {
        gfx_pixel(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void gfx_thick_line(int x0, int y0, int x1, int y1, int half_w, uint16_t c) {
    float dx = (float)(x1 - x0), dy = (float)(y1 - y0);
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1.0f) { gfx_fill_circle(x0, y0, half_w, c); return; }
    float nx = -dy / len, ny = dx / len;
    for (int k = -half_w; k <= half_w; k++) {
        gfx_line(x0 + (int)lroundf(nx * k), y0 + (int)lroundf(ny * k),
                 x1 + (int)lroundf(nx * k), y1 + (int)lroundf(ny * k), c);
    }
}

void gfx_circle(int cx, int cy, int r, uint16_t c) {
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        gfx_pixel(cx + x, cy + y, c); gfx_pixel(cx - x, cy + y, c);
        gfx_pixel(cx + x, cy - y, c); gfx_pixel(cx - x, cy - y, c);
        gfx_pixel(cx + y, cy + x, c); gfx_pixel(cx - y, cy + x, c);
        gfx_pixel(cx + y, cy - x, c); gfx_pixel(cx - y, cy - x, c);
        y++;
        if (err < 0) err += 2 * y + 1;
        else { x--; err += 2 * (y - x) + 1; }
    }
}

void gfx_fill_circle(int cx, int cy, int r, uint16_t c) {
    for (int y = -r; y <= r; y++) {
        int w = (int)sqrtf((float)(r * r - y * y));
        hspan(cx - w, cx + w, cy + y, c);
    }
}

void gfx_ring(int cx, int cy, int r_in, int r_out, uint16_t c) {
    for (int y = -r_out; y <= r_out; y++) {
        int wo = (int)sqrtf((float)(r_out * r_out - y * y));
        if (abs(y) >= r_in) {
            hspan(cx - wo, cx + wo, cy + y, c);
        } else {
            int wi = (int)sqrtf((float)(r_in * r_in - y * y));
            hspan(cx - wo, cx - wi, cy + y, c);
            hspan(cx + wi, cx + wo, cy + y, c);
        }
    }
}

void gfx_arc(int cx, int cy, int r_in, int r_out, float a0, float a1, uint16_t c) {
    float step = 1.0f / (float)r_out;
    for (float a = a0; a <= a1; a += step) {
        float s = sinf(a), co = cosf(a);
        for (int r = r_in; r <= r_out; r++) {
            gfx_pixel(cx + (int)lroundf(r * s), cy - (int)lroundf(r * co), c);
        }
    }
}

static int imin3(int a, int b, int c) { return a < b ? (a < c ? a : c) : (b < c ? b : c); }
static int imax3(int a, int b, int c) { return a > b ? (a > c ? a : c) : (b > c ? b : c); }

void gfx_fill_tri(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t c) {
    int minx = imin3(x0, x1, x2), maxx = imax3(x0, x1, x2);
    int miny = imin3(y0, y1, y2), maxy = imax3(y0, y1, y2);
    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx >= LCD_W) maxx = LCD_W - 1;
    if (maxy >= LCD_H) maxy = LCD_H - 1;
    int area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    if (area == 0) return;
    for (int y = miny; y <= maxy; y++) {
        for (int x = minx; x <= maxx; x++) {
            int w0 = (x1 - x0) * (y - y0) - (x - x0) * (y1 - y0);
            int w1 = (x2 - x1) * (y - y1) - (x - x1) * (y2 - y1);
            int w2 = (x0 - x2) * (y - y2) - (x - x2) * (y0 - y2);
            if (area > 0 ? (w0 >= 0 && w1 >= 0 && w2 >= 0) : (w0 <= 0 && w1 <= 0 && w2 <= 0)) {
                fb[y * LCD_W + x] = c;
            }
        }
    }
}

static void glyph(int x, int y, char ch, int scale, uint16_t c) {
    if (ch >= 'a' && ch <= 'z') ch -= 32;
    if (ch < FONT_FIRST || ch > FONT_LAST) ch = '?';
    const uint8_t *g = font5x7[ch - FONT_FIRST];
    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if (g[col] & (1 << row)) {
                for (int dy = 0; dy < scale; dy++) {
                    hspan(x + col * scale, x + col * scale + scale - 1, y + row * scale + dy, c);
                }
            }
        }
    }
}

int gfx_text_width(const char *s, int scale) {
    int n = (int)strlen(s);
    return n ? n * 6 * scale - scale : 0;
}

void gfx_text(int x, int y, const char *s, int scale, uint16_t c) {
    for (; *s; s++) {
        glyph(x, y, *s, scale, c);
        x += 6 * scale;
    }
}

void gfx_text_center(int cx, int y, const char *s, int scale, uint16_t c) {
    gfx_text(cx - gfx_text_width(s, scale) / 2, y, s, scale, c);
}

void gfx_present(void) {
    gc9a01_flush(fb);
}
