#pragma once
#include <stdint.h>
#include "esp_err.h"

#define RGB565_RAW(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
#define RGB565(r, g, b) ((uint16_t)((RGB565_RAW(r, g, b) >> 8) | (RGB565_RAW(r, g, b) << 8)))

esp_err_t gfx_init(void);
uint16_t *gfx_fb(void);
void gfx_clear(uint16_t c);
void gfx_pixel(int x, int y, uint16_t c);
void gfx_line(int x0, int y0, int x1, int y1, uint16_t c);
void gfx_thick_line(int x0, int y0, int x1, int y1, int half_w, uint16_t c);
void gfx_circle(int cx, int cy, int r, uint16_t c);
void gfx_fill_circle(int cx, int cy, int r, uint16_t c);
void gfx_fill_tri(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t c);
void gfx_ring(int cx, int cy, int r_in, int r_out, uint16_t c);
void gfx_arc(int cx, int cy, int r_in, int r_out, float a0, float a1, uint16_t c);
void gfx_text(int x, int y, const char *s, int scale, uint16_t c);
void gfx_text_center(int cx, int y, const char *s, int scale, uint16_t c);
int gfx_text_width(const char *s, int scale);
void gfx_present(void);
