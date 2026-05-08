/*
 * display.c — SSD1306 128×32 OLED driver + ECG watch UI.
 *
 * Layout:
 *   Left panel (0..74 px):  "BPM" label, large BPM value, battery icon
 *   Right panel (75..127):  52×26 px BPM trend graph (30 s window)
 *   Top-right corner:       BLE dot + arrhythmia label
 *
 * Reuses the SSD1306 low-level driver and framebuffer code from shock_watch.
 * The glyph/font table (FONT5x7) covers ASCII 32-90, giving A-Z without
 * needing individual glyph entries for every letter.
 */

#include "display.h"
#include "app_config.h"
#include "board_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define SCREEN_W 128
#define SCREEN_H  32

#define GRAPH_X   76
#define GRAPH_Y    4
#define GRAPH_W   52
#define GRAPH_H   24

#define CHART_DURATION_MS  30000u
#define CHART_DT_MS        ((CHART_DURATION_MS + GRAPH_W - 1) / GRAPH_W)

#define GRAPH_MIN_BPM  40.0f
#define GRAPH_MAX_BPM 140.0f

static i2c_port_t s_port = I2C_NUM_0;

// =========================================================================
// Framebuffer
// =========================================================================

static uint8_t fb[SCREEN_W * SCREEN_H / 8];

static inline void fb_clear(void) { memset(fb, 0, sizeof(fb)); }

static inline void fb_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= SCREEN_W || y < 0 || y >= SCREEN_H) return;
    int     page = y >> 3;
    int     idx  = page * SCREEN_W + x;
    uint8_t bit  = (uint8_t)(1 << (y & 7));
    if (on) fb[idx] |= bit; else fb[idx] &= (uint8_t)~bit;
}

static void fb_hline(int x0, int x1, int y)
{
    if (y < 0 || y >= SCREEN_H) return;
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    if (x0 < 0) x0 = 0;
    if (x1 >= SCREEN_W) x1 = SCREEN_W - 1;
    for (int x = x0; x <= x1; ++x) fb_set_pixel(x, y, true);
}

static void fb_vline(int x, int y0, int y1)
{
    if (x < 0 || x >= SCREEN_W) return;
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
    if (y0 < 0) y0 = 0;
    if (y1 >= SCREEN_H) y1 = SCREEN_H - 1;
    for (int y = y0; y <= y1; ++y) fb_set_pixel(x, y, true);
}

static void fb_line(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    while (true) {
        fb_set_pixel(x0, y0, true);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// =========================================================================
// 5×7 font  (ASCII 32-90 = space..Z)
// =========================================================================

static const uint8_t FONT5x7[][5] = {
  {0,0,0,0,0},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
  {0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
  {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
  {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
  {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
  {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
  {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
  {0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
  {0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},
};

/* Explicit 5×7 glyphs — FONT5x7 is off-by-one for uppercase letters
 * and is missing X/Y/Z entirely.  These override it for every character
 * we actually render on the display. */
static const uint8_t GLYPH_0[5]     = {0x3E,0x51,0x49,0x45,0x3E};
static const uint8_t GLYPH_1[5]     = {0x00,0x42,0x7F,0x40,0x00};
static const uint8_t GLYPH_2[5]     = {0x42,0x61,0x51,0x49,0x46};
static const uint8_t GLYPH_3[5]     = {0x21,0x41,0x45,0x4B,0x31};
static const uint8_t GLYPH_4[5]     = {0x18,0x14,0x12,0x7F,0x10};
static const uint8_t GLYPH_5[5]     = {0x27,0x45,0x45,0x45,0x39};
static const uint8_t GLYPH_6[5]     = {0x3C,0x4A,0x49,0x49,0x30};
static const uint8_t GLYPH_7[5]     = {0x01,0x71,0x09,0x05,0x03};
static const uint8_t GLYPH_8[5]     = {0x36,0x49,0x49,0x49,0x36};
static const uint8_t GLYPH_9[5]     = {0x06,0x49,0x49,0x29,0x1E};
static const uint8_t GLYPH_MINUS[5] = {0x08,0x08,0x08,0x08,0x08};
/* Letters used on-screen: "BPM", "BRADY", "TACHY", "IRREG" */
static const uint8_t GLYPH_A[5]     = {0x7E,0x11,0x11,0x11,0x7E};
static const uint8_t GLYPH_B[5]     = {0x7F,0x49,0x49,0x49,0x36};
static const uint8_t GLYPH_C[5]     = {0x3E,0x41,0x41,0x41,0x22};
static const uint8_t GLYPH_D[5]     = {0x7F,0x41,0x41,0x41,0x3E};
static const uint8_t GLYPH_E[5]     = {0x7F,0x49,0x49,0x49,0x41};
static const uint8_t GLYPH_G[5]     = {0x3E,0x41,0x49,0x49,0x7A};
static const uint8_t GLYPH_H[5]     = {0x7F,0x08,0x08,0x08,0x7F};
static const uint8_t GLYPH_I[5]     = {0x00,0x41,0x7F,0x41,0x00};
static const uint8_t GLYPH_M[5]     = {0x7F,0x06,0x18,0x06,0x7F};
static const uint8_t GLYPH_P[5]     = {0x7F,0x09,0x09,0x09,0x06};
static const uint8_t GLYPH_R[5]     = {0x7F,0x09,0x19,0x29,0x46};
static const uint8_t GLYPH_T[5]     = {0x01,0x01,0x7F,0x01,0x01};
static const uint8_t GLYPH_Y[5]     = {0x07,0x08,0x70,0x08,0x07};

static bool glyph_lookup(char c, const uint8_t **out)
{
    switch (c) {
        case '0': *out = GLYPH_0;     return true;
        case '1': *out = GLYPH_1;     return true;
        case '2': *out = GLYPH_2;     return true;
        case '3': *out = GLYPH_3;     return true;
        case '4': *out = GLYPH_4;     return true;
        case '5': *out = GLYPH_5;     return true;
        case '6': *out = GLYPH_6;     return true;
        case '7': *out = GLYPH_7;     return true;
        case '8': *out = GLYPH_8;     return true;
        case '9': *out = GLYPH_9;     return true;
        case '-': *out = GLYPH_MINUS; return true;
        case 'A': *out = GLYPH_A;     return true;
        case 'B': *out = GLYPH_B;     return true;
        case 'C': *out = GLYPH_C;     return true;
        case 'D': *out = GLYPH_D;     return true;
        case 'E': *out = GLYPH_E;     return true;
        case 'G': *out = GLYPH_G;     return true;
        case 'H': *out = GLYPH_H;     return true;
        case 'I': *out = GLYPH_I;     return true;
        case 'M': *out = GLYPH_M;     return true;
        case 'P': *out = GLYPH_P;     return true;
        case 'R': *out = GLYPH_R;     return true;
        case 'T': *out = GLYPH_T;     return true;
        case 'Y': *out = GLYPH_Y;     return true;
        default:  return false;
    }
}

static void fb_draw_char(int x, int y, char c, int scale)
{
    const uint8_t *col = NULL;
    if (!glyph_lookup(c, &col)) {
        if (c < 32 || c > 90) c = '?';
        col = FONT5x7[(int)c - 32];
    }

    for (int cx = 0; cx < 5; cx++) {
        uint8_t bits = col[cx];
        for (int cy = 0; cy < 7; cy++) {
            bool on = ((bits >> cy) & 1) != 0;
            for (int sx = 0; sx < scale; sx++)
                for (int sy = 0; sy < scale; sy++)
                    fb_set_pixel(x + cx*scale + sx, y + cy*scale + sy, on);
        }
    }
    /* clear separator column */
    for (int sy = 0; sy < 7 * scale; sy++)
        fb_set_pixel(x + 5*scale, y + sy, false);
}

static void fb_draw_text(int x, int y, const char *s, int scale)
{
    while (*s) {
        fb_draw_char(x, y, *s, scale);
        x += 6 * scale;
        s++;
    }
}

// =========================================================================
// SSD1306 minimal driver
// =========================================================================

static esp_err_t ssd1306_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(c, buf, 2, true);
    i2c_master_stop(c);
    esp_err_t e = i2c_master_cmd_begin(s_port, c, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(c);
    return e;
}

static bool ssd1306_try_init(void)
{
    const uint8_t cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
        0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x02, 0x81, 0x8F, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    for (size_t i = 0; i < sizeof(cmds); i++) {
        if (ssd1306_cmd(cmds[i]) != ESP_OK) return false;
    }
    return true;
}

static void ssd1306_force_clear(void)
{
    (void)ssd1306_cmd(0xAE);
    for (int page = 0; page < 4; page++) {
        (void)ssd1306_cmd((uint8_t)(0xB0 | page));
        (void)ssd1306_cmd(0x00);
        (void)ssd1306_cmd(0x10);
        uint8_t buf[1 + 128];
        buf[0] = 0x40;
        memset(&buf[1], 0x00, 128);
        i2c_cmd_handle_t c = i2c_cmd_link_create();
        i2c_master_start(c);
        i2c_master_write_byte(c, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(c, buf, sizeof(buf), true);
        i2c_master_stop(c);
        (void)i2c_master_cmd_begin(s_port, c, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(c);
    }
    (void)ssd1306_cmd(0xA4);
    (void)ssd1306_cmd(0xAF);
}

static esp_err_t ssd1306_flush(void)
{
    for (int page = 0; page < 4; page++) {
        if (ssd1306_cmd((uint8_t)(0xB0 | page)) != ESP_OK) return ESP_FAIL;
        if (ssd1306_cmd(0x00) != ESP_OK) return ESP_FAIL;
        if (ssd1306_cmd(0x10) != ESP_OK) return ESP_FAIL;
        uint8_t buf[1 + 128];
        buf[0] = 0x40;
        memcpy(&buf[1], &fb[page * 128], 128);
        i2c_cmd_handle_t c = i2c_cmd_link_create();
        i2c_master_start(c);
        i2c_master_write_byte(c, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(c, buf, sizeof(buf), true);
        i2c_master_stop(c);
        esp_err_t e = i2c_master_cmd_begin(s_port, c, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(c);
        if (e != ESP_OK) return e;
    }
    return ESP_OK;
}

// =========================================================================
// Chart (30 s BPM trend)
// =========================================================================

static int8_t  chart_y[GRAPH_W];
static int     chart_head = 0;

uint32_t display_chart_interval_ms(void) { return CHART_DT_MS; }

void display_chart_clear(void)
{
    for (int i = 0; i < GRAPH_W; i++) chart_y[i] = -1;
    chart_head = 0;
}

static int bpm_to_y(float bpm)
{
    if (bpm != bpm) return -1;   /* NAN */
    if (bpm < GRAPH_MIN_BPM) bpm = GRAPH_MIN_BPM;
    if (bpm > GRAPH_MAX_BPM) bpm = GRAPH_MAX_BPM;
    float frac = (bpm - GRAPH_MIN_BPM) / (GRAPH_MAX_BPM - GRAPH_MIN_BPM);
    int yoff = (int)lroundf((GRAPH_H - 1) * (1.0f - frac));
    if (yoff < 0) yoff = 0;
    if (yoff >= GRAPH_H) yoff = GRAPH_H - 1;
    return yoff;
}

void display_chart_push_bpm(float bpm)
{
    int yoff = bpm_to_y(bpm);
    chart_y[chart_head] = (int8_t)(yoff >= 0 ? yoff : -1);
    chart_head = (chart_head + 1) % GRAPH_W;
}

static void draw_graph_box(void)
{
    fb_hline(GRAPH_X - 1, GRAPH_X + GRAPH_W, GRAPH_Y - 1);
    fb_hline(GRAPH_X - 1, GRAPH_X + GRAPH_W, GRAPH_Y + GRAPH_H);
    fb_vline(GRAPH_X - 1, GRAPH_Y - 1, GRAPH_Y + GRAPH_H);
    fb_vline(GRAPH_X + GRAPH_W, GRAPH_Y - 1, GRAPH_Y + GRAPH_H);
}

/* Dashed guide lines for brady (60) and tachy (100) thresholds */
static void draw_guide_lines(void)
{
    const float thresholds[2] = { BRADY_BPM_THRESHOLD, TACHY_BPM_THRESHOLD };
    for (int t = 0; t < 2; t++) {
        float thresh = thresholds[t];
        if (thresh < GRAPH_MIN_BPM || thresh > GRAPH_MAX_BPM) continue;
        int y = GRAPH_Y + bpm_to_y(thresh);
        for (int x = 0; x < GRAPH_W; x++)
            if ((x & 3) < 2) fb_set_pixel(GRAPH_X + x, y, true);
    }
}

static void draw_graph_plot(void)
{
    int prev_y = -1;
    for (int i = 0; i < GRAPH_W; i++) {
        int idx  = (chart_head + i) % GRAPH_W;
        int yoff = chart_y[idx];
        int x    = GRAPH_X + i;
        if (yoff >= 0) {
            int y = GRAPH_Y + yoff;
            fb_set_pixel(x, y, true);
            if (prev_y >= 0) fb_line(x - 1, prev_y, x, y);
            prev_y = y;
        } else {
            prev_y = -1;
        }
    }
}

// =========================================================================
// Battery icon
// =========================================================================

static void draw_battery(int x, int y, int pct)
{
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    fb_hline(x, x + 13, y);
    fb_hline(x, x + 13, y + 7);
    fb_vline(x,      y, y + 7);
    fb_vline(x + 13, y, y + 7);
    fb_vline(x + 14, y + 2, y + 5);
    int fill = (pct * 12) / 100;
    for (int fx = 0; fx < fill; fx++) fb_vline(x + 1 + fx, y + 1, y + 6);
}

// =========================================================================
// Arrhythmia label helper
// =========================================================================

static void draw_arrhythmia_label(uint8_t flags, bool blink_on)
{
    const char *label = NULL;

    if (flags & ARR_TACHYCARDIA)  label = "TACHY";
    else if (flags & ARR_BRADYCARDIA) label = "BRADY";
    else if (flags & ARR_IRREGULAR)   label = "IRREG";

    if (label && blink_on) {
        fb_draw_text(GRAPH_X + 2, 0, label, 1);
    }
}

// =========================================================================
// Main render
// =========================================================================

static void ui_draw(float bpm, uint8_t arrhythmia,
                    bool ble_connected, bool blink_on, int battery_pct)
{
    fb_clear();

    /* "BPM" label */
    fb_draw_text(2, 0, "BPM", 1);

    /* Large BPM value */
    char buf[8];
    if (bpm != bpm) {
        strcpy(buf, "--");
    } else {
        int v = (int)lroundf(bpm);
        if (v < 0)   v = 0;
        if (v > 999) v = 999;
        snprintf(buf, sizeof(buf), "%d", v);
    }
    fb_draw_text(2, 10, buf, 3);

    /* Battery icon top-left after label */
    draw_battery(32, 0, battery_pct);

    /* BLE connected dot (top right of left panel) */
    if (ble_connected) {
        fb_set_pixel(72, 1, true);
        fb_set_pixel(73, 1, true);
        fb_set_pixel(72, 2, true);
        fb_set_pixel(73, 2, true);
    }

    /* Right-side graph */
    draw_graph_box();
    draw_guide_lines();
    draw_graph_plot();

    /* Arrhythmia label in top of graph area (blinks) */
    draw_arrhythmia_label(arrhythmia, blink_on);

    (void)ssd1306_flush();
}

// =========================================================================
// Public API
// =========================================================================

bool display_init(i2c_port_t port)
{
    s_port = port;
    if (!ssd1306_try_init()) return false;
    ssd1306_force_clear();
    display_chart_clear();
    display_render(NAN, ARR_NONE, false, false, 100);
    return true;
}

void display_render(float bpm, uint8_t arrhythmia,
                    bool ble_connected, bool blink_on, int battery_pct)
{
    ui_draw(bpm, arrhythmia, ble_connected, blink_on, battery_pct);
}
