/*
 * main.c — ECG Watch firmware (ESP32-C3)
 *
 * Hardware: ESP32-C3 + MAX30102 (I2C SDA=8, SCL=9) + SSD1306 OLED (same bus)
 *           + optional shock/buzzer on GPIO10 + battery ADC on GPIO4.
 *
 * Signal flow:
 *   MAX30102 IR channel → Pan-Tompkins pipeline → QRS detection →
 *   R-R interval analysis → BPM + arrhythmia flags →
 *   OLED display + BLE notify to desktop app.
 *
 * BLE protocol (NUS-style):
 *   Watch → App (notify): "H:<bpm>,A:<flags>,B:<battery>\n"
 *   App → Watch (write):  "SHOCK\n"  fires the GPIO10 shock pulse.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "board_config.h"
#include "pan_tompkins.h"
#include "display.h"
#include "battery.h"
#include "ble_ecg.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

/* Low-level GPIO for shock pulse (same approach as shock_watch) */
#include "esp_bit_defs.h"
#include "soc/soc.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"

static const char *TAG = "ECG";

// =========================================================================
// Shock / buzzer helpers
// =========================================================================

static inline void shock_init_ll(void)
{
#if defined(PIN_FUNC_SELECT) && defined(IO_MUX_GPIO10_REG) && defined(PIN_FUNC_GPIO)
    PIN_FUNC_SELECT(IO_MUX_GPIO10_REG, PIN_FUNC_GPIO);
#endif
    REG_WRITE(GPIO_ENABLE_W1TS_REG, BIT(SHOCK_GPIO));
#if SHOCK_ACTIVE_HIGH
    REG_WRITE(GPIO_OUT_W1TC_REG, BIT(SHOCK_GPIO));
#else
    REG_WRITE(GPIO_OUT_W1TS_REG, BIT(SHOCK_GPIO));
#endif
}

static inline void shock_set(bool on)
{
#if SHOCK_ACTIVE_HIGH
    if (on) REG_WRITE(GPIO_OUT_W1TS_REG, BIT(SHOCK_GPIO));
    else    REG_WRITE(GPIO_OUT_W1TC_REG, BIT(SHOCK_GPIO));
#else
    if (on) REG_WRITE(GPIO_OUT_W1TC_REG, BIT(SHOCK_GPIO));
    else    REG_WRITE(GPIO_OUT_W1TS_REG, BIT(SHOCK_GPIO));
#endif
}

// =========================================================================
// I2C helpers
// =========================================================================

static esp_err_t i2c_init(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_GPIO,
        .scl_io_num       = I2C_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t e = i2c_param_config(I2C_PORT, &cfg);
    if (e != ESP_OK) return e;
    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((addr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t e = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return e;
}

static esp_err_t i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((addr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((addr << 1) | I2C_MASTER_READ), true);
    if (n > 1) i2c_master_read(cmd, buf, n - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + n - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t e = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return e;
}

// =========================================================================
// MAX30102 driver
// =========================================================================

static esp_err_t max30102_init(void)
{
    esp_err_t e;

    /* Software reset */
    e = i2c_write_byte(MAX30102_ADDR, 0x09, 0x40);
    if (e != ESP_OK) { ESP_LOGE("MAX", "reset: %s", esp_err_to_name(e)); return e; }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* FIFO: no sample averaging (SMP_AVE=0 → 1 sample/entry), rollover enabled */
    e = i2c_write_byte(MAX30102_ADDR, 0x08, 0x10);  /* SMP_AVE=0, ROLLOVER=1 */
    if (e != ESP_OK) return e;

    /* Mode: SpO2 (IR + RED) */
    e = i2c_write_byte(MAX30102_ADDR, 0x09, 0x03);
    if (e != ESP_OK) return e;

    /* SpO2 config: ADC range 4096 nA, 200 Hz sample rate, 215 µs pulse width (16-bit).
     * 200 Hz is the practical max for MAX30102; Pan-Tompkins paper used 360 Hz. */
    e = i2c_write_byte(MAX30102_ADDR, 0x0A, (uint8_t)((0b11 << 5) | (0b010 << 2) | 0b10));
    if (e != ESP_OK) return e;

    /* LED currents: RED ~7 mA, IR ~10 mA */
    e = i2c_write_byte(MAX30102_ADDR, 0x0C, 0x70); if (e != ESP_OK) return e;
    e = i2c_write_byte(MAX30102_ADDR, 0x0D, 0x80); if (e != ESP_OK) return e;

    /* Clear FIFO pointers */
    e = i2c_write_byte(MAX30102_ADDR, 0x04, 0x00); if (e != ESP_OK) return e;
    e = i2c_write_byte(MAX30102_ADDR, 0x05, 0x00); if (e != ESP_OK) return e;
    e = i2c_write_byte(MAX30102_ADDR, 0x06, 0x00); if (e != ESP_OK) return e;

    ESP_LOGI("MAX", "Initialised @ %d Hz", SAMPLE_RATE_HZ);
    return ESP_OK;
}

static int max30102_available(void)
{
    uint8_t wr = 0, rd = 0;
    if (i2c_read_bytes(MAX30102_ADDR, 0x04, &wr, 1) != ESP_OK) return 0;
    if (i2c_read_bytes(MAX30102_ADDR, 0x06, &rd, 1) != ESP_OK) return 0;
    return ((int)wr - (int)rd) & 0x1F;
}

static bool max30102_read(uint32_t *red_out, uint32_t *ir_out)
{
    uint8_t d[6];
    if (i2c_read_bytes(MAX30102_ADDR, 0x07, d, 6) != ESP_OK) return false;
    *red_out = ((uint32_t)(d[0] & 0x03) << 16) | ((uint32_t)d[1] << 8) | d[2];
    *ir_out  = ((uint32_t)(d[3] & 0x03) << 16) | ((uint32_t)d[4] << 8) | d[5];
    return true;
}

// =========================================================================
// app_main
// =========================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "ECG Watch starting");
    esp_log_level_set("*", ESP_LOG_INFO);

    /* NVS must be initialised before BLE */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    /* Hardware init */
    shock_init_ll();

    esp_err_t e = i2c_init();
    if (e != ESP_OK) { ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(e)); return; }

    battery_init();

    bool ui_ok = display_init(I2C_PORT);
    ESP_LOGI(TAG, "OLED: %s", ui_ok ? "OK" : "FAIL");

    e = max30102_init();
    if (e != ESP_OK) ESP_LOGE(TAG, "MAX30102 init failed: %s", esp_err_to_name(e));

    ble_ecg_init();

    pt_init();
    display_chart_clear();

    ESP_LOGI(TAG, "Sampling @ %d Hz", SAMPLE_RATE_HZ);

    /* Timing */
    /* FreeRTOS tick = 10 ms; poll at that rate and drain all FIFO samples each iteration */
    const TickType_t sample_tick = pdMS_TO_TICKS(10);
    TickType_t last_wake   = xTaskGetTickCount();
    TickType_t t_ui        = last_wake;
    TickType_t t_ble       = last_wake;
    TickType_t t_bat       = last_wake;
    TickType_t t_chart     = last_wake;
    const TickType_t ui_dt    = pdMS_TO_TICKS(100);
    const TickType_t ble_dt   = pdMS_TO_TICKS(BLE_NOTIFY_MS);
    const TickType_t bat_dt   = pdMS_TO_TICKS(1000);
    const TickType_t chart_dt = pdMS_TO_TICKS(display_chart_interval_ms());

    /* State */
    float    show_bpm        = NAN;
    uint8_t  show_arr        = ARR_NONE;
    bool     finger_prev     = false;
    uint32_t finger_since_ms = 0;
    bool     blink_on        = false;
    uint32_t next_blink_ms   = 0;
    int      battery_pct     = 100;
    uint32_t shock_end_ms    = 0;  /* 0 = shock not active */
    uint32_t last_qrs_ms     = 0;  /* 0 = no beat seen yet  */
    bool     feed_mode       = false; /* true = playback via BLE FEED commands */
    bool     raw_mode        = false; /* true = stream raw IR back to app (REC_START) */
    int32_t  raw_batch[5];
    int      raw_batch_n     = 0;

    pt_result_t pt_res = { .bpm = NAN, .arrhythmia = ARR_NONE, .qrs_count = 0 };

    while (1) {
        vTaskDelayUntil(&last_wake, sample_tick);
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        /* --- Battery update (1 Hz) --- */
        if ((int32_t)(xTaskGetTickCount() - t_bat) >= 0) {
            t_bat += bat_dt;
            battery_pct = battery_read_percent();
        }

        /* --- BLE command handling --- */
        {
            char cmd[64];
            if (ble_ecg_get_command(cmd, sizeof(cmd))) {
                if (strncmp(cmd, "SHOCK", 5) == 0) {
                    shock_set(true);
                    shock_end_ms = now_ms + SHOCK_PULSE_MS;
                    ESP_LOGI(TAG, "Shock triggered");

                } else if (strncmp(cmd, "FEED_START", 10) == 0) {
                    /* Enter playback mode: ignore real sensor, reset PT */
                    feed_mode    = true;
                    finger_prev  = false;
                    pt_reset();
                    show_bpm     = NAN;
                    show_arr     = ARR_NONE;
                    last_qrs_ms  = 0;
                    display_chart_clear();
                    ESP_LOGI(TAG, "Feed mode ON");

                } else if (strncmp(cmd, "FEED_STOP", 9) == 0) {
                    /* Leave playback mode, resume normal sensor operation */
                    feed_mode    = false;
                    finger_prev  = false;
                    pt_reset();
                    show_bpm     = NAN;
                    show_arr     = ARR_NONE;
                    last_qrs_ms  = 0;
                    display_chart_clear();
                    ESP_LOGI(TAG, "Feed mode OFF");

                } else if (feed_mode && strncmp(cmd, "FEED:", 5) == 0) {
                    /* Process batch of comma-separated raw samples: FEED:v1,v2,... */
                    char *p = cmd + 5;
                    while (*p != '\0' && *p != '\n') {
                        int32_t val = (int32_t)strtol(p, &p, 10);
                        bool qrs = pt_process(val, &pt_res);
                        if (qrs) {
                            last_qrs_ms = now_ms;
                            ESP_LOGD(TAG, "Feed QRS bpm=%.1f", (double)pt_res.bpm);
                        }
                        if (pt_res.bpm == pt_res.bpm) {
                            show_bpm = pt_res.bpm;
                            show_arr = pt_res.arrhythmia;
                        }
                        if (*p == ',') p++;
                        else break;
                    }
                }
                } else if (strncmp(cmd, "REC_START", 9) == 0) {
                    raw_mode    = true;
                    raw_batch_n = 0;
                    ESP_LOGI(TAG, "Raw stream ON");

                } else if (strncmp(cmd, "REC_STOP", 8) == 0) {
                    raw_mode    = false;
                    raw_batch_n = 0;
                    ESP_LOGI(TAG, "Raw stream OFF");
            }
        }

        /* --- Shock pulse timer --- */
        if (shock_end_ms != 0 && now_ms >= shock_end_ms) {
            shock_set(false);
            shock_end_ms = 0;
        }

        /* --- Drain MAX30102 FIFO (skipped during BLE playback) --- */
        int guard = 32;
        while (!feed_mode && guard-- > 0 && max30102_available() > 0) {
            uint32_t red = 0, ir = 0;
            if (!max30102_read(&red, &ir)) break;

            bool finger = (ir > FINGER_IR_MIN && ir < IR_SAT_HARD_MAX);

            if (finger && !finger_prev) {
                finger_since_ms = now_ms;
                last_qrs_ms = 0;
                pt_reset();
                show_bpm = NAN;
                show_arr = ARR_NONE;
                display_chart_clear();
                ESP_LOGI(TAG, "Finger detected — waiting for settle");
            } else if (!finger && finger_prev) {
                last_qrs_ms = 0;
                show_bpm = NAN;
                show_arr = ARR_NONE;
                ESP_LOGI(TAG, "Finger removed");
            }
            finger_prev = finger;

            bool settled = finger &&
                           ((now_ms - finger_since_ms) >= FINGER_SETTLE_MS);

            if (settled) {
                bool qrs = pt_process((int32_t)ir, &pt_res);
                if (qrs) {
                    last_qrs_ms = now_ms;
                    ESP_LOGI(TAG, "QRS #%lu  bpm=%.1f arr=0x%02X",
                             (unsigned long)pt_res.qrs_count,
                             (double)pt_res.bpm, pt_res.arrhythmia);
                }

                /* Update display values once BPM is valid */
                if (pt_res.bpm == pt_res.bpm) {   /* not NAN */
                    show_bpm = pt_res.bpm;
                    show_arr = pt_res.arrhythmia;
                }

                /* Stale-signal timeout: no beat for 3 s → clear display */
                if (last_qrs_ms != 0 &&
                    (now_ms - last_qrs_ms) > 3000u) {
                    show_bpm = NAN;
                    show_arr = ARR_NONE;
                }
            }

            /* Raw streaming: batch 5 samples then send as "RAW:v1,v2,v3,v4,v5\n" */
            if (raw_mode) {
                raw_batch[raw_batch_n++] = (int32_t)ir;
                if (raw_batch_n >= 5) {
                    char rbuf[64];
                    int  rlen = snprintf(rbuf, sizeof(rbuf),
                                         "RAW:%ld,%ld,%ld,%ld,%ld\n",
                                         (long)raw_batch[0], (long)raw_batch[1],
                                         (long)raw_batch[2], (long)raw_batch[3],
                                         (long)raw_batch[4]);
                    if (rlen > 0) ble_ecg_send_str(rbuf, (uint16_t)rlen);
                    raw_batch_n = 0;
                }
            }
        } /* FIFO drain */

        /* --- Chart update --- */
        if ((int32_t)(xTaskGetTickCount() - t_chart) >= 0) {
            t_chart += chart_dt;
            if (show_bpm == show_bpm) display_chart_push_bpm(show_bpm);
        }

        /* --- Blink timer (2.5 Hz) --- */
        if (now_ms >= next_blink_ms) {
            next_blink_ms = now_ms + 400u;
            blink_on = !blink_on;
        }

        /* --- Display update (~10 Hz) --- */
        if (ui_ok && (int32_t)(xTaskGetTickCount() - t_ui) >= 0) {
            t_ui += ui_dt;
            display_render(show_bpm, show_arr,
                           ble_ecg_connected(), blink_on, battery_pct);
        }

        /* --- BLE notify --- */
        if ((int32_t)(xTaskGetTickCount() - t_ble) >= 0) {
            t_ble += ble_dt;
            uint8_t bpm_u8 = (show_bpm == show_bpm && show_bpm >= 0.0f)
                             ? (uint8_t)(show_bpm + 0.5f) : 255u;
            ble_ecg_notify(bpm_u8, show_arr, (uint8_t)battery_pct);
        }

    } /* main loop */
}
