/*
 * battery.c — Battery percent via ADC voltage divider.
 * Identical to shock_watch: reads GPIO4 through a 100k/100k divider,
 * maps 3.3-4.2 V (Li-ion) to 0-100%, with IIR smoothing and deadband.
 */

#include "battery.h"
#include "board_config.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t             s_channel    = ADC_CHANNEL_4;
static adc_cali_handle_t         s_cali       = NULL;
static bool                      s_cali_en    = false;
static float                     s_pct_smooth = -1.0f;
static float                     s_pct_out    = -1.0f;

static adc_channel_t gpio_to_channel(int gpio)
{
    switch (gpio) {
        case 0: return ADC_CHANNEL_0;
        case 1: return ADC_CHANNEL_1;
        case 2: return ADC_CHANNEL_2;
        case 3: return ADC_CHANNEL_3;
        case 4: return ADC_CHANNEL_4;
        default:
            ESP_LOGW("BAT", "Unsupported GPIO %d, using GPIO4", gpio);
            return ADC_CHANNEL_4;
    }
}

void battery_init(void)
{
    s_pct_smooth = s_pct_out = -1.0f;

    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    (void)adc_oneshot_new_unit(&init_cfg, &s_adc);

    s_channel = gpio_to_channel(BATTERY_ADC_GPIO);
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    (void)adc_oneshot_config_channel(s_adc, s_channel, &chan_cfg);

    adc_cali_curve_fitting_config_t cal_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = chan_cfg.atten,
        .bitwidth = chan_cfg.bitwidth,
    };
    s_cali_en = (adc_cali_create_scheme_curve_fitting(&cal_cfg, &s_cali) == ESP_OK);
    if (!s_cali_en) {
        s_cali = NULL;
        ESP_LOGW("BAT", "ADC calibration unavailable — using fallback scaling");
    }
}

static float voltage_to_pct(float v)
{
    static const struct { float v; float pct; } pts[] = {
        {3.30f,  0.0f}, {3.40f,  1.0f}, {3.50f,  4.0f}, {3.60f, 10.0f},
        {3.70f, 25.0f}, {3.80f, 45.0f}, {3.90f, 65.0f}, {4.00f, 82.0f},
        {4.10f, 94.0f}, {4.20f,100.0f}, {4.25f,100.0f},
    };
    if (v <= pts[0].v) return 0.0f;
    for (size_t i = 1; i < sizeof(pts)/sizeof(pts[0]); i++) {
        if (v <= pts[i].v) {
            float t = (v - pts[i-1].v) / (pts[i].v - pts[i-1].v);
            return pts[i-1].pct + (pts[i].pct - pts[i-1].pct) * t;
        }
    }
    return 100.0f;
}

int battery_read_percent(void)
{
    if (!s_adc) return 0;

    int sum = 0;
    for (int i = 0; i < 32; i++) {
        int raw = 0;
        (void)adc_oneshot_read(s_adc, s_channel, &raw);
        sum += raw;
    }
    float avg = (float)sum / 32.0f;

    int mv = 0;
    if (s_cali_en) {
        (void)adc_cali_raw_to_voltage(s_cali, (int)avg, &mv);
    } else {
        mv = (int)(3100.0f * avg / 4095.0f);
    }
    float v_batt = (float)mv / 500.0f;   /* ×2 for divider, /1000 for volts */
    float pct    = voltage_to_pct(v_batt);

    if (s_pct_smooth < 0.0f) s_pct_smooth = pct;
    else                      s_pct_smooth += 0.12f * (pct - s_pct_smooth);

    if (s_pct_out < 0.0f) s_pct_out = s_pct_smooth;
    else {
        float d = s_pct_smooth - s_pct_out;
        if (d > 1.0f || d < -1.0f) s_pct_out += d;
    }
    if (s_pct_out < 0.0f)   s_pct_out = 0.0f;
    if (s_pct_out > 100.0f) s_pct_out = 100.0f;
    return (int)(s_pct_out + 0.5f);
}
