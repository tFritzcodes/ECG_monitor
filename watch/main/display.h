#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "pan_tompkins.h"  /* for ARR_* flags */

/* Chart (30 s BPM trend) helpers */
uint32_t display_chart_interval_ms(void);
void     display_chart_clear(void);
void     display_chart_push_bpm(float bpm);

bool display_init(i2c_port_t port);

/* Render one frame.
 *   bpm           : current BPM (NAN = no signal)
 *   arrhythmia    : ARR_* bitmask
 *   ble_connected : true when BLE client is subscribed
 *   blink_on      : toggles at 2.5 Hz for alert blinking
 *   battery_pct   : 0-100
 */
void display_render(float bpm, uint8_t arrhythmia,
                    bool ble_connected, bool blink_on, int battery_pct);

#endif
