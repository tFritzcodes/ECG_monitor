#ifndef BLE_ECG_H
#define BLE_ECG_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*
 * NimBLE-based BLE peripheral exposing a Nordic UART Service (NUS).
 *
 * Watch → App  (TX notify characteristic):
 *   ASCII line:  "H:<bpm>,A:<arrhythmia>,B:<battery>\n"
 *   bpm         : integer BPM (255 = no signal)
 *   arrhythmia  : bitmask  0=none, 1=brady, 2=tachy, 4=irreg
 *   battery     : 0-100 %
 *
 * App → Watch  (RX write characteristic):
 *   "SHOCK\n"        — fire shock pulse (SHOCK_PULSE_MS)
 *   "REC_START\n"    — begin raw IR streaming (watch → app as RAW:v1,v2,v3,v4,v5)
 *   "REC_STOP\n"     — stop raw IR streaming
 *
 * Raw streaming (watch → app while recording):
 *   "RAW:v1,v2,v3,v4,v5\n"  — 5 raw IR ADC samples, 200 Hz effective rate
 */

void ble_ecg_init(void);

bool ble_ecg_connected(void);     /* true when app is subscribed to TX */

/* Send one BPM/arrhythmia/battery frame.  No-op if not connected. */
void ble_ecg_notify(uint8_t bpm, uint8_t arrhythmia, uint8_t battery);

/* Send an arbitrary string as a BLE notify (used for RAW: sample streaming). */
void ble_ecg_send_str(const char *msg, uint16_t len);

/* Returns true (once) when a new command arrives from the app. */
bool ble_ecg_get_command(char *buf, size_t len);

#endif
