#ifndef APP_CONFIG_H
#define APP_CONFIG_H

// MAX30102 sampling
#define SAMPLE_RATE_HZ      200
#define FINGER_IR_MIN       20000u
#define IR_SAT_HARD_MAX     260000u
#define FINGER_SETTLE_MS    500u    // PT bootstrap (2s) handles its own warmup

// BPM display EMA smoothing
#define BPM_EMA_ALPHA       0.25f
#define BPM_STEP_CLAMP_FRAC 0.30f
#define MIN_BPM             35.0f
#define MAX_BPM             220.0f

// Arrhythmia thresholds
#define BRADY_BPM_THRESHOLD  60.0f
#define TACHY_BPM_THRESHOLD 100.0f
#define IRREG_RR_FRAC        0.35f  // >35% RR deviation = irregular (PPG jitter > ECG)
#define IRREG_MIN_OUTLIERS   2      // need this many outlier beats before flagging

// BLE notify interval
#define BLE_NOTIFY_MS       500u

// Shock pulse duration (ms) on BLE "SHOCK" command
#define SHOCK_PULSE_MS      200u

#endif
