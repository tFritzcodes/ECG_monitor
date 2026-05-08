#ifndef PAN_TOMPKINS_H
#define PAN_TOMPKINS_H

/*
 * Pan-Tompkins QRS detector at 200 Hz (exact paper parameters).
 *
 *   Pan, J. and Tompkins, W. J., "A Real-Time QRS Detection Algorithm,"
 *   IEEE Trans. Biomed. Eng., vol. 32, no. 3, pp. 230-236, Mar. 1985.
 *
 * Pipeline: DC removal -> LPF -> HPF -> derivative -> squaring -> MWI -> threshold
 *
 * Arrhythmia detection via R-R interval analysis (AHA/ACC clinical thresholds):
 *   Bradycardia : BPM < 60
 *   Tachycardia : BPM > 100
 *   Irregular   : ≥2 of the last 8 RR intervals deviate >35% from their median
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>   // for NAN

#define ARR_NONE        0x00
#define ARR_BRADYCARDIA 0x01
#define ARR_TACHYCARDIA 0x02
#define ARR_IRREGULAR   0x04

typedef struct {
    float    bpm;           // current BPM (NAN until first valid estimate)
    uint8_t  arrhythmia;    // bitmask of ARR_* flags
    uint32_t qrs_count;     // total QRS detections since init
} pt_result_t;

void pt_init(void);
void pt_reset(void);

// Process one IR sample. Returns true on QRS detection.
// result->bpm and result->arrhythmia are updated when a QRS is detected.
bool pt_process(int32_t ir_raw, pt_result_t *result);

#endif
