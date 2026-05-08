/*
 * pan_tompkins.c
 *
 * Pan-Tompkins QRS detection at 200 Hz (exact paper parameters).
 *
 *   Pan, J. and Tompkins, W. J., "A Real-Time QRS Detection Algorithm,"
 *   IEEE Trans. Biomed. Eng., vol. 32, no. 3, pp. 230-236, Mar. 1985.
 *
 * All filter delays, window lengths, and timing constants match the original
 * 200 Hz design exactly — no scaling required.
 *
 *   LPF:  H(z) = (1-z^-6)^2 / (1-z^-1)^2          [N=6, DC gain = 36]
 *         y(n) = 2y(n-1) - y(n-2) + x(n) - 2x(n-6) + x(n-12)
 *         Normalize >>5 (÷32; within 11% of exact ÷36).
 *         Delay line: 13 slots.
 *
 *   HPF:  allpass - running-sum LPF                  [32-sample window]
 *         Allpass delay = 16 samples.
 *         LPF  = (1/32)(1 - z^-32)/(1 - z^-1)  i.e. 32-sample running sum ÷ 32
 *         HPF  = allpass - LPF:
 *         y(n) = y(n-1) - x(n)/32 + x(n-16) - x(n-17) + x(n-32)/32
 *         Delay line: 33 slots.
 *
 *   Derivative:  5-point                             [paper eq. 4]
 *         y(n) = (1/8)[2x(n) + x(n-1) - x(n-3) - 2x(n-4)]
 *
 *   Squaring:  clamp ±3000 then x*x (prevents int32 overflow in MWI).
 *
 *   MWI:  150 ms = 30 samples @ 200 Hz.
 *
 *   Refractory:  200 ms = 40 samples.
 *   Bootstrap:     2 s  = 400 samples (seeds adaptive thresholds).
 *
 * Bootstrap seeding for PPG:
 *   During the 2 s bootstrap, local MWI maxima are collected.  After
 *   bootstrap the MEDIAN peak is used as SPKI (not the raw maximum).
 *   This prevents the finger-placement transient from inflating the
 *   threshold and causing the first several beats to be missed.
 *   All filter delay lines are wiped after bootstrap; the 40-sample
 *   refractory period fills them with real data before detection begins.
 *
 * Searchback (Paper Section II-E):
 *   If no QRS is detected for > 1.66 × RR_avg, re-examine the last
 *   1.66 × RR_avg samples using THRESHOLD II = 0.5 × THRESHOLD I.
 *   The highest MWI peak above THR2 in that window is declared a QRS.
 *
 * Adaptive thresholds (Paper Section II-D):
 *   SPKI  = 0.125 × signal_peak + 0.875 × SPKI   (1/8 IIR)
 *   NPKI  = 0.125 × noise_peak  + 0.875 × NPKI
 *   THR1  = NPKI + 0.25 × (SPKI - NPKI)           (THRESHOLD I)
 *   THR2  = 0.5 × THR1                             (THRESHOLD II, searchback)
 *
 * Artifact rejection:
 *   Peak > 3 × SPKI → silently discarded; SPKI, NPKI, refractory unchanged.
 *
 * No-beat watchdog:
 *   5 s without a beat → halve both SPKI and NPKI proportionally so THR1
 *   drops without distorting the signal/noise ratio.
 *
 * BPM reporting:
 *   BPM is not reported until 3 RR intervals (4 beats) have been collected.
 *   The median of the buffered RR intervals is then used, making the first
 *   reading immune to a single missed or doubled beat.
 *   An EMA (alpha = BPM_EMA_ALPHA) smooths subsequent updates with no
 *   hard step-clamp, so the display converges quickly to the true rate.
 *
 * PPG DC removal:
 *   Slow IIR low-pass (alpha = 0.005, tau ~ 2 s) tracks the baseline.
 *   Residual AC is fed into the pipeline.
 */

#include "pan_tompkins.h"
#include "app_config.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

// =========================================================================
// Filter state
// =========================================================================

#define LPF_N   13
static int32_t lpf_x[LPF_N];
static int32_t lpf_y1, lpf_y2;
static uint8_t lpf_head;

#define HPF_N   33
static int32_t hpf_x[HPF_N];
static int32_t hpf_y;
static uint8_t hpf_head;

static int32_t drv_x[5];
static uint8_t drv_head;

#define MWI_N   30
static int32_t mwi_x[MWI_N];
static int32_t mwi_sum;
static uint8_t mwi_head;

static float   ppg_dc;
static bool    dc_seeded;

/* Set when bootstrap completes; pt_process resets all filter state before
 * the very next sample so detection starts on a clean pipeline. */
static bool filter_reset_needed;

// =========================================================================
// Detector state
// =========================================================================

#define REFRACTORY_SAMP  50   /* 250 ms — covers PPG dicrotic notch (ECG paper used 200 ms) */
#define BOOT_SAMPLES    400
#define NO_BEAT_TIMEOUT 1000

#define SB_N            600
static int32_t  sb_buf[SB_N];
static uint16_t sb_head;

static int32_t  spki, npki, thresh1;
static bool     pt_initialized;
static uint16_t refractory;
static uint16_t no_beat_cnt;
static int32_t  prev_mwi;
static bool     is_rising;
/* When searchback fires this holds how many samples back the best peak was.
 * pt_process subtracts it from sample_idx so RR intervals are accurate even
 * when the recovered beat is near the start of the search window. */
static uint32_t sb_qrs_back;

/* Bootstrap peak collection — used to seed SPKI from the signal median
 * rather than the raw maximum, which is dominated by placement transients. */
#define BOOT_MAX_PEAKS  32
static uint16_t boot_count;
static int32_t  boot_max;
static int32_t  boot_peaks[BOOT_MAX_PEAKS];
static uint8_t  boot_peak_cnt;
static int32_t  boot_prev_mwi;
static bool     boot_was_rising;

// =========================================================================
// R-R interval / arrhythmia state
// =========================================================================

#define RR_BUF  8
static uint32_t rr_buf[RR_BUF];
static int      rr_count;
static int      rr_head;
static uint32_t last_qrs_sample;
static uint32_t sample_idx;

static pt_result_t s_result;

// =========================================================================
// Filters
// =========================================================================

static int32_t lpf(int32_t x)
{
    lpf_x[lpf_head] = x;
    uint8_t i6  = (uint8_t)((lpf_head + LPF_N -  6) % LPF_N);
    uint8_t i12 = (uint8_t)((lpf_head + LPF_N - 12) % LPF_N);
    int32_t y = (lpf_y1 << 1) - lpf_y2
              + lpf_x[lpf_head]
              - (lpf_x[i6] << 1)
              + lpf_x[i12];
    lpf_y2   = lpf_y1;
    lpf_y1   = y;
    lpf_head = (uint8_t)((lpf_head + 1) % LPF_N);
    return y >> 5;
}

static int32_t hpf(int32_t x)
{
    hpf_x[hpf_head] = x;
    uint8_t i16 = (uint8_t)((hpf_head + HPF_N - 16) % HPF_N);
    uint8_t i17 = (uint8_t)((hpf_head + HPF_N - 17) % HPF_N);
    uint8_t i32 = (uint8_t)((hpf_head + HPF_N - 32) % HPF_N);
    hpf_y = hpf_y
          - (x            >> 5)
          + hpf_x[i16]
          - hpf_x[i17]
          + (hpf_x[i32]   >> 5);
    hpf_head = (uint8_t)((hpf_head + 1) % HPF_N);
    return hpf_y;
}

static int32_t derivative(int32_t x)
{
    drv_x[drv_head] = x;
    uint8_t n1 = (uint8_t)((drv_head + 4) % 5);
    uint8_t n3 = (uint8_t)((drv_head + 2) % 5);
    uint8_t n4 = (uint8_t)((drv_head + 1) % 5);
    int32_t y = ((drv_x[drv_head] << 1) + drv_x[n1]
                - drv_x[n3] - (drv_x[n4] << 1)) >> 3;
    drv_head = (uint8_t)((drv_head + 1) % 5);
    return y;
}

static int32_t squarer(int32_t x)
{
    if (x >  3000) x =  3000;
    if (x < -3000) x = -3000;
    return x * x;
}

static int32_t mwi(int32_t x)
{
    mwi_sum        -= mwi_x[mwi_head];
    mwi_x[mwi_head] = x;
    mwi_sum        += x;
    mwi_head        = (uint8_t)((mwi_head + 1) % MWI_N);
    return mwi_sum / MWI_N;
}

// =========================================================================
// R-R / arrhythmia helpers
// =========================================================================

static void rr_push(uint32_t ibi)
{
    rr_buf[rr_head] = ibi;
    rr_head = (rr_head + 1) % RR_BUF;
    if (rr_count < RR_BUF) rr_count++;
}

static uint32_t rr_median(void)
{
    if (rr_count < 2) return 0;
    uint32_t tmp[RR_BUF];
    int n = rr_count;
    for (int i = 0; i < n; i++)
        tmp[i] = rr_buf[(rr_head - n + i + RR_BUF) % RR_BUF];
    for (int i = 1; i < n; i++) {
        uint32_t key = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = key;
    }
    return tmp[n / 2];
}

static uint8_t classify_arrhythmia(float bpm)
{
    if (bpm != bpm) return ARR_NONE;
    uint8_t flags = ARR_NONE;
    if (bpm < BRADY_BPM_THRESHOLD) flags |= ARR_BRADYCARDIA;
    if (bpm > TACHY_BPM_THRESHOLD) flags |= ARR_TACHYCARDIA;
    if (rr_count >= 4) {
        uint32_t med = rr_median();
        if (med > 0) {
            uint32_t thr = (uint32_t)(med * IRREG_RR_FRAC);
            int outliers = 0;
            for (int i = 0; i < rr_count; i++) {
                int idx = (rr_head - rr_count + i + RR_BUF) % RR_BUF;
                uint32_t d = (rr_buf[idx] > med) ? (rr_buf[idx] - med)
                                                  : (med - rr_buf[idx]);
                if (d > thr) outliers++;
            }
            if (outliers >= IRREG_MIN_OUTLIERS) flags |= ARR_IRREGULAR;
        }
    }
    return flags;
}

// =========================================================================
// Core detector
// =========================================================================

static bool detect_qrs(int32_t mwi_val)
{
    sb_buf[sb_head] = mwi_val;
    sb_head = (uint16_t)((sb_head + 1) % SB_N);

    /* ---- Bootstrap: 2 s of data to seed adaptive thresholds ----------- */
    if (!pt_initialized) {
        if (mwi_val > boot_max) boot_max = mwi_val;

        /* Collect local MWI maxima for median-based SPKI seeding */
        if (boot_was_rising && mwi_val < boot_prev_mwi
                && boot_peak_cnt < BOOT_MAX_PEAKS) {
            boot_peaks[boot_peak_cnt++] = boot_prev_mwi;
        }
        boot_was_rising = (mwi_val >= boot_prev_mwi);
        boot_prev_mwi   = mwi_val;

        if (++boot_count >= BOOT_SAMPLES) {
            /* Seed SPKI from median of observed peaks — robust to transients */
            if (boot_peak_cnt >= 2) {
                /* Insertion sort on the small peak array */
                for (int i = 1; i < (int)boot_peak_cnt; i++) {
                    int32_t key = boot_peaks[i]; int j = i - 1;
                    while (j >= 0 && boot_peaks[j] > key) {
                        boot_peaks[j+1] = boot_peaks[j]; j--;
                    }
                    boot_peaks[j+1] = key;
                }
                spki = boot_peaks[boot_peak_cnt / 2];   /* median */
            } else {
                /* Fallback: no clear peaks observed — be conservative */
                spki = boot_max >> 1;
            }
            if (spki < 1) spki = 1;
            npki   = spki >> 2;
            thresh1 = npki + ((spki - npki) >> 2);

            pt_initialized     = true;
            refractory         = REFRACTORY_SAMP;
            prev_mwi           = 0;
            is_rising          = false;
            /* Signal pt_process to wipe all filter delay lines before next
             * sample, then rely on the refractory period to refill them. */
            filter_reset_needed = true;
        }
        return false;
    }

    /* ---- Refractory blanking ------------------------------------------ */
    if (refractory > 0) {
        refractory--;
        prev_mwi = mwi_val;
        return false;
    }

    /* ---- Primary peak detector --------------------------------------- */
    bool qrs = false;

    if (mwi_val > prev_mwi) {
        is_rising = true;
    } else if (is_rising && mwi_val < prev_mwi) {
        bool artifact = (spki > 0 && prev_mwi > spki + (spki << 1)); /* > 3×SPKI */

        if (!artifact) {
            if (prev_mwi >= thresh1) {
                qrs        = true;
                spki       = spki + ((prev_mwi - spki) >> 3);
                refractory = REFRACTORY_SAMP;
            } else {
                npki = npki + ((prev_mwi - npki) >> 3);
            }
        }
        is_rising = false;
    }

    prev_mwi = mwi_val;

    thresh1 = npki + ((spki - npki) >> 2);
    if (thresh1 < 1) thresh1 = 1;

    /* ---- Searchback (Section II-E) ----------------------------------- */
    sb_qrs_back = 0;
    if (!qrs && rr_count >= 2 && last_qrs_sample > 0) {
        uint32_t rr_ms    = rr_median();
        if (rr_ms > 0) {
            uint32_t rr_samp  = rr_ms * (uint32_t)SAMPLE_RATE_HZ / 1000u;
            uint32_t sb_limit = (rr_samp * 5u) / 3u;
            if (sb_limit > (uint32_t)SB_N) sb_limit = (uint32_t)SB_N;

            if ((sample_idx - last_qrs_sample) >= sb_limit) {
                int32_t  thr2   = thresh1 >> 1;
                int32_t  best   = thr2;
                uint32_t best_i = 0;
                for (uint32_t i = 0; i < sb_limit; i++) {
                    uint32_t idx = ((uint32_t)sb_head + SB_N - sb_limit + i)
                                   % (uint32_t)SB_N;
                    if (sb_buf[idx] > best) { best = sb_buf[idx]; best_i = i; }
                }
                if (best > thr2) {
                    qrs           = true;
                    /* i=0 is oldest, i=sb_limit-1 is most recent (1 sample ago).
                     * Back-offset gives the actual sample position of the QRS so
                     * IBI is measured to the real peak, not the current sample. */
                    sb_qrs_back   = sb_limit - 1u - best_i;
                    spki          = spki + ((best - spki) >> 3);
                    thresh1       = npki + ((spki - npki) >> 2);
                    refractory    = REFRACTORY_SAMP;
                }
            }
        }
    }

    /* ---- No-beat watchdog -------------------------------------------- */
    if (qrs) {
        no_beat_cnt = 0;
    } else if (++no_beat_cnt >= NO_BEAT_TIMEOUT) {
        /* Halve both SPKI and NPKI together to maintain their ratio and
         * keep THR1 proportional — avoids false positives after recovery. */
        spki        >>= 1;
        npki        >>= 1;
        no_beat_cnt   = 0;
    }

    return qrs;
}

// =========================================================================
// Public API
// =========================================================================

static void reset_filters(void)
{
    memset(lpf_x, 0, sizeof(lpf_x));
    lpf_y1 = lpf_y2 = 0;
    lpf_head = 0;
    memset(hpf_x, 0, sizeof(hpf_x));
    hpf_y = 0;
    hpf_head = 0;
    memset(drv_x, 0, sizeof(drv_x));
    drv_head = 0;
    memset(mwi_x, 0, sizeof(mwi_x));
    mwi_sum = 0;
    mwi_head = 0;
}

void pt_init(void)
{
    reset_filters();
    memset(rr_buf, 0, sizeof(rr_buf));
    memset(sb_buf, 0, sizeof(sb_buf));
    sb_head = 0;

    spki = npki = thresh1 = 0;
    pt_initialized = false;
    boot_count     = 0;
    boot_max       = 0;
    memset(boot_peaks, 0, sizeof(boot_peaks));
    boot_peak_cnt  = 0;
    boot_prev_mwi  = 0;
    boot_was_rising = false;
    filter_reset_needed = false;

    refractory   = 0;
    no_beat_cnt  = 0;
    prev_mwi     = 0;
    is_rising    = false;
    sb_qrs_back  = 0;

    ppg_dc    = 0.0f;
    dc_seeded = false;

    rr_count = rr_head = 0;
    last_qrs_sample = 0;
    sample_idx = 0;

    s_result.bpm        = NAN;
    s_result.arrhythmia = ARR_NONE;
    s_result.qrs_count  = 0;
}

void pt_reset(void) { pt_init(); }

bool pt_process(int32_t ir_raw, pt_result_t *result)
{
    sample_idx++;

    /* DC removal */
    if (!dc_seeded) { ppg_dc = (float)ir_raw; dc_seeded = true; }
    else            { ppg_dc += 0.005f * ((float)ir_raw - ppg_dc); }
    int32_t ac = (int32_t)((float)ir_raw - ppg_dc);

    /* Wipe filter delay lines immediately after bootstrap ends so the
     * refractory period (next 40 samples) refills them cleanly. */
    if (filter_reset_needed) {
        filter_reset_needed = false;
        reset_filters();
    }

    int32_t s1 = lpf(ac);
    int32_t s2 = hpf(s1);
    int32_t s3 = derivative(s2);
    int32_t s4 = squarer(s3);
    int32_t s5 = mwi(s4);

    bool qrs = detect_qrs(s5);

    if (qrs) {
        s_result.qrs_count++;
        /* Searchback sets sb_qrs_back to the offset of the recovered peak so
         * we timestamp the QRS at the actual peak, not at the current sample. */
        uint32_t qrs_idx = sample_idx - sb_qrs_back;

        if (last_qrs_sample > 0) {
            uint32_t ibi_ms = ((qrs_idx - last_qrs_sample) * 1000u)
                              / SAMPLE_RATE_HZ;

            if (ibi_ms > 273u && ibi_ms < 1714u) {   /* 35–220 BPM gate */
                rr_push(ibi_ms);

                /* Wait for 3 RR intervals (4 beats) before trusting the
                 * median — prevents a single missed beat from printing ~40 BPM. */
                if (rr_count >= 3) {
                    uint32_t med_ms = rr_median();
                    float new_bpm = (med_ms > 0)
                                    ? (60000.0f / (float)med_ms) : NAN;

                    if (new_bpm == new_bpm) {
                        if (s_result.bpm != s_result.bpm) {
                            /* First reading: snap directly to measured value */
                            s_result.bpm = new_bpm;
                        } else {
                            /* EMA smoothing — no hard step-clamp so the display
                             * converges quickly rather than ramping slowly. */
                            s_result.bpm += BPM_EMA_ALPHA
                                            * (new_bpm - s_result.bpm);
                        }
                    }

                    s_result.arrhythmia = classify_arrhythmia(s_result.bpm);
                }
            }
        }

        last_qrs_sample = qrs_idx;
    }

    if (result) *result = s_result;
    return qrs;
}
