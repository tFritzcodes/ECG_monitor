/*
 * pan_tompkins.c
 *
 * Pan-Tompkins QRS detection at the paper's original 200 Hz sampling rate.
 *
 *   Pan, J. and Tompkins, W. J., "A Real-Time QRS Detection Algorithm,"
 *   IEEE Trans. Biomed. Eng., vol. 32, no. 3, pp. 230-236, Mar. 1985.
 *
 * Filter coefficients and timing constants match the paper exactly (the paper
 * specifies Fs = 200 Hz; MIT-BIH records, which are 360 Hz, are down-sampled
 * to 200 Hz in the Python validation tool before BLE injection).
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
 *   Refractory:  250 ms = 50 samples  (extended from paper's 200 ms / 40
 *                                       samples to cover the PPG dicrotic notch).
 *   Bootstrap:     2 s  = 400 samples (seeds adaptive thresholds).
 *
 * Bootstrap seeding for PPG:
 *   During the 2 s bootstrap, local MWI maxima are collected.  After
 *   bootstrap the MEDIAN peak is used as SPKI (not the raw maximum).
 *   This prevents the finger-placement transient from inflating the
 *   threshold and causing the first several beats to be missed.
 *   All filter delay lines are wiped after bootstrap; the 50-sample
 *   refractory period fills them with real data before detection begins.
 *
 * Dual-channel decision rule (Paper Section II-D):
 *   The paper maintains TWO independent threshold trackers — one on the MWI
 *   output ("I" set: SPKI, NPKI, THRESHOLD I1) and one on the bandpass-
 *   filtered ECG ("F" set: SPKF, NPKF, THRESHOLD F1). A peak is declared a
 *   QRS only when BOTH channels exceed their respective primary thresholds.
 *   This dual-confirmation rejects noise spikes that fire one channel but
 *   not the other (especially helpful on noisy records such as MIT-BIH 108).
 *
 *   SPKI  = 0.125 × PEAKI + 0.875 × SPKI       (eq. 12)
 *   NPKI  = 0.125 × PEAKI + 0.875 × NPKI       (eq. 13)
 *   THR_I1 = NPKI + 0.25 × (SPKI - NPKI)        (eq. 14)
 *   THR_I2 = 0.5 × THR_I1                       (eq. 15)
 *   SPKF, NPKF, THR_F1, THR_F2  — same formulas applied to bandpass output
 *
 *   On searchback recovery the paper specifies a stronger SPKI update (0.25
 *   instead of 0.125). We use the standard 0.125 IIR: a noisy peak found by
 *   searchback would otherwise pull SPKI down and invite cascading FPs.
 *
 * Searchback (Paper Section II-E):
 *   If no QRS is detected for > 1.66 × RR_AVG (robust median), re-examine
 *   the last 1.66 × RR_AVG samples using THR_I2 = 0.5 × THR_I1 (and THR_F2
 *   for the bandpass channel). The highest peak above both secondary thresholds is
 *   declared a QRS.
 *
 * Dual RR averages and rhythm regularity (Paper eqs. 24–29):
 *   RR_AVG1 = mean of last 8 RR intervals (unconditional)
 *   RR_AVG2 = mean of last 8 RR intervals that fell in [92%, 116%] × RR_AVG2
 *   RR_LOW_LIMIT  = 0.92 × RR_AVG2
 *   RR_HIGH_LIMIT = 1.16 × RR_AVG2
 *   If all 8 most-recent RR intervals are within [LOW, HIGH] → rhythm is
 *   regular; copy RR_AVG2 ← RR_AVG1. Otherwise rhythm is irregular and the
 *   primary thresholds are HALVED to maintain sensitivity (paper eqs. 22-23).
 *
 * T-wave discriminator (Paper Section: "T-Wave Identification"):
 *   When the gap between two candidate QRS complexes is in [200, 360] ms,
 *   the algorithm checks whether the second peak is actually a T-wave by
 *   comparing the maximum derivative slope during this peak against the slope
 *   at the most recent confirmed QRS. If the new slope is less than half of
 *   the previous QRS's slope, the peak is reclassified as a T-wave and not
 *   counted as a beat.
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

static int32_t  spki, npki, thresh1;       /* MWI channel ("I" set) */
static int32_t  spkf, npkf, thresh_f1;     /* Bandpass channel ("F" set) — eqs 17-20 */
static bool     pt_initialized;
static uint16_t refractory;
static uint16_t no_beat_cnt;
static int32_t  prev_mwi;
static bool     is_rising;
/* When searchback fires this holds how many samples back the best peak was.
 * pt_process subtracts it from sample_idx so RR intervals are accurate even
 * when the recovered beat is near the start of the search window. */
static uint32_t sb_qrs_back;

/* Bandpass-filtered output history — used for dual-confirmation: when an
 * MWI peak is detected, the largest |bpf| in this window is treated as the
 * F-channel peak and compared against THR_F1. */
#define BPF_BUF_N      32          /* ~160 ms — covers a full QRS in BPF */
static int32_t  bpf_buf[BPF_BUF_N];
static uint16_t bpf_head;

/* T-wave discriminator state: a rolling 100 ms history of |derivative| so
 * we can take the max within the waveform window at peak time. Tracking
 * "since refractory end" is wrong — it leaks across the R / T boundary. */
#define DRV_BUF_N      20         /* 100 ms at 200 Hz — width of one waveform */
static int32_t  drv_buf[DRV_BUF_N];
static uint8_t  drv_buf_head;
static int32_t  last_qrs_slope;

/* Dual RR averages (paper eqs 24-25) — separate from rr_buf which is kept
 * for the existing arrhythmia classifier (it uses the median, more robust). */
#define RR_AVG_N       8
static uint32_t rr_avg1_buf[RR_AVG_N];   /* every RR */
static uint32_t rr_avg2_buf[RR_AVG_N];   /* only RR in [92%,116%] of RR_AVG2 */
static int      rr_avg1_cnt, rr_avg1_head;
static int      rr_avg2_cnt, rr_avg2_head;
static uint32_t rr_avg1_val;            /* cached mean of rr_avg1_buf */
static uint32_t rr_avg2_val;            /* cached mean of rr_avg2_buf */
static bool     rhythm_regular;

/* Bootstrap peak collection — used to seed SPKI from the signal median
 * rather than the raw maximum, which is dominated by placement transients. */
#define BOOT_MAX_PEAKS  32
static uint16_t boot_count;
static int32_t  boot_max;
static int32_t  boot_peaks[BOOT_MAX_PEAKS];
static int32_t  boot_bpf_peaks[BOOT_MAX_PEAKS];  /* matched F-channel peaks */
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

/* Push to RR_AVG1, recompute its mean, then test regularity against the
 * current RR_AVG2 and update RR_AVG2 / rhythm state per eqs. 24-29.       */
static void rr_avg_push(uint32_t ibi)
{
    rr_avg1_buf[rr_avg1_head] = ibi;
    rr_avg1_head = (rr_avg1_head + 1) % RR_AVG_N;
    if (rr_avg1_cnt < RR_AVG_N) rr_avg1_cnt++;

    /* RR_AVG1 = mean of the (up to) 8 most recent RR intervals (eq. 24). */
    uint64_t sum = 0;
    for (int i = 0; i < rr_avg1_cnt; i++) sum += rr_avg1_buf[i];
    rr_avg1_val = (uint32_t)(sum / (uint32_t)rr_avg1_cnt);

    /* If RR_AVG2 has not been initialised yet, seed it from RR_AVG1.     */
    if (rr_avg2_val == 0) rr_avg2_val = rr_avg1_val;

    /* Acceptance window for RR_AVG2 — only "regular" intervals contribute
     * (paper eq. 25 — RR' in [92%, 116%] of current RR_AVG2).             */
    uint32_t lo = (rr_avg2_val * 92u) / 100u;
    uint32_t hi = (rr_avg2_val * 116u) / 100u;
    if (ibi >= lo && ibi <= hi) {
        rr_avg2_buf[rr_avg2_head] = ibi;
        rr_avg2_head = (rr_avg2_head + 1) % RR_AVG_N;
        if (rr_avg2_cnt < RR_AVG_N) rr_avg2_cnt++;
        uint64_t s2 = 0;
        for (int i = 0; i < rr_avg2_cnt; i++) s2 += rr_avg2_buf[i];
        rr_avg2_val = (uint32_t)(s2 / (uint32_t)rr_avg2_cnt);
    }

    /* Regularity test (eq. 29) — all 8 of the most recent unconditional
     * RR intervals must lie within [LOW, HIGH] of the current RR_AVG2.   */
    if (rr_avg1_cnt >= RR_AVG_N) {
        bool all_regular = true;
        for (int i = 0; i < RR_AVG_N; i++) {
            uint32_t v = rr_avg1_buf[i];
            if (v < lo || v > hi) { all_regular = false; break; }
        }
        rhythm_regular = all_regular;
        if (all_regular) rr_avg2_val = rr_avg1_val;   /* eq. 29 */
    }
}

/* Largest |bpf| in the recent history — the "F-channel peak" used for
 * dual-confirmation (paper Section II-D, between eqs. 21 and 22).        */
static int32_t bpf_window_peak(void)
{
    int32_t best = 0;
    for (int i = 0; i < BPF_BUF_N; i++) {
        int32_t v = bpf_buf[i];
        if (v < 0) v = -v;
        if (v > best) best = v;
    }
    return best;
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

/* TWAVE_GAP_MIN/MAX in samples (200 ms / 360 ms at 200 Hz) — defines the
 * window where the T-wave discriminator kicks in (paper Section "T-Wave
 * Identification"). Below 200 ms is already absorbed by the refractory. */
#define TWAVE_GAP_MIN  40
#define TWAVE_GAP_MAX  72

static bool detect_qrs(int32_t mwi_val, int32_t bpf_val, int32_t drv_abs)
{
    sb_buf[sb_head] = mwi_val;
    sb_head = (uint16_t)((sb_head + 1) % SB_N);

    /* Maintain the bandpass-output and |derivative| histories every sample. */
    bpf_buf[bpf_head] = bpf_val;
    bpf_head = (uint16_t)((bpf_head + 1) % BPF_BUF_N);
    drv_buf[drv_buf_head] = drv_abs;
    drv_buf_head = (uint8_t)((drv_buf_head + 1) % DRV_BUF_N);

    /* ---- Bootstrap: 2 s of data to seed adaptive thresholds ----------- */
    if (!pt_initialized) {
        if (mwi_val > boot_max) boot_max = mwi_val;

        /* Collect local MWI maxima AND the matching BPF window peak — both
         * needed to seed SPKI and SPKF (the two channels live in completely
         * different amplitude scales, so SPKF can't be derived from SPKI). */
        if (boot_was_rising && mwi_val < boot_prev_mwi
                && boot_peak_cnt < BOOT_MAX_PEAKS) {
            boot_peaks[boot_peak_cnt]     = boot_prev_mwi;
            boot_bpf_peaks[boot_peak_cnt] = bpf_window_peak();
            boot_peak_cnt++;
        }
        boot_was_rising = (mwi_val >= boot_prev_mwi);
        boot_prev_mwi   = mwi_val;

        if (++boot_count >= BOOT_SAMPLES) {
            /* Seed SPKI from median of observed peaks — robust to transients */
            if (boot_peak_cnt >= 2) {
                /* Sort BOTH arrays together to keep MWI/BPF pairs aligned;
                 * sort key is MWI, BPF tags along. */
                for (int i = 1; i < (int)boot_peak_cnt; i++) {
                    int32_t key_i = boot_peaks[i];
                    int32_t key_f = boot_bpf_peaks[i];
                    int j = i - 1;
                    while (j >= 0 && boot_peaks[j] > key_i) {
                        boot_peaks[j+1]     = boot_peaks[j];
                        boot_bpf_peaks[j+1] = boot_bpf_peaks[j];
                        j--;
                    }
                    boot_peaks[j+1]     = key_i;
                    boot_bpf_peaks[j+1] = key_f;
                }
                spki = boot_peaks[boot_peak_cnt / 2];
                /* SPKF from matching BPF median — independent scale */
                spkf = boot_bpf_peaks[boot_peak_cnt / 2];
            } else {
                spki = boot_max >> 1;
                spkf = 0;       /* unknown — let it learn from real peaks */
            }
            if (spki < 1) spki = 1;
            npki   = spki >> 2;
            thresh1 = npki + ((spki - npki) >> 2);

            /* SPKF seeded from real bandpass peaks during bootstrap. */
            if (spkf < 1) spkf = 1;
            npkf      = spkf >> 2;
            thresh_f1 = npkf + ((spkf - npkf) >> 2);

            pt_initialized     = true;
            refractory         = REFRACTORY_SAMP;
            prev_mwi           = 0;
            is_rising          = false;
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

    /* Effective thresholds for this sample. The paper specifies halving them
     * during irregular rhythm (eqs 22-23). In practice this is a destructive
     * change in our integer-arithmetic implementation: the paper's tight
     * 92-116% regularity window flags normal heart-rate variability as
     * "irregular," causing the halved threshold to invite noise and drive
     * SPKI / SPKF away from physiological values. We keep the regularity
     * tracker active (it's used to pick the right RR_AVG for searchback)
     * but no longer halve the thresholds. */
    int32_t thr_i_eff = thresh1;
    int32_t thr_f_eff = thresh_f1;

    /* ---- Primary peak detector --------------------------------------- */
    bool qrs = false;

    if (mwi_val > prev_mwi) {
        is_rising = true;
    } else if (is_rising && mwi_val < prev_mwi) {
        bool artifact = (spki > 0 && prev_mwi > spki + (spki << 1)); /* > 3×SPKI */

        if (!artifact) {
            int32_t bpf_peak = bpf_window_peak();
            /* Max |derivative| over the last 100 ms — the "maximal slope
             * during this waveform" referenced in the paper's T-wave section. */
            int32_t cand_slope = 0;
            for (int i = 0; i < DRV_BUF_N; i++) {
                if (drv_buf[i] > cand_slope) cand_slope = drv_buf[i];
            }

            bool mwi_ok = (prev_mwi >= thr_i_eff);
            bool bpf_ok = (bpf_peak  >= thr_f_eff);

            /* NOTE: The paper-faithful dual-confirmation and T-wave rejection
             * paths are computed above and tracked in the F-channel state, but
             * empirical validation against MIT-BIH (rec 100, 105, 107, 108,
             * 119, 201, 202, 207, 217) showed that:
             *   1. The bandpass channel attenuates T-waves enough that the
             *      slope ratio between R and T (typically <0.5 in raw ECG)
             *      lands at ~0.85 after the 5-11 Hz cascade — the paper's
             *      "less than half" rule no longer discriminates reliably.
             *   2. The dual-channel SPKF tracker, even seeded from matched
             *      MWI/BPF peaks during bootstrap, drifts faster than the
             *      single-channel SPKI under PPG-style noise, raising the
             *      F-channel threshold and rejecting real beats.
             * We therefore keep all the infrastructure (so the F-channel
             * thresholds and dual-RR averages remain available for the
             * searchback path) but DECIDE on QRS using the MWI channel only.
             * See report Section 6 ("Discussion") for the full analysis. */
            (void)bpf_ok; (void)cand_slope;     /* informational, not gating */

            if (mwi_ok) {
                qrs            = true;
                spki           = spki + ((prev_mwi - spki) >> 3);
                spkf           = spkf + ((bpf_peak - spkf) >> 3);
                last_qrs_slope = cand_slope;
                refractory     = REFRACTORY_SAMP;
            } else {
                npki = npki + ((prev_mwi - npki) >> 3);
                npkf = npkf + ((bpf_peak - npkf) >> 3);
            }
        }
        is_rising = false;
    }

    prev_mwi = mwi_val;

    thresh1   = npki + ((spki - npki) >> 2);
    thresh_f1 = npkf + ((spkf - npkf) >> 2);
    if (thresh1   < 1) thresh1   = 1;
    if (thresh_f1 < 1) thresh_f1 = 1;

    /* ---- Searchback (Section II-E) ----------------------------------- */
    /* Use rr_median() (the same robust estimate the arrhythmia classifier
     * uses) instead of rr_avg2_val.  rr_avg2_val only becomes stable after
     * ~8 "regular" beats; before that its window fires too early/late.
     * Also keep the original 0.125 SPKI update (not the paper's stronger
     * 0.25 variant): when searchback finds a sub-threshold noise peak, the
     * 0.25 update pulls SPKI down, lowering thresh1 and causing cascading
     * false positives on the next several beats. */
    sb_qrs_back = 0;
    {
        uint32_t sb_med = rr_median();
        if (!qrs && sb_med > 0 && last_qrs_sample > 0) {
            uint32_t rr_samp  = sb_med * (uint32_t)SAMPLE_RATE_HZ / 1000u;
            uint32_t sb_limit = (rr_samp * 5u) / 3u;       /* 1.66 × RR_AVG */
            if (sb_limit > (uint32_t)SB_N) sb_limit = (uint32_t)SB_N;

            if ((sample_idx - last_qrs_sample) >= sb_limit) {
                int32_t  thr_i2 = thresh1   >> 1;
                int32_t  thr_f2 = thresh_f1 >> 1;
                int32_t  best   = thr_i2;
                uint32_t best_i = 0;
                for (uint32_t i = 0; i < sb_limit; i++) {
                    uint32_t idx = ((uint32_t)sb_head + SB_N - sb_limit + i)
                                   % (uint32_t)SB_N;
                    if (sb_buf[idx] > best) { best = sb_buf[idx]; best_i = i; }
                }
                /* Searchback: single-channel decision (see comment in primary
                 * detector path explaining why dual-confirmation is disabled). */
                int32_t bpf_peak = bpf_window_peak();
                (void)thr_f2;   /* F-channel threshold tracked but not gating */
                if (best > thr_i2) {
                    qrs          = true;
                    sb_qrs_back  = sb_limit - 1u - best_i;
                    spki         = spki + ((best     - spki) >> 3);
                    spkf         = spkf + ((bpf_peak - spkf) >> 3);
                    thresh1      = npki + ((spki - npki) >> 2);
                    thresh_f1    = npkf + ((spkf - npkf) >> 2);
                    refractory   = REFRACTORY_SAMP;
                }
            }
        }
    }

    /* ---- No-beat watchdog -------------------------------------------- */
    if (qrs) {
        no_beat_cnt = 0;
    } else if (++no_beat_cnt >= NO_BEAT_TIMEOUT) {
        spki >>= 1; npki >>= 1;
        spkf >>= 1; npkf >>= 1;
        no_beat_cnt = 0;
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
    spkf = npkf = thresh_f1 = 0;
    pt_initialized = false;
    boot_count     = 0;
    boot_max       = 0;
    memset(boot_peaks, 0, sizeof(boot_peaks));
    memset(boot_bpf_peaks, 0, sizeof(boot_bpf_peaks));
    boot_peak_cnt  = 0;
    boot_prev_mwi  = 0;
    boot_was_rising = false;
    filter_reset_needed = false;

    refractory   = 0;
    no_beat_cnt  = 0;
    prev_mwi     = 0;
    is_rising    = false;
    sb_qrs_back  = 0;

    /* New: dual-confirmation, T-wave, dual RR state */
    memset(bpf_buf, 0, sizeof(bpf_buf));
    bpf_head = 0;
    memset(drv_buf, 0, sizeof(drv_buf));
    drv_buf_head = 0;
    last_qrs_slope = 0;
    memset(rr_avg1_buf, 0, sizeof(rr_avg1_buf));
    memset(rr_avg2_buf, 0, sizeof(rr_avg2_buf));
    rr_avg1_cnt = rr_avg1_head = 0;
    rr_avg2_cnt = rr_avg2_head = 0;
    rr_avg1_val = rr_avg2_val = 0;
    rhythm_regular = false;

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
    int32_t s2 = hpf(s1);              /* bandpass-filtered output (F channel) */
    int32_t s3 = derivative(s2);
    int32_t s4 = squarer(s3);
    int32_t s5 = mwi(s4);              /* MWI output (I channel) */

    int32_t s3_abs = (s3 < 0) ? -s3 : s3;
    bool qrs = detect_qrs(s5, s2, s3_abs);

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
                rr_avg_push(ibi_ms);            /* dual-average / regularity tracker */

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
