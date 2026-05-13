"""
pan_tompkins.py — Exact Python port of pan_tompkins.c (integer arithmetic).

Mirrors every change in the C file:
  • Bootstrap seeds SPKI from median of collected MWI peaks, not boot_max.
  • All filter delay lines are reset after bootstrap; refractory refills them.
  • No-beat watchdog halves both SPKI and NPKI to preserve the threshold ratio.
  • BPM not reported until rr_count >= 3 (4 beats) to prevent ~40 BPM cold start.
  • No step-clamp on BPM EMA — converges quickly once first valid reading lands.
"""

import math

ARR_NONE        = 0x00
ARR_BRADYCARDIA = 0x01
ARR_TACHYCARDIA = 0x02
ARR_IRREGULAR   = 0x04

_FS          = 200
_LPF_N       = 13
_HPF_N       = 33
_MWI_N       = 30
_REFRAC      = 50   # 250 ms — covers PPG dicrotic notch (ECG paper used 200 ms)
_BOOT        = 400
_NO_BEAT_TO  = 1000
_SB_N        = 600
_BRADY       = 60.0
_TACHY       = 100.0
_IRREG_FRAC  = 0.35
_IRREG_MIN   = 2
_EMA_ALPHA   = 0.25
_BOOT_PEAKS  = 32
# Paper-faithful additions
_BPF_BUF_N      = 32     # ~160 ms bandpass history for F-channel peak search
_DRV_BUF_N      = 20     # 100 ms |derivative| history — the "waveform" window
_RR_AVG_N       = 8      # paper's RR_AVG1/AVG2 length
_TWAVE_GAP_MIN  = 40     # 200 ms in samples
_TWAVE_GAP_MAX  = 72     # 360 ms in samples


class PanTompkins:
    """Stateful Pan-Tompkins QRS detector, exact integer port of the C code."""

    def __init__(self):
        self.reset()

    def reset(self):
        # filters
        self._lpf_x    = [0] * _LPF_N
        self._lpf_y1   = 0
        self._lpf_y2   = 0
        self._lpf_head = 0

        self._hpf_x    = [0] * _HPF_N
        self._hpf_y    = 0
        self._hpf_head = 0

        self._drv_x    = [0] * 5
        self._drv_head = 0

        self._mwi_x    = [0] * _MWI_N
        self._mwi_sum  = 0
        self._mwi_head = 0

        self._ppg_dc    = 0.0
        self._dc_seeded = False
        self._filter_reset_needed = False

        # detector — I channel (MWI)
        self._sb_buf   = [0] * _SB_N
        self._sb_head  = 0
        self._spki     = 0
        self._npki     = 0
        self._thresh1  = 0
        # detector — F channel (bandpass)
        self._spkf     = 0
        self._npkf     = 0
        self._thresh_f1 = 0
        self._bpf_buf  = [0] * _BPF_BUF_N
        self._bpf_head = 0

        self._inited   = False
        self._refrac   = 0
        self._no_beat  = 0
        self._prev_mwi = 0
        self._rising   = False
        self._sb_qrs_back = 0

        # T-wave discriminator state — rolling |derivative| history
        self._drv_buf        = [0] * _DRV_BUF_N
        self._drv_buf_head   = 0
        self._last_qrs_slope = 0

        # Dual RR averages (paper eqs 24-29)
        self._rr_avg1     = [0] * _RR_AVG_N
        self._rr_avg2     = [0] * _RR_AVG_N
        self._rr1_cnt     = 0
        self._rr1_head    = 0
        self._rr2_cnt     = 0
        self._rr2_head    = 0
        self._rr_avg1_val = 0
        self._rr_avg2_val = 0
        self._regular     = False

        # bootstrap peak collection — paired MWI + BPF peaks so SPKF can be
        # seeded from real bandpass-channel amplitudes rather than a proxy.
        self._boot_cnt       = 0
        self._boot_max       = 0
        self._boot_peaks     = []   # list of (mwi_peak, bpf_peak) tuples
        self._boot_prev      = 0
        self._boot_was_rising = False

        # RR / arrhythmia (legacy median buffer — kept for classifier)
        self._rr       = [0] * 8
        self._rr_cnt   = 0
        self._rr_head  = 0
        self._last_qrs = 0
        self._idx      = 0

        self.bpm        = float('nan')
        self.arrhythmia = ARR_NONE
        self.qrs_count  = 0

    # ------------------------------------------------------------------ filters

    def _reset_filters(self):
        self._lpf_x    = [0] * _LPF_N
        self._lpf_y1   = self._lpf_y2 = 0
        self._lpf_head = 0
        self._hpf_x    = [0] * _HPF_N
        self._hpf_y    = 0
        self._hpf_head = 0
        self._drv_x    = [0] * 5
        self._drv_head = 0
        self._mwi_x    = [0] * _MWI_N
        self._mwi_sum  = 0
        self._mwi_head = 0

    def _lpf(self, x):
        self._lpf_x[self._lpf_head] = x
        i6  = (self._lpf_head + _LPF_N -  6) % _LPF_N
        i12 = (self._lpf_head + _LPF_N - 12) % _LPF_N
        y = ((self._lpf_y1 << 1) - self._lpf_y2
             + self._lpf_x[self._lpf_head]
             - (self._lpf_x[i6] << 1)
             + self._lpf_x[i12])
        self._lpf_y2   = self._lpf_y1
        self._lpf_y1   = y
        self._lpf_head = (self._lpf_head + 1) % _LPF_N
        return y >> 5

    def _hpf(self, x):
        self._hpf_x[self._hpf_head] = x
        i16 = (self._hpf_head + _HPF_N - 16) % _HPF_N
        i17 = (self._hpf_head + _HPF_N - 17) % _HPF_N
        i32 = (self._hpf_head + _HPF_N - 32) % _HPF_N
        self._hpf_y = (self._hpf_y
                       - (x >> 5)
                       + self._hpf_x[i16]
                       - self._hpf_x[i17]
                       + (self._hpf_x[i32] >> 5))
        self._hpf_head = (self._hpf_head + 1) % _HPF_N
        return self._hpf_y

    def _deriv(self, x):
        self._drv_x[self._drv_head] = x
        n1 = (self._drv_head + 4) % 5
        n3 = (self._drv_head + 2) % 5
        n4 = (self._drv_head + 1) % 5
        y = ((self._drv_x[self._drv_head] << 1) + self._drv_x[n1]
             - self._drv_x[n3] - (self._drv_x[n4] << 1)) >> 3
        self._drv_head = (self._drv_head + 1) % 5
        return y

    def _square(self, x):
        x = max(-3000, min(3000, x))
        return x * x

    def _mwi(self, x):
        self._mwi_sum -= self._mwi_x[self._mwi_head]
        self._mwi_x[self._mwi_head] = x
        self._mwi_sum += x
        self._mwi_head = (self._mwi_head + 1) % _MWI_N
        return self._mwi_sum // _MWI_N

    # ------------------------------------------------------------------ RR

    def _rr_push(self, ibi):
        self._rr[self._rr_head] = ibi
        self._rr_head = (self._rr_head + 1) % 8
        if self._rr_cnt < 8:
            self._rr_cnt += 1

    def _rr_median(self):
        if self._rr_cnt < 2:
            return 0
        n = self._rr_cnt
        tmp = sorted(self._rr[(self._rr_head - n + i + 8) % 8] for i in range(n))
        return tmp[n // 2]

    def _rr_avg_push(self, ibi):
        """Update RR_AVG1, RR_AVG2, and rhythm regularity per paper eqs 24-29."""
        self._rr_avg1[self._rr1_head] = ibi
        self._rr1_head = (self._rr1_head + 1) % _RR_AVG_N
        if self._rr1_cnt < _RR_AVG_N:
            self._rr1_cnt += 1
        self._rr_avg1_val = sum(self._rr_avg1[:self._rr1_cnt]) // self._rr1_cnt

        if self._rr_avg2_val == 0:
            self._rr_avg2_val = self._rr_avg1_val

        lo = (self._rr_avg2_val * 92) // 100
        hi = (self._rr_avg2_val * 116) // 100
        if lo <= ibi <= hi:
            self._rr_avg2[self._rr2_head] = ibi
            self._rr2_head = (self._rr2_head + 1) % _RR_AVG_N
            if self._rr2_cnt < _RR_AVG_N:
                self._rr2_cnt += 1
            self._rr_avg2_val = sum(self._rr_avg2[:self._rr2_cnt]) // self._rr2_cnt

        if self._rr1_cnt >= _RR_AVG_N:
            all_regular = all(lo <= v <= hi for v in self._rr_avg1)
            self._regular = all_regular
            if all_regular:
                self._rr_avg2_val = self._rr_avg1_val   # eq. 29

    def _bpf_window_peak(self):
        """Largest |bpf| in the recent window — F-channel peak for confirmation."""
        return max(abs(v) for v in self._bpf_buf)

    def _classify(self, bpm):
        if math.isnan(bpm):
            return ARR_NONE
        flags = ARR_NONE
        if bpm < _BRADY:
            flags |= ARR_BRADYCARDIA
        if bpm > _TACHY:
            flags |= ARR_TACHYCARDIA
        if self._rr_cnt >= 4:
            med = self._rr_median()
            if med > 0:
                thr = med * _IRREG_FRAC
                out = sum(1 for i in range(self._rr_cnt)
                          if abs(self._rr[(self._rr_head - self._rr_cnt + i + 8) % 8]
                                 - med) > thr)
                if out >= _IRREG_MIN:
                    flags |= ARR_IRREGULAR
        return flags

    # ------------------------------------------------------------------ detector

    def _detect(self, mwi_val, bpf_val, drv_abs):
        self._sb_buf[self._sb_head] = mwi_val
        self._sb_head = (self._sb_head + 1) % _SB_N
        self._bpf_buf[self._bpf_head] = bpf_val
        self._bpf_head = (self._bpf_head + 1) % _BPF_BUF_N
        self._drv_buf[self._drv_buf_head] = drv_abs
        self._drv_buf_head = (self._drv_buf_head + 1) % _DRV_BUF_N

        if not self._inited:
            if mwi_val > self._boot_max:
                self._boot_max = mwi_val
            if (self._boot_was_rising and mwi_val < self._boot_prev
                    and len(self._boot_peaks) < _BOOT_PEAKS):
                # Capture MWI peak with matching F-channel window peak
                self._boot_peaks.append((self._boot_prev, self._bpf_window_peak()))
            self._boot_was_rising = (mwi_val >= self._boot_prev)
            self._boot_prev = mwi_val

            self._boot_cnt += 1
            if self._boot_cnt >= _BOOT:
                if len(self._boot_peaks) >= 2:
                    pairs = sorted(self._boot_peaks, key=lambda p: p[0])
                    mid = len(pairs) // 2
                    self._spki = pairs[mid][0]
                    self._spkf = pairs[mid][1]
                else:
                    self._spki = max(self._boot_max >> 1, 1)
                    self._spkf = 0
                if self._spki < 1: self._spki = 1
                if self._spkf < 1: self._spkf = 1
                self._npki    = self._spki >> 2
                self._thresh1 = self._npki + ((self._spki - self._npki) >> 2)
                self._npkf     = self._spkf >> 2
                self._thresh_f1 = self._npkf + ((self._spkf - self._npkf) >> 2)
                self._inited  = True
                self._refrac  = _REFRAC
                self._prev_mwi = 0
                self._rising   = False
                self._filter_reset_needed = True
            return False

        if self._refrac > 0:
            self._refrac  -= 1
            self._prev_mwi = mwi_val
            return False

        # Effective thresholds. The paper's irregular-rhythm halving (eqs
        # 22-23) is disabled — its 92-116% regularity window flags normal
        # HR variability as irregular, halving lets in noise, and SPKI/SPKF
        # run away. We keep the regularity tracker (used for RR_AVG2-based
        # searchback) but no longer halve.
        thr_i_eff = self._thresh1
        thr_f_eff = self._thresh_f1

        qrs = False
        if mwi_val > self._prev_mwi:
            self._rising = True
        elif self._rising and mwi_val < self._prev_mwi:
            artifact = (self._spki > 0 and
                        self._prev_mwi > self._spki + (self._spki << 1))
            if not artifact:
                bpf_peak = self._bpf_window_peak()
                # Paper's "maximal slope during this waveform"
                cand_slope = max(self._drv_buf)
                mwi_ok = (self._prev_mwi >= thr_i_eff)
                bpf_ok = (bpf_peak       >= thr_f_eff)
                # See pan_tompkins.c for the long-form explanation: paper's
                # dual-confirmation and T-wave-slope rules don't translate
                # cleanly to our fixed-point cascade — the BPF threshold
                # tracker drifts and the post-filter R/T slope ratio sits
                # well above 0.5, so neither rule discriminates correctly.
                # Infrastructure is kept (SPKF / NPKF / slope buffer all
                # update normally) but the QRS decision uses the MWI
                # channel only.
                _ = (bpf_ok, cand_slope)

                if mwi_ok:
                    qrs = True
                    self._spki = self._spki + ((self._prev_mwi - self._spki) >> 3)
                    self._spkf = self._spkf + ((bpf_peak       - self._spkf) >> 3)
                    self._last_qrs_slope = cand_slope
                    self._refrac = _REFRAC
                else:
                    self._npki = self._npki + ((self._prev_mwi - self._npki) >> 3)
                    self._npkf = self._npkf + ((bpf_peak       - self._npkf) >> 3)
            self._rising = False

        self._prev_mwi = mwi_val
        self._thresh1   = self._npki + ((self._spki - self._npki) >> 2)
        self._thresh_f1 = self._npkf + ((self._spkf - self._npkf) >> 2)
        if self._thresh1   < 1: self._thresh1   = 1
        if self._thresh_f1 < 1: self._thresh_f1 = 1

        # Searchback — use the same robust median that the arrhythmia classifier
        # uses (rr_median), not rr_avg2_val.  rr_avg2_val only becomes reliable
        # after ~8 "regular" beats; before that it sits near the first IBI and
        # causes premature/wrong searchback windows.  The original 0.125 SPKI
        # update (not the paper's stronger 0.25 variant) is also restored here:
        # the stronger update causes SPKI to collapse when a sub-threshold noise
        # peak is found during searchback, which then admits a cascade of FPs.
        self._sb_qrs_back = 0
        sb_med = self._rr_median()
        if not qrs and sb_med > 0 and self._last_qrs > 0:
            rr_samp  = sb_med * _FS // 1000
            sb_limit = min((rr_samp * 5) // 3, _SB_N)
            if self._idx - self._last_qrs >= sb_limit:
                thr_i2 = self._thresh1   >> 1
                thr_f2 = self._thresh_f1 >> 1
                best, best_i = thr_i2, 0
                for i in range(sb_limit):
                    idx = (self._sb_head + _SB_N - sb_limit + i) % _SB_N
                    if self._sb_buf[idx] > best:
                        best   = self._sb_buf[idx]
                        best_i = i
                bpf_peak = self._bpf_window_peak()
                _ = thr_f2     # F-channel threshold tracked but not gating
                if best > thr_i2:
                    qrs = True
                    self._sb_qrs_back = sb_limit - 1 - best_i
                    self._spki = self._spki + ((best     - self._spki) >> 3)
                    self._spkf = self._spkf + ((bpf_peak - self._spkf) >> 3)
                    self._thresh1   = self._npki + ((self._spki - self._npki) >> 2)
                    self._thresh_f1 = self._npkf + ((self._spkf - self._npkf) >> 2)
                    self._refrac = _REFRAC

        if qrs:
            self._no_beat = 0
        else:
            self._no_beat += 1
            if self._no_beat >= _NO_BEAT_TO:
                self._spki >>= 1; self._npki >>= 1
                self._spkf >>= 1; self._npkf >>= 1
                self._no_beat = 0

        return qrs

    # ------------------------------------------------------------------ public

    def process(self, ir_raw: int) -> bool:
        self._idx += 1

        if not self._dc_seeded:
            self._ppg_dc    = float(ir_raw)
            self._dc_seeded = True
        else:
            self._ppg_dc += 0.005 * (float(ir_raw) - self._ppg_dc)

        ac = int(ir_raw - self._ppg_dc)

        if self._filter_reset_needed:
            self._filter_reset_needed = False
            self._reset_filters()

        s1 = self._lpf(ac)
        s2 = self._hpf(s1)              # bandpass output (F channel)
        s3 = self._deriv(s2)
        s4 = self._square(s3)
        s5 = self._mwi(s4)              # MWI output (I channel)
        s3_abs = -s3 if s3 < 0 else s3
        qrs = self._detect(s5, s2, s3_abs)

        if qrs:
            self.qrs_count += 1
            qrs_idx = self._idx - self._sb_qrs_back
            if self._last_qrs > 0:
                ibi_ms = (qrs_idx - self._last_qrs) * 1000 // _FS
                if 273 < ibi_ms < 1714:
                    self._rr_push(ibi_ms)
                    self._rr_avg_push(ibi_ms)
                    if self._rr_cnt >= 3:
                        med = self._rr_median()
                        new_bpm = 60000.0 / med if med > 0 else float('nan')
                        if not math.isnan(new_bpm):
                            if math.isnan(self.bpm):
                                self.bpm = new_bpm
                            else:
                                self.bpm += _EMA_ALPHA * (new_bpm - self.bpm)
                        self.arrhythmia = self._classify(self.bpm)
            self._last_qrs = qrs_idx

        return qrs

    def process_array(self, samples) -> list:
        beats = []
        for i, s in enumerate(samples):
            if self.process(int(s)):
                beats.append(i)
        return beats
