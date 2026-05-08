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

        # detector
        self._sb_buf   = [0] * _SB_N
        self._sb_head  = 0
        self._spki     = 0
        self._npki     = 0
        self._thresh1  = 0
        self._inited   = False
        self._refrac   = 0
        self._no_beat  = 0
        self._prev_mwi = 0
        self._rising   = False
        self._sb_qrs_back = 0  # searchback offset; non-zero only when searchback fires

        # bootstrap peak collection
        self._boot_cnt       = 0
        self._boot_max       = 0
        self._boot_peaks     = []
        self._boot_prev      = 0
        self._boot_was_rising = False

        # RR / arrhythmia
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

    def _detect(self, mwi_val):
        self._sb_buf[self._sb_head] = mwi_val
        self._sb_head = (self._sb_head + 1) % _SB_N

        if not self._inited:
            if mwi_val > self._boot_max:
                self._boot_max = mwi_val

            # Collect local maxima
            if (self._boot_was_rising and mwi_val < self._boot_prev
                    and len(self._boot_peaks) < _BOOT_PEAKS):
                self._boot_peaks.append(self._boot_prev)
            self._boot_was_rising = (mwi_val >= self._boot_prev)
            self._boot_prev = mwi_val

            self._boot_cnt += 1
            if self._boot_cnt >= _BOOT:
                if len(self._boot_peaks) >= 2:
                    peaks = sorted(self._boot_peaks)
                    self._spki = peaks[len(peaks) // 2]   # median
                else:
                    self._spki = max(self._boot_max >> 1, 1)
                if self._spki < 1:
                    self._spki = 1
                self._npki    = self._spki >> 2
                self._thresh1 = self._npki + ((self._spki - self._npki) >> 2)
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

        qrs = False

        if mwi_val > self._prev_mwi:
            self._rising = True
        elif self._rising and mwi_val < self._prev_mwi:
            artifact = (self._spki > 0 and
                        self._prev_mwi > self._spki + (self._spki << 1))
            if not artifact:
                if self._prev_mwi >= self._thresh1:
                    qrs = True
                    self._spki  = self._spki + ((self._prev_mwi - self._spki) >> 3)
                    self._refrac = _REFRAC
                else:
                    self._npki = self._npki + ((self._prev_mwi - self._npki) >> 3)
            self._rising = False

        self._prev_mwi = mwi_val
        self._thresh1  = self._npki + ((self._spki - self._npki) >> 2)
        if self._thresh1 < 1:
            self._thresh1 = 1

        # Searchback
        self._sb_qrs_back = 0
        if not qrs and self._rr_cnt >= 2 and self._last_qrs > 0:
            rr_ms = self._rr_median()
            if rr_ms > 0:
                rr_samp  = rr_ms * _FS // 1000
                sb_limit = min((rr_samp * 5) // 3, _SB_N)
                if self._idx - self._last_qrs >= sb_limit:
                    thr2   = self._thresh1 >> 1
                    best   = thr2
                    best_i = 0
                    for i in range(sb_limit):
                        idx = (self._sb_head + _SB_N - sb_limit + i) % _SB_N
                        if self._sb_buf[idx] > best:
                            best   = self._sb_buf[idx]
                            best_i = i
                    if best > thr2:
                        qrs = True
                        # i=0 is oldest; i=sb_limit-1 is most recent (1 sample ago)
                        self._sb_qrs_back = sb_limit - 1 - best_i
                        self._spki   = self._spki + ((best - self._spki) >> 3)
                        self._thresh1 = self._npki + ((self._spki - self._npki) >> 2)
                        self._refrac  = _REFRAC

        # No-beat watchdog — halve both to keep threshold ratio intact
        if qrs:
            self._no_beat = 0
        else:
            self._no_beat += 1
            if self._no_beat >= _NO_BEAT_TO:
                self._spki   >>= 1
                self._npki   >>= 1
                self._no_beat  = 0

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

        s = self._mwi(self._square(self._deriv(self._hpf(self._lpf(ac)))))
        qrs = self._detect(s)

        if qrs:
            self.qrs_count += 1
            qrs_idx = self._idx - self._sb_qrs_back
            if self._last_qrs > 0:
                ibi_ms = (qrs_idx - self._last_qrs) * 1000 // _FS
                if 273 < ibi_ms < 1714:
                    self._rr_push(ibi_ms)
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
