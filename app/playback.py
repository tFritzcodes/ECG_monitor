"""
playback.py — Offline .mat / WFDB playback window for ECG Watch app.

Supports:
  • MIT-BIH WFDB records (.hea file → loads signal + beat annotations)
  • Raw sample .mat files with key 'ir_raw' or 'signal' (+ optional 'fs')
  • Our own recording .mat files (key 'bpm' — shows BPM trace, no re-detection)

Runs the Pan-Tompkins algorithm locally in Python (exact port of firmware C code).
Optionally streams raw samples to the watch via BLE FEED:<value> commands.
"""

import csv
import os, math, threading, time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.io import loadmat
from scipy.signal import resample_poly
from math import gcd

from pan_tompkins import PanTompkins, ARR_NONE, ARR_BRADYCARDIA, ARR_TACHYCARDIA, ARR_IRREGULAR

TARGET_FS      = 200
PIPELINE_DELAY = 39   # samples; compensate for filter group delay in plots

BEAT_TYPES = set('NLRBAbejAaJSVrFYWfQe')

# BLE can't sustain 200 individual writes/sec (min connection interval ~7.5ms).
# Batch BATCH_SIZE samples per write, send every BATCH_INTERVAL seconds.
# BATCH_SIZE / BATCH_INTERVAL = TARGET_FS (200 Hz effective sample rate).
BATCH_SIZE     = 5
BATCH_INTERVAL = BATCH_SIZE / TARGET_FS   # 25 ms


def _arr_label(flags):
    parts = []
    if flags & ARR_BRADYCARDIA: parts.append("BRADY")
    if flags & ARR_TACHYCARDIA: parts.append("TACHY")
    if flags & ARR_IRREGULAR:   parts.append("IRREG")
    return " | ".join(parts) if parts else "NORMAL"


class PlaybackWindow:
    """Toplevel window that runs PT algorithm on loaded data and shows results."""

    def __init__(self, parent: tk.Tk, ble_client=None):
        self._ble   = ble_client   # optional BLEClient for FEED streaming
        self._win   = tk.Toplevel(parent)
        self._win.title("Playback / Validation")
        self._win.resizable(True, True)
        self._win.minsize(700, 500)

        self._signal    = None   # int32 array at TARGET_FS
        self._ann_samps = None   # beat annotation sample indices (optional)
        self._detected  = None   # detected QRS sample indices
        self._bpm_trace = None   # (sample_idx, bpm) pairs
        self._fs        = TARGET_FS
        self._rec_name  = ""

        self._streaming = False
        self._stop_flag = threading.Event()

        self._build_ui()

    def _build_ui(self):
        PAD = 8

        # ── toolbar ──────────────────────────────────────────────────────
        bar = ttk.Frame(self._win)
        bar.pack(fill=tk.X, padx=PAD, pady=PAD)

        ttk.Button(bar, text="Load WFDB (.hea)",
                   command=self._load_wfdb).pack(side=tk.LEFT, padx=4)
        ttk.Button(bar, text="Run Algorithm",
                   command=self._run_algorithm).pack(side=tk.LEFT, padx=4)

        self._btn_stream = ttk.Button(bar, text="▶ Stream to Watch",
                                      command=self._toggle_stream,
                                      state=tk.DISABLED)
        self._btn_stream.pack(side=tk.LEFT, padx=4)

        # ── stats bar ─────────────────────────────────────────────────────
        self._lbl_stats = ttk.Label(self._win, text="No file loaded.",
                                    font=("Helvetica", 10))
        self._lbl_stats.pack(fill=tk.X, padx=PAD)

        # ── figure ────────────────────────────────────────────────────────
        self._fig = Figure(figsize=(8, 4), dpi=96)
        self._ax_bpm = self._fig.add_subplot(111)
        self._ax_bpm.set_xlabel("Time (s)")
        self._ax_bpm.set_ylabel("BPM")
        self._ax_bpm.set_title("BPM trace — no data")
        self._fig.tight_layout()

        canvas = FigureCanvasTkAgg(self._fig, master=self._win)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True,
                                    padx=PAD, pady=(0, PAD))
        self._canvas = canvas

        # ── progress bar ──────────────────────────────────────────────────
        self._progress = ttk.Progressbar(self._win, mode="determinate",
                                          maximum=100)
        self._progress.pack(fill=tk.X, padx=PAD, pady=(0, PAD))

    # ── loaders ──────────────────────────────────────────────────────────

    def _load_wfdb(self):
        path = filedialog.askopenfilename(
            title="Open WFDB header file",
            filetypes=[("WFDB header", "*.hea"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            import wfdb
            base = os.path.splitext(path)[0]
            rec  = wfdb.rdrecord(base)
            self._rec_name = os.path.basename(base)

            sig_orig = rec.p_signal[:, 0]
            orig_fs  = rec.fs
            g        = gcd(TARGET_FS, orig_fs)
            sig      = resample_poly(sig_orig, TARGET_FS // g, orig_fs // g)
            self._signal = (sig * 1000).astype(np.int32)
            self._fs     = TARGET_FS

            # Load annotations if present
            try:
                ann = wfdb.rdann(base, 'atr')
                all_samps  = np.round(np.array(ann.sample) * TARGET_FS / orig_fs).astype(int)
                beat_mask  = [s in BEAT_TYPES for s in ann.symbol]
                self._ann_samps = all_samps[beat_mask]
            except Exception:
                self._ann_samps = None

            n_ann = len(self._ann_samps) if self._ann_samps is not None else 0
            self._lbl_stats.config(
                text=f"Loaded WFDB '{self._rec_name}'  "
                     f"{len(self._signal)/TARGET_FS:.1f}s  "
                     f"{n_ann} annotated beats  — press Run Algorithm"
            )
            self._detected = None
            self._bpm_trace = None
        except Exception as exc:
            messagebox.showerror("Load error", str(exc))

    def _load_mat(self):
        path = filedialog.askopenfilename(
            title="Open .mat file",
            filetypes=[("MATLAB files", "*.mat"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            data = loadmat(path)
            self._rec_name = os.path.basename(path)
            self._ann_samps = None

            if 'ir_raw' in data:
                sig = np.array(data['ir_raw']).flatten().astype(np.int32)
                fs  = int(np.array(data.get('fs', [[TARGET_FS]])).flatten()[0])
            elif 'signal' in data:
                sig = np.array(data['signal']).flatten()
                fs  = int(np.array(data.get('fs', [[TARGET_FS]])).flatten()[0])
                sig = (sig * 1000).astype(np.int32)
            elif 'bpm' in data:
                # Our own recording format — no raw samples, just show BPM trace
                bpm  = np.array(data['bpm']).flatten()
                t    = np.array(data.get('time', np.arange(len(bpm)) * 0.5)).flatten()
                self._signal    = None
                self._bpm_trace = list(zip(t * TARGET_FS, bpm))
                self._detected  = []
                self._lbl_stats.config(
                    text=f"Loaded BPM recording '{self._rec_name}'  "
                         f"{len(bpm)} samples  "
                         f"BPM range {bpm.min():.0f}–{bpm.max():.0f}"
                )
                self._plot_results()
                return
            else:
                messagebox.showerror("Load error",
                                     "Expected key 'ir_raw', 'signal', or 'bpm'.")
                return

            # Resample to TARGET_FS if needed
            if fs != TARGET_FS:
                g   = gcd(TARGET_FS, fs)
                sig = resample_poly(sig.astype(float),
                                    TARGET_FS // g, fs // g).astype(np.int32)

            self._signal = sig
            self._fs     = TARGET_FS
            self._lbl_stats.config(
                text=f"Loaded '{self._rec_name}'  "
                     f"{len(sig)/TARGET_FS:.1f}s  — press Run Algorithm"
            )
            self._detected  = None
            self._bpm_trace = None
        except Exception as exc:
            messagebox.showerror("Load error", str(exc))

    def _load_csv(self):
        """Load a CSV recording saved by the desktop app (time_s, bpm columns)."""
        path = filedialog.askopenfilename(
            title="Open CSV recording",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            with open(path, newline="") as f:
                rows = list(csv.DictReader(f))
            if not rows or "bpm" not in rows[0]:
                messagebox.showerror("Load error",
                                     "Expected columns: time_s, bpm")
                return
            t   = [float(r["time_s"]) for r in rows]
            bpm = [float(r["bpm"])    for r in rows]
            self._rec_name   = os.path.basename(path)
            self._signal     = None
            self._ann_samps  = None
            self._bpm_trace  = [(ti * TARGET_FS, b) for ti, b in zip(t, bpm)]
            self._detected   = []
            self._lbl_stats.config(
                text=f"Loaded CSV '{self._rec_name}'  "
                     f"{len(bpm)} samples  "
                     f"BPM range {min(bpm):.0f}–{max(bpm):.0f}"
            )
            self._plot_results()
        except Exception as exc:
            messagebox.showerror("Load error", str(exc))

    # ── algorithm ────────────────────────────────────────────────────────

    def _run_algorithm(self):
        if self._signal is None:
            messagebox.showwarning("No data", "Load a file first.")
            return

        n = len(self._signal)
        self._progress["value"] = 0

        pt = PanTompkins()
        detected  = []
        bpm_trace = []   # (sample_idx, bpm) — only when bpm is valid
        # Track arrhythmia flag counts across 5-second windows
        arr_counts = {ARR_BRADYCARDIA: 0, ARR_TACHYCARDIA: 0, ARR_IRREGULAR: 0}
        arr_windows = 0
        SNAP_INTERVAL = TARGET_FS * 5   # snapshot every 5 s
        CHUNK = 2000

        for start in range(0, n, CHUNK):
            chunk = self._signal[start:start + CHUNK]
            for i, s in enumerate(chunk):
                idx = start + i
                if pt.process(int(s)):
                    detected.append(idx)
                if not math.isnan(pt.bpm):
                    bpm_trace.append((idx, pt.bpm))
                if idx % SNAP_INTERVAL == 0 and not math.isnan(pt.bpm):
                    arr_windows += 1
                    for flag in (ARR_BRADYCARDIA, ARR_TACHYCARDIA, ARR_IRREGULAR):
                        if pt.arrhythmia & flag:
                            arr_counts[flag] += 1
            self._progress["value"] = int(100 * (start + len(chunk)) / n)
            self._win.update_idletasks()

        self._detected  = detected
        self._bpm_trace = bpm_trace

        # Score against annotations if available
        BOOT_SAMPS = 400
        WINDOW     = 30
        stats_txt  = ""
        if self._ann_samps is not None:
            eval_ann = self._ann_samps[self._ann_samps >= BOOT_SAMPS]
            det_comp = np.array(detected) - PIPELINE_DELAY
            matched  = set()
            tp = fp  = 0
            for d in det_comp:
                hit = False
                for j, a in enumerate(eval_ann):
                    if j not in matched and abs(d - a) <= WINDOW:
                        tp += 1; matched.add(j); hit = True; break
                if not hit:
                    fp += 1
            fn  = len(eval_ann) - len(matched)
            Se  = 100.0 * tp / (tp + fn)  if (tp + fn)  > 0 else 0.0
            PPV = 100.0 * tp / (tp + fp)  if (tp + fp)  > 0 else 0.0
            stats_txt = (f"  Se={Se:.1f}%  PPV={PPV:.1f}%  "
                         f"TP={tp} FP={fp} FN={fn}")

        # Build arrhythmia summary showing % of record each flag was active
        arr_parts = []
        if arr_windows > 0:
            for flag, label in ((ARR_BRADYCARDIA, "BRADY"),
                                (ARR_TACHYCARDIA, "TACHY"),
                                (ARR_IRREGULAR,   "IRREG")):
                pct = 100 * arr_counts[flag] // arr_windows
                if pct > 0:
                    arr_parts.append(f"{label} {pct}%")
        arr_str = "  [" + (", ".join(arr_parts) if arr_parts else "NORMAL") + "]"

        bpm_str = f"{pt.bpm:.1f}" if not math.isnan(pt.bpm) else "N/A"
        self._lbl_stats.config(
            text=(f"'{self._rec_name}'  detected {len(detected)} beats  "
                  f"final BPM={bpm_str}" + arr_str + stats_txt)
        )
        self._plot_results()

        if self._ble:
            self._btn_stream.config(state=tk.NORMAL)

    def _plot_results(self):
        self._ax_bpm.clear()

        if self._bpm_trace:
            xs = [s / TARGET_FS for s, _ in self._bpm_trace]
            ys = [b for _, b in self._bpm_trace]
            self._ax_bpm.plot(xs, ys, "r-", linewidth=1.2, label="BPM")

        if self._detected:
            det_t = [(s - PIPELINE_DELAY) / TARGET_FS for s in self._detected]
            if self._bpm_trace:
                beat_bpms = []
                bpm_arr = np.array([b for _, b in self._bpm_trace])
                samp_arr = np.array([s for s, _ in self._bpm_trace])
                for d in self._detected:
                    idx = np.searchsorted(samp_arr, d - PIPELINE_DELAY)
                    idx = min(idx, len(bpm_arr) - 1)
                    beat_bpms.append(bpm_arr[idx])
                self._ax_bpm.scatter(det_t, beat_bpms, s=12,
                                     color="green", zorder=5, label="QRS")
            else:
                yref = 75
                self._ax_bpm.vlines(det_t, yref - 10, yref + 10,
                                    colors="green", linewidth=0.5)

        if self._ann_samps is not None:
            ann_t = self._ann_samps / TARGET_FS
            self._ax_bpm.vlines(ann_t, 0, 1,
                                 transform=self._ax_bpm.get_xaxis_transform(),
                                 colors="blue", linewidth=0.4, alpha=0.4,
                                 label="Annotation")

        self._ax_bpm.axhline(60,  color="blue",   linestyle="--",
                              alpha=0.4, linewidth=0.8)
        self._ax_bpm.axhline(100, color="orange", linestyle="--",
                              alpha=0.4, linewidth=0.8)
        self._ax_bpm.set_xlabel("Time (s)")
        self._ax_bpm.set_ylabel("BPM")
        self._ax_bpm.set_title(f"Playback: {self._rec_name}")
        if self._bpm_trace or self._detected:
            self._ax_bpm.legend(fontsize=8, loc="upper right")
        self._fig.tight_layout()
        self._canvas.draw()

    # ── BLE streaming ────────────────────────────────────────────────────

    def _toggle_stream(self):
        if self._streaming:
            self._stop_flag.set()
            self._streaming = False
            self._btn_stream.config(text="▶ Stream to Watch")
        else:
            if self._signal is None:
                messagebox.showwarning("No data", "Run algorithm first.")
                return
            self._stop_flag.clear()
            self._streaming = True
            self._btn_stream.config(text="■ Stop Streaming")
            t = threading.Thread(target=self._stream_worker, daemon=True)
            t.start()

    def _stream_worker(self):
        """
        Send raw samples to the watch via batched FEED BLE commands.

        Sends BATCH_SIZE samples per write every BATCH_INTERVAL seconds,
        giving an effective 200 Hz sample rate without exceeding BLE bandwidth.
        FEED_START puts the firmware into feed mode (ignores real sensor).
        FEED_STOP resumes normal sensor operation.
        """
        self._ble.send_command("FEED_START")
        time.sleep(0.1)   # let firmware process FEED_START before first batch

        samples = self._signal
        n = len(samples)
        i = 0
        while i < n and not self._stop_flag.is_set():
            batch = samples[i:i + BATCH_SIZE]
            payload = "FEED:" + ",".join(str(int(v)) for v in batch)
            self._ble.send_command(payload)
            i += BATCH_SIZE

            # Update progress bar from main thread
            pct = int(100 * i / n)
            self._win.after_idle(
                lambda p=pct: self._progress.configure(value=p)
            )
            time.sleep(BATCH_INTERVAL)

        self._ble.send_command("FEED_STOP")
        self._streaming = False
        self._win.after_idle(lambda: (
            self._btn_stream.config(text="▶ Stream to Watch"),
            self._progress.configure(value=0),
        ))
