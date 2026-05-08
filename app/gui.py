"""
gui.py — Tkinter + Matplotlib desktop GUI for the ECG Watch.

Features:
  • Live BPM chart (rolling 60-second window)
  • Current BPM and arrhythmia status display
  • Battery % indicator
  • BLE connection status
  • Start / Stop recording button
  • Timed recording (user-specified seconds)
  • Shock button (sends SHOCK command to watch)
  • Save to CSV (time_s, bpm, arrhythmia_flags, arrhythmia_label)
    Loadable in MATLAB: data = readtable('recording.csv')
"""

import csv
import os
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog
from typing import Optional

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

from ble_client import BLEClient, WatchData, arrhythmia_label, ARR_NONE
from playback import PlaybackWindow

CHART_WINDOW_S  = 60    # seconds of history shown in plot
UPDATE_INTERVAL = 250   # ms between plot refreshes


class ECGApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("ECG Watch Monitor")
        root.resizable(False, False)

        # --- BLE client ---
        self.ble = BLEClient(on_data=self._on_ble_data, on_raw=self._on_raw_data)
        self.ble.start()

        # --- Recording state ---
        self._recording      = False
        self._rec_start_time: Optional[float] = None
        self._rec_bpm:        list[float] = []
        self._rec_times:      list[float] = []
        self._rec_arrhythmia: list[int]   = []
        self._rec_raw:        list[int]   = []   # raw IR samples streamed from watch
        self._timed_duration: Optional[float] = None  # seconds; None = manual stop

        # --- Rolling chart data ---
        self._chart_times: list[float] = []
        self._chart_bpms:  list[float] = []

        # --- Build UI ---
        self._build_ui()

        # --- Animation ---
        self._anim = FuncAnimation(
            self._fig, self._update_plot,
            interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False
        )

        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        PAD = 8

        # Top status bar
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, padx=PAD, pady=(PAD, 0))

        self._lbl_ble = ttk.Label(status_frame, text="BLE: Disconnected",
                                  foreground="red")
        self._lbl_ble.pack(side=tk.LEFT, padx=4)

        self._lbl_bat = ttk.Label(status_frame, text="Bat: --%")
        self._lbl_bat.pack(side=tk.RIGHT, padx=4)

        # BPM + arrhythmia display
        info_frame = ttk.Frame(self.root)
        info_frame.pack(fill=tk.X, padx=PAD, pady=4)

        self._lbl_bpm = ttk.Label(info_frame, text="--",
                                  font=("Helvetica", 64, "bold"))
        self._lbl_bpm.pack(side=tk.LEFT, padx=12)

        bpm_sub = ttk.Frame(info_frame)
        bpm_sub.pack(side=tk.LEFT, padx=4)
        ttk.Label(bpm_sub, text="BPM", font=("Helvetica", 14)).pack(anchor=tk.W)
        self._lbl_arr = ttk.Label(bpm_sub, text="NORMAL",
                                  font=("Helvetica", 14, "bold"),
                                  foreground="green")
        self._lbl_arr.pack(anchor=tk.W)

        # Matplotlib chart
        self._fig = Figure(figsize=(7, 2.8), dpi=96)
        self._ax  = self._fig.add_subplot(111)
        self._ax.set_xlim(-CHART_WINDOW_S, 0)
        self._ax.set_ylim(30, 180)
        self._ax.set_xlabel("Time (s)", fontsize=9)
        self._ax.set_ylabel("BPM", fontsize=9)
        self._ax.set_title("Heart Rate (live)", fontsize=10)
        self._ax.axhline(60,  color="blue",   linestyle="--",
                         alpha=0.5, linewidth=0.8, label="60 BPM")
        self._ax.axhline(100, color="orange", linestyle="--",
                         alpha=0.5, linewidth=0.8, label="100 BPM")
        self._ax.legend(loc="upper right", fontsize=7)
        self._fig.tight_layout()

        self._line, = self._ax.plot([], [], "r-", linewidth=1.5)
        self._rec_patch = Rectangle(
            (0, 0), 0, 1,
            transform=self._ax.get_xaxis_transform(),
            alpha=0.12, color="red", visible=False
        )
        self._ax.add_patch(self._rec_patch)

        canvas = FigureCanvasTkAgg(self._fig, master=self.root)
        canvas.get_tk_widget().pack(padx=PAD, pady=4)
        self._canvas = canvas

        # Control buttons
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill=tk.X, padx=PAD, pady=(0, PAD))

        self._btn_rec = ttk.Button(btn_frame, text="Start Recording",
                                   command=self._toggle_record)
        self._btn_rec.pack(side=tk.LEFT, padx=4)

        ttk.Button(btn_frame, text="Timed Record",
                   command=self._start_timed_record).pack(side=tk.LEFT, padx=4)

        ttk.Button(btn_frame, text="Save .hea",
                   command=self._save_wfdb).pack(side=tk.LEFT, padx=4)

        ttk.Button(btn_frame, text="Playback / Validate",
                   command=self._open_playback).pack(side=tk.LEFT, padx=4)

        self._btn_shock = ttk.Button(btn_frame, text="⚡ SHOCK",
                                     command=self._send_shock,
                                     style="Shock.TButton")
        self._btn_shock.pack(side=tk.RIGHT, padx=4)

        # Shock button style
        style = ttk.Style()
        style.configure("Shock.TButton", foreground="red",
                         font=("Helvetica", 10, "bold"))

        # Recording status bar
        self._lbl_rec_status = ttk.Label(self.root, text="", foreground="gray")
        self._lbl_rec_status.pack(pady=(0, 4))

    # ------------------------------------------------------------------
    # BLE callback (called from BLE thread)
    # ------------------------------------------------------------------

    def _on_ble_data(self, wd: WatchData):
        now = time.time()
        if wd.bpm is not None:
            self._chart_times.append(now)
            self._chart_bpms.append(wd.bpm)
            # Trim to window
            cutoff = now - CHART_WINDOW_S - 5
            while self._chart_times and self._chart_times[0] < cutoff:
                self._chart_times.pop(0)
                self._chart_bpms.pop(0)

            # Record
            if self._recording:
                self._rec_times.append(now)
                self._rec_bpm.append(wd.bpm)
                self._rec_arrhythmia.append(wd.arrhythmia)

        # Schedule GUI updates on the main thread
        self.root.after_idle(self._refresh_labels, wd)

        # Auto-stop timed recording
        if self._recording and self._timed_duration is not None:
            elapsed = now - (self._rec_start_time or now)
            if elapsed >= self._timed_duration:
                self.root.after_idle(self._stop_recording)

    # ------------------------------------------------------------------
    # GUI updates (main thread)
    # ------------------------------------------------------------------

    def _refresh_labels(self, wd: WatchData):
        # BPM
        if wd.bpm is None:
            self._lbl_bpm.config(text="--")
        else:
            self._lbl_bpm.config(text=f"{int(round(wd.bpm))}")

        # Arrhythmia
        label = arrhythmia_label(wd.arrhythmia)
        color = "green" if wd.arrhythmia == ARR_NONE else "red"
        self._lbl_arr.config(text=label, foreground=color)

        # Battery
        self._lbl_bat.config(text=f"Bat: {wd.battery}%")

        # BLE status
        if self.ble.connected:
            self._lbl_ble.config(text="BLE: Connected", foreground="green")
        else:
            self._lbl_ble.config(text="BLE: Disconnected", foreground="red")

        # Recording status
        if self._recording and self._rec_start_time:
            elapsed = time.time() - self._rec_start_time
            n = len(self._rec_bpm)
            if self._timed_duration:
                remaining = max(0, self._timed_duration - elapsed)
                self._lbl_rec_status.config(
                    text=f"● Recording — {n} samples — {remaining:.0f}s remaining",
                    foreground="red"
                )
            else:
                self._lbl_rec_status.config(
                    text=f"● Recording — {n} samples — {elapsed:.0f}s elapsed",
                    foreground="red"
                )

    def _update_plot(self, _frame):
        """Called by FuncAnimation every UPDATE_INTERVAL ms."""
        if not self._chart_times:
            return

        now   = time.time()
        times = np.array(self._chart_times) - now   # relative seconds (≤0)
        bpms  = np.array(self._chart_bpms)

        self._line.set_data(times, bpms)

        # Shade the recording region
        if self._recording and self._rec_start_time:
            rec_start_rel = max(self._rec_start_time - now, -CHART_WINDOW_S)
            self._rec_patch.set_x(rec_start_rel)
            self._rec_patch.set_width(-rec_start_rel)
            self._rec_patch.set_visible(True)
        else:
            self._rec_patch.set_visible(False)

        # Dynamic y-axis
        if len(bpms) > 1:
            lo = max(30,  float(bpms.min()) - 10)
            hi = min(220, float(bpms.max()) + 10)
            self._ax.set_ylim(lo, hi)

    # ------------------------------------------------------------------
    # Recording controls
    # ------------------------------------------------------------------

    def _toggle_record(self):
        if self._recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _on_raw_data(self, samples: list):
        """Called from BLE thread with a batch of raw IR samples."""
        if self._recording:
            self._rec_raw.extend(samples)

    def _start_recording(self, duration: Optional[float] = None):
        self._rec_bpm.clear()
        self._rec_times.clear()
        self._rec_arrhythmia.clear()
        self._rec_raw.clear()
        self._rec_start_time  = time.time()
        self._timed_duration  = duration
        self._recording       = True
        self._btn_rec.config(text="Stop Recording")
        self.ble.send_command("REC_START")
        print(f"[APP] Recording started (duration={duration}s)")

    def _stop_recording(self):
        self._recording = False
        self._btn_rec.config(text="Start Recording")
        self._lbl_rec_status.config(
            text=f"Recording stopped — {len(self._rec_bpm)} samples saved",
            foreground="gray"
        )
        self.ble.send_command("REC_STOP")
        elapsed = time.time() - (self._rec_start_time or time.time())
        print(f"[APP] Recording stopped: {len(self._rec_bpm)} samples, "
              f"{elapsed:.1f}s")

    def _start_timed_record(self):
        answer = simpledialog.askinteger(
            "Timed Recording",
            "Record for how many seconds?",
            minvalue=1, maxvalue=3600, initialvalue=30
        )
        if answer:
            self._start_recording(duration=float(answer))

    # ------------------------------------------------------------------
    # Save as WFDB (.hea + .dat) — same format as test data
    # ------------------------------------------------------------------

    def _save_wfdb(self):
        if not self._rec_raw and not self._rec_bpm:
            messagebox.showwarning("No Data",
                                   "No recording data to save.\n"
                                   "Start a recording first.")
            return

        path = filedialog.asksaveasfilename(
            defaultextension=".hea",
            filetypes=[("WFDB header", "*.hea"), ("All files", "*.*")],
            initialfile="ecg_watch_recording.hea"
        )
        if not path:
            return

        import numpy as np
        try:
            import wfdb
        except ImportError:
            messagebox.showerror("Missing dependency", "pip install wfdb")
            return

        rec_name = os.path.splitext(os.path.basename(path))[0]
        rec_dir  = os.path.dirname(path) or "."

        if self._rec_raw:
            # Full raw IR signal — loadable in Playback for algorithm validation
            sig = np.array(self._rec_raw, dtype=np.float64).reshape(-1, 1)
            wfdb.wrsamp(rec_name, fs=200, units=["ADC"], sig_name=["IR"],
                        p_signal=sig, fmt=["32"], write_dir=rec_dir)
            detail = f"{len(self._rec_raw)} raw IR samples @ 200 Hz"
        else:
            # No raw stream received — save BPM trace as signal (fallback)
            t0  = self._rec_times[0] if self._rec_times else 0.0
            bpm = np.array(self._rec_bpm, dtype=np.float64).reshape(-1, 1)
            # Compute effective sample rate from timestamps
            if len(self._rec_times) > 1:
                dt = (self._rec_times[-1] - self._rec_times[0]) / (len(self._rec_times) - 1)
                fs = max(1, round(1.0 / dt))
            else:
                fs = 2
            wfdb.wrsamp(rec_name, fs=fs, units=["BPM"], sig_name=["BPM"],
                        p_signal=bpm, fmt=["16"], write_dir=rec_dir)
            detail = f"{len(self._rec_bpm)} BPM samples (no raw signal — ensure BLE connected during recording)"

        messagebox.showinfo("Saved",
                            f"Saved to:\n{rec_dir}/{rec_name}.hea\n\n{detail}")
        print(f"[APP] Saved WFDB → {rec_dir}/{rec_name}.hea")

    # ------------------------------------------------------------------
    # Shock
    # ------------------------------------------------------------------

    def _open_playback(self):
        PlaybackWindow(self.root, ble_client=self.ble)

    def _send_shock(self):
        if not self.ble.connected:
            messagebox.showwarning("Not Connected",
                                   "Watch is not connected via BLE.")
            return
        self.ble.send_command("SHOCK")
        print("[APP] SHOCK command sent")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def _on_close(self):
        if self._recording:
            self._stop_recording()
        self.root.destroy()
