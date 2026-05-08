#!/usr/bin/env python3
"""
ECG Watch Desktop App
=====================
Connects to the ECG Watch via BLE (NUS protocol) and displays:
  - Live heart-rate graph (60-second rolling window)
  - Current BPM and arrhythmia classification
  - Battery level
  - Recording controls (manual / timed)
  - Save to .mat for MATLAB analysis
  - Shock button

Requirements:
    pip install bleak matplotlib scipy numpy

Usage:
    python main.py
"""

import tkinter as tk
from gui import ECGApp


def main():
    root = tk.Tk()
    root.minsize(680, 480)
    app = ECGApp(root)   # noqa: F841 — held alive by root mainloop
    root.mainloop()


if __name__ == "__main__":
    main()
