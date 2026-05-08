"""
ble_client.py — Async BLE client for the ECG Watch.

Connects to a device advertising as "ECG-Watch" using the NUS protocol.
Parses incoming notify lines: "H:<bpm>,A:<arrhythmia>,B:<battery>"
Sends commands via the RX characteristic.

Usage:
    Run via asyncio.  All public state is thread-safe via threading.Lock().
"""

import asyncio
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Callable

from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic

# Nordic UART Service UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify (watch→app)
NUS_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write  (app→watch)

DEVICE_NAME = "ECG-Watch"

# Arrhythmia flag bitmask (mirrors watch firmware)
ARR_NONE        = 0x00
ARR_BRADYCARDIA = 0x01
ARR_TACHYCARDIA = 0x02
ARR_IRREGULAR   = 0x04

ARR_LABELS = {
    ARR_BRADYCARDIA: "BRADY",
    ARR_TACHYCARDIA: "TACHY",
    ARR_IRREGULAR:   "IRREG",
}


@dataclass
class WatchData:
    bpm:        Optional[float] = None   # None = no signal
    arrhythmia: int             = ARR_NONE
    battery:    int             = 0
    timestamp:  float           = field(default_factory=time.time)


class BLEClient:
    """Manages connection to ECG-Watch in a background asyncio thread."""

    def __init__(self, on_data: Optional[Callable[[WatchData], None]] = None,
                 on_raw:  Optional[Callable[[list], None]] = None):
        self._on_data    = on_data
        self._on_raw     = on_raw
        self._lock       = threading.Lock()
        self._latest     = WatchData()
        self._connected  = False
        self._cmd_queue: asyncio.Queue = None  # type: ignore[assignment]
        self._loop: asyncio.AbstractEventLoop = None  # type: ignore[assignment]
        self._thread     = threading.Thread(target=self._run_loop, daemon=True)

    # ------------------------------------------------------------------
    # Public thread-safe API (call from tkinter / main thread)
    # ------------------------------------------------------------------

    def start(self):
        """Start the BLE background thread."""
        self._thread.start()

    def send_command(self, cmd: str):
        """Queue a command string for transmission (e.g. 'SHOCK', 'REC_START')."""
        if self._loop and self._cmd_queue:
            self._loop.call_soon_threadsafe(
                self._cmd_queue.put_nowait, cmd.rstrip("\n") + "\n"
            )

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    def latest(self) -> WatchData:
        with self._lock:
            return WatchData(
                bpm        = self._latest.bpm,
                arrhythmia = self._latest.arrhythmia,
                battery    = self._latest.battery,
                timestamp  = self._latest.timestamp,
            )

    # ------------------------------------------------------------------
    # Internal async logic
    # ------------------------------------------------------------------

    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._cmd_queue = asyncio.Queue()
        self._loop.run_until_complete(self._connect_loop())

    async def _connect_loop(self):
        """Scan → connect → reconnect on disconnect, forever."""
        while True:
            print(f"[BLE] Scanning for '{DEVICE_NAME}'...")
            device = None
            while device is None:
                device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=5.0)
                if device is None:
                    print("[BLE] Not found, retrying...")
                    await asyncio.sleep(2)

            print(f"[BLE] Found {device.address}, connecting...")
            try:
                async with BleakClient(device) as client:
                    with self._lock:
                        self._connected = True
                    print("[BLE] Connected")

                    await client.start_notify(NUS_TX_UUID, self._on_notify)

                    # Drain the command queue and forward to RX char
                    while client.is_connected:
                        try:
                            cmd = await asyncio.wait_for(
                                self._cmd_queue.get(), timeout=0.2
                            )
                            data = cmd.encode()
                            await client.write_gatt_char(NUS_RX_UUID, data,
                                                         response=False)
                            print(f"[BLE] TX: {cmd.strip()}")
                        except asyncio.TimeoutError:
                            pass

            except Exception as exc:
                print(f"[BLE] Disconnected / error: {exc}")
            finally:
                with self._lock:
                    self._connected = False
            await asyncio.sleep(2)

    def _on_notify(self, _char: BleakGATTCharacteristic, data: bytearray):
        """Parse incoming BLE notify line.

        Two formats:
          H:<bpm>,A:<arr>,B:<bat>   — live data frame
          RAW:v1,v2,v3,v4,v5        — raw IR samples during recording
        """
        try:
            line = data.decode().strip()

            if line.startswith("RAW:"):
                if self._on_raw:
                    vals = [int(v) for v in line[4:].split(",") if v]
                    self._on_raw(vals)
                return

            parts = {kv.split(":")[0]: kv.split(":")[1]
                     for kv in line.split(",") if ":" in kv}

            bpm_raw = int(parts.get("H", "255"))
            bpm     = None if bpm_raw == 255 else float(bpm_raw)
            arr     = int(parts.get("A", "0"))
            bat     = int(parts.get("B", "0"))

            wd = WatchData(bpm=bpm, arrhythmia=arr, battery=bat,
                           timestamp=time.time())
            with self._lock:
                self._latest = wd

            if self._on_data:
                self._on_data(wd)
        except Exception as exc:
            print(f"[BLE] Parse error: {exc}  raw={data!r}")


def arrhythmia_label(flags: int) -> str:
    """Return a display string for arrhythmia flags."""
    if flags == ARR_NONE:
        return "NORMAL"
    labels = [lbl for mask, lbl in ARR_LABELS.items() if flags & mask]
    return " + ".join(labels)
