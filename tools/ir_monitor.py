#!/usr/bin/env python3
"""
IR sensor live monitor — connects to ESP32-S3 over USB serial,
parses lines like: "L  1064 ( 619/1841/3909) LF ..."
or simpler:        "L:1064 LF:1359 RF:1172 R:1153"
and shows live bars + rolling history.

Usage:
    pip install pyserial
    python tools/ir_monitor.py            # auto-pick port
    python tools/ir_monitor.py /dev/cu.usbmodem1101
"""

import re
import sys
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk

import serial
import serial.tools.list_ports

BAUD       = 115200
ADC_MAX    = 4095
HIST_LEN   = 200
SENSORS    = ["L", "LF", "RF", "R"]
COLORS     = {"L": "#1f77b4", "LF": "#2ca02c", "RF": "#ff7f0e", "R": "#d62728"}

# Matches "L  1064" / "L:1064" / "L 1064"
PAT = re.compile(r"(L|LF|RF|R)[\s:]+(-?\d+)")


def pick_port(arg=None):
    if arg:
        return arg
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if any(k in (p.device + (p.description or "")).lower()
               for k in ["usbmodem", "wchusb", "slab", "cp210", "ch340", "esp"]):
            return p.device
    if ports:
        return ports[0].device
    sys.exit("no serial ports found")


class IRMonitor(tk.Tk):
    def __init__(self, port):
        super().__init__()
        self.title(f"IR Monitor — {port}")
        self.geometry("760x520")
        self.configure(bg="#111")

        self.port = port
        self.ser = None
        self.values = {s: 0 for s in SENSORS}
        self.peak   = {s: 0 for s in SENSORS}
        self.hist   = {s: deque([0]*HIST_LEN, maxlen=HIST_LEN) for s in SENSORS}
        self.running = True

        self._build_ui()
        self._connect()
        threading.Thread(target=self._reader, daemon=True).start()
        self.after(50, self._tick)

    def _build_ui(self):
        top = tk.Frame(self, bg="#111"); top.pack(fill="x", padx=10, pady=6)
        self.status = tk.Label(top, text="…", fg="#9f9", bg="#111", font=("Menlo", 11))
        self.status.pack(side="left")
        tk.Button(top, text="Reset peaks", command=self._reset_peaks,
                  bg="#222", fg="white", relief="flat").pack(side="right")

        bars = tk.Frame(self, bg="#111"); bars.pack(fill="x", padx=10, pady=10)
        self.bar_rows = {}
        for s in SENSORS:
            row = tk.Frame(bars, bg="#111"); row.pack(fill="x", pady=4)
            tk.Label(row, text=s, width=3, font=("Menlo", 14, "bold"),
                     fg=COLORS[s], bg="#111").pack(side="left")
            canv = tk.Canvas(row, height=28, bg="#222", highlightthickness=0, width=560)
            canv.pack(side="left", padx=8, fill="x", expand=True)
            rect = canv.create_rectangle(0, 0, 0, 28, fill=COLORS[s], width=0)
            peak_line = canv.create_line(0, 0, 0, 28, fill="white", width=2)
            val_lbl = tk.Label(row, text="0", width=6, font=("Menlo", 13),
                               fg="white", bg="#111", anchor="e")
            val_lbl.pack(side="left")
            peak_lbl = tk.Label(row, text="pk:0", width=8, font=("Menlo", 11),
                                fg="#aaa", bg="#111", anchor="w")
            peak_lbl.pack(side="left", padx=4)
            self.bar_rows[s] = (canv, rect, peak_line, val_lbl, peak_lbl)

        self.plot = tk.Canvas(self, bg="#0a0a0a", height=200,
                              highlightthickness=0)
        self.plot.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    def _connect(self):
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0.2)
            self.status.config(text=f"connected {self.port} @ {BAUD}", fg="#9f9")
        except Exception as e:
            self.status.config(text=f"open failed: {e}", fg="#f66")

    def _reader(self):
        buf = b""
        while self.running:
            if not self.ser:
                time.sleep(0.5); self._connect(); continue
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                self.status.config(text=f"serial error: {e}", fg="#f66")
                self.ser = None; continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    self._parse(line.decode("utf-8", errors="ignore"))
                except Exception:
                    pass

    def _parse(self, line):
        hits = PAT.findall(line)
        if not hits:
            return
        seen = {}
        for k, v in hits:
            if k not in seen:
                seen[k] = int(v)
        if not any(s in seen for s in SENSORS):
            return
        for s in SENSORS:
            if s in seen:
                v = max(0, min(ADC_MAX, seen[s]))
                self.values[s] = v
                if v > self.peak[s]:
                    self.peak[s] = v
                self.hist[s].append(v)

    def _reset_peaks(self):
        for s in SENSORS: self.peak[s] = 0

    def _tick(self):
        for s in SENSORS:
            canv, rect, peak_line, val_lbl, peak_lbl = self.bar_rows[s]
            w = canv.winfo_width() or 560
            frac = self.values[s] / ADC_MAX
            canv.coords(rect, 0, 0, int(w * frac), 28)
            pf = self.peak[s] / ADC_MAX
            px = int(w * pf)
            canv.coords(peak_line, px, 0, px, 28)
            val_lbl.config(text=f"{self.values[s]}")
            peak_lbl.config(text=f"pk:{self.peak[s]}")
        self._draw_plot()
        self.after(50, self._tick)

    def _draw_plot(self):
        c = self.plot
        c.delete("all")
        w = c.winfo_width() or 740
        h = c.winfo_height() or 200
        if w < 2 or h < 2: return
        # gridlines at 25/50/75% of ADC range
        for frac in (0.25, 0.5, 0.75):
            y = h - int(h * frac)
            c.create_line(0, y, w, y, fill="#222")
        n = HIST_LEN
        dx = w / (n - 1)
        for s in SENSORS:
            pts = []
            for i, v in enumerate(self.hist[s]):
                pts.append(i * dx)
                pts.append(h - (v / ADC_MAX) * h)
            c.create_line(*pts, fill=COLORS[s], width=1.5, smooth=False)
        # legend
        for i, s in enumerate(SENSORS):
            c.create_text(10 + i * 50, 12, text=s,
                          fill=COLORS[s], font=("Menlo", 11, "bold"), anchor="w")

    def destroy(self):
        self.running = False
        try:
            if self.ser: self.ser.close()
        except Exception:
            pass
        super().destroy()


if __name__ == "__main__":
    port = pick_port(sys.argv[1] if len(sys.argv) > 1 else None)
    app = IRMonitor(port)
    app.mainloop()
