# tools/

Helper scripts for Micromouse26 firmware development.

## `ir_monitor.py`

Live GUI for 4 IR sensors over USB serial. Bars + rolling plot.

```bash
pip install pyserial               # only dep beyond stdlib tkinter
python tools/ir_monitor.py         # auto-pick ESP32 port
python tools/ir_monitor.py /dev/cu.usbmodem1101   # explicit port
```

Parses both `L:1234 LF:... RF:... R:...` and `L 1234 (min/avg/max) ...`
output from `test/ir-test.cpp` / `test/sensor_cal.cpp`. Click **Reset peaks**
to clear the white tick marks.

## `ble-debug.html`

Web Bluetooth dashboard for production firmware device **`bromouse`** (`env:main`).

- 6×3 maze grid, IR bars, motion panel, state badge
- Commands: `EXPLORE`, `FAST`, `STOP`, `DUMP`, `CLEAR_NVS`
- Telemetry JSON: `ST`, `POS`, `WALL`, `MOT`, `BAT`, `CRASH`, `MAZE`

```bash
# Must use localhost or HTTPS — not file://
python3 -m http.server 8080
# open http://localhost:8080/tools/ble-debug.html in Chrome
```

**macOS:** System Settings → Privacy & Security → Bluetooth → enable **Google Chrome** if the device picker is empty.

See also: `docs/2026-06-24-firmware-progress.md` (BLE section).

## `notify_upload.py`

PlatformIO post-upload hook — plays a sound on successful `pio run -t upload`.
Already wired via `platformio.ini` (`extra_scripts = post:tools/notify_upload.py`).
No manual invocation. macOS uses `Glass.aiff`; Windows uses `playsound` + MP3;
Linux falls back to terminal bell.

## `.venv/`

Local Python venv for the scripts above. Activate:
```bash
source tools/.venv/bin/activate
```
