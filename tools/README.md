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

Web Bluetooth dashboard for `pivot-mouse-v1` (`test/motor-ble-drive.cpp` etc).
Sends single-char commands (`1`-`5` cells, `R`/`L` turns, `s` stop, `?`
status) and streams TX notifications.

```bash
open tools/ble-debug.html          # macOS
# or serve any HTTP and open in Chrome/Edge (Web BT needs https/localhost)
```

Requires browser with Web Bluetooth (Chrome, Edge, Opera; Safari ✗).

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
