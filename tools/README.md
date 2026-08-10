# tools/

**Optional** developer utilities. Firmware build/flash only needs PlatformIO —
nothing under `tools/` is required to compile `[env:main]`.

## Layout

| Path | Purpose |
|---|---|
| `notify_upload.py` | PlatformIO post-upload chime (already wired in `platformio.ini`) |
| `ir_monitor.py` | Tk IR bar/plot UI over USB serial |
| `ble-debug.html` | Simple Web Bluetooth debug page |
| `ble-car-app/` | SvelteKit Web Bluetooth RC UI for OLED **BLE Car** mode |
| `vision/` | Optional OpenCV + Flask maze-vision helpers |

## `notify_upload.py`

Runs only on `pio run -t upload` (not during compile-only CI). Uses system
sounds (macOS `Glass.aiff`, Linux `paplay`/`aplay` when available, else terminal
bell). On Windows, optional `pip install playsound` plus a local
`tools/upload-chime.wav` if you want a custom sound.

## `ir_monitor.py`

```bash
python3 -m venv tools/.venv
source tools/.venv/bin/activate
pip install pyserial
# Debian/Ubuntu may also need: sudo apt install python3-tk
python tools/ir_monitor.py                 # auto-pick ESP32 port
python tools/ir_monitor.py /dev/ttyACM0    # explicit port
```

## `ble-debug.html`

Open in Chrome/Edge (Web Bluetooth). Prefer serving over `http://localhost` or HTTPS.

## `ble-car-app/`

See [`ble-car-app/README.md`](ble-car-app/README.md). Uses **npm** + committed
`package-lock.json` (`npm ci`).

## `vision/`

See [`vision/README.md`](vision/README.md). Runtime files such as `map.json` /
`corners.json` are gitignored — calibrate locally.
