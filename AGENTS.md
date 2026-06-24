# neuromouse26 — Agent Guide

Micromouse26: ESP32-S3 maze robot (flood-fill explore + NVS fast run, IR wall sense, MPU-6500 heading hold, BLE telemetry).

## Docs

| File | Purpose |
|---|---|
| `CLAUDE.md` | Module map, state machine, build commands, BLE status, invariants |
| `GEMINI.md` | Architecture deep-dive, hardware table, motor/IR/PID patterns |
| `OVERVIEW.md` | High-level summary |
| `docs/2026-06-24-firmware-progress.md` | **Session progress** — explore flow, cal, experiments, current banner |
| `docs/IR-CALIBRATION.md` | Front/side IR values and thresholds |
| `include/README` | Header inventory |

## Build

```bash
~/.platformio/penv/bin/pio run -e main              # build production
~/.platformio/penv/bin/pio run -e main -t upload      # flash
~/.platformio/penv/bin/pio device monitor             # serial @ 115200
```

Default env: `main`. Diagnostic envs: `wall-follow-pcnt`, `sensor-cal-ble`, `encoder-test`, `imu-turn`, etc. — see `platformio.ini` and `CLAUDE.md`.

## Layout

- `src/main.cpp` — state machine + motion executor
- `include/` — header-only subsystems (text-included once)
- `test/` — per-subsystem PlatformIO sketches
- `tools/ble-debug.html` — Web Bluetooth monitor for `bromouse`
- `hardware/` — PCB netlists and pick-and-place (2026-06-23)

## Conventions

- Firmware change → upload (`main` env) without extra confirmation.
- No commits unless explicitly requested.
- Platform: stock `espressif32` Arduino 2.x (not Arduino 3.x LEDC API).
