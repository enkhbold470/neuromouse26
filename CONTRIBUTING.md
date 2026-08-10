# Contributing to NeuroMouse26

Thanks for your interest! This project was built for the 2026 All America Micromouse Contest (AAMC @ UCLA IEEE). We welcome bug fixes, tuning insights, and hardware adaptations.

## Ground Rules

- **Hardware-specific constants** (`CELL_TICKS`, `RIGHT_ENC_SCALE`, IR cal values) are chassis-specific — note your hardware if you change them.
- **No magic numbers** — all pins → `PinConfig.h`, all tuning → `Tuning.h`.
- **Keep the single-loop architecture** in `[env:main]`. Do not add FreeRTOS task splits or pull `WifiDebug.h` into main.
- **BLE RC** (`BLECarControl.h`) is an intentional OLED-menu mode in main; do not expand it into a second control architecture.
- **LEDC API is Arduino 2.x** (`ledcSetup` + `ledcAttachPin`). Don't mix with 3.x API.
- **Test sketches** belong in `test/` with a matching `[env:<name>]` block in `platformio.ini`.
- **Never commit WiFi passwords.** `WifiDebug.h` uses placeholder `WIFI_SSID` / `WIFI_PASS` (override locally or via `-D` build flags).

## Workflow

1. Fork → branch from `main`.
2. Make changes. Build: `pio run -e main`.
3. Upload and verify with serial monitor at 115200 baud.
4. Open a PR with a clear description of what changed and why.

## What's in Scope

- Bug fixes in motion control, planner, or maze logic.
- Porting to different hardware (new IR sensors, different MCU) — open an issue first.
- Documentation improvements.
- New test sketches (`test/` + `platformio.ini` env).

## Out of Scope

- Pulling WiFi / `WifiDebug.h` into `[env:main]`.
- External dependencies beyond `NimBLE-Arduino`, `U8g2`, and `FastLED`.
- FreeRTOS multi-task rewrites.

## Reporting Issues

Please include:
- Your hardware (MCU board, motor specs, encoder CPR).
- PlatformIO / espressif32 platform version.
- Serial output (enable `TELEMETRY=true` in `Tuning.h` only while capturing the log; default is `false`).

## Header map

See [`include/README.md`](include/README.md) for the module inventory, include order, and which headers are LEGACY / dormant.