# Contributing to NeuroMouse26

Thanks for your interest! This project was built for the 2026 All America Micromouse Contest (AAMC @ UCLA IEEE). We welcome bug fixes, tuning insights, and hardware adaptations.

## Ground Rules

- **Hardware-specific constants** (`CELL_TICKS`, `RIGHT_ENC_SCALE`, IR cal values) are chassis-specific — note your hardware if you change them.
- **No magic numbers** — all pins → `PinConfig.h`, all tuning → `Tuning.h`.
- **No FreeRTOS task splits, no WiFi** in `[env:main]`. Keep the single-loop architecture intact.
- **LEDC API is Arduino 2.x** (`ledcSetup` + `ledcAttachPin`). Don't mix with 3.x API.
- **Test sketches** belong in `test/` with a matching `[env:<name>]` block in `platformio.ini`.

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

- WiFi / BLE features in `[env:main]` (use `WifiDebug.h` as a side env).
- External dependencies beyond `NimBLE-Arduino` and `U8g2`.
- FreeRTOS multi-task rewrites.

## Reporting Issues

Please include:
- Your hardware (MCU board, motor specs, encoder CPR).
- PlatformIO / espressif32 platform version.
- Serial output (with `TELEMETRY=true` in `Tuning.h`).
