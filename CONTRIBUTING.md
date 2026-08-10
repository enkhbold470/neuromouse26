# Contributing to NeuroMouse26

Thanks for your interest! This project was built for the 2026 All America Micromouse Contest (AAMC @ UCLA IEEE). We welcome bug fixes, tuning insights, and hardware adaptations.

Please follow our [Code of Conduct](CODE_OF_CONDUCT.md).

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
2. Make changes. Build: `pio run -e main` (CI also builds the listed test envs).
3. Upload and verify with serial monitor at 115200 baud when hardware-related.
4. Open a PR using the pull-request template.

## Commit style

Prefer short imperative subjects (≤72 chars). Optional Conventional Commits:
`fix:`, `feat:`, `docs:`, `test:`, `chore:`.

Example: `fix: scale right encoder via rTicks() in forward PID`

## What's in Scope

- Bug fixes in motion control, planner, or maze logic.
- Porting to different hardware (new IR sensors, different MCU) — open an issue first.
- Documentation improvements.
- New test sketches (`test/` + `platformio.ini` env).

## Out of Scope

- Pulling WiFi / `WifiDebug.h` into `[env:main]`.
- External dependencies beyond `NimBLE-Arduino`, `U8g2`, and `FastLED`.
- FreeRTOS multi-task rewrites.

## License of contributions

By contributing, you agree that your contributions are licensed under the
[MIT License](LICENSE) (same terms as this repository). No separate CLA is required.

## Reporting Issues

Please include:

- Your hardware (MCU board, motor specs, encoder CPR).
- PlatformIO / espressif32 platform version.
- Serial output (enable `TELEMETRY=true` in `Tuning.h` only while capturing the log; default is `false`).

Security issues: [SECURITY.md](SECURITY.md). Support routing: [SUPPORT.md](SUPPORT.md).

## Docs map

| Doc | Use |
|---|---|
| [`include/README.md`](include/README.md) | Header inventory, include order, LEGACY notes |
| [`CLAUDE.md`](CLAUDE.md) | Full firmware / hardware guide for contributors & agents |
| [`docs/00_INDEX.md`](docs/00_INDEX.md) | Historical engineering notes (may lag current stack) |
| [`THIRD_PARTY.md`](THIRD_PARTY.md) | Third-party licenses & attribution |
| [`tools/README.md`](tools/README.md) | Optional developer utilities |
