# NeuroMouse V2.0 — ESP32-S3 Micromouse Firmware

[![PlatformIO CI](https://github.com/enkhbold470/neuromouse26/actions/workflows/build.yml/badge.svg)](https://github.com/enkhbold470/neuromouse26/actions/workflows/build.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32--S3-orange.svg)](https://platformio.org/)
[![Award](https://img.shields.io/badge/AAMC%202026-3rd%20Place-brightgreen.svg)](https://www.youtube.com/watch?v=2M4ZANPrZ4s)

ESP32-S3 micromouse firmware (Arduino / PlatformIO) with flood-fill maze solving
and phase-script position-PID motion. **3rd place overall** at the 2026 All
America Micromouse Contest (AAMC @ UCLA IEEE).

Not affiliated with or endorsed by IEEE, UCLA, or AAMC organizers.

## Why this repo

- Single translation-unit firmware — fast iteration, zero link-time ambiguity
- Hardware **PCNT 4×** quadrature encoders (no ISR jitter)
- Fast-run **straight-cell fusion** (one trapezoid across a corridor)
- NVS-persisted wall map for explore → fast-run replay
- Competition-proven on a hand-built proto-board chassis

## Quickstart

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- ESP32-S3-WROOM (board env: `esp32-s3-devkitc-1`) over USB-CDC
- 2S LiPo for motor load (`MOTOR_SLEEP` GPIO41 must be HIGH — firmware does this)

### Build & upload

```bash
pio run -e main               # build
pio run -e main -t upload     # build + upload
pio device monitor            # serial @ 115200 baud
```

USB-CDC flags are already set in `platformio.ini`
(`ARDUINO_USB_MODE=1`, `ARDUINO_USB_CDC_ON_BOOT=1`). If the serial port does not
appear, reset the board (or hold BOOT) after plugging in.

### First run

1. Power on → OLED shows the main menu
2. Blue keycap switch selects mode; encoder wheel adjusts Fast Run Speed
3. **Explore** — flood-fill mapping; walls saved to NVS
4. **Fast Run** — replays the optimal path at `fastRunCruiseTps`

Leave `TELEMETRY=false` in `Tuning.h` for timing-sensitive runs.

## Competition demos

[![Watch Full Competition Run on YouTube](https://img.shields.io/badge/YouTube-Watch%20Full%20AAMC%20Run-red?style=for-the-badge&logo=youtube)](https://www.youtube.com/watch?v=2M4ZANPrZ4s)

| Explore (flood-fill) | Fast run (straight fusion) | Goal celebration |
|---|---|---|
| ![Maze Explore Run](docs/images/maze_explore.gif) | ![Fast Run Acceleration](docs/images/maze_fastrun.gif) | ![Goal Cell Reached](docs/images/maze_goal.gif) |

## Hardware & debugger gallery

| Competition chassis | Web debugger (historical home bench) |
|---|---|
| ![V2.0 on maze wall](docs/images/20260523_204208.jpg) | ![Web Debugger GUI](docs/images/sim_gui_run.png) |

| Bench debugging | Turn telemetry UI |
|---|---|
| ![Dev loop](docs/images/20260523_232430.jpg) | ![Turn telemetry](docs/images/sim_gui_turn.png) |

The HTTP web debugger (`WifiDebug.h`) is a **dormant lab tool** — not part of
`[env:main]`. See [SECURITY.md](SECURITY.md).

## Hardware stack

```
MCU:            ESP32-S3-WROOM (Xtensa LX7 dual-core, 240 MHz)
Motors:         GA-N20 brushed DC — 1:30 gear ratio, 500 RPM @ 6V
Driver:         DRV8833 dual H-bridge (one channel per side)
Encoders:       7 CPR magnetic quadrature disk on motor shaft (PCNT 4× decode)
IR Emitters:    SFH4545 (950 nm narrow-angle IR LEDs)
IR Receivers:   TEFT4300 (NPN phototransistors)
                - LF/RF → straight forward (front detect + align)
                - L/R   → 90° perpendicular (side-wall reads)
Gyro/IMU:       MPU-6500 (I2C 0x68) — yaw integration for spot turns
Display:        0.96" SSD1306 128×64 OLED (I2C 0x3C)
Tactile Switch: Linear Blue Keycap Switch (GPIO 42)
Battery:        300 mAh 2S LiPo (7.4V nominal)
```

### Pin summary

| Function | GPIO | Notes |
|---|---|---|
| Motor L IN1 / IN2 | 18 / 17 | DRV8833 |
| Motor R IN3 / IN4 | 15 / 16 | DRV8833 |
| Motor nSLEEP | 41 | Must be HIGH |
| Enc L A/B | 21 / 14 | PCNT |
| Enc R A/B | 38 / 39 | PCNT |
| IR RX L / LF / RF / R | 10 / 4 / … | See `PinConfig.h` |
| Button / Buzzer | 42 / 40 | |
| Onboard WS2812 | 48 | Status LED |
| Vbat sense | see `BAT_V_SENSE` | Divider `BAT_VDIV_MULT` |

Full map: [`include/PinConfig.h`](include/PinConfig.h).

**Key measured constants** (chassis-specific — re-measure if wheels/tires change;
values match current `Tuning.h` / `PinConfig.h`):

| Constant | Value | Notes |
|---|---|---|
| `CELL_TICKS` | 1373 | Hand-measured per cell pitch |
| `RIGHT_ENC_SCALE` | 1.0028f | L/R mechanical balance |
| `MOTOR_PWM_FREQ_HZ` | 200 Hz | Breakaway torque on this chassis |
| `WHEEL_DIAMETER` | 33.4 mm | |
| `WHEEL_TRACK_MM` | 80 mm | |

## Firmware architecture

Newcomers: [`src/main.cpp`](src/main.cpp) → [`include/README.md`](include/README.md)
→ `Tuning.h` / `PinConfig.h`. Deep dive: [`CLAUDE.md`](CLAUDE.md).

```
src/main.cpp                 PID + setup()/loop() state machine
include/Tuning.h             Tunables [A]–[H]
include/PinConfig.h          Pins + PWM/IR caps
include/MicromouseEncoderPCNT.h   Hardware PCNT encoders
include/Planner.h / MicromouseMaze.h   Motion script + flood-fill
include/IMU.h                MPU-6500 yaw (I2C)
test/*.cpp                   Standalone sketches (see platformio.ini)
```

### State machine (`loop()`)

```
IDLE → EXPLORE_THINK → RUN → GOAL
                      ↘ CRASH
IDLE → BLE_CAR_DRIVE → IDLE
```

- **`PH_FORWARD`** — trapezoidal position-PID, IR centering + yaw hold
- **`PH_SPOT`** — IMU yaw-PID in-place turn
- **`PH_ALIGN_FRONT`** — creep to calibrated front-wall gap (dead-end exit)

## Test sketches

Environments that currently build (see `platformio.ini`):

```bash
pio run -e encoder-test -t upload   # PCNT ticks + RPM
pio run -e imu-turn     -t upload   # gyro bias + yaw (test/mpu6500.cpp)
pio run -e ws2812b      -t upload   # onboard status LED
pio run -e wall-follow-pcnt -t upload
pio run -e goal-blink   -t upload
pio run -e sensor-cal   -t upload
pio run -e batt-volt    -t upload
pio run -e ble-test     -t upload
```

CI builds the same matrix on every PR to `main`.

## Optional tools

Bench helpers under [`tools/`](tools/README.md) (IR monitor, BLE car web UI,
vision experiments). Not required to build firmware.

## Engineering story

**V1.0** was a custom PCB that became an iteration bottleneck. **V2.0** (the
competition robot) is a hand-soldered proto-board — faster rewiring and sensor
re-angling. Home practice used a compact sub-region; competition ran the full
**16×16** maze with classical centre-4 goals. A dormant HTTP debugger
(`WifiDebug.h`) helped tune PID/IR on the bench without constant reflashing.

## Documentation

| Doc | Purpose |
|---|---|
| [CLAUDE.md](CLAUDE.md) | Full firmware / hardware guide |
| [include/README.md](include/README.md) | Header map |
| [docs/00_INDEX.md](docs/00_INDEX.md) | Historical engineering notes |
| [CONTRIBUTING.md](CONTRIBUTING.md) | How to contribute |
| [SECURITY.md](SECURITY.md) | Vulnerability reporting |
| [SUPPORT.md](SUPPORT.md) | Where to get help |
| [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) | Community standards |
| [THIRD_PARTY.md](THIRD_PARTY.md) | Dependency & media attribution |
| [CHANGELOG.md](CHANGELOG.md) | Release notes |

## Team & acknowledgments

**Inky Ganbold · Yu Hong (Elijah) Chen**  
*UCLA IEEE All America Micromouse Contest 2026 (AAMC)*

Thanks to PlatformIO, Espressif, and the authors of **U8g2**, **FastLED**, and
**NimBLE-Arduino**. Thanks to IEEE Student Branch at UCLA for hosting the contest.

## License

Project source and original documentation: [MIT License](LICENSE)
© 2026 Inky Ganbold, Yu Hong (Elijah) Chen.

Exceptions and third-party notices: [THIRD_PARTY.md](THIRD_PARTY.md).
