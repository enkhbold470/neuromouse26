# Micromouse26 header layout

Domain code is split into **header-only modules** (single translation unit —
every header is text-included exactly once into `src/main.cpp`). To touch any
subsystem, open the matching header. Sections `[A]`–`[H]` referenced below are
the tuning blocks in `Tuning.h`.

> **Start here as a newcomer:** `src/main.cpp` (top comment + `setup`/`loop`) →
> `Tuning.h` → `PinConfig.h` → then the subsystem you care about.

## Project-owned headers (this directory)

| Header | Role |
|---|---|
| `Tuning.h` | Every tunable constant. Sections: `[A]` geometry, `[B0]` `BASE_BREAKAWAY_PWM`, `[B]` forward, `[C]` turns, `[D]` IR centering, `[E]` dead-end, `[F]` script geometry, `[H]` debug flags (`USE_IMU`, `USE_IR`, `TELEMETRY`). |
| `IMU.h` | MPU-6500 register stack, bias capture, yaw integration. Owns: `yawDeg`, `gzFilt`, `gyroBiasZ`, `yawTargetDeg`, `imuReady`. Call `updateYaw()` once per loop. |
| `IRSensors.h` | 4-sensor IR array (LF, L, R, RF). Owns: `PAIRS`, `irVal`, EMA state, `calL`/`calR`. `readIR()` / `sampleIR()`. |
| `Battery.h` | Vbat ADC + 6.4–8.4 V → 0–100 % linear SOC. |
| `Pose.h` | Robot pose + mode flags. Owns: `robotRow`/`Col`/`Heading`, planned pose, `exploreMode`, `fastRunMode`, `fastRunCruiseTps`, `pendingOffsetTicks`. |
| `MotionScript.h` | Phase-script data + pushers. Owns: `RunPhase` / `TurnDir`, `PhaseStep`, `script[]`, script indices. Stateful hooks (`phaseEnter`, `onPhaseActivate`, `scriptKick`) live in `main.cpp`. |
| `Persistence.h` | NVS save/load for walls (`"walls"`) and fast cruise (`"fspeed"`). Namespace `"mm26"`. Externs `maze` (defined in `main.cpp`). |
| `Planner.h` | `setupMaze`, `senseAndStoreWalls`, `buildMoveScript` (+ fast-run straight fusion). |
| `OLED.h` | U8G2 instance + menu + run + diag screens + auto gyro-cal. |
| `BLECarControl.h` | Optional BLE RC mode (NimBLE NUS). Selected from the OLED menu; drives `BLE_CAR_DRIVE` in `main.cpp`. |

## Library / shared hardware headers

| Header | Role |
|---|---|
| `PinConfig.h` | Pins, PWM/IR caps, `RIGHT_ENC_SCALE`, `BAT_VDIV_MULT`, IR cal defaults. Also contains **legacy constants** used only by older `test/` wall-follow sketches — see the file's section banners. |
| `MicromouseMotor.h` | DRV8833 wrapper (LEDC PWM, drive/brake/coast). |
| `MicromouseEncoderPCNT.h` | ESP32 PCNT 4× quadrature decoder — **what `[env:main]` uses**. |
| `MicromouseMaze.h` | 16×16 maze + flood-fill BFS + `bestDirectionBiased`. |
| `IRCalibration.h` | `IRCal::` namespace, `IR_DIST_TABLE`, `estimateFrontDistMM`. |

## Legacy / dormant (do not use for new main-firmware work)

| Header | Status |
|---|---|
| `MicromouseEncoder.h` | **LEGACY** ISR half-quadrature encoder. Unused by `[env:main]`; still referenced by a few older `test/` sketches. Prefer `MicromouseEncoderPCNT.h`. |
| `PID.h` | **LEGACY** generic PID helper. Unused — `main.cpp` embeds its own IR-centering `PID` struct. |
| `WifiDebug.h` | **DORMANT** HTTP debug server. **Not** included by `[env:main]`. Requires local WiFi credentials (never commit real passwords). |

## What lives in `src/main.cpp`

- Hardware instances: `maze`, `leftMotor`, `rightMotor`, `leftEnc`, `rightEnc`.
- IR-centering PID struct + instance (`pid`).
- Helpers: `rTicks`, `stopMotors`, `buttonEdge`.
- Phase hooks: `onPhaseActivate`, `phaseEnter`, `scriptKick`.
- `setup()` and the full `loop()` state machine:
  `IDLE` / `ENC_TEST` / `IR_TEST` / `FAST_SPEED_EDIT` / `EXPLORE_THINK` /
  `RUN` / `GOAL` / `CRASH` / `BLE_CAR_DRIVE`.

## Include order in `src/main.cpp`

1. Tuning + libs first (no shared mutable state):  
   `Arduino.h`, `Wire.h`, `PinConfig.h`, `Tuning.h`, `MicromouseMotor.h`,
   `MicromouseEncoderPCNT.h`, `MicromouseMaze.h`, `IRCalibration.h`.
2. Leaf state headers: `IMU.h`, `IRSensors.h`, `Battery.h`, `Pose.h`, `MotionScript.h`.
3. Hardware objects + `pid` + helpers defined in `main.cpp`.
4. Headers that need `maze` / encoders in scope:  
   `Persistence.h`, `Planner.h`, `OLED.h`, `BLECarControl.h`.
5. `setup()` and `loop()`.

## Status LED (onboard WS2812 on `rgb_pin` = GPIO 48)

Driven non-blocking from `loop()`:

| Pattern | Meaning |
|---|---|
| Off | Menu / running / non-terminal |
| Green flash → rainbow | `GOAL` (repeats until button) |
| Red blink (300 ms) | `CRASH` (flood unreachable / boxed in) |

On `GOAL`, the robot also spins 180° in place before the LED celebration.

## Conventions

- File-scope `static` in headers is OK because `main.cpp` is the only TU
  (`build_src_filter = +<main.cpp>`). Adding a second `.cpp` to `env:main`
  requires converting shared state to `inline` (C++17) or one definition site.
- `extern MicromouseMaze maze;` in `Persistence.h` / `Planner.h` points at the
  single definition in `main.cpp`.
- New tuning knob → `Tuning.h`. New screen → `OLED.h`. Motion *numerics* →
  `Tuning.h`; motion *behavior* (PID structure, settle logic) → `src/main.cpp`
  `RUN` case.
- Include-guard prefixes are mixed (`MM26_*` for newer modules,
  `MICROMOUSE_*` / `PIN_CONFIG_H` for older). Do not rename casually — no
  functional benefit, breaks muscle memory.
