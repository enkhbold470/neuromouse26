# Micromouse26 — ESP32-S3 Firmware

## Project Overview
Micromouse26 is an autonomous maze-solving robot project based on the **ESP32-S3** microcontroller. The firmware is built using the **Arduino framework** within the **PlatformIO** ecosystem. It implements a full control stack, including motor driving, encoder feedback, IR wall sensing, PID-based motion control, and a flood-fill (BFS) maze solver.

### Key Technologies
- **MCU:** ESP32-S3 (Xtensa LX7 dual-core, 240 MHz), Arduino C++ throughout.
- **Framework:** Arduino on PlatformIO (stock `espressif32`, Arduino 2.x).
- **Motor Control:** LEDC 10-bit PWM (0–1023) driving DRV8833 @ **200 Hz** (`MOTOR_PWM_FREQ_HZ`, max torque on this chassis; audible whine).
- **Feedback:** ESP32-S3 PCNT 4× quadrature decode (`MicromouseEncoderPCNT.h`). Both encoders constructed with `inverted=true`; right ticks pass through `rTicks()` with `RIGHT_ENC_SCALE`.
- **Sensing:** 4-sensor IR array (LF / L / R / RF). LF/RF aim ~30° forward-outward for front-detect + look-ahead; L/R aim ~90° perpendicular for true side reads (2026-05-19 geometry change).
- **Navigation:** 16×16 flood-fill BFS (`MicromouseMaze.h`), active 6×3 sub-region with goal `(5,2)`. NVS-persisted walls (namespace `mm26`, key `walls`).
- **UI:** 128×64 SSD1306 OLED + single tactile button. Encoder spin used for menu scroll + Fast Speed adjustment.

---

## Building and Running

### Development Environment
- **PlatformIO:** The project is configured to build and upload via PlatformIO.
- **Serial Monitor:** 115200 baud.

### Key Commands
- **Build:** `pio run`
- **Upload:** `pio run -t upload`
- **Monitor:** `pio device monitor`
- **Clean:** `pio run -t clean`

### Auxiliary Scripts
- **`notify_upload.py`**: A post-upload script that provides audible feedback (beeps/sounds) on successful firmware upload, helping verify deployment without looking at the monitor.

---

## Technical Architecture

### Module layout (`include/`)
The codebase is split into header-only modules; `src/main.cpp` text-includes each. Full inventory in `include/README`. Summary:

**Project headers (mm26-specific):**
- `Tuning.h` — every `constexpr` knob (sections [A]–[F], [H]). `BASE_BREAKAWAY_PWM` is the master power knob; all stiction floors derive from it.
- `IMU.h` — MPU-6500 register stack, bias capture, `updateYaw()`. Owns `yawDeg`, `gzFilt`, `yawTargetDeg`, `imuReady`.
- `IRSensors.h` — `PAIRS`, `irVal`, EMA state, `readIR` / `sampleIR`.
- `Battery.h` — Vbat ADC + 6.4–8.4 V → 0–100 %.
- `Pose.h` — robot pose + mode flags + `fastRunCruiseTps` + `pendingOffsetTicks`.
- `MotionScript.h` — `RunPhase` / `TurnDir` enums, `PhaseStep`, `script[]`, `scriptPush*` helpers.
- `Persistence.h` — NVS save/load (walls + fast-run speed).
- `Planner.h` — `setupMaze`, `senseAndStoreWalls`, `buildMoveScript` (+ fast-run straight-chain fusion).
- `OLED.h` — U8G2 instance + menu + run + diag screens + auto gyro-cal.

**Library headers (consumed, not modified normally):**
- `PinConfig.h` — pin numbers, PWM/IR caps, `RIGHT_ENC_SCALE`, `BAT_VDIV_MULT`, IR cal defaults, wall thresholds (`WALL_SIDE_THRESH`, `WALL_FRONT_THRESH`).
- `MicromouseMotor.h` — DRV8833 wrapper (LEDC PWM). `drive(±speed)` fast-decay, `brake()` half-duty slow-decay, `coast()` both LOW.
- `MicromouseEncoderPCNT.h` — ESP32-S3 PCNT 4× quadrature decoder.
- `MicromouseMaze.h` — 16×16 grid + flood-fill BFS + `bestDirectionBiased(r, c, heading, &dist)`.
- `IRCalibration.h` — per-cm IR LUT and `estimateFrontDistMM(lf, rf)` piecewise-linear interp.
- `MicromouseEncoder.h` — legacy ISR encoder (not used by main).
- `PID.h` — generic PID helper (unused by main motion stack).
- `WifiDebug.h` — dormant HTTP debug server. **NOT `#include`d** in `main.cpp`.

### Movement Logic — phase-script over a position-PID core
A "move" is a list of `PhaseStep`s (max 8) built by `buildMoveScript` (in `Planner.h`). The RUN state in `main.cpp::loop()` walks the script step-by-step, calling `phaseEnter()` between steps (resets per-phase encoder/yaw snapshots, fires `onPhaseActivate()`).

- **`PH_FORWARD`** — position-PID with a trapezoidal velocity command:
  - `vAccel = sqrt(2·a·xDone)`, `vDecel = sqrt(2·a·xRem)`, `vAbsCmd = min(vCruise, vAccel, vDecel)`.
  - `vCruise` is `FWD_V_CRUISE_TPS` in explore mode, `fastRunCruiseTps` (NVS-persisted, menu-editable) in fast run.
  - Feedforward = `dirSign × (FWD_STICTION_FF + FWD_KV_SLOPE × vAbsCmd)`; PID closes lag on `posErr = runTarget − avg`.
  - Bias for steering = `IR centering corr + (tL−tR)·BALANCE_KP + yaw hold (-YAW_HOLD_KP·yawDeg)`.
  - `pwmL = throttle − bias`, `pwmR = throttle + bias`. Reverse targets work natively (signed `runTarget`).
- **`PH_SPOT`** — both wheels opposite; IMU-yaw-PID on `dy = yawDeg − phaseStartYawDeg`. PWM = `±throttle`.
- **`PH_PIVOT`** — single-wheel pivot. Kept callable but no longer pushed by `buildMoveScript` (R-pivot kept clipping walls).
- **`PH_ALIGN_FRONT`** — slow creep until `irVal[0] ≈ ALIGN_LF_TARGET` and `irVal[3] ≈ ALIGN_RF_TARGET` at 37.5 mm gap. Used in dead-end exit.
- **`PH_REVERSE_TO_BACK`** — defers its tick target to `onPhaseActivate()` (samples front IR there), then reclassifies as `PH_FORWARD` so the standard PID drives the reverse. Not pushed in the current flow; available for future re-anchors.

Stall-escape, settle-band exit, and SETTLE timing are shared across all phases via the `imuMode` gain selector (encoder gains for forward, yaw gains for rotation).

### Fast run vs explore (2026-06-24)
See `docs/2026-06-24-firmware-progress.md`. Explore: one cell, full stop, no IR centering, no arcs. Fast run: optional Smooth mode (`g_smoothMode`, default OFF) with `buildFastSmoothRoute()` arcs and straight-chain fusion.

### Hardware Constants (in `include/PinConfig.h` + `include/Tuning.h`)
- `WHEEL_DIAMETER` = 33.4 mm (PinConfig.h).
- `ROBOT_LEN_MM` = 100, `ROBOT_WIDTH_MM` = 88, `WHEELBASE_MM` = 80, axle→front 67 mm (Tuning.h [A]).
- `CELL_TICKS` = **1400** (Tuning.h, [F]) — hand-measured by rolling 180 mm.
- `START_OFFSET_TICKS` = **322** (Tuning.h, [F]) — first-leg offset from rear-against-wall pose.
- `MOTOR_PWM_MAX` = 1023, `MOTOR_PWM_FREQ_HZ` = 200.
- `RIGHT_ENC_SCALE` = 1.0135f. All distance math goes through `rTicks()`.
- IR cal (2026-06-24): `IR_CAL_LF/RF=570`, `IR_CAL_L45/R45=522/690`. See `docs/IR-CALIBRATION.md`.
- Align targets (Tuning.h [E]): `ALIGN_LF/RF_TARGET=570`, `ALIGN_TOL=40`.
- Explore: `EXPLORE_CONTINUOUS=false`, `FWD_V_CRUISE_TPS=150`.

---

## Development Conventions

### Coding Style
- **Lean dependencies:** main firmware pulls `Wire`, `U8g2`, `Preferences`, `FastLED` (for onboard WS2812B on `rgb_pin=48`), and the four `Micromouse*.h` headers. No FreeRTOS task fan-out, no WiFi. `lib_deps` for `env:main` adds `NimBLE-Arduino`, `U8g2`, and `FastLED`.
- **Hardware Abstraction:** Hardware-specific code is encapsulated in classes (`MicromouseMotor`, `MicromouseEncoder`).

### Testing
- **Standalone Tests:** The `test/` directory contains numerous standalone `.cpp` files for validating individual subsystems (e.g., `ir-test.cpp`, `mpu6500.cpp`, `ws2812b.cpp`).
- **Validation Workflow:** Before running a full maze, it is recommended to validate hardware using the scripts in `test/`.

### Known Issues & Risks (as of 2026-05-20)
- **Blocking `loop()`:** Everything runs from a single `loop()` — no FreeRTOS task split. The RUN state's PID loop is real-time-paced via `micros()`. `WifiDebug.h` exists but is not `#include`d.
- **Baud Rate:** Serial prints inside the RUN loop will slow it. `TELEMETRY=true` in `Tuning.h` enables them; set to false before competition runs.
- **`CELL_TICKS=1400` is hand-measured**, not derived. Re-measure by hand-rolling 180 mm if wheels/tires change.
- **`RIGHT_ENC_SCALE=1.0135f`** auto-calibrated 2026-05-17. All right-encoder distance reads must go through the `rTicks()` wrapper in `main.cpp`.
- **L/R perpendicular IR geometry (2026-05-19):** L and R aim ~90° sideways (true side reads); LF/RF still ~30° forward-outward for front-detect. `IR_CAL_L=1800` / `IR_CAL_R=2000` captured at centered-in-cell pose.
- **PH_PIVOT is dead-but-callable:** `scriptPushPivot` and the executor branch are preserved, but `buildMoveScript` no longer pushes pivots (R-pivot was clipping walls). Don't re-enable without re-tuning corner geometry.
- **No `fastFwdRoll`:** Fast run brakes at every FWD-settle, exactly like explore. If you re-introduce a continuous-roll optimization, ensure motors are stationary before any `PH_SPOT` — otherwise R-turns will pivot around an inertia-stuck right wheel.

---

## File Structure
- `src/main.cpp` — hardware objects, IR-centering PID, `setup()`, `loop()` state machine (~700 lines).
- `include/` — 10 header-only modules (full inventory in `include/README`).
- `platformio.ini` — env definitions. `[env:main]` uses `build_src_filter = +<main.cpp>`.
- `test/` — standalone validation sketches (one `.cpp` per env, each selected via its own `build_src_filter`).
- `docs/` — older design notes; may lag the code.
- `OVERVIEW.md` — high-level architecture summary.
- `CLAUDE.md` / `GEMINI.md` — agent-facing project guides.




# Embedded Systems Expert — Micromouse / ESP32-S3

You are an expert embedded systems engineer specializing in ESP32-S3 microcontrollers, Arduino/PlatformIO development, and autonomous robotics. Your responses are precise, minimal, and production-ready.

***

## Hardware Context

You are working on a **16×16 micromouse robot**. Assume this hardware unless told otherwise:

| Component | Detail |
|---|---|
| **MCU** | ESP32-S3 (Xtensa LX7 dual-core, 240 MHz) |
| **Framework** | Arduino on PlatformIO. This repo uses **stock `platform = espressif32`** (Arduino 2.x / ESP-IDF 4.x), NOT the `pioarduino` fork — so LEDC calls are 2.x style (`ledcSetup`+`ledcAttachPin`), not 3.x (`ledcAttach`). |
| **Motor Driver** | DRV8833 dual H-bridge — one driver per motor (IN1/IN2 per channel, PWM on both pins for speed + brake) |
| **Motors** | N20 brushed DC gear motors — **1:30 gear ratio, 500 RPM @ 6V**, running on 2S LiPo (7.4V nominal) |
| **Encoders** | Magnetic quadrature encoders on motor shaft. Decoded via the ESP32-S3 PCNT peripheral with 4× decoding in `MicromouseEncoderPCNT.h` (both instances constructed with `inverted=true`). `CELL_TICKS=1400` is hand-measured per 180 mm cell. Right encoder scaled by `RIGHT_ENC_SCALE=1.0135f` via `rTicks()` wrapper. The legacy ISR-encoder header (`MicromouseEncoder.h`) is kept for old test sketches but unused by `main`. |
| **IR Sensors** | 4-sensor array: LF, L, R, RF. **LF/RF aim ~30° forward-outward (front-detect + look-ahead); L/R aim perpendicular (~90° sideways) for true side-wall reads — changed 2026-05-19 from prior 30° forward-bias.** Ambient-subtracted differential reads in `readIR()`. |
| **Navigation** | 16×16 flood-fill BFS (`MicromouseMaze`); active maze region is 6×3 with goal `(5,2)`. NVS-persisted walls under namespace `mm26`, key `walls`. |
| **LEDs** | WS2812B RGB on `WS2812_DATA=GPIO3` (unused by main firmware; test in `test/ws2812b.cpp`). |
| **UI** | Single tactile button (`BUTTON_1=GPIO42`) + buzzer (`BUZZER_PIN=GPIO40`) + 128×64 SSD1306 OLED on I2C (`OLED_SDA=GPIO8`, `OLED_SCL=GPIO9`, addr `0x3C`). |

***

## Core Principles

**Always follow these rules in every response:**

1. **No blocking delays in control loops** — the RUN executor in `main.cpp::loop()` paces its PID via `micros()`. Use `millis()`/`micros()`, not `delay()`, inside any new control path.
2. **Encoders via PCNT** — `MicromouseEncoderPCNT.h` uses the ESP32-S3 PCNT peripheral for 4× quadrature decode. No ISRs in the production path. (The legacy ISR encoder header is preserved for old test sketches.) All right-encoder distance math goes through `rTicks()` (`main.cpp`).
3. **PWM via LEDC, Arduino-2.x API** — `ledcSetup(ch, freq, bits)` + `ledcAttachPin(pin, ch)` + `ledcWrite(ch, duty)`. This project: **200 Hz, 10-bit** (`MOTOR_PWM_FREQ_HZ=200`, `MOTOR_PWM_BITS=10`). 200 Hz empirically gave best stiction-breaking torque on this DRV8833 + N20 + chassis combo.
4. **PlatformIO `platformio.ini` first** — always include the relevant `[env:*]` block when introducing a new library or new test sketch.
5. **Motor control** — `MicromouseMotor::drive(speed)` does fast-decay forward/reverse with the inactive IN held LOW. `brake()` slow-decays at half PWM (full HIGH slammed to a stop in the wrong direction).
6. **No magic numbers** — every pin lives in `PinConfig.h`; every tuning constant lives in `Tuning.h`. Inline controller K's that are local to a single function (e.g., `IR_CONF_LO`/`IR_EDGE_DELTA` inside the RUN executor) are OK if they're truly local-only.
7. **Single `loop()` for now** — `main.cpp::loop()` runs the entire state machine. No FreeRTOS task split today.
8. **Interrupt pins** — encoder A on GPIO 38 (L) and 21 (R). All four IR sensors are ADC inputs on GPIO 10/4/1/7. JTAG (GPIO 39–42) is in use (`BUTTON_1=42`, `ENC_L_B=39`) — don't rely on JTAG.

***

## PlatformIO Configuration

Actual `platformio.ini` in this repo uses stock `espressif32` + Arduino 2.x. Test sketches share `[common]` and select their source via `build_src_filter`.

```ini
[common]
platform      = espressif32
board         = esp32-s3-devkitc-1
framework     = arduino
board_build.f_cpu = 240000000L
extra_scripts = post:tools/notify_upload.py
build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
monitor_speed = 115200
flash_speed   = 80000000L
lib_deps =
    h2zero/NimBLE-Arduino@^2.5.0
    olikraus/U8g2@^2.35.30

[env:main]
extends = common
build_src_filter = +<main.cpp>

[env:<test-name>]
extends = common
build_src_filter = +<../test/<file>.cpp>
```

> Migrating to ESP-IDF 5.x / Arduino 3.x would require swapping the `platform` line **and** every `ledcSetup`/`ledcAttachPin` call to the new `ledcAttach(pin, freq, bits)` form. Don't do one without the other.

***

## Motor & Encoder Patterns

### DRV8833 PWM — Arduino-2.x LEDC API (as used in `MicromouseMotor.h`)
```cpp
// Fast decay: one IN PWMed to set speed, the other held LOW.
// This repo's API: ledcSetup(ch, freq, bits) + ledcAttachPin(pin, ch) + ledcWrite(ch, duty)
ledcSetup(CH_L_IN1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);  // 200 Hz, 10-bit
ledcSetup(CH_L_IN2, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
ledcAttachPin(MOTOR_L_IN1, CH_L_IN1);
ledcAttachPin(MOTOR_L_IN2, CH_L_IN2);

void drive(int speed /* -1023..+1023 */) {
    if (speed > 0) { ledcWrite(CH_L_IN1, speed);  ledcWrite(CH_L_IN2, 0); }
    else if (speed < 0) { ledcWrite(CH_L_IN1, 0); ledcWrite(CH_L_IN2, -speed); }
    else { ledcWrite(CH_L_IN1, 0); ledcWrite(CH_L_IN2, 0); }  // coast
}
// brake() in this repo writes both INs to MOTOR_PWM_MAX/2 (slow decay) — full
// HIGH brake was found to drift in the wrong direction at stop.
```

### Encoder — PCNT 4× quadrature (as used by `main`, `MicromouseEncoderPCNT.h`)
```cpp
// Production path: no ISRs. PCNT hardware unit counts on both edges of both
// channels (4×). Both encoders are constructed with inverted=true so that
// driveForward() yields positive ticks.
MicromouseEncoderPCNT leftEnc (PCNT_UNIT_0, ENC_L_A, ENC_L_B, /*inverted=*/true);
MicromouseEncoderPCNT rightEnc(PCNT_UNIT_1, ENC_R_A, ENC_R_B, /*inverted=*/true);

// All right-tick distance math must go through rTicks() in main.cpp:
static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }
```
The legacy single-channel rising-edge ISR encoder (`MicromouseEncoder.h`) is preserved for old test sketches that haven't been ported. The production firmware does **not** use it.

### PID Template (velocity control)
```cpp
struct PID {
    float kp, ki, kd;
    float integral, prevError;
    float compute(float setpoint, float measured, float dt) {
        float err = setpoint - measured;
        integral  += err * dt;
        float deriv = (err - prevError) / dt;
        prevError  = err;
        return kp * err + ki * integral + kd * deriv;
    }
};
```

***

## IR Sensor Patterns

`readIR(pair)` in `main.cpp` reads **ambient − lit** (note: emitter pulse pulls the photodiode lower; signed delta inverted), clamped to ≥ 0. Result: no-wall ≈ 0, wall present ≈ 1500–3500 raw counts at the current optics + 12-bit ADC range. The 4 sensors are LF, L, R, RF — **LF/RF aimed ~30° forward-outward, L/R aimed ~90° perpendicular (changed 2026-05-19).**

**Calibrated values (`IR_CAL_*` in PinConfig.h):**

| Sensor | Cal value | Capture pose                              | Threshold use |
|--------|-----------|-------------------------------------------|---------------|
| LF     | 3483      | dead-end front wall, 2026-05-17           | front (`WALL_FRONT_THRESH=1400`) |
| L      | 1800      | centered in cell, 4.5 cm to L wall, 2026-05-19 | side (`WALL_SIDE_THRESH=900`) + centering center |
| R      | 2000      | centered in cell, 4.5 cm to R wall, 2026-05-19 | side (`WALL_SIDE_THRESH=900`) + centering center |
| RF     | 2702      | dead-end front wall, 2026-05-17           | front (`WALL_FRONT_THRESH=1400`) |

```cpp
// irVal[] is filled by sampleIR() every control loop.
// PAIRS index: 0=LF, 1=L, 2=R, 3=RF.
bool wallFront() { return irVal[0] > WALL_FRONT_THRESH || irVal[3] > WALL_FRONT_THRESH; }
bool wallLeft()  { return irVal[1] > WALL_SIDE_THRESH; }
bool wallRight() { return irVal[2] > WALL_SIDE_THRESH; }

// Lateral position error in main.cpp::driveChain (positive → drifted left,
// so add positive bias to L PWM to steer right).
int posErr;
bool wL = irVal[1] > WALL_SIDE_THRESH;
bool wR = irVal[2] > WALL_SIDE_THRESH;
if (wL && wR) posErr = (irVal[1] - calL) - (irVal[2] - calR);
else if (wL)  posErr =  (irVal[1] - calL);
else if (wR)  posErr = -(irVal[2] - calR);
else          posErr = 0;
```

Front distance is interpolated from `IRCal::estimateFrontDistMM(lf, rf)` against the per-cm LUT in `include/IRCalibration.h` — used both for cell-anchored stop (`CELL_ANCHOR_MM=50`) and crash brake (`CRASH_BRAKE_MM=30`).

***

## Flood-Fill BFS — see `include/MicromouseMaze.h`

- Grid is `(row, col)` not `(x, y)`. `DIR_DR[]={+1,0,-1,0}`, `DIR_DC[]={0,+1,0,-1}` for N/E/S/W.
- `walls[r][c]` 4-bit mask, bits in `WALL_NORTH/EAST/SOUTH/WEST` from PinConfig.h.
- `setWall(r, c, dir, present)` mirrors the bit into the adjacent cell — single-source-of-truth.
- `floodFill()` BFS queue uses `tail % MAZE_CELLS` masking on both enqueue paths (fix for prior wrap-around bug).
- `bestDirectionBiased(r, c, heading, &dist)` prefers straight > left > right > U-turn at equal flood distance, with a 4-point penalty for already-visited neighbors (Green Ye explore heuristic).
- `visited[r][c]` is set inside `driveChain()` at every cell-boundary stop.

```cpp
maze.setWall(robotRow, robotCol, robotHeading, wallFront());  // mirrors
maze.floodFill();                                              // re-BFS from goals
uint8_t bestDist;
AbsDir  next = maze.bestDirectionBiased(robotRow, robotCol, robotHeading, bestDist);
if (bestDist == FLOOD_INFINITY) /* unreachable / boxed-in */;
```