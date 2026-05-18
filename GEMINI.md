# Micromouse26 — ESP32-S3 Firmware

## Project Overview
Micromouse26 is an autonomous maze-solving robot project based on the **ESP32-S3** microcontroller. The firmware is built using the **Arduino framework** within the **PlatformIO** ecosystem. It implements a full control stack, including motor driving, encoder feedback, IR wall sensing, PID-based motion control, and a flood-fill (BFS) maze solver.

### Key Technologies
- **MCU:** Universal Portability (Currently ESP32-S3 but written in standard Arduino C++)
- **Framework:** Arduino / PlatformIO
- **Motor Control:** LEDC 10-bit PWM (0-1023 scale) driving DRV8833 @ **200 Hz** (`MOTOR_PWM_FREQ_HZ`, max torque on this chassis; audible whine)
- **Feedback:** Single-channel rising-edge encoders. ISR samples pin-B level for direction (`MicromouseEncoder.h`).
- **Sensing:** 4-sensor IR array (LF, L45, R45, RF) for wall detection and centering
- **Navigation:** 16x16 Flood-fill BFS algorithm
- **UI:** Standard `tone()` buzzer, basic button

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

### Core Modules (in `include/`)
- **`MicromouseMotor`**: DRV8833 wrapper using LEDC PWM (Arduino-2.x `ledcSetup` + `ledcAttachPin` API). `drive(±speed)` for fast-decay forward/reverse, `brake()` = half-duty slow decay, `coast()` = both INs LOW.
- **`MicromouseEncoder`**: Single-channel rising-edge ISR encoder. Direction inferred by sampling pin-B level in the ISR. Up to 4 instances via static dispatch (`isr0..isr3`). ISRs in flash, not IRAM.
- **`MicromouseMaze`**: 16×16 grid + flood-fill BFS + `bestDirectionBiased(r, c, heading, &dist)`. Goal defaults to classical 4-cell center; `main.cpp::setupMaze()` overrides to `setGoalSingle(5, 2)`.
- **`IRCalibration`**: per-cm IR LUT and `estimateFrontDistMM(lf, rf)` piecewise-linear interpolation.
- **`PID`** (`include/PID.h`): generic templated PID helper. Not used by the production motion stack — `driveChain()` and `doTurn()` inline their own gain math. Available for new code if needed.
- **`WifiDebug`** (`include/WifiDebug.h`): HTTP debug server. **NOT `#include`d** in current `main.cpp` — dormant. Re-wire only if you intentionally bring WiFi back.

### Movement Logic
- **Forward Drive (`driveChain` in main.cpp):** cascaded velocity-PI inner loop at 200 Hz (`VPID_LOOP_US=5000`).
  - speed PI on `(vL+vR)/2` → `CELL_TARGET_MMS=250 mm/s` (`VPID_LOOP_KP=1.0`, `VPID_LOOP_KI=1.5`)
  - straight-line PI on `(curL−curR)` tick mismatch → 0 (`VPID_STRAIGHT_KP=6.0`)
  - feed-forward = `target / kV_avg`, battery-comp-scaled by `NOMINAL_VBAT / Vbat`
  - lateral bias adds `POS_KP×IR_pos_err + YAW_KP×yaw` directly to L/R PWM
- **Cell-boundary stop:** IR-anchored when a front wall is in range (`CELL_ANCHOR_MM=50`), else encoder-anchored at `TICKS_PER_CELL`. Hard crash brake at `CRASH_BRAKE_MM=30`. After stop, optional `squareToFrontWall()` if `35 < fdMM < 80`.
- **Right encoder scaling:** `rTicks() = rightEnc.getTicks() * RIGHT_ENC_SCALE` (currently `1.0135f`, auto-calibrated 2026-05-17). Always use `rTicks()` — not `rightEnc.getTicks()` — for distance math.
- **Pivot Turns:** MPU-6500 gyro-integrated, trapezoidal ω profile + KFF + PD on yaw. `TURN_PEAK_OMEGA_DPS=360`, `TURN_ACCEL_DPS2=1800`. No encoder-tick path. `TURN_TIMEOUT_MS=4000`. `TURN_PWM=200` is a saturation cap only.
- **Explore Loop:** `driveChain()` runs continuous, sensing walls + flood-filling at each cell boundary, returning only on direction change or goal.
- **Default maze:** 6 rows × 3 cols, goal at `(5,2)`. Allocated inside the 16×16 array (bottom-left corner). Geometry constants in `main.cpp`, not configurable at runtime.

### Hardware Constants (Adjust in `include/PinConfig.h`)
- `WHEEL_DIAMETER`: 33.4mm
- `TICKS_PER_REV`: 205.0f — empirical (raw ≈ 14 PPR × 30 gear = 420, halved by rising-edge-only ISR + 200µs filter at running speed). Do NOT "fix" to 210 or 420 — distance math will break.
- `TICKS_PER_CELL`: 350 — tape-validated, **not** computed from `MM_PER_TICK`.
- `WHEEL_TRACK_MM`: 74.0mm
- PWM is 10-bit (0-1023). `MOTOR_PWM_MAX` = 1023, `MOTOR_PWM_FREQ_HZ` = 200.
- `BAT_VDIV_MULT` = 3.751f (calibrated 2026-05-17; divider 39k / (39k+100k)).
- IR cal defaults (PinConfig.h, dead-end 32-sample mean 2026-05-17): `IR_CAL_LF=3483`, `IR_CAL_L=1491`, `IR_CAL_R=1648`, `IR_CAL_RF=2702`.
- Wall thresholds: `WALL_SIDE_THRESH=900`, `WALL_FRONT_THRESH=1400`.

---

## Development Conventions

### Coding Style
- **Lean dependencies:** main firmware pulls only `Wire`, `U8g2`, `Preferences`, and the four `Micromouse*.h` headers. No FastLED, no FreeRTOS task fan-out, no WiFi. `lib_deps` adds `NimBLE-Arduino` and `U8g2`; FastLED is not a dep of `env:main`.
- **Hardware Abstraction:** Hardware-specific code is encapsulated in classes (`MicromouseMotor`, `MicromouseEncoder`).

### Testing
- **Standalone Tests:** The `test/` directory contains numerous standalone `.cpp` files for validating individual subsystems (e.g., `ir-test.cpp`, `mpu6500.cpp`, `ws2812b.cpp`).
- **Validation Workflow:** Before running a full maze, it is recommended to validate hardware using the scripts in `test/`.

### Known Issues & Risks (as of 2026-05-17)
- **Blocking Loops:** `driveChain()`, `advanceToCellCenter()`, `doTurn()` block the main loop fully. No FreeRTOS task split. `WifiDebug.h` exists but is not `#include`d.
- **Baud Rate:** Serial prints inside the 200 Hz inner control loop will slow it. Comment out debug prints before speed runs.
- **`TICKS_PER_REV=205` is empirical, not theoretical.** Do NOT change to 210 or 420 — breaks distance and cell-pitch math.
- **`TICKS_PER_CELL=350` is tape-validated**, not derived. Re-measure with a tape against an actual cell rather than recomputing from `MM_PER_TICK`.
- **Right-encoder scaling:** `RIGHT_ENC_SCALE=1.0135f` (re-enabled 2026-05-17, value from on-device auto-cal). All distance reads must go through `rTicks()` wrapper.
- **Turn convergence:** `doTurn()` reports `OK`/`FAIL` via Serial; main.cpp sets `crashFlag` on FAIL and refuses to issue further `driveChain()` until reset. Hold phase is the only place `TURN_MIN_HOLD_PWM=260` is applied — do not turn it into an always-on stiction floor (fights the ramp).
- **IR side cone catches front wall at close range:** at `frontMM < 60 mm`, side sensors pick up the front wall via their 30° forward-bias cone. `driveChain()` filters these samples out of the look-ahead. Don't re-enable side-wall sensing at cell-center stop — the geometry guarantees wrong-cell reads.

---

## File Structure
- `src/main.cpp`: Main firmware entry point and logic.
- `platformio.ini`: Project configuration and dependencies.
- `test/`: Subsystem validation scripts.
- `docs/`: Technical documentation and design decisions (may vary from `src/main.cpp`).
- `OVERVIEW.md`: High-level summary and bug tracking.




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
| **Encoders** | Single-channel magnetic encoders on motor shaft (~14 PPR × 30 = ~420 raw ticks/output-rev). Rising-edge ISR samples pin-B level for direction. Effective `TICKS_PER_REV=205` at running speed (200µs noise filter halves count). Right encoder scaled by `RIGHT_ENC_SCALE=1.0135f` via `rTicks()` wrapper. |
| **IR Sensors** | 4-sensor array: LF, L, R, RF (all ~30° forward-outward). Ambient-subtracted differential reads in `readIR()`. Side walls sensed by look-ahead during cell traversal, not at stop. |
| **Navigation** | 16×16 flood-fill BFS (`MicromouseMaze`); active maze region is 6×3 with goal `(5,2)`. NVS-persisted walls under namespace `mm26`, key `walls`. |
| **LEDs** | WS2812B RGB on `WS2812_DATA=GPIO3` (unused by main firmware; test in `test/ws2812b.cpp`). |
| **UI** | Single tactile button (`BUTTON_1=GPIO42`) + buzzer (`BUZZER_PIN=GPIO40`) + 128×64 SSD1306 OLED on I2C (`OLED_SDA=GPIO8`, `OLED_SCL=GPIO9`, addr `0x3C`). |

***

## Core Principles

**Always follow these rules in every response:**

1. **No blocking delays in control loops** — `driveChain()` paces itself via `micros()` against `VPID_LOOP_US`. Use `millis()`/`micros()`, not `delay()`, inside any new control loop.
2. **ISR discipline** — encoder ISRs in `MicromouseEncoder.h` are flat (no `this` ptr) and live in flash (no `IRAM_ATTR`, deliberate — firmware never writes flash at runtime). Shared counters are `volatile`. No Serial or heap inside ISRs.
3. **PWM via LEDC, Arduino-2.x API** — `ledcSetup(ch, freq, bits)` + `ledcAttachPin(pin, ch)` + `ledcWrite(ch, duty)`. This project: **200 Hz, 10-bit** (`MOTOR_PWM_FREQ_HZ=200`, `MOTOR_PWM_BITS=10`). 200 Hz empirically gave best stiction-breaking torque on this DRV8833 + N20 + heavy-chassis combo. If you change it, **recalibrate kV/KV_L/R** on-device with `test/velocity-pid-ble.cpp` and paste new values into `PinConfig.h`.
4. **PlatformIO `platformio.ini` first** — always include the relevant `[env:*]` block when introducing a new library or new test sketch.
5. **Motor control** — `MicromouseMotor::drive(speed)` does fast-decay forward/reverse with the inactive IN held LOW. `brake()` slow-decays at half PWM (full HIGH was found to slam to a stop in the wrong direction).
6. **No magic numbers** — every pin, threshold, and tuning constant lives as `constexpr` in `PinConfig.h`. Inline controller K's inside a single function (e.g., `POS_KP_PWM` in `driveChain()`) are OK if they're truly local-only.
7. **Single `loop()` for now** — main firmware runs everything in `loop()`. No FreeRTOS task split today; the velocity-PI inner loop and turn loops are already real-time-paced via `micros()`.
8. **Interrupt pins** — encoder A on GPIO 38 (L) and 21 (R). All four IR sensors are ADC inputs on GPIO 10/4/1/7. JTAG (GPIO 39-42) is in use (`BUTTON_1=42`, `ENC_L_B=39`) — don't rely on JTAG.

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

### Encoder ISR — direction from pin-B level on rising A (as in `MicromouseEncoder.h`)
```cpp
// Flat ISRs, one per encoder, no `this` ptr (avoids Xtensa l32r literal issues).
// Counters live in flash (no IRAM_ATTR) — safe because firmware never writes
// flash at runtime.
namespace _mmenc {
    static volatile long c0 = 0;
    static uint8_t pinB0 = 0xFF;
    inline void isr0() {
        c0 += (pinB0 != 0xFF && digitalRead(pinB0)) ? -1 : 1;
    }
}
// setup():
attachInterrupt(digitalPinToInterrupt(ENC_L_A), _mmenc::isr0, RISING);
```

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

`readIR(pair)` in `main.cpp` reads **ambient − lit** (note: emitter pulse pulls the photodiode lower; signed delta inverted), clamped to ≥ 0. Result: no-wall ≈ 0, wall present ≈ 1500–3500 raw counts at the current optics + 12-bit ADC range. The 4 sensors are LF, L, R, RF — all aimed ~30° forward-outward.

**Calibrated dead-end values (2026-05-17, all 4 walls present, 32-sample mean — `IR_CAL_*` in PinConfig.h):**

| Sensor | Cal value | Threshold use |
|--------|-----------|---------------|
| LF     | 3483      | front (`WALL_FRONT_THRESH=1400`) |
| L      | 1491      | side (`WALL_SIDE_THRESH=900`) + centering center |
| R      | 1648      | side (`WALL_SIDE_THRESH=900`) + centering center |
| RF     | 2702      | front (`WALL_FRONT_THRESH=1400`) |

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