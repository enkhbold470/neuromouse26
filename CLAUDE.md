# CLAUDE.md — Micromouse26 ESP32-S3 Firmware Guide

This file is the single, authoritative guide for AI assistants (Claude, Gemini, etc.) and developers working in this repository.

> **🏆 3rd Place Overall · All America Micromouse Contest 2026 (AAMC @ UCLA IEEE)**  
> **MCU:** ESP32-S3 (Xtensa LX7 dual-core, 240 MHz) | **Framework:** Arduino on PlatformIO (stock `espressif32`, Arduino 2.x)

---

## 1. Hardware Architecture & Hardware Context

| Component | Specification & Implementation Details |
|---|---|
| **MCU** | ESP32-S3-WROOM (Xtensa LX7 dual-core, 240 MHz). Single translation unit build (`src/main.cpp`). |
| **Framework & API** | Arduino on PlatformIO using **stock `platform = espressif32`** (Arduino 2.x / ESP-IDF 4.x). LEDC API is 2.x style (`ledcSetup` + `ledcAttachPin` + `ledcWrite`), NOT 3.x `ledcAttach`. |
| **Motor Driver** | DRV8833 dual H-bridge — one driver channel per motor. Fast decay `drive(±speed)` (inactive IN held LOW); half-duty slow decay `brake()` (writes both INs to `MOTOR_PWM_MAX / 2`). |
| **Motors** | GA-N20 brushed DC gear motors — **1:30 gear ratio, 500 RPM @ 6V**, powered by 2S LiPo (7.4V nominal). |
| **Encoders** | Magnetic quadrature encoders (7 CPR disk on motor shaft). Decoded via ESP32-S3 **PCNT hardware peripheral (4× decode)** in `MicromouseEncoderPCNT.h`. Both encoders constructed with `inverted=true`. `CELL_TICKS=1400` is hand-measured per 180 mm cell. Right ticks are scaled by `RIGHT_ENC_SCALE=1.0135f` via the `rTicks()` wrapper. |
| **IR Sensors** | 4-sensor differential array (SFH4545 narrow 950nm emitters + TEFT4300 phototransistors): LF, L, R, RF. **LF/RF aim ~30° forward-outward** (front detect + look-ahead); **L/R aim perpendicular (~90° sideways)** for true side-wall reads. Differential ambient-subtracted reads in `readIR()`. |
| **IMU / Gyro** | MPU-6500 (SPI) — DLPF=3 (41 Hz BW) to reject PWM harmonics. Integrated Z-axis yaw (`updateYaw()`) for spot turns and forward yaw-hold. |
| **Navigation** | 16×16 flood-fill BFS (`MicromouseMaze.h`); home practice active region is 6×3 with goal `(5,2)`. NVS-persisted walls (namespace `mm26`, key `walls`). |
| **Display & UI** | 0.96" 128×64 SSD1306 OLED (I2C `0x3C`) + single tactile Linear Blue Switch (`BUTTON_1=GPIO42`) + Buzzer (`GPIO40`). Rotary encoder used for OLED menu scrolling and Fast Speed adjustment. |
| **Battery** | 300 mAh 2S LiPo (7.4V nominal). Resistor divider `BAT_VDIV_MULT=3.751f` → 0–100% linear SOC. |

---

## 2. Core Development Principles

1. **Single Translation Unit Architecture:** `build_src_filter = +<main.cpp>` means `src/main.cpp` is the only `.cpp` compiled into `[env:main]`. All project headers in `include/` are text-included exactly once. File-scope `static` variables in headers are safe under this pattern.
2. **No Blocking Delays in Control Loops:** The RUN executor in `main.cpp::loop()` paces its position-PID via `micros()`. Never use `delay()` inside any motion control path.
3. **Encoders via Hardware PCNT:** `MicromouseEncoderPCNT.h` uses the ESP32-S3 PCNT peripheral for 4× quadrature decoding with zero CPU overhead. All right-encoder distance reads MUST go through the `rTicks()` wrapper (`main.cpp`).
4. **Arduino 2.x LEDC PWM (200 Hz):** `MOTOR_PWM_FREQ_HZ = 200` (10-bit PWM, 0–1023). 200 Hz empirically yields maximum breakaway torque on this DRV8833 + N20 chassis.
5. **No Magic Numbers:** Every pin assignment lives in `include/PinConfig.h`; every tuning parameter lives in `include/Tuning.h`.
6. **Automatic Gyro Calibration:** `autoCalGyroBeforeStart()` (in `include/OLED.h`) runs automatically before every Explore or Fast Run leg.
7. **Flash On Every Code Edit:** Standard workflow: after making any code edit, compile and flash immediately using `pio run -e main -t upload`. Post-upload chime script is `tools/notify_upload.py`.

---

## 3. Project File Layout

```
neuromouse26/
├── include/
│   ├── README                      Module inventory & header include map
│   ├── Tuning.h                    Every tunable constant (Sections [A]–[H]). Master knob: BASE_BREAKAWAY_PWM
│   ├── PinConfig.h                 Pin mappings, PWM/IR limits, IR_CAL defaults, wall thresholds
│   ├── IMU.h                       MPU-6500 register stack, bias capture, updateYaw() integration
│   ├── IRSensors.h                 4-sensor IR array, ambient-subtracted reads, EMA filters
│   ├── IRCalibration.h             Per-cm front IR LUT & estimateFrontDistMM()
│   ├── MotionScript.h              RunPhase enums, PhaseStep struct, script[] array, pusher helpers
│   ├── Planner.h                   setupMaze(), senseAndStoreWalls(), buildMoveScript() + straight fusion
│   ├── MicromouseMaze.h            16×16 grid + flood-fill BFS + bestDirectionBiased()
│   ├── MicromouseMotor.h           DRV8833 wrapper (LEDC PWM drive/brake/coast)
│   ├── MicromouseEncoderPCNT.h     ESP32-S3 PCNT 4× quadrature decoder
│   ├── OLED.h                      SSD1306 U8G2 instance, menu, diagnostic screens, auto gyro-cal
│   ├── Persistence.h               ESP32 NVS save/load for walls & fast-run cruise speed
│   ├── Battery.h                   Vbat ADC sampling & 0–100% linear SOC calculation
│   ├── Pose.h                      Robot pose (row, col, heading), mode flags, fastRunCruiseTps
│   ├── MicromouseEncoder.h         Legacy ISR encoder (unused by main firmware)
│   ├── PID.h                       Generic PID struct (unused by main motion stack)
│   └── WifiDebug.h                 Live HTTP telemetry server & web debugger UI (dormant side env)
├── src/
│   └── main.cpp                    Hardware instances, PID controller, setup() & loop() state machine (~700 lines)
├── test/                           Standalone test sketches (one env per file in platformio.ini)
├── tools/
│   └── notify_upload.py            Post-upload audible chime script
├── platformio.ini                  PlatformIO environment definitions
├── LICENSE                         MIT License
├── CONTRIBUTING.md                 Contribution rules
├── CLAUDE.md                       Authoritative agent & project guide (this file)
└── README.md                       Public open-source landing page
```

---

## 4. Build, Upload & Test Commands

```bash
# Production Firmware (Main Robot Stack)
pio run -e main -t upload     # Build and flash main firmware
pio device monitor            # Serial monitor @ 115200 baud

# Standalone Subsystem Test Environments
pio run -e ir-test -t upload       # 4-channel IR diagnostics & front distance LUT
pio run -e encoder-test -t upload  # PCNT tick counts & wheel RPM
pio run -e mpu6500 -t upload       # Gyro bias capture & yaw integration stream
pio run -e ws2812b -t upload       # RGB status LED test
pio run -e wall-follow-pcnt -t upload # Drivetrain reference sketch (no solver)

# Clean Build
pio run -t clean
```

---

## 5. Motion Control & Drivetrain Architecture

Movement is managed via a **phase-script over a position-PID core**. A single move produces a list of `PhaseStep` records (max 8) constructed by `buildMoveScript()` in `include/Planner.h`.

### Phase Types (`RunPhase` in `include/MotionScript.h`)

| Phase | Description & Mechanism |
|---|---|
| `PH_FORWARD` | Position-PID with trapezoidal velocity profiling (`vAccel = sqrt(2·a·xDone)`, `vDecel = sqrt(2·a·xRem)`, cruise at `vCruise`). Signed tick targets (negative = reverse). Steering bias combines IR centering + encoder balance + IMU yaw hold. |
| `PH_SPOT` | Both wheels rotate in opposite directions. Yaw-PID closed on `dy = yawDeg − phaseStartYawDeg`. |
| `PH_PIVOT` | Single-wheel pivot (inner wheel braked, outer wheel drives). Preserved in code but no longer pushed by `buildMoveScript` (R-pivots bumped side walls). |
| `PH_ALIGN_FRONT` | Slow creep until front IR sensors reach calibrated wall distance (`irVal[0] ≈ ALIGN_LF_TARGET`, `irVal[3] ≈ ALIGN_RF_TARGET` at 37.5 mm). Used in dead-end 180° exit sequence. |
| `PH_REVERSE_TO_BACK` | Samples front IR on activation to compute reverse ticks = `−(frontMm + BACKUP_OFFSET_MM) × ticksPerMm`, then reclassifies as `PH_FORWARD`. |

### Steering Bias in Forward Motion

```cpp
pwmL = throttle - bias;
pwmR = throttle + bias;
```

Where `bias` is the sum of:
1. **IMU Yaw Hold:** `-YAW_HOLD_KP * yawDeg` (reset to `0` at every phase boundary so each leg has its own straight reference).
2. **IR Lateral Centering:** Confidence-weighted `cR * (irVal[2] - calR) - cL * (irVal[1] - calL)` using side sensors L (ch 1) & R (ch 2).
3. **Encoder Balance:** `(tL - tR) * BALANCE_KP`.

### Fast-Run Straight-Chain Fusion

In Fast Run mode, `buildMoveScript()` fuses consecutive straight cells into a single `PH_FORWARD` phase. The trapezoidal profile stretches acceleration, cruise, and deceleration over the entire corridor, eliminating per-cell braking.

---

## 6. Navigation & Maze Solver

- **Grid System:** `(row, col)` 16×16 grid (`MicromouseMaze.h`). Active practice region is 6×3 with start `(0,0)` facing North and goal `(5,2)`.
- **Flood-Fill BFS:** `bestDirectionBiased(r, c, heading, &dist)` evaluates distance to goal, preferring straight > left > right > U-turn at equal flood distance, with a +4 penalty for already-visited cells.
- **NVS Persistence:** On reaching the goal cell, `nvsSaveWalls()` writes the 256-byte wall bitmask to ESP32 NVS namespace `"mm26"` under key `"walls"`. Fast Run reloads this map and skips sensing.

---

## 7. State Machine (`src/main.cpp`)

```
IDLE
 ├─ menu Explore     → EXPLORE_THINK (auto gyro-cal first)
 ├─ menu Fast Run    → EXPLORE_THINK (loads NVS walls first; bails if empty)
 ├─ menu Fast Speed  → FAST_SPEED_EDIT (encoder knob adjusts fastRunCruiseTps)
 ├─ menu Enc Test    → ENC_TEST
 ├─ menu IR Test     → IR_TEST
 └─ menu Clear NVS   → IDLE (wipes saved walls)

FAST_SPEED_EDIT → IDLE      (button saves to NVS, returns to menu)
EXPLORE_THINK → RUN         (kicks current move's script)
RUN → EXPLORE_THINK         (script done, pose committed, next move)
EXPLORE_THINK → GOAL        (reached target cell; saves NVS)
EXPLORE_THINK → CRASH       (flood = FLOOD_INFINITY, robot boxed in)

GOAL / CRASH → IDLE         (button press clears LED and returns to menu)
```

---

## 8. Serial Telemetry & Event Logging

Enable serial logging by setting `TELEMETRY = true` in `include/Tuning.h` (section [H]). Always set `TELEMETRY = false` before competition runs to prevent Serial print latency from slowing the 199 Hz PID loop.

Log tags: `[IMU]`, `[GCAL/AUTO]`, `[SENSE]`, `[PLAN]`, `[FAST]`, `[EVENT]`, `--- STEP END ---`, `--- MOVE DONE ---`.

---

## 9. Critical Invariants & Known Rules

- **`frictionZone <= holdBand`:** Required so the robot never stalls in a dead zone between breakaway power and settle detection.
- **`MAX_SCRIPT >= 4`:** Required for the 4-step dead-end exit sequence.
- **`RIGHT_ENC_SCALE = 1.0135f`:** All right-tick math must use `rTicks()`.
- **`CELL_TICKS = 1400`:** Hand-measured per 180 mm cell. Re-measure if tires or wheels change.
- **No `fastFwdRoll`:** End of phase always brakes to a full stop before executing `PH_SPOT` turns to prevent rotational inertia drift.
