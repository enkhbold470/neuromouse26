# Micromouse26 — ESP32-S3 Firmware

## Project Overview
Micromouse26 is an autonomous maze-solving robot project based on the **ESP32-S3** microcontroller. The firmware is built using the **Arduino framework** within the **PlatformIO** ecosystem. It implements a full control stack, including motor driving, encoder feedback, IR wall sensing, PID-based motion control, and a flood-fill (BFS) maze solver.

### Key Technologies
- **MCU:** Universal Portability (Currently ESP32-S3 but written in standard Arduino C++)
- **Framework:** Arduino / PlatformIO
- **Motor Control:** LEDC 10-bit PWM (0-1023 scale) driving DRV8833
- **Feedback:** Single-channel encoders (rising edge only, direction from motor command)
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
- **`MicromouseMotor`**: Wrapper for DRV8833 using universal `analogWrite()`. Extremely lean, no mapping math.
- **`MicromouseEncoder`**: Interrupt-driven tick counter for single-channel encoders. Supports direction detection if B-channel is available.
- **`MicromouseMaze`**: Implements up to 16×16 grid and flood-fill. Default practice maze is 3×6 (configurable via web UI). `bestDirectionBiased` prefers straight over turns.
- **`WifiDebug`** (`include/WifiDebug.h`): WiFi HTTP debug server (port 80). Serves live sensor dashboard, real-time maze canvas, and config form. All tuning constants in `TuningConfig` struct are editable at runtime without reflash. Config POST rebuilds PIDs and maze immediately.
- **`PID`**: Generic PID controller used for both wall-centering (`wallPid`) and encoder-matching (`encPid`). Uses `micros()` for accurate `dt` calculation and features anti-windup clamping.

### Movement Logic
- **Forward Drive (`moveCells`):** per-motor independent stop — each motor coasts when its own tick count hits target. PWM decel ramp over last `DECEL_TICKS=200` ticks (100mm) from `DRIVE_PWM` → `DRIVE_PWM_MIN=100`. Balance P correction (`BALANCE_KP=3`) only while both motors still running. No brake — pure coast stop.
- **Stop accuracy:** at DRIVE_PWM=450, exit speed ~112mm/s → coast ~23mm residual. Tune `DRIVE_PWM_MIN` toward 80 to reduce further.
- **Right encoder scaling:** `rTicks() = rightEnc.getTicks() * (410.0/413.0)` — compensates hardware tick-count difference (calibrated: 1-rev test L=410, R=413).
- **Pivot Turns:** 90-degree turns use encoder counts (`TICKS_PER_90`), 2s timeout. `TURN_PWM=380`.
- **Explore Loop:** Sense Walls → Update Maze → Re-flood → Decide Direction → Move.
- **Default maze:** 3×6 practice maze (6 rows, 3 cols). Goal at (5,1). Configurable via web UI.

### Hardware Constants (Adjust in `include/PinConfig.h`)
- `WHEEL_DIAMETER`: 33.4mm
- `TICKS_PER_REV`: 210.0f — empirically calibrated for single-channel rising-edge ISR at running speed (motor shaft encoder ~14 PPR × 1:30 gear = ~420 raw, but 200µs noise filter and ISR behavior yield effective 210 at speed)
- `TICKS_PER_CELL`: 360 (calculated: 180mm / (π×33.4/210) — validated matches physical 1-cell travel)
- `WHEEL_TRACK_MM`: 74.0mm
- PWM is 10-bit (0-1023). `MOTOR_PWM_MAX` = 1023.

---

## Development Conventions

### Coding Style
- **Ruthless MVP:** The code is stripped of all non-essential bloat (no FastLED, no IMU calculus). It relies on standard Arduino functions (`analogWrite`, `tone`) so it can run on virtually any chip (AVR, STM32, ESP32).
- **Hardware Abstraction:** Hardware-specific code is encapsulated in classes (`MicromouseMotor`, `MicromouseEncoder`).

### Testing
- **Standalone Tests:** The `test/` directory contains numerous standalone `.cpp` files for validating individual subsystems (e.g., `ir-test.cpp`, `mpu6500.cpp`, `ws2812b.cpp`).
- **Validation Workflow:** Before running a full maze, it is recommended to validate hardware using the scripts in `test/`.

### Known Issues & Risks (as of 2026-05-09)
- **Blocking Loops:** Motion functions like `moveCells()` are blocking. WiFi debug runs on a separate FreeRTOS task (Core 0) to avoid starvation.
- **Baud Rate:** Serial prints inside move loop every 150ms — disable `bleSend` log lines during speed runs.
- **IR thresholds fixed 2026-05-07:** Previous thresholds (LF/RF=1500, L45/R45=650) exceeded maximum possible wall readings (~574) — wall detection was completely non-functional. Fixed to 50. Always validate thresholds against actual `irRead()` output before testing.
- **Encoder effective TICKS_PER_REV:** Physical encoder is ~14 PPR × 30 gear = ~420 raw ticks/rev. ISR with 200µs noise filter yields effective 210 at running speed. Do NOT change `TICKS_PER_REV` to 420 — breaks distance calculation. Value 210 is empirically validated.
- **Coast overshoot:** Without brake, residual overshoot ~20-35mm at `DRIVE_PWM=450`, `DRIVE_PWM_MIN=100`. Physics limit: N20 1:30 at 7.4V, 140g robot, μ≈0.028. Tune `DRIVE_PWM_MIN` toward 80 if motor still runs; below ~70 risks stall mid-ramp.
- **Right encoder offset:** Right encoder reads ~0.7% more ticks than left over same distance. Compensated by `RIGHT_ENC_SCALE = 410.0/413.0` applied via `rTicks()` wrapper. Recalibrate by spinning each wheel one full revolution and measuring counts.
- **Encoder direction:** `driveEncoder()` / `moveCells()` uses `abs(getTicks())` so works regardless of pinB wiring — if ticks stay near 0, check pinA interrupt wiring and encoder power.

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
| **Framework** | Arduino on PlatformIO (`pioarduino` fork for ESP-IDF 5.x support) |
| **Motor Driver** | DRV8833 dual H-bridge — one driver per motor (IN1/IN2 per channel, PWM on both pins for speed + brake) |
| **Motors** | N20 brushed DC gear motors — **1:30 gear ratio, 500 RPM @ 6V**, running on 2S LiPo (7.4V nominal) |
| **Encoders** | Single-channel magnetic encoders on motor shaft (~14 PPR at motor shaft × 30 = ~420 raw ticks/output-rev). ISR counts rising edge only. Effective `TICKS_PER_REV=210` at running speed. Right encoder scaled by `410/413` to equalize L/R. |
| **IR Sensors** | 4-sensor array: LF, L45, R45, RF — analog reads for wall detection and cell centering |
| **Navigation** | 16×16 flood-fill BFS algorithm |
| **LEDs** | WS2812B RGB via FastLED |
| **UI** | Single tactile button + buzzer |

***

## Core Principles

**Always follow these rules in every response:**

1. **No blocking delays** — use `millis()` or hardware timers for timing. `delay()` is forbidden except in setup/debug one-liners.
2. **ISR discipline** — encoder ISRs are `IRAM_ATTR`. Shared variables between ISR and main loop are `volatile`. No Serial or heap allocation inside ISRs.
3. **PWM via LEDC** — ESP32-S3 has no analogWrite(); use `ledcAttach()` / `ledcWrite()` (ESP-IDF 5.x Arduino API). Default: 20 kHz, 10-bit resolution.
4. **PlatformIO `platformio.ini` first** — always include the relevant config block when introducing a new library or build flag.
5. **Motor control** — DRV8833 fast decay = both INs driven (one HIGH, one PWM). Slow decay = one IN HIGH, one PWM. State this when writing motor code.
6. **No magic numbers** — every pin, threshold, and constant must be a `#define` or `constexpr` at the top of the file.
7. **Prefer FreeRTOS tasks over `loop()`** — use `xTaskCreatePinnedToCore()` for sensor reads (Core 0) and navigation/control (Core 1).
8. **Interrupt pins** — on ESP32-S3, any GPIO supports interrupts. Prefer GPIOs not shared with JTAG (GPIO 39-42) unless JTAG is disabled.

***

## PlatformIO Configuration

Use this as the base `platformio.ini` for all ESP32-S3 work:

```ini
[env:esp32-s3]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/51.03.07/platform-espressif32.zip
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
board_build.flash_mode = qio
board_build.psram_type = opi
board_upload.flash_size = 4MB
build_flags =
    -DCORE_DEBUG_LEVEL=0
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
lib_deps =
    fastled/FastLED
    ; add others here
```

> **Note:** Use `pioarduino` (the community fork) for ESP-IDF 5.x / Arduino 3.x compatibility. The official PlatformIO Espressif platform is stuck on Arduino 2.x.

***

## Motor & Encoder Patterns

### DRV8833 PWM (Fast Decay, preferred for responsive control)
```cpp
// Fast decay: IN1 = direction, IN2 = PWM (or vice versa)
// ledcAttach(pin, freq, resolution) — ESP-IDF 5.x API
ledcAttach(IN1_LEFT, 20000, 10);
ledcAttach(IN2_LEFT, 20000, 10);

void setMotor(uint8_t in1, uint8_t in2, int16_t speed) {
    // speed: -1023 to +1023
    if (speed >= 0) {
        ledcWrite(in1, speed);
        ledcWrite(in2, 0);
    } else {
        ledcWrite(in1, 0);
        ledcWrite(in2, -speed);
    }
}
```

### Encoder ISR (single-channel, direction from motor command)
```cpp
volatile int32_t leftTicks = 0;
volatile int32_t rightTicks = 0;
extern int8_t leftDir;   // set by motor control: +1 or -1
extern int8_t rightDir;

void IRAM_ATTR leftEncoderISR()  { leftTicks  += leftDir; }
void IRAM_ATTR rightEncoderISR() { rightTicks += rightDir; }

// setup():
attachInterrupt(ENC_LEFT_PIN,  leftEncoderISR,  RISING);
attachInterrupt(ENC_RIGHT_PIN, rightEncoderISR, RISING);
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

`irRead()` uses **differential (ambient-subtracted)** reads — emitter OFF → read ambient, emitter ON → read lit, return `max(0, lit - amb)`. Result: no-wall ≈ 0–10, wall present ≈ 400–550. Threshold of 50 gives clean separation with no false positives possible.

**Calibrated values (2026-05-07, dead-end centered, all 4 walls present):**

| Sensor | No-wall | Wall avg | Wall min | Wall max | CENTER (PID) |
|--------|---------|----------|----------|----------|--------------|
| LF     | <10     | 552      | 512      | 574      | —            |
| L45    | <10     | 421      | 397      | 491      | 421          |
| R45    | <10     | 504      | 494      | 508      | 504          |
| RF     | <10     | 530      | 526      | 534      | —            |

```cpp
// irRead() = differential (ambient-subtracted), so no-wall ~0, wall ~400-550
// Single threshold works for all sensors — huge gap between states
#define WALL_THRESH  50

bool wallFront() { return irRead(IR_LF) > WALL_THRESH || irRead(IR_RF) > WALL_THRESH; }
bool wallLeft()  { return irRead(IR_L45) > WALL_THRESH; }
bool wallRight() { return irRead(IR_R45) > WALL_THRESH; }

// Wall-follow PID centering error (positive = drifted right)
// Uses calibrated centers: L45_CENTER=421, R45_CENTER=504
float centeringError() {
    return (float)(irRead(IR_L45) - L45_CENTER) - (float)(irRead(IR_R45) - R45_CENTER);
}
```

***

## Flood-Fill BFS (16×16)

```cpp
constexpr uint8_t MAZE_SIZE = 16;
uint8_t floodMap[MAZE_SIZE][MAZE_SIZE];
uint8_t walls[MAZE_SIZE][MAZE_SIZE];  // bitmask: bit0=N, bit1=E, bit2=S, bit3=W

void floodFill(uint8_t goalX, uint8_t goalY) {
    memset(floodMap, 255, sizeof(floodMap));
    floodMap[goalY][goalX] = 0;
    // BFS queue — static array to avoid heap allocation
    uint8_t queue[MAZE_SIZE * MAZE_SIZE][2];
    uint16_t head = 0, tail = 0;
    queue[tail][0] = goalX; queue[tail][1] = goalY; tail++;
    while (head != tail) {
        uint8_t x = queue[head][0], y = queue[head][1];
        head++;
        // ... BFS neighbor expansion ...
    }
}
```