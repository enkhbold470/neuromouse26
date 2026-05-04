# wall-follow-encoder-count-cell — Development Notes

**File:** `test/wall-follow-encoder-count-cell.cpp`
**Platform:** ESP32-S3 / Arduino framework / PlatformIO env `wall-follow-encoder-count-cell`

---

## What This Test Does

1. Button press starts the robot
2. Robot drives forward using dual PID (IR wall centering + encoder straight-keeping)
3. After every `TICKS_PER_CELL` average encoder ticks (1 cell = 180mm ≈ 360 ticks), cell counter increments
4. After `MAX_CELLS` cells, robot pauses then executes a 90° encoder pivot turn
5. Robot stops. Button press again resets and restarts.

---

## Hardware Interrupts vs Polling

| Subsystem | Mechanism | Notes |
|---|---|---|
| Encoder counting | **Hardware GPIO interrupt** (`IRAM_ATTR`, RISING edge on ENC_A) | Runs in IRAM, never misses ticks |
| Motor PWM | **Hardware LEDC timer** (20 kHz) | Zero CPU overhead during drive |
| PID control loop | Free-running `loop()` polling | Uses `micros()` for accurate `dt` regardless of loop rate |
| IR read | `delayMicroseconds(50)` per sensor | 2 sensors × 50µs = ~100µs blocking per loop → ~10kHz max loop rate |

---

## Tunable Constants

### Straight driving
| Define | Value | What it does |
|---|---|---|
| `BASE_PWM` | 250 | Base motor speed (0–1023) |
| `WALL_KP/KI/KD` | 500 / 50 / 300 | IR wall centering PID |
| `WALL_MAX_CORR` | 400 | Max IR correction applied to motors |
| `ERROR_TRIM` | 0.10 | Shifts setpoint left; increase if robot hugs right wall |
| `ENC_KP/KI/KD` | 9.0 / 0.8 / 0.5 | Encoder differential straight-keeping PID |
| `ENC_MAX_CORR` | 120 | Max encoder correction applied to motors |

### Cell tracking
| Define | Value | What it does |
|---|---|---|
| `TICKS_PER_CELL` | 360 | Encoder ticks = 1 cell (180mm). Math: 180 × 210/(π×33.4) |
| `MAX_CELLS` | 3 | How many cells before turn (currently 3, was 5) |
| `CELL_PAUSE_MS` | 50 | Pause before turn after last cell |

### 90° pivot turn
| Define | Value | What it does |
|---|---|---|
| `WHEEL_TRACK_MM` | 74.0 | Centre-to-centre axle width — measure physically |
| `TICKS_PER_90` | ~116 | Computed: `WHEEL_TRACK_MM × 210 / (4 × 33.4)` |
| `TURN_PWM` | 200 | Motor speed during turn |
| `TURN_DIR` | 1 | `1` = right turn, `-1` = left turn |

---

## Control Architecture

```
loop()
 ├── IR reads (L45, R45)
 ├── Wall centering PID  →  wallCorr
 ├── Encoder diff PID    →  encCorr   (tL - tR within current cell)
 ├── totalCorr = wallCorr + encCorr
 ├── pwmL = BASE_PWM - totalCorr
 ├── pwmR = BASE_PWM + totalCorr
 └── Cell boundary check
      ├── cellCount < MAX_CELLS → resetCellBase(), reset PIDs, continue
      └── cellCount >= MAX_CELLS → pause → turn90(TURN_DIR) → stop
```

### Correction direction convention
- Positive `totalCorr` → left motor slows, right motor speeds → steers left
- Used to correct rightward drift (left encoder ran more ticks than right)

### turn90(int dir)
```
dir =  1 → right: leftMotor(+PWM),  rightMotor(-PWM)  [left fwd, right back]
dir = -1 → left:  leftMotor(-PWM),  rightMotor(+PWM)  [left back, right fwd]
```
Stops when average of both wheel tick contributions reaches `TICKS_PER_90`.

---

## Cell Distance Math

```
wheel circumference = π × 33.4mm = 104.93mm
ticks per mm        = 210 / 104.93 = 2.001 ticks/mm
1 cell (180mm)      = 180 × 2.001 = 360.2 → TICKS_PER_CELL = 360
```

## Turn Angle Math

```
arc per wheel (90°) = π × WHEEL_TRACK_MM / 4
ticks               = arc × ticks_per_mm
                    = (π × 74 / 4) × (210 / (π × 33.4))
                    = 74 × 210 / (4 × 33.4)
                    ≈ 116 ticks
```

---

## Tuning Guide

**Robot drifts right on straight:** increase `ERROR_TRIM` (0.05 steps)
**Robot oscillates side to side:** reduce `WALL_KP` or increase `WALL_KD`
**Robot doesn't hold straight without walls:** increase `ENC_KP`
**Encoder correction fights wall correction:** reduce `ENC_MAX_CORR`
**Turn overshoots:** increase `WHEEL_TRACK_MM` by 2–3mm steps
**Turn undershoots:** decrease `WHEEL_TRACK_MM` by 2–3mm steps
**Turn too fast / imprecise:** lower `TURN_PWM` (try 150)

---

## IMU Turn (Attempted — Reverted)

An IMU-based `turn90()` using `MicromouseIMU` (MPU-6500, gyro Z integration) was attempted.
Approach: P-controller on remaining yaw (`pwm = KP × (90 - |yaw|)`), early brake by `TURN_BRAKE_DEG`.
Result: did not work reliably on hardware. Reverted to encoder-based turn.
Possible causes: IMU Z-axis sign ambiguity, gyro latency during fast pivot, motor vibration noise.

---

## Related Files

| File | Purpose |
|---|---|
| `test/wall-follow-simple.cpp` | Original IR-only wall follow (no encoders, no cell tracking) |
| `test/wall-follow-encoder.cpp` | Encoder + wall follow, continuous (no stop between cells) |
| `include/MicromouseEncoder.h` | Quadrature encoder driver, interrupt-based |
| `include/MicromouseMotor.h` | DRV8833 motor driver, LEDC PWM |
| `include/MicromouseIMU.h` | MPU-6500 gyro driver, yaw integration |
| `include/PinConfig.h` | All GPIO pin assignments + physics constants |
