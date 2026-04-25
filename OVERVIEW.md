# Micromouse26 — Firmware Overview

> ESP32-S3 · PlatformIO + Arduino framework · Verbose test build

---

## Project Structure

```
micromouse26/
├── include/
│   ├── PinConfig.h
│   ├── MicromouseMotor.h
│   ├── MicromouseEncoder.h
│   ├── MicromouseIMU.h
│   ├── MicromouseIR.h
│   └── MicromouseMaze.h
├── src/
│   └── main.cpp
├── platformio.ini
└── OVERVIEW.md  ← this file
```

---

## File Descriptions

### `include/PinConfig.h`
All GPIO pin assignments, physical constants (wheel diameter, encoder ticks/rev), and I2C pin definitions in one place. Change hardware wiring here — nowhere else.

---

### `include/MicromouseMotor.h`
DRV8833-driven DC motor wrapper.

| Method | Description |
|---|---|
| `begin()` | Sets up LEDC PWM channels |
| `drive(int pwm)` | Forward (positive) or reverse (negative), range −1023 … +1023 |
| `brake()` | Both inputs HIGH — active brake |
| `coast()` | Both inputs LOW — freewheeling |

**Verbose logging:** every `drive()`, `brake()`, and `coast()` call prints the label ("LEFT"/"RIGHT"), direction string, and raw PWM value.

Constructor signature:
```cpp
MicromouseMotor(uint8_t in1, uint8_t in2,
                uint8_t ch1, uint8_t ch2,
                const char* label);
```

---

### `include/MicromouseEncoder.h`
Single-channel quadrature encoder (RISING-edge count only).

| Method | Description |
|---|---|
| `begin(isr_fn)` | Attaches ISR on channel A |
| `handleInterrupt()` | Called from ISR — increments tick counter |
| `getTicks()` | Thread-safe read (interrupt guard) |
| `reset()` | Zero the tick counter |
| `printStatus()` | Prints label + current tick count |

**Verbose logging:** `getTicks()` and `reset()` both print current values to Serial.

---

### `include/MicromouseIMU.h`
MPU-6050 gyroscope integration for yaw tracking.

| Method | Description |
|---|---|
| `begin(sda, scl)` | I2C init + WHO_AM_I check + auto-calibrate |
| `calibrate()` | Averages 500 samples at rest to find gyro Z bias |
| `update()` | Integrates gyro Z → `_yaw` (call every loop) |
| `resetYaw()` | Zero the yaw accumulator |
| `getYaw()` | Returns current yaw in degrees |
| `printStatus()` | Prints bias, current yaw, and loop count |

**Verbose logging:** calibration prints a dot every 50 samples; `update()` logs raw reading, bias-corrected value, dt, and accumulated yaw.

---

### `include/MicromouseIR.h`
6-sensor IR array with ambient-light cancellation.

Sensor layout assumed:
```
   [FL]  [F]  [FR]
   [L]        [R]
```
(FL = front-left diagonal, F = front centre, FR = front-right diagonal, L = left side, R = right side; plus one spare channel.)

| Method | Description |
|---|---|
| `begin()` | Prints all GPIO assignments |
| `update()` | Reads ambient then lit value for every sensor; prints `ambient`, `lit`, `diff` |
| `wallFront()` | Returns `true` if front sensor exceeds `IR_THRESH_FRONT` |
| `wallLeft()` | Returns `true` if left sensor exceeds `IR_THRESH_SIDE` |
| `wallRight()` | Returns `true` if right sensor exceeds `IR_THRESH_SIDE` |
| `printStatus()` | Prints all 6 diff values + combined front/left/right boolean |

**Thresholds to calibrate** (in `MicromouseIR.h`):

| Constant | Default | Meaning |
|---|---|---|
| `IR_THRESH_FRONT` | 800 | Front wall detected |
| `IR_THRESH_SIDE` | 500 | Side wall detected |
| `IR_THRESH_DIAG` | 600 | Diagonal sensor wall detected |

---

### `include/MicromouseMaze.h`
16×16 flood-fill maze solver.

#### Data layout
```cpp
uint8_t walls  [MAZE_SIZE][MAZE_SIZE]; // 4-bit wall mask per cell (N/E/S/W)
uint8_t flood  [MAZE_SIZE][MAZE_SIZE]; // BFS distance to goal
bool    visited[MAZE_SIZE][MAZE_SIZE]; // has robot entered this cell?
```

Wall bit masks:
```cpp
WALL_N = 0x01   WALL_E = 0x02   WALL_S = 0x04   WALL_W = 0x08
```

#### Key methods

| Method | Description |
|---|---|
| `reset()` | Clears all walls, marks only the outer border walls |
| `floodFill()` | BFS from goal cells outward; fills `flood[][]` |
| `setWall(r,c,dir,present)` | Sets wall bit and mirrors to the adjacent cell |
| `hasWall(r,c,dir)` | Reads wall bit |
| `bestDirection(r,c,&dist)` | Returns `AbsDir` with the lowest flood neighbour value |
| `isGoal(r,c)` | True if the cell is one of the goal cells |
| `setGoalSingle(r,c)` | Override goal to a single cell (useful for testing) |
| `printFlood()` | 16×16 grid of flood distances |
| `printWalls()` | 16×16 grid of wall hex bytes |
| `printVisited()` | 16×16 grid of `V`/`.` visited marks |

#### Direction enum
```cpp
enum AbsDir : uint8_t { DIR_NORTH=0, DIR_EAST=1, DIR_SOUTH=2, DIR_WEST=3 };

// Row/col deltas for each direction:
// DIR_NORTH: dr=+1, dc= 0
// DIR_EAST:  dr= 0, dc=+1
// DIR_SOUTH: dr=-1, dc= 0
// DIR_WEST:  dr= 0, dc=-1
```

---

### `src/main.cpp`
Full robot firmware — 701 lines.

---

## Robot FSM

```
         ┌──────────┐
  reset  │          │
 ──────► │   IDLE   │ ◄─────────────────────────────┐
         │          │  button press (from STOP)      │
         └────┬─────┘                                │
    button    │                                      │
    press     ▼                                      │
         ┌──────────┐                                │
         │CALIBRATE │  IMU bias calibration,         │
         │          │  encoder + maze reset           │
         └────┬─────┘                                │
    2 s delay │                                      │
              ▼                                      │
         ┌──────────┐   goal cell                    │
         │  EXPLORE │ ─────────────► ┌─────────────┐ │
         │  (loop)  │                │GOAL_REACHED │ │
         └──────────┘                └──────┬──────┘ │
                                            │        │
                                    auto    ▼        │
                                     ┌──────────┐    │
                                     │   STOP   │ ───┘
                                     │          │  button press
                                     └──────────┘
```

### State details

| State | Entry condition | What it does |
|---|---|---|
| **IDLE** | Power-on / restart | Waits for falling edge on BUTTON_1 |
| **CALIBRATE** | Button press | Re-calibrates IMU gyro bias (500-sample average), resets encoders, resets maze to border-walls-only, re-floods |
| **EXPLORE** | After 2 s delay | Per-cell loop (see below) |
| **GOAL_REACHED** | Robot enters a goal cell | Brakes motors, beeps buzzer 3×, dumps flood + visited maps, transitions to STOP |
| **RETURN_HOME** | *(placeholder)* | Prints TODO, goes to STOP |
| **STOP** | Error or goal reached | Motors coast; prints status every 5 s; button press goes back to IDLE |

---

## Explore Loop (one iteration per cell)

```
1. Mark current cell as visited
2. Check if current cell is a goal cell → GOAL_REACHED
3. senseWallsAndUpdate()
   └─ ir.update() → wallFront/Left/Right → maze.setWall() (auto-mirrors)
4. maze.floodFill()  (BFS re-run with new wall knowledge)
5. Every 10 steps: printFlood() + printVisited()
6. maze.bestDirection() → targetDir (lowest flood neighbour)
7. Compute turn steps = (targetDir − heading + 4) % 4
   └─ 0 → no turn
   └─ 1 → turnRight()
   └─ 2 → turnAround()
   └─ 3 → turnLeft()
8. moveForwardOneCell()
9. Update robotRow / robotCol
10. stepCount++  → printRobotPosition()
```

---

## Movement Functions

### `moveForwardOneCell()`
- Resets encoders and IMU yaw
- 50 Hz PID loop until both encoders reach `TICKS_PER_CELL`
- Ramps speed down to 50% in the last 20% of the cell
- IMU yaw correction: `leftMotor.drive(outL − yaw×0.5)` / `rightMotor.drive(outR + yaw×0.5)`
- 5-second safety timeout
- Brakes then coasts on completion

### `turnRight()` / `turnLeft()`
- Resets IMU yaw
- Spins motors in opposite directions at `PID_TARGET_TURN` PWM
- Polls `imu.update()` every 2 ms; stops when `|yaw| ≥ 88°`
- 3-second safety timeout

### `turnAround()`
- Two consecutive `turnRight()` calls with 100 ms gap

---

## PID Controller

One `computePID()` instance per motor (independent state structs `pidLeft`, `pidRight`).

```
speed  = (ticks − prevTicks) / dt          (ticks/second)
speed  = 0.5×prevSpeed + 0.5×speed         (low-pass filter, α=0.5)
error  = target − speed
I     += error × dt                        (clamped ±1278 — anti-windup)
D      = (error − prevError) / dt
output = Kp×error + Ki×I + Kd×D
```

| Parameter | Default | Notes |
|---|---|---|
| `Kp` | 1.5 | Raise until motor responds; back off if oscillating |
| `Ki` | 0.8 | Raise slowly after Kp is stable |
| `Kd` | 0.05 | Keep low — encoder quantisation amplifies derivative |
| `PID_TARGET_EXPLORE` | 800 t/s | ≈45% of N20 free-run at 6V |
| `PID_TARGET_TURN` | 400 t/s | Slower for accurate IMU turns |
| PID interval | 20 ms (50 Hz) | Fixed-interval via `micros()` |

---

## Physical Constants

| Constant | Location | Default | What to do |
|---|---|---|---|
| `WHEEL_DIAMETER` | `PinConfig.h` | 32 mm | Measure your wheel |
| `TICKS_PER_REV` | `PinConfig.h` | 210 | Count ticks for one full wheel rotation |
| `CELL_MM` | `main.cpp` | 180.0 mm | Standard competition cell size |
| `TRACK_WIDTH_MM` | `main.cpp` | 74.0 mm | Measure wheel contact patch spacing |
| `IR_THRESH_FRONT` | `MicromouseIR.h` | 800 | Set by placing robot at cell-centre facing a wall |
| `IR_THRESH_SIDE` | `MicromouseIR.h` | 500 | Set by placing robot at cell-centre with side wall |
| `IR_THRESH_DIAG` | `MicromouseIR.h` | 600 | Set by placing robot at cell-centre with diagonal wall |

Derived constants (computed automatically at compile time):
```
MM_PER_TICK    = (π × WHEEL_DIAMETER) / TICKS_PER_REV
TICKS_PER_CELL = CELL_MM / MM_PER_TICK            ≈ 376 ticks
TICKS_PER_90°  = (π × TRACK_WIDTH / 4) / MM_PER_TICK
```

---

## Serial Telemetry

Every 100 ms the firmware prints two lines:
```
[TELE] step=12  pos=(3,2)  heading=NORTH  yaw=0.3°  Lticks=0  Rticks=0
[TELE] flood_here=11  best_dir=NORTH(10)  state=EXPLORE
```

Other tagged prefixes:
| Tag | Source |
|---|---|
| `[INIT]` | `setup()` hardware initialisation |
| `[FSM]` | State transitions |
| `[IDLE]` | Button detection |
| `[CALIBRATE]` | IMU gyro calibration |
| `[EXPLORE]` | Per-cell navigation logic |
| `[SENSE]` | IR reading + wall registration |
| `[MOVE]` | `moveForwardOneCell()` progress |
| `[TURN-R]` / `[TURN-L]` | Turn yaw feedback loop |
| `[PID]` | Per-motor PID values (every 20 ms during movement) |
| `[GOAL]` | Goal-reached actions |
| `[STOP]` | Stop-state heartbeat |
| `[POS]` | Verbose position dump after each cell |
| `[IMU]` | IMU integration values |
| `[IR]` | Raw IR sensor values |
| `[MOTOR]` | Motor drive commands |
| `[ENC]` | Encoder tick reads |

---

## Quick-Start Checklist

1. **Wire up hardware** and fill in `PinConfig.h` with your actual GPIO numbers.
2. **Build and flash** — open Serial Monitor at 115200 baud.
3. **Check `[INIT]`** lines — confirm IMU WHO_AM_I passes, all pins print.
4. **Place robot on a flat surface**, press BUTTON_1 — watch `[CALIBRATE]` complete.
5. **Put robot at maze start cell (0,0) facing NORTH**, press button again.
6. **Watch `[EXPLORE]`** — verify IR walls match physical walls in `[SENSE]` lines.
7. **Tune IR thresholds** until wall detection is reliable.
8. **Tune `TRACK_WIDTH_MM`** until `turnRight()` consistently reaches 88–92°.
9. **Tune PID** — start with `Kp` only (set Ki=0, Kd=0); add Ki once straight-line is stable.

---

## Future Work

- `STATE_RETURN_HOME` — re-flood with goal at (0,0), navigate back
- Speed-run mode — second pass using known wall map at full speed
- Diagonal movement
- Battery voltage monitoring via ADC
- OLED display for status without Serial
