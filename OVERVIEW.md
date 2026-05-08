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

---

## Code Review — Bugs & Risks Found (2026-04-27)

> This section records findings from a full static analysis of all source files.
> Fix the **Critical Bugs** before running the robot on a real maze.

### Verdict
The architecture is sound (FSM + flood-fill BFS + PID + IMU yaw turns), but
two bugs make the forward-drive yaw correction effectively dead code, one bug
corrupts PID output for both motors simultaneously, and one bug can break the
BFS queue on complex mazes.

---

### Critical Bugs

#### BUG-1 — Shared `static` inside `computePID()` corrupts both motors
**File:** `src/main.cpp` — `computePID()` function

```cpp
static float filteredSpeed = 0.0f;  // NOTE: this static is shared
```

`filteredSpeed` is `static`, so it is one variable shared across every call
to `computePID()`. Left motor and right motor both call this function. The
left motor's filtered speed bleeds into the right motor's PID on the very
next call and vice versa. Both motors receive wrong proportional and
derivative terms whenever they differ in speed (i.e., almost always).

**Fix:** Add `filteredSpeed` as a field to `PIDState`:
```cpp
struct PIDState {
    float integral      = 0.0f;
    float prevError     = 0.0f;
    long  prevTicks     = 0;
    float filteredSpeed = 0.0f;  // <-- add this
};
```
Then replace the static line inside `computePID()` with `pid.filteredSpeed = ...`.

---

#### BUG-2 — `imu.update()` never called inside `moveForwardOneCell()` — yaw correction is dead code
**File:** `src/main.cpp` — `moveForwardOneCell()` while-loop

The yaw correction code:
```cpp
float yaw = imu.getYaw();
float yawCorr = yaw * 0.5f;
leftMotor.drive((int)(outL - yawCorr));
rightMotor.drive((int)(outR + yawCorr));
```
`imu.getYaw()` returns `currentYaw`, which only changes when `imu.update()`
is called. `imu.update()` is called at the top of `loop()`, but `loop()` is
blocked for the entire duration of the move. The while-loop inside
`moveForwardOneCell()` never calls `imu.update()`, so `yaw` is always 0.0°
(the value after `imu.resetYaw()` at move start). Yaw correction does nothing.

**Fix:** Call `imu.update()` at the top of the PID while-loop:
```cpp
while (ticksL < TICKS_PER_CELL && ticksR < TICKS_PER_CELL) {
    imu.update();   // <-- add this line
    if (millis() - moveStart > 5000) { ... }
    ...
}
```

---

#### BUG-3 — PID output units do not match `drive()` input units; integral saturates
**File:** `src/main.cpp` — `computePID()` and `moveForwardOneCell()`

`computePID()` works in **ticks/second** (target ≈ 800 t/s). With `Kp=1.5`
the raw output at zero speed is `1.5 × 800 = 1200`, which exceeds the
`drive()` PWM range of `±1023`. `drive()` clamps the value (safe), but the
integrator (`Ki=0.8`) continues winding up well past the useful range. The
anti-windup clamp of `±1278` was chosen to match this accidental overflow
rather than the actual control range, so the integrator provides almost no
benefit during normal operation.

**Fix (minimal):** Scale the PID output before calling `drive()`:
```cpp
const float pwmScale = (float)PWM_MAX / PID_TARGET_EXPLORE; // 1023/800 ≈ 1.28
leftMotor.drive( (int)((outL - yawCorr) * pwmScale));
rightMotor.drive((int)((outR + yawCorr) * pwmScale));
```
Also tighten anti-windup clamp to `±(PWM_MAX / Ki)`.

---

#### BUG-4 — BFS queue in `floodFill()` mixes modulo-wrapped and bare indexing
**File:** `include/MicromouseMaze.h` — `floodFill()`

Goal cells are enqueued without `% MAZE_CELLS`:
```cpp
qRow[tail] = goalRow[i];   // bare index — safe only while tail < 256
qCol[tail] = goalCol[i];
tail++;
```
BFS expansion does use `tail % MAZE_CELLS`. When `tail` (uint16_t) grows
beyond 255, the head/tail comparison `head != tail` can fail to terminate
correctly because the indices are no longer in the same numeric domain.
For the default 4-goal seeding this cannot overflow, but the BFS itself
can enqueue all 256 cells, pushing `tail` to ~260 while `head` is still
below 256 — the loop may then spin far too long or terminate early.

**Fix:** Use `% MAZE_CELLS` consistently for every enqueue and dequeue, or
use a power-of-two ring buffer (size 256, mask 0xFF).

---

### Design Risks (not outright bugs, but will hurt real-world performance)

| # | Location | Issue |
|---|---|---|
| R1 | `turnRight()` / `turnLeft()` | No ramp-down before target angle. At 400 PWM, mechanical overshoot is typically 5–15°. Add a slow zone when `|yaw| > 75°`. |
| R2 | `wallFront()` in `MicromouseIR.h` | Requires **both** front sensors to trigger (`&&`). One dirty or misaligned sensor causes missed front walls and the robot drives into them. Consider `||` or a dedicated single front sensor. |
| R3 | All motion functions | Entire `loop()` is blocked during movement. Top-level `imu.update()` and `printTelemetry()` do not run. All real-time updates must be inlined into motion functions (BUG-2 is a direct consequence of this). |
| R4 | `IR_THRESH_*` in `MicromouseIR.h` | Placeholder values (800/600/500). Must be calibrated for your specific hardware and maze wall material before exploration is reliable. |
| R5 | Everywhere | Every `drive()`, `getTicks()`, PID tick, and IR sample prints to Serial. At 115200 baud this adds milliseconds of blocking UART time per PID cycle, skewing `dt` and risking cell overshoot. Comment out verbose prints before any speed run. |
| R6 | `turnAround()` | Implemented as two `turnRight()` calls. Works because `turnRight()` resets yaw internally. Correct but fragile — document this implicit dependency. |

---

### What is correct and can be trusted

- FSM state transitions and button debounce logic
- Flood-fill BFS algorithm (correct aside from BUG-4 edge case)
- Wall mirroring in `setWall()` — neighbour cell always updated symmetrically
- Outer border wall initialisation in `reset()`
- Encoder direction detection — B-pin sampled on A-pin rising edge
- Differential IR reading — ambient subtraction cancels background IR correctly
- Motor `drive()` minimum-power mapping avoids stall zone
- Turn direction calculation: `(targetDir − heading + 4) % 4`
- Safety timeouts in all motion functions (3 s turn, 5 s forward)
- `imu.resetYaw()` called correctly before each turn and each forward move




Bash(~/.platformio/penv/bin/pio run -e main 2>&1 | tail -8)
  ⎿  Merged 2 ELF sections                                                                            
     Successfully created esp32s3 image.                                                              
     ========================= [SUCCESS] Took 2.72 seconds =========================                  
     … +5 lines (ctrl+o to expand)                                                                    
                                                                                                      
⏺ Clean. Web calibration flow (scroll to bottom of page):
                                                                                                      
  ┌─────────────────────┬──────────────────────────────┬──────────────────────────────────────────┐   
  │        Step         │          What to do          │             What it measures             │
  ├─────────────────────┼──────────────────────────────┼──────────────────────────────────────────┤   
  │ 1 Open Space        │ Robot on table, no walls     │ Ambient no-wall baseline, warns if > 30  │
  ├─────────────────────┼──────────────────────────────┼──────────────────────────────────────────┤
  │ 2 Side Wall Centers │ Robot in corridor, walls     │ Sets L45_CENTER + R45_CENTER             │   
  │                     │ left+right                   │                                          │   
  ├─────────────────────┼──────────────────────────────┼──────────────────────────────────────────┤   
  │ 3 Front Wall        │ Robot facing wall ~5cm       │ Verifies LF/RF sensors work              │   
  │ (optional)          │                              │                                          │
  ├─────────────────────┼──────────────────────────────┼──────────────────────────────────────────┤   
  │ Apply               │ Click button                 │ POSTs centers + auto-computed threshold  │
  │                     │                              │ to /config                               │   
  └─────────────────────┴──────────────────────────────┴──────────────────────────────────────────┘   
   
  Threshold auto-computed as midpoint between max no-wall reading and min wall reading. "Apply" button
   stays disabled until steps 1+2 complete.                 
                                                                                                      
  Right motor: reflash with new 100 PWM diagnostic — if still dead at 100, it's wiring (check IN3/IN4 
  continuity to DRV8833, check MOTOR_R_INV polarity).
                                                                                                      
✻ Crunched for 2m 53s                                       