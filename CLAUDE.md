# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

@GEMINI.md

---

## Build & Flash Commands

```bash
pio run                          # build main firmware
pio run -t upload                # build + flash main firmware
pio device monitor               # serial monitor @ 115200 baud
pio run -e sensor-cal -t upload  # flash IR calibration test
pio run -e encoder-test -t upload
pio run -e ir-test -t upload
pio run -e imu-turn -t upload
pio run -e wall-follow -t upload
pio run -e motor-ble -t upload
pio run -t clean
```

Each `test/` file is a self-contained firmware — select via `-e <env>`. Environments are defined in `platformio.ini`.

---

## Cross-File Architecture

### Constants: PinConfig.h vs main.cpp (conflict)
`PinConfig.h` defines IR thresholds (`L_THRESH`, `R_THRESH`, `LF_THRESH`, `RF_THRESH = 450`) and PID gains (`WALL_KP`, `ENC_KP`, etc.) that are **never used**. `main.cpp` defines its own local values:
- `WALL_SIDE_THRESH = 1000`, `WALL_FRONT_THRESH = 1500` (main.cpp line 85–86)
- `CENTER_KP = 0.12`, `CENTER_KI = 0.0`, `CENTER_KD = 0.03` (main.cpp line 94–96)

When editing thresholds or PID gains, **edit main.cpp**, not PinConfig.h.

### Maze sizing: MAZE_SIZE=16 vs 3×6 practice config
`MicromouseMaze` allocates a full 16×16 grid (`MAZE_SIZE` from PinConfig.h). `main.cpp` configures a 3×6 practice maze via `MAZE_ROWS=6`, `MAZE_COLS=3`, `GOAL_ROW=5`, `GOAL_COL=2`. Border walls are set in `setupMaze()` to match the 3×6 footprint. The robot only ever occupies the bottom-left 6×3 region of the 16×16 array.

### Motion call chain (RUN state)
```
loop() RUN
  ├── senseWalls() + floodFill() + bestDirectionBiased()
  ├── if turn needed → advanceToCellCenter()  (half-cell forward to pivot at cell center)
  ├── rotateToHeading()
  │     └── ensureFrontClearance() → doTurn()
  └── driveChain()  ← chains multiple cells, updates robotRow/Col, returns on direction change
```
`driveChain()` is the only function that advances `robotRow`/`robotCol`. It does so at each cell-boundary crossing internally.

### Gyro turn vs encoder turn
Turns use **MPU-6500 gyro yaw integration** (`doTurn()`), not encoder tick counts. `TICKS_PER_90`, `TURN_TICKS_90_L/R` in PinConfig.h are unused legacy constants. The encoder-based turn path was replaced.

### RIGHT_ENC_SCALE is disabled
`RIGHT_ENC_SCALE = 1.0f` — compensation is off. L/R encoders were empirically equalized at `TICKS_PER_REV=205`. Do not set `TICKS_PER_REV=210` or `420` — breaks distance math.

### IR calibration at runtime
Cal values (`calLF`, `calL`, `calR`, `calRF`) are runtime variables in main.cpp (not compile-time constants). Defaults are hardcoded (3300/1800/1800/2500). Robot menu → "Cal IR" → place in dead-end → press button to capture live readings. Cal values feed `MID_BRAKE_FRAC` and `TURN_CLEAR_FRAC` checks as well as the IR centering bias in `driveChain()`.

### WifiDebug.h exists but is not active
`include/WifiDebug.h` has a full HTTP debug server. It is not `#include`d in current `main.cpp`. Do not assume it runs unless explicitly re-enabled.
