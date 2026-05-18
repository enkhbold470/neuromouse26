# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

@GEMINI.md

---

## Build & Flash Commands

```bash
pio run                                # build main firmware (env:main)
pio run -t upload                      # build + flash main firmware
pio device monitor                     # serial monitor @ 115200 baud
pio run -t clean

# Test/diagnostic firmwares — each is a self-contained sketch.
# Select via `-e <env>`; flash with `pio run -e <env> -t upload`.
pio run -e sensor-cal                  # IR calibration (serial-driven)
pio run -e sensor-cal-ble              # IR calibration over NimBLE
pio run -e motor-ble                   # BLE-driven motor jog
pio run -e encoder-test                # live L/R tick stream
pio run -e ir-test                     # raw IR readouts
pio run -e wall-follow                 # straight-line wall follower
pio run -e wall-follow-cells           # encoder-counted cell follower
pio run -e imu-turn                    # MPU-6500 trapezoid+PID turn test
pio run -e side-open-turn              # opportunistic side-open pivot test
pio run -e motor-freq                  # PWM frequency sweep
pio run -e velocity-pid                # cascaded velocity-PI standalone
pio run -e velocity-pid-ble            # velocity PID + live BLE telemetry
```

`platformio.ini` selects each via `build_src_filter`. Sources `test/ble-test.cpp`, `test/batt-volt.cpp`, `test/buzzer.cpp`, `test/ws2812b.cpp` have no env — adapt one of the existing `[env:*]` blocks if you need them.

Post-upload sound is emitted by `tools/notify_upload.py` (`extra_scripts` in `[common]`).

---

## Cross-File Architecture

### Calibrated constants live in PinConfig.h
`src/main.cpp` line 6 spells it out: "Calibrated globals live in PinConfig.h". This includes IR cal/thresholds, encoder scale, PWM freq, kV feed-forward, velocity-PID gains, turn-profile params, brake fractions. **Edit `include/PinConfig.h`, not main.cpp**, when tuning. main.cpp only holds maze geometry (`MAZE_ROWS=6`, `MAZE_COLS=3`, `GOAL_ROW=5`, `GOAL_COL=2`) and local-only controller K's inside `driveChain()` / `advanceToCellCenter()` (`POS_KP_PWM=0.04`, `YAW_KP_PWM=4.0`).

### Maze sizing: 16×16 alloc, 6×3 used
`MicromouseMaze` always allocates the full 16×16 grid (`MAZE_SIZE` from PinConfig.h, default `setGoalCentre4()` at classical 7-8 center). `main.cpp::setupMaze()` overrides — closes north/east border at `MAZE_ROWS-1` / `MAZE_COLS-1` and re-points goal to `(5,2)`. Robot only ever occupies bottom-left 6×3 of the 16×16 array.

### Motion stack (RUN state) — cascaded velocity-PI
```
loop() RUN
  ├── driveChain()          ← inner loop: vpidStep() every VPID_LOOP_US (5 ms = 200 Hz)
  │     ├── speed PI    on (vL+vR)/2 → target mm/s              (VPID_LOOP_KP/KI)
  │     ├── straight PI on (curL−curR) tick mismatch → 0        (VPID_STRAIGHT_KP)
  │     ├── feed-forward = target / kV_avg, battery-comp-scaled (KV_L/R, NOMINAL_VBAT)
  │     ├── lateralBias = POS_KP×IR_pos_err + YAW_KP×yaw        (direct PWM)
  │     ├── side-IR look-ahead samples populate NEXT cell walls
  │     └── stops on IR-anchored front wall (≤ CELL_ANCHOR_MM=50 mm)
  │                       OR encoder ≥ TICKS_PER_CELL
  │                       OR crash brake (≤ CRASH_BRAKE_MM=30 mm)
  │   then: senseWallsFrontOnly() + applyLookaheadSides() + floodFill() +
  │         bestDirectionBiased() → continue straight or return
  ├── (on direction change) advanceToCellCenter()  ← half-cell to pivot point
  └── rotateToHeading()
        └── ensureFrontClearance() → doTurn() (trapezoid ω + KFF + PD on yaw)
```
**`driveChain()` is the only function that advances `robotRow`/`robotCol`**. It does so at each cell-boundary stop. Heading is updated by `rotateToHeading()` only.

### Turns are MPU-6500 gyro-integrated, not encoder-based
`doTurn()` runs a trapezoidal angular-velocity profile (`TURN_PEAK_OMEGA_DPS=360`, `TURN_ACCEL_DPS2=1800`) with feed-forward (`TURN_KFF_PWM_PER_DPS=1.0`) and PD on integrated yaw (`TURN_KP_PWM_PER_DEG=12.0`, `TURN_KD_PWM_PER_DPS=0.4`). Stiction-floor `TURN_MIN_HOLD_PWM=260` applies during hold phase only — never during ramp. `turnAround()` chains two `−90°` turns, carrying first-turn residual into the second target so total is exactly `−180°`. `quickGyroRecal()` re-zeros bias before every turn (~80 ms standstill samples).

### IR side-wall sensing is look-ahead, not stop-time
The 4 IR sensors point ~30° forward-outward, so at a cell-center stop they're aimed into cell N+2 — wrong cell. `driveChain()` therefore:
- Reads side walls **during** the approach (after 17% into the cell, while sensor cone covers cell N+1's sides), accumulating `sideLook.{Lhigh, Llow, Rhigh, Rlow}`.
- At cell boundary, `applyLookaheadSides()` writes those walls into the just-entered cell. Requires ≥ `MIN_LOOKAHEAD_SAMPLES=3` valid samples (otherwise leaves prior state).
- Rejects samples taken at `frontMM < SIDE_POISON_MM=60`: at close front-wall range, the side sensors' forward-bias cone catches the front wall and would record a phantom side wall.

Front wall is sampled at cell-center via `senseWallsFrontOnly()`.

### Right-encoder scaling lives in `rTicks()`
`RIGHT_ENC_SCALE=1.0135f` is active (re-enabled 2026-05-17 after auto-cal). All distance math in main.cpp uses `rTicks()` (which applies the scale), never `rightEnc.getTicks()` directly. If you add new code consuming right-encoder ticks for distance, use `rTicks()`.

### `TICKS_PER_REV=205` is empirical, not theoretical
N20 14 PPR × 1:30 gear = ~420 raw ticks/rev, but the rising-edge ISR with 200 µs filter loses half at running speed → effective 205. `TICKS_PER_CELL=350` is tape-validated, **not** computed from `MM_PER_TICK`. Do not "fix" either value to a theoretical number — distance math will break.

### IR calibration: compile-time defaults, runtime overrides
`IR_CAL_LF/L/R/RF` in PinConfig.h are baked-in. main.cpp copies them into RAM vars `calLF, calL, calR, calRF` at boot. The on-device menu and `test/sensor-cal-ble.cpp` can rewrite the RAM copies, but the compile-time defaults are restored on every boot (no NVS persistence for IR cal). Re-capture procedure: place robot in dead-end, run "Cal IR", copy printed values back into PinConfig.h, reflash.

### Wall persistence in NVS
`Preferences` namespace `"mm26"`, key `"walls"` holds the 256-byte `walls[][]` bitmask grid. Menu items:
- **START** — fresh run, walls reset to border-only. On reaching GOAL, button-press from goal screen saves walls to NVS.
- **FAST (load)** — loads NVS walls and re-floods, runs without explore overhead. Bails if no save exists.
- **Clear Save** — `prefs.remove("walls")`.

### LEDC API is Arduino 2.x style (espressif32 platform)
`MicromouseMotor` uses `ledcSetup(ch, freq, bits)` + `ledcAttachPin(pin, ch)`, not the Arduino 3.x `ledcAttach(pin, freq, bits)` API. `platformio.ini` uses `platform = espressif32` (stock), not `pioarduino`. If you migrate to ESP-IDF 5.x / Arduino 3.x, both the platform line **and** every `ledc*` call must change together.

### WifiDebug.h exists, is not built
`include/WifiDebug.h` (~500 lines, full HTTP debug server) is not `#include`d in current `main.cpp`. main.cpp's includes are only `PinConfig.h`, `IRCalibration.h`, `MicromouseMotor.h`, `MicromouseEncoder.h`, `MicromouseMaze.h`. Do not assume WiFi runs.

### FSM states (main.cpp:797)
```
IDLE → (button on menu item) → TEST_MOTOR | TEST_ENC | TEST_IR | RUN
RUN  → GOAL (success) | CRASH (turn failed to converge, or stop reasons)
GOAL → IDLE (button) — save walls before leaving
CRASH→ IDLE (button)
```
No automatic STOP / RETURN_HOME state. CRASH is set when `doTurn()` reports failure (final error > 2× deadband) so the main loop won't issue a `driveChain()` against unreliable heading.

### Battery voltage feeds turn + drive feed-forward
`readVbat()` (ADC on `BAT_V_SENSE=GPIO5`, divider `BAT_VDIV_MULT=3.751`) is read once at the start of each `doTurn()` and each `driveChain()` / `advanceToCellCenter()` to set `vScale = NOMINAL_VBAT / vbat`. PWM feed-forward is scaled by `vScale` so target angular and linear velocities stay constant as the pack sags. PID gains are not scaled.
