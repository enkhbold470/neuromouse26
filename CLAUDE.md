# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

@GEMINI.md

---

## Where to make changes

Domain code is split into header-only modules in `include/`, each text-included once into `src/main.cpp` (single TU). Open the header that owns the subsystem:

| File | Owns |
|---|---|
| `include/Tuning.h` | Every `constexpr` tuning knob (sections [A]–[F], [H]). ★ `BASE_BREAKAWAY_PWM` is the master power knob. |
| `include/IMU.h` | MPU-6500 + `yawDeg`/`gzFilt`/`yawTargetDeg`/`imuReady` + `updateYaw()`. |
| `include/IRSensors.h` | `PAIRS`, `irVal`, EMA centering state, `readIR`/`sampleIR`. |
| `include/Pose.h` | `robotRow/Col/Heading`, mode flags, `fastRunCruiseTps`, `pendingOffsetTicks`. |
| `include/MotionScript.h` | `RunPhase`/`TurnDir` enums, `PhaseStep`, `script[]`, `scriptPush*` helpers. |
| `include/Persistence.h` | NVS save/load (walls + fast-run speed, namespace `mm26`). |
| `include/BLETelemetry.h` | NimBLE GATT server (Nordic UART Service). `bleInit()` in setup; `bleGetCmd()` polled in loop; `bleState/blePos/bleWall/bleMotion/bleCrash/bleMazeDump` called at key events. |
| `include/Planner.h` | `setupMaze`, `senseAndStoreWalls`, `buildMoveScript` (+ fast-run chain fusion). |
| `include/OLED.h` | U8G2 + menu + run + diag screens + `autoCalGyroBeforeStart()`. |
| `src/main.cpp` | Hardware objects, IR-centering PID, `rTicks`/`stopMotors`/`buttonEdge`, `onPhaseActivate`/`phaseEnter`/`scriptKick`, `setup()`, `loop()` state machine. |

- **New tuning knob** → `Tuning.h` only.
- **New screen** → `OLED.h` only.
- **Motion control numerics** → `Tuning.h`. **Motion control behavior** (PID structure, settle logic) → `src/main.cpp` RUN case.
- **New code path reading right ticks** → use `rTicks()`, never `rightEnc.getTicks()` directly.

If you add a second `.cpp` to `[env:main]`, convert file-scope `static` globals in headers to `inline` (C++17).

---

## Build & Flash

```bash
~/.platformio/penv/bin/pio run -e main -t upload     # build + flash production firmware
~/.platformio/penv/bin/pio device monitor             # serial @ 115200
~/.platformio/penv/bin/pio run -e <env>               # build only (no upload)
~/.platformio/penv/bin/pio run -t clean
```

**Convention:** every code change is followed by an upload. The user has standing approval for `pio run -e <env> -t upload` after any code modification — don't ask for confirmation.

### Test environments

| Env | Purpose |
|---|---|
| `main` | Production firmware — PCNT + IMU + IR + flood-fill solver |
| `wall-follow-pcnt` | Drivetrain reference (same script/PID/IMU stack, no flood-fill). Tune knobs here first. |
| `sensor-cal-ble` | Re-capture IR cal → paste output into `PinConfig.h::IR_CAL_*` |
| `encoder-test` | Live tick stream for hand-roll measurement |
| `imu-turn` | Standalone MPU-6500 trapezoidal turn (legacy) |
| `motor-ble`, `ir-test`, `ir-turn-test`, etc. | Older diagnostics for hardware bring-up |

`test/ble-test.cpp`, `test/batt-volt.cpp`, `test/buzzer.cpp`, `test/ws2812b.cpp` have no `[env:*]` block — adapt an existing block if needed.

---

## Explore/Fast-run move cycle

```
EXPLORE_THINK:
  senseAndStoreWalls()                    // F/L/R relative to heading
  if isGoal → nvsSaveWalls() → GOAL
  maze.floodFill()
  bestDir = maze.bestDirectionBiased(...) // straight > L > R > U, +4 penalty visited
  buildMoveScript(bestDir):
    diff==0: FWD(cellTicks + pendingOffset)
    diff==1: SPOT_R 90° + FWD(cellTicks)
    diff==3: SPOT_L 90° + FWD(cellTicks)
    diff==2: ALIGN_FRONT + SPOT_R 180° + FWD(−2 cm) + FWD(1 cell pitch)
    fast-run only: trailing FWD chains fused across consecutive straight cells
  scriptKick() → RUN

RUN → on last-step settle: commit plannedRow/Col/Heading → EXPLORE_THINK
```

`autoCalGyroBeforeStart()` (in `OLED.h`) fires automatically before every Explore/Fast Run: shows "CAL GYRO stay still" → `delay(300)` → 300-sample bias capture → zero `yawDeg`/`yawTargetDeg`. No button action needed.

---

## State machine

```
IDLE
 ├─ Explore    → EXPLORE_THINK (auto gyro-cal)
 ├─ Fast Run   → EXPLORE_THINK (loads NVS walls; bails if empty)
 ├─ Fast Speed → FAST_SPEED_EDIT (encoder knob adjusts fastRunCruiseTps)
 ├─ Enc Test   → ENC_TEST
 ├─ IR Test    → IR_TEST
 └─ Clear NVS  → IDLE

FAST_SPEED_EDIT → IDLE      (button saves NVS)
EXPLORE_THINK   → RUN       (script kicked)
RUN             → EXPLORE_THINK (script done)
EXPLORE_THINK   → GOAL      (at goal; explore saves NVS)
EXPLORE_THINK   → CRASH/BOXED (bestDist == FLOOD_INFINITY; explore tries 180° recovery first)
GOAL / CRASH    → IDLE      (button)
```

---

## Critical invariants

- **`frictionZone ≤ holdBand`** — violating this creates a dead zone where stiction is disabled but settle hasn't triggered; robot stalls.
- **`MAX_SCRIPT ≥ 4`** — the 180° dead-end sequence needs 4 steps. Currently 8.
- **LEDC is Arduino 2.x API** — `ledcSetup(ch, freq, bits)` + `ledcAttachPin(pin, ch)`. Not 3.x's `ledcAttach(pin, freq, bits)`. Migrating to Arduino 3.x requires changing the `platform` line **and** every `ledc*` call together.
- **`yawDeg` resets at every phase boundary** (not just rotational ones). Forward heading-hold depends on this.
- **Encoder polarity** — both PCNT ctors use `inverted=true`; `MOTOR_L_INV=false`/`MOTOR_R_INV=false`. Changing motor orientation means flipping both `MOTOR_*_INV` AND encoder `inverted` together.
- **`MOTOR_SLEEP=GPIO41`** — DRV8833 nSLEEP; `setup()` drives it HIGH. Any new env must replicate `pinMode(MOTOR_SLEEP, OUTPUT); digitalWrite(MOTOR_SLEEP, HIGH);`.
- **Reverse targets use `PH_FORWARD` with negative target** — the PID handles this natively. Don't add a separate reverse phase.
- **`CELL_TICKS=1400` is hand-measured** (rolling 180 mm). Do not derive from `TICKS_PER_REV`. Re-measure if wheels/tires change.
- **`TELEMETRY=true` in `Tuning.h` slows the RUN loop** — set false before competition.
- **`WifiDebug.h` is not compiled into `main`** — don't assume WiFi runs.
- **No `fastFwdRoll`** — fast run brakes at every FWD-settle, exactly like explore. Don't reintroduce continuous-roll; it breaks R-turns when the right motor can't reverse against forward inertia.
- **`BOXED` state** — when `bestDist == FLOOD_INFINITY` in fast run, OLED shows `!! BOXED !!` (maze has no reachable path from current cell); in explore mode a 180° recovery is attempted first.
- **`SIDE_ADAPTIVE=true`** — side IR uses per-run live calibration (`calibrateSideRefs()`) + relative threshold + saturation skip + 5-sample median. Set `false` to revert to legacy fixed threshold.
- **`g_smoothMode`** (in `Pose.h`) — runtime toggle (OLED "Mode: Smooth/Classic") enabling `PH_CURVE` continuous arcs in fast run. Off by default; arc code is gated behind `CURVE_ENABLE` in `Tuning.h [I]`.
- **BLE device name** — robot advertises as `"bromouse"`. Browser connects via `tools/ble-debug.html` using Web Bluetooth (NUS UUIDs). Commands: `EXPLORE`, `FAST`, `STOP`, `DUMP`, `CLEAR_NVS`. Telemetry JSON types: `ST` (state), `POS`, `WALL`, `MOT` (motion, gated 10 Hz), `BAT`, `CRASH`, `MAZE`.
- **Side sensor contamination** — L45/R45 sensors at 45° (PCB-verified from pick-and-place U6=45°, U9=315°). Front wall at 90 mm appears at 127 mm diagonal → phantom side reads. Fix: `senseAndStoreWalls()` skips side re-sensing when front wall confirmed; mid-cell sense (at 50% forward) already ran without contamination.

---

## Hardware files

PCB design exports are in `hardware/` (KiCad / EasyEDA outputs, 2026-06-23):

| File | Contents |
|---|---|
| `Netlist_Schematic1_2026-06-23.tel` | Schematic netlist |
| `Netlist_PCB1_2026-06-23.tel` | PCB netlist |
| `PickAndPlace_PCB1_2026-06-23.csv` | SMT pick-and-place coordinates |
