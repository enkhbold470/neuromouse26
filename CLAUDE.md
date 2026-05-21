# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

@GEMINI.md

---

## File layout (post-refactor 2026-05-20)

Domain code is split into header-only modules in `include/`. Each is text-included once into `src/main.cpp` (single TU; `build_src_filter = +<main.cpp>`). To touch a subsystem, open the matching header. The README in `include/` has the full map; the short version:

| File | Owns |
|---|---|
| `include/Tuning.h` | Every `constexpr` tuning knob (sections [A]–[F], [H]). The ★★★ MAIN POWER KNOB ★★★ is `BASE_BREAKAWAY_PWM`. |
| `include/IMU.h` | MPU-6500 + `yawDeg`/`gzFilt`/`yawTargetDeg`/`imuReady` + `updateYaw()`. |
| `include/IRSensors.h` | `PAIRS`, `irVal`, IR-centering EMA state (`irLSm`/`irRSm`/`irFirstSample`), `readIR`/`sampleIR`. |
| `include/Battery.h` | `readVbat`, `batPct`. |
| `include/Pose.h` | `robotRow/Col/Heading`, `plannedRow/Col/Heading`, `exploreMode`, `fastRunMode`, `fastRunCruiseTps`, `pendingOffsetTicks`. |
| `include/MotionScript.h` | `RunPhase`/`TurnDir` enums, `PhaseStep`, `script[]`, `scriptLen/Idx`, `runPhase/Target/TurnDir`, `phaseStartTL/TR/Us`, the `scriptPush*` helpers. |
| `include/Persistence.h` | NVS save/load for walls + fast-run speed (namespace `mm26`). |
| `include/Planner.h` | `setupMaze`, `senseAndStoreWalls`, `buildMoveScript` (+ fast-run straight-chain fusion). |
| `include/OLED.h` | U8G2 instance + menu + run + diag screens + auto gyro-cal. |
| `src/main.cpp` | Hardware objects (motors, encoders, maze), IR-centering PID, `rTicks`/`stopMotors`/`buttonEdge`, `onPhaseActivate`/`phaseEnter`/`scriptKick`, `setup()`, full `loop()` state machine. ~700 lines. |

**Adding a new tuning knob** → put it in `Tuning.h`, not in any other header.
**Adding a new screen** → put it in `OLED.h`.
**Changing motion control numerics** → `Tuning.h`. **Changing motion control behavior** (PID structure, settle logic) → `src/main.cpp` RUN case.

If you ever add a second `.cpp` to `[env:main]`, convert the file-scope `static` globals in the headers to `inline` (C++17) or move definitions into a `.cpp`.

---

## Build & Flash

```bash
~/.platformio/penv/bin/pio run -e main -t upload     # build + flash production firmware
~/.platformio/penv/bin/pio device monitor             # serial @ 115200
~/.platformio/penv/bin/pio run -e <env>               # build only
~/.platformio/penv/bin/pio run -t clean
```

**Convention in this repo:** *every code change is followed by an upload, not just a build.* When iterating on a sketch, flash on every edit so the user can immediately re-test on hardware. The user has standing approval for `pio run -e <env> -t upload` after a code modification — don't ask.

Post-upload chime: `tools/notify_upload.py` (wired as `extra_scripts` in `[common]`).

### Live test sketches (each is a standalone `.cpp` selected via `build_src_filter`)

| Env | Purpose |
|---|---|
| `main` | Production firmware — PCNT + IMU + IR + flood-fill solver |
| `wall-follow-pcnt` | Drivetrain reference. Same script/PID/IMU stack as main, no flood-fill. Tune knobs here first. |
| `sensor-cal-ble` | Re-capture IR cal (paste output into `PinConfig.h::IR_CAL_*`) |
| `encoder-test` | Live tick stream for hand-roll measurement |
| `imu-turn` | Standalone MPU-6500 trapezoidal turn (legacy) |
| `wall-follow-cells` | Legacy ISR-encoder cell counter (predecessor of `wall-follow-pcnt`) |
| `motor-ble`, `motor-freq`, `velocity-pid*`, `wall-follow`, `side-open-turn`, `ir-test`, `ir-turn-test` | Older diagnostics, kept for hardware bring-up |

`test/ble-test.cpp`, `test/batt-volt.cpp`, `test/buzzer.cpp`, `test/ws2812b.cpp` have no `[env:*]` block — adapt one of the existing blocks if you need them.

---

## Architecture

The production stack (in `src/main.cpp`) and the reference drivetrain (in `test/wall-follow-encoder-pcnt.cpp`) share the same motion code. Tune one, port to the other.

### Drivetrain: phase-script over a position-PID core

Movement is expressed as a sequence of `PhaseStep` records (max 8). Each step picks a `RunPhase` and a tick/degree target. The executor walks the script, resets encoders + zero-yaw between steps, and exits to a higher layer (IDLE for one-shot tests, EXPLORE_THINK for flood-fill) when the last step settles.

```
RunPhase variants:
  PH_FORWARD          — encoder-tick target. Signed (negative = reverse).
                        IR centering + IMU yaw hold + encoder balance bias.
  PH_PIVOT            — single-wheel pivot. Inner wheel braked, outer drives.
                        Target in degrees (IMU) or ticks (fallback).
  PH_SPOT             — both wheels opposite. Robot rotates about its centre.
                        90° and 180° both expressed in degrees.
  PH_FWD_TO_WALL      — open-loop slow forward, stops when IR-estimated front
                        distance ≤ T.wallTouchDistMm. Yaw-held during travel.
  PH_REVERSE_TO_BACK  — at activation, samples front IR and computes a reverse
                        tick target = −(frontMm + T.backupOffsetMm) × ticks/mm,
                        then re-classifies itself as PH_FORWARD so the
                        standard PID handles the reverse motion. Used after
                        180° to bump the rear against the (now) back wall.
```

`onPhaseActivate()` is called from `scriptKick()` and every endPhase-advance. It's the hook for deferred-target phases (currently only `PH_REVERSE_TO_BACK`).

### Position PID is one block, parameterised per phase

The PID code in the `RUN` state is mode-blind. The `imuMode` selector picks one of two gain banks:
- `kp/kd/maxPwm/stictionPwm/frictionZone/holdBand/...` — encoder ticks (forward).
- `yawKp/yawKd/yawMaxPwm/yawStictionPwm/yawFrictionZone/yawHoldBand/...` — degrees (pivot, spot).

Both banks share the same stall-escape + settle-band exit logic, just different bands/units. **Don't fork the PID** when adding a new phase — pick which bank you want and plumb through.

Forward velocity damping uses `(Δavg)/dt` LPF. IMU velocity damping uses `gzFilt` directly (already low-passed in `updateYaw()`).

### Steering bias during forward = sum of three sources

`pwmL = throttle − bias`, `pwmR = throttle + bias`, with `bias` = sum of:
1. **IMU yaw hold** — `−T.yawHoldKp × yawDeg`. Primary heading reference. Yaw is reset to 0 at every phase boundary, so each leg has its own "straight ahead".
2. **IR centering** — confidence-weighted `cR·(irR − calR) − cL·(irL − calL)`. The `conf` ramp 200…800 raw counts replaces threshold-based wall detection so there's no on/off snap when a wall fades in/out. Edge events (`dL/dR > ±300`) print one-shot `[EVENT] L wall opened` etc. **Sensor geometry (changed 2026-05-19):** L/R aim ~90° perpendicular (true side reads); LF/RF still ~30° forward-outward (front-detect only). `IR_CAL_L=1800` / `IR_CAL_R=2000` re-captured 2026-05-19 with robot centered in cell (4.5 cm to each side wall). The old "side cone catches front wall at `frontMm < 60`" caveat no longer applies.
3. **Encoder balance** — `(tL − tR) × T.balanceKp`. Secondary symmetry trim.

Pivot and spot phases skip IR and balance — they're rotational, no lateral position to correct.

### MPU-6500 yaw integration

Configured DLPF=3 (41 Hz BW) to reject motor-PWM harmonics. Z-axis bias captured by 300 samples at boot (robot must be still). `updateYaw()` runs at the top of every loop iteration, integrating `gz − bias` with a `|gz| < 0.05 °/s` deadband and a `gzFilt` LPF (0.7/0.3) for the PID derivative term.

**Convention:** right rotation → `yawDeg < 0`. Pivot/spot progress = `−yawDeg` for right turns, `+yawDeg` for left. The yaw-hold sign in the forward path is `−T.yawHoldKp × yawDeg` to compensate (right drift → negative yaw → positive bias → steer left).

### Cal Gyro is automatic, not button-triggered

The `GYRO_CAL` menu state shows IR LF/RF readings vs `IR_CAL_LF/IR_CAL_RF` (visual alignment aid only — wall-against-robot is geometric, not a stillness signal). Cal **fires automatically** when encoder Δticks ≤ 1 and `|gzFilt| < 1 °/s` are both true for 100 ms continuously. Move the robot mid-cal and the still-timer resets. This protects the bias capture from hand jitter.

### Encoder polarity is inverted on both wheels

```cpp
MicromouseEncoderPCNT leftEnc (PCNT_UNIT_0, ENC_L_A, ENC_L_B, /*inverted=*/true);
MicromouseEncoderPCNT rightEnc(PCNT_UNIT_1, ENC_R_A, ENC_R_B, /*inverted=*/true);
```

`MOTOR_L_INV` / `MOTOR_R_INV` flip the IN1/IN2 pins, which made the magnetic encoders' native +1 direction mechanical reverse. The PCNT `inverted=true` flag mirrors them so `driveForward → positive ticks`. **Do not** unset this without re-tuning all forward distances.

### Right-encoder scale + `rTicks()`

`RIGHT_ENC_SCALE = 1.0135f` (`PinConfig.h`). All distance math goes through `rTicks()`, never `rightEnc.getTicks()`. If you add a new code path consuming right ticks for distance, use `rTicks()`.

### `TICKS_PER_CELL = 1405` is hand-measured

PCNT 4× decode gives ~7.81 ticks/mm. `T.ticksPerCell = 1405` was captured by rolling the robot 180 mm by hand on this exact chassis. **It is not derived from `TICKS_PER_REV`** — re-measure by hand-roll if the wheel/tire changes.

`ticksPerMm = T.ticksPerCell / 180.0f` is the conversion factor used by `PH_REVERSE_TO_BACK`.

---

## Flood-fill explore (`src/main.cpp` only)

Active maze region: **6 rows × 3 cols**, start `(0,0)` facing North, goal `(5,2)`. `MicromouseMaze` allocates the full 16×16 grid (from `MAZE_SIZE` in `PinConfig.h`); `setupMaze()` closes the far north and east borders so flood-fill never wanders, then overrides the goal to `(5,2)`.

### Move cycle

```
EXPLORE_THINK:
  senseAndStoreWalls()                      // F/L/R relative to heading
  if isGoal: nvsSaveWalls() → state = GOAL
  maze.floodFill()
  bestDir = maze.bestDirectionBiased(...)   // GreenYe: straight > L > R > U,
                                            // +4 penalty for visited cells
  buildMoveScript(bestDir):
    diff==0: FWD(cellTicks + pendingOffset)
    diff==1: SPOT_R 90 + FWD(...)
    diff==3: SPOT_L 90 + FWD(...)
    diff==2: SPOT_R 180 + REVERSE_TO_BACK + FWD(cellTicks + startOffsetTicks)
  scriptKick(); state = RUN
RUN:
  ... runs script ...
  on last step settle: commit plannedRow/Col/Heading → EXPLORE_THINK
```

### Start-offset and 180° re-anchor

At boot the robot sits with its rear pressed against the back wall of cell `(0,0)` — its centre is `~4.5 cm` behind cell-(0,0)-centre. The first forward leg must travel `cellTicks + startOffsetTicks` (1405 + 351 ≈ 22.5 cm) to land at cell-(1,0)-centre. `pendingOffsetTicks` carries this extra distance into the next forward leg, then clears.

After a 180° (only happens at dead-ends in this maze): `SPOT_R 180` then `PH_REVERSE_TO_BACK` (which samples front IR at activation and reverses by `frontMm + T.backupOffsetMm` mm of ticks). Robot now sits with rear against the new back wall — same `-4.5 cm` pose as start — so `pendingOffsetTicks` is set to `startOffsetTicks` again for the subsequent forward.

`T.backupOffsetMm = 15` mm (1.5 cm) compensates for the front-sensor-to-axle distance being ~1.5 cm longer than the back-edge-to-axle distance on this chassis.

### Wall sensing & NVS

`senseAndStoreWalls()` runs at every cell-centre arrival in EXPLORE mode (skipped in FAST mode). `setWall` mirrors each bit into the adjacent cell — single source of truth. On goal: `nvsSaveWalls()` writes the 256-byte `walls[][]` bitmask to NVS namespace `"mm26"` key `"walls"`. **Fast Run** reads it back and runs the same flood/decide/move loop with sensing disabled. **Clear NVS** menu wipes the key.

IR cal (`IR_CAL_LF/L/R/RF`) is **not** persisted to NVS. Compile-time defaults in `PinConfig.h` are restored on every boot. Re-capture via `sensor-cal-ble` or the on-device menu (if added) and paste the new values into `PinConfig.h`.

---

## Critical invariants (easy to break)

- **`frictionZone ≤ holdBand`** — otherwise the robot stalls in a dead zone between them where the stiction floor is disabled but settle hasn't triggered. `Tuning` comment flags this.
- **`MAX_SCRIPT ≥ 4`** — the 180° anchor sequence is 3 steps plus optional pre-turn. Currently 8.
- **LEDC API is Arduino 2.x** — `ledcSetup(ch, freq, bits)` + `ledcAttachPin(pin, ch)`, not 3.x's `ledcAttach(pin, freq, bits)`. `platformio.ini` uses `platform = espressif32` (stock), not `pioarduino`. Migrating to ESP-IDF 5.x / Arduino 3.x requires changing the platform line **and** every `ledc*` call together.
- **`yawDeg` resets at every phase boundary**, not just rotational ones. Forward-leg heading hold relies on this.
- **Encoder polarity** — both PCNT ctors take `inverted=true`. Don't change without re-tuning.
- **Reverse targets work natively in `PH_FORWARD`** — the position PID handles negative targets symmetrically (encoders go negative as robot reverses, `posErr = target − avg` stays the right sign). Don't add a separate reverse phase; just use a negative `target`.
- **`WifiDebug.h` is not compiled into `main`** — it's a dormant ~500-line HTTP server. Don't assume WiFi runs.

---

## State machine (`src/main.cpp`)

```
IDLE
 ├─ menu Explore   → EXPLORE_THINK
 ├─ menu Fast Run  → EXPLORE_THINK   (loads NVS first, bails if empty)
 ├─ menu Cal Gyro  → GYRO_CAL        (auto-cal on stillness, button = abort)
 ├─ menu Enc Test  → ENC_TEST
 ├─ menu Turn R/L  → RUN             (single SPOT_90 script for HW test)
 └─ menu Clear NVS → IDLE            (wipes saved walls)

EXPLORE_THINK → RUN          (kicks current move's script)
RUN → EXPLORE_THINK          (script done, more cells)
RUN → IDLE                   (script done, not in explore/fast — e.g. menu turn)
EXPLORE_THINK → GOAL         (at goal; explore saves NVS first)
EXPLORE_THINK → CRASH        (flood = FLOOD_INFINITY, robot boxed in)

GOAL / CRASH → IDLE          (button)
```

`bestDirectionBiased()` returning `FLOOD_INFINITY` is the only crash condition. Unlike the legacy stack, there's no `doTurn()` convergence-failure path because the script's stall-escape exits any phase that can't make progress.

---

## Battery feed-forward removed

The legacy `vScale = NOMINAL_VBAT / Vbat` PWM scaling that wrapped the cascaded velocity-PI loop is gone. Position PID's stiction floor + KD damping handle pack sag implicitly. If long-run distance accuracy starts drifting with battery voltage, re-add the scale at the throttle-magnitude step (`mag = mag * vScale`).
