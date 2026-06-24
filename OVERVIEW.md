# Micromouse26 — Firmware Overview

> ESP32-S3 · PlatformIO + Arduino · PCNT encoders · MPU-6500 yaw · flood-fill solver

---

## Project Structure

```
neuromouse26/
├── include/
│   ├── README                      module map (read this first)
│   ├── Tuning.h                    every tunable constant
│   ├── IMU.h                       MPU-6500 + yaw integration
│   ├── IRSensors.h                 4-sensor IR array
│   ├── Battery.h                   Vbat + state-of-charge
│   ├── Pose.h                      robot pose + mode flags
│   ├── MotionScript.h              RunPhase enum + script[] + pushers
│   ├── Persistence.h               NVS save/load
│   ├── Planner.h                   setupMaze, sense, buildMoveScript
│   ├── OLED.h                      menu + run + diag screens
│   ├── PinConfig.h                 pin numbers + hardware constants
│   ├── MicromouseMotor.h           DRV8833 wrapper (LEDC PWM)
│   ├── MicromouseEncoderPCNT.h     PCNT 4× quadrature decoder
│   ├── MicromouseMaze.h            16×16 grid + flood-fill BFS
│   ├── IRCalibration.h             per-cm IR LUT + front-distance interp
│   ├── MicromouseEncoder.h         legacy ISR encoder (unused by main)
│   ├── PID.h                       generic PID (unused by main)
│   └── WifiDebug.h                 dormant HTTP debug server (not included)
├── src/
│   └── main.cpp                    hardware objects + setup + loop FSM
├── test/                           one .cpp per env, bring-up sketches
├── tools/
│   └── notify_upload.py            post-upload audible chime
├── platformio.ini
├── CLAUDE.md                       agent guide (Claude)
├── GEMINI.md                       agent guide (Gemini)
├── AGENTS.md                       Cursor agent entry + doc index
├── docs/
│   ├── 2026-06-24-firmware-progress.md   session progress (explore, BLE, cal)
│   └── IR-CALIBRATION.md                 front/side IR bench values
└── OVERVIEW.md                     this file
```

---

## State Machine

`enum State { IDLE, ENC_TEST, IR_TEST, FAST_SPEED_EDIT, EXPLORE_THINK, RUN, GOAL, CRASH };` (in `src/main.cpp`).

```
                    ┌──────────────┐
              boot  │     IDLE     │ ◄────────────────────────────┐
             ─────► │ encoder=menu │  button (from any leaf state) │
                    │ button=enter │                               │
                    └──────┬───────┘                               │
                           │                                       │
        menu choice ───────┴──────────────────────────────         │
        Explore / Fast Run                                │        │
                  │                                       │        │
                  ▼                                       │        │
            autoCalGyro + 3-2-1 countdown                 │        │
                  │                                       │        │
                  ▼                                       │        │
            ┌──────────────┐    senseAndStoreWalls        │        │
            │EXPLORE_THINK │──────────────────────┐       │        │
            │ flood + plan │                      │       │        │
            └──────┬───────┘                      │       │        │
                   │ buildMoveScript + kick       │       │        │
                   ▼                              ▼       │        │
            ┌──────────────┐                ┌─────────┐   │        │
            │     RUN      │ ────────────► │  GOAL   │   │        │
            │ script exec  │  reached goal └────┬────┘   │        │
            └──────┬───────┘                    │ button  │        │
                   │ script done                ▼         │        │
                   └─► EXPLORE_THINK    ┌────────────┐   │        │
                                        │   CRASH    │   │        │
                                        │ boxed-in   │   │        │
                                        └─────┬──────┘   │        │
                                              │ button   │        │
                                              ▼          ▼        ▼
                                              └──────────┴────────┘
```

Other menu leaves: `ENC_TEST` (live tick stream), `IR_TEST` (live sensor read-out), `FAST_SPEED_EDIT` (encoder-knob adjustment of `fastRunCruiseTps`, saved to NVS on button press). Each returns to `IDLE` on a button press.

---

## One move cycle (EXPLORE_THINK → RUN → EXPLORE_THINK)

> **2026-06-24:** Simple safe explore — one cell per script, full stop, no arcs. Full detail: `docs/2026-06-24-firmware-progress.md`.

```
EXPLORE_THINK:
  yawDeg / yawTargetDeg ← 0
  senseAndStoreWalls() on first visit
  maze.visited[r][c] = true
  if first goal (5,2): save NVS, exploreFwdGoalSaved, keep sweeping
  if sweep done: returnHomeMode → flood home to (0,0)
  if home (0,0): 180° spin → GOAL
  floodFillExplore (forward) or floodFill (return home)
  buildMoveScript(bestDir):
      one cell — SPOT 90° if needed + FWD(CELL_TICKS)
      dead end: SPOT 180 + reverse 20 mm + forward 180 mm (no ALIGN in explore)
  scriptKick() → RUN

RUN:
  explore: FWD_V_CRUISE_TPS (150), POS_SETTLE_MS brake, no IR centering
  front IR hard stop @ 530 during forward explore
  fast run: optional Smooth arcs + straight-chain via buildFastSmoothRoute()
```

---

## Motion control: phase-script over a position-PID core

A move is a sequence of `PhaseStep`s. Each step picks a `RunPhase` and a target. The RUN executor walks the script. Per-phase yaw / encoder snapshots are captured in `phaseEnter()`.

| Phase | What it does |
|---|---|
| `PH_FORWARD` | Signed tick target. Position-PID + trapezoidal velocity command (`vAccel = sqrt(2·a·xDone)`, `vDecel = sqrt(2·a·xRem)`, cruise at `vCruise`). Forward bias = IR centering corr + (tL−tR)·`BALANCE_KP` + IMU yaw hold. |
| `PH_SPOT` | Both wheels opposite. IMU-yaw-PID on `dy = yawDeg − phaseStartYawDeg`. Signed by `runTurnDir`. |
| `PH_PIVOT` | Single-wheel pivot (other wheel braked). Still callable, but `buildMoveScript` no longer pushes it (R-pivot kept clipping walls). |
| `PH_ALIGN_FRONT` | Slow creep until `irVal[0] ≈ ALIGN_LF_TARGET` and `irVal[3] ≈ ALIGN_RF_TARGET` at 37.5 mm. Used in the dead-end exit. |
| `PH_REVERSE_TO_BACK` | Defers its tick target to `onPhaseActivate` (samples front IR there), then reclassifies as `PH_FORWARD`. Not pushed in the current flow; available for future re-anchor primitives. |

All phases share the same stall-escape + settle-band exit logic in the RUN executor — the `imuMode` flag selects gain banks (`POS_*` for forward, `YAW_*` for rotation).

### Fast run vs explore (2026-06-24)

| | Explore | Fast run |
|---|---|---|
| Speed | `FWD_V_CRUISE_TPS` (150) | `fastRunCruiseTps` (menu/NVS) |
| Cells per script | **1** | 1 (Classic) or fused straights (Smooth) |
| Turns | SPOT 90° / 180° | SPOT or PH_CURVE arcs when Smooth ON |
| IR centering | **Off** | On (fast run forward) |
| Settle | Full `POS_SETTLE_MS` every cell | 0 ms on fast-run FWD (trapezoid parks) |

`g_smoothMode` default **OFF** — opt-in via OLED **Mode** menu for fast run only.

---

## Tuning knobs

All tunables are `constexpr` in `include/Tuning.h`. Sections:

| Section | Covers |
|---|---|
| [A] | Robot + maze geometry (mm) |
| [B0] | Main power knob (`BASE_BREAKAWAY_PWM`); all stiction floors derived |
| [B] | Forward motion: trapezoid (`FWD_V_CRUISE_TPS`, `FWD_ACCEL_TPS2`), position PID (`POS_KP/KD`), settle/stall bands |
| [C] | Pivot / spot turns: yaw PID (`YAW_KP/KD`), settle/stall, `YAW_HOLD_KP/KD` for forward heading lock |
| [D] | IR centering: `IR_CENTER_KP/KI/KD`, `IR_CENTER_MAX` |
| [E] | Dead-end handling: `ALIGN_LF_TARGET`, `ALIGN_RF_TARGET`, `DEADEND_REVERSE_MM`, `DEADEND_FWD_MM` |
| [F] | Script geometry: `CELL_TICKS`, `START_OFFSET_TICKS`, fallback tick counts |
| [H] | Debug flags: `USE_IMU`, `USE_IR`, `TELEMETRY` |

**Bump `BASE_BREAKAWAY_PWM` and the whole robot gets stronger proportionally** — every stiction floor scales with it. PID gains (`POS_KP`/etc.) are intentionally NOT derived because they're independent physics.

---

## Hardware Constants (`include/PinConfig.h`)

| Constant | Value | Notes |
|---|---|---|
| `WHEEL_DIAMETER` | 33.4 mm | Measured |
| `RIGHT_ENC_SCALE` | 1.0135f | Auto-calibrated 2026-05-17 |
| `WHEEL_TRACK_MM` | 80 mm | |
| `MOTOR_PWM_MAX` | 1023 | 10-bit LEDC |
| `MOTOR_PWM_FREQ_HZ` | 200 | Empirical sweet spot for stiction + this DRV8833 + N20 |
| `MOTOR_PWM_BITS` | 10 | |
| `BAT_VDIV_MULT` | 3.751f | 39k / (39k + 100k) divider, calibrated 2026-05-17 |
| `IR_CAL_LF` / `IR_CAL_RF` | 570 / 570 | Cell centre front gap, 2026-06-24 |
| `IR_CAL_L45` / `IR_CAL_R45` | 522 / 690 | Side @ cell centre, 2026-06-24 |
| `WALL_FRONT_THRESH` | 300 | Front detect (stall path) |
| `WALL_FRONT_STOP_THRESH` | 530 | Explore hard stop (Tuning.h) |
| `SIDE_OPEN_CEIL` | 350 | Below = open side (Tuning.h) |

Full IR doc: `docs/IR-CALIBRATION.md`. Legacy values in older docs are superseded.

`CELL_TICKS=1400` and `START_OFFSET_TICKS=322` live in `Tuning.h` (section [F]) — both hand-measured on this chassis. Do NOT derive them from `TICKS_PER_REV`.

---

## Serial Telemetry

Per-loop while in RUN (`TELEMETRY=true` in Tuning.h, [H]):
```
t=12345 ph=FWD/TRAP tgt=1400 xCmd=+1400 vCmd=+150 avg=+782.5 err=+617.5 v=+148.2 thr=+128 pwmL=+132 pwmR=+124 yaw=+0.21 tL=782 tR=783
t=13456 ph=SPOT/IMU tgt=90 avg=+45.2 err=+44.8 v=+12.3 thr=+165 pwmL=+165 pwmR=-165 yaw=-45.20 tL=80 tR=-78
```

Tagged events:

| Tag | Source |
|---|---|
| `[IMU]` | Setup-time mpu init / bias result |
| `[GCAL/AUTO]` | autoCalGyroBeforeStart result |
| `[SENSE]` | senseAndStoreWalls — sampled wall booleans + raw IR |
| `[PLAN]` | EXPLORE_THINK — current pose, flood distance, chosen direction |
| `[FAST]` | Straight-chain fusion — how many cells got chained |
| `[BACKUP]` | onPhaseActivate computing a PH_REVERSE_TO_BACK target |
| `[EVENT]` | IR centering edge — wall opened / appeared |
| `--- STEP END idx=I/N ph=PH reason=... ---` | endPhase advance |
| `--- MOVE DONE pos=... ---` | script complete, pose committed |
| `[FSPEED]` | Fast Speed load / save |
| `[NVS]` | Wall persistence |

`TELEMETRY=false` before competition runs — Serial prints inside the PID loop slow it.

---

## Build & Flash

```bash
~/.platformio/penv/bin/pio run -e main -t upload     # production firmware
~/.platformio/penv/bin/pio device monitor             # 115200 baud
~/.platformio/penv/bin/pio run -e <env>               # build only
```

Convention: every code change is followed by an upload, not just a build. The post-upload chime (`tools/notify_upload.py`) confirms the flash completed.

---

## Where to make changes

| You want to… | Open |
|---|---|
| Change a tuning constant | `include/Tuning.h` |
| Tune a screen / add an OLED diagnostic | `include/OLED.h` |
| Change wall-sense thresholds or geometry | `include/PinConfig.h` (thresholds, cal values) + `include/Planner.h` (sense logic) |
| Change the move-decision rule | `include/Planner.h::buildMoveScript` |
| Touch motion control behavior (PID, settle, stall) | `src/main.cpp` RUN case |
| Add a new RunPhase | `include/MotionScript.h` (enum + pusher) + `src/main.cpp` RUN case (executor branch) |
| Wire up a new sensor | new `include/*.h` + `#include` it in `src/main.cpp` |
| Persist something to NVS | `include/Persistence.h` |

---

## Conventions worth knowing

- **Single translation unit.** `build_src_filter = +<main.cpp>` means `src/main.cpp` is the only `.cpp` compiled into `[env:main]`. File-scope `static` in headers works because each header is text-included exactly once.
- **`extern MicromouseMaze maze;`** in `Persistence.h` / `Planner.h` points at the single definition in `main.cpp`. Same pattern for `leftEnc`/`rightEnc` in `OLED.h`.
- **Right rotation → `yawDeg < 0`.** Pivot/spot progress = `−yawDeg` for right turns, `+yawDeg` for left. The forward-leg yaw-hold sign in main.cpp compensates with a leading minus.
- **All right-tick distance math goes through `rTicks()`**, not `rightEnc.getTicks()`.
- **`yawDeg` is reset to 0 at every `phaseEnter`** — each leg has its own "straight ahead".
- **No hooks (`--no-verify`, etc.) on git commits.** No FastLED, no FreeRTOS tasks, no WiFi in `[env:main]`.
