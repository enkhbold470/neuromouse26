# Firmware Progress — 2026-06-24

> Branch: `trust-me-bro` · Production env: `main` · Boot banner: **`[FW] 2026-06-24 simple-safe explore maze6x3`**

This document captures the full explore/fast-run/BLE/IR work through 2026-06-24, including experiments that were tried and reverted, current calibration, and the **simple safe explore** baseline now on the robot.

---

## Physical setup

| Item | Value |
|---|---|
| Test maze | **6×3** cells inside 16×16 flood grid |
| Start | `(0,0)` facing North |
| Goal | `(5,2)` top-right |
| Cell pitch | 180 mm centre-to-centre |
| Robot body | 100 × 88 mm |
| Wheel track | 80 mm |
| Axle → front | 67 mm |
| Axle → rear | 33 mm |
| Weight | ~161 g with batteries |

PCB exports: `hardware/Netlist_PCB1_2026-06-23.tel`, `PickAndPlace_PCB1_2026-06-23.csv`.

---

## Current explore behavior (simple safe)

**Philosophy:** one predictable move per plan — stop, sense, turn, go. No fused straights, no arcs, no IR centering during explore.

### Move cycle

```
EXPLORE_THINK:
  auto gyro zero (yawDeg = 0 at each cell)
  senseAndStoreWalls() on first visit to cell
  mark cell visited
  if at (5,2) first time → nvsSaveWalls(), exploreFwdGoalSaved = true, OLED "GOAL OK / sweeping"
  if sweep done (unvisited == 0) → setGoalSingle(0,0), returnHomeMode = true
  if at (0,0) in returnHomeMode → final 180° spin → GOAL
  floodFillExplore (forward) or floodFill (return home)
  buildMoveScript(bestDir):
    one cell only — SPOT 90° if needed, then PH_FORWARD one CELL_TICKS
    dead end: SPOT 180° + reverse 20 mm + forward 180 mm (no ALIGN in explore)
    preTurnAlignNeeded() always returns false
  scriptKick() → RUN

RUN:
  FWD at FWD_V_CRUISE_TPS (150 tps ≈ 19 mm/s)
  full POS_SETTLE_MS (80 ms) brake at end of each phase
  no IR centering during explore
  front IR hard stop if LF/RF peak ≥ 530 → mark wall, backup, replan
  fast run only: IR centering + smooth arcs when Mode = Smooth ON
```

### Explore completion flow

1. Reach `(5,2)` → save walls to NVS → **keep exploring** (sweep)
2. When all cells visited → route home to `(0,0)`
3. At `(0,0)` → save NVS → 180° celebration spin → **GOAL** blink

Flags in `Pose.h`:

- `exploreFwdGoalSaved` — set after first goal arrival
- `returnHomeMode` — set after sweep complete
- `finalTurnPending` — set when final spin script is kicked

---

## Fast run (unchanged from explore motion split)

| Setting | Behavior |
|---|---|
| `g_smoothMode` | OLED **Mode: Smooth/Classic** (default **OFF**) |
| Smooth ON + `CURVE_ENABLE` | `buildFastSmoothRoute()` — FWD + PH_CURVE arcs, straight-chain fusion |
| Smooth OFF | Classic one-cell SPOT + FWD (same script builder path as explore) |
| Cruise | `fastRunCruiseTps` — menu **Fast Speed**, NVS key `fspeed` |

Arc geometry (`Tuning.h [I]`):

- `CURVE_RADIUS_MM = 90` (= half cell pitch, **not** derived from PCB netlist)
- Fast-run only; explore never uses PH_CURVE

---

## IR calibration (2026-06-24 bench)

### Front (LF / RF) — cell centre, correct front gap

| Sensor | Value | Use |
|---|---|---|
| LF | 570 | `IR_CAL_LF`, `ALIGN_LF_TARGET` |
| RF | 570 | `IR_CAL_RF`, `ALIGN_RF_TARGET` |

Front stop thresholds: brake @ 480, hard stop @ 530.

### Side (L45 / R45) — user measured

| Pose | L45 | R45 |
|---|---|---|
| Cell centre | 522 | 690 |
| Touching left wall | 3600 | 270 |
| Touching right wall | 180 | 4000 |

In firmware (`PinConfig.h` + `Tuning.h [J]`):

- `IR_CAL_L45 = 522`, `IR_CAL_R45 = 690`
- `SIDE_OPEN_CEIL = 350` — below = open side (do not steer / mark wall)
- `SIDE_WALL_FRAC = 0.45` — wall if read > 45% of per-run `sideRef`
- `calibrateSideRefs()` at explore start from cell `(0,0)` west wall

IR centering (`main.cpp`) uses `wallConf()` with `SIDE_OPEN_CEIL` guard — **disabled entirely during explore** (`!exploreMode` gate).

---

## BLE telemetry

| Item | Detail |
|---|---|
| Device name | `bromouse` |
| Service | Nordic UART (NUS) |
| Monitor | `tools/ble-debug.html` (localhost or HTTPS; not `file://`) |
| Commands | `EXPLORE`, `FAST`, `STOP`, `DUMP`, `CLEAR_NVS` |
| JSON types | `ST`, `POS`, `WALL`, `MOT`, `BAT`, `CRASH`, `MAZE` |

**macOS:** System Settings → Privacy & Security → Bluetooth → enable **Google Chrome** if picker is empty.

Serial confirms: `[BLE] advertising as 'bromouse'`.

---

## OLED indication

| Screen | When |
|---|---|
| `oledExploreThink()` | Between moves — mode EXPLORE / SWEEP / HOME + unvisited count |
| `oledRun()` | During RUN — header shows mode + phase (FWD/SPOT/CRV) |
| `oledTerminal()` | GOAL OK, SWEEP DONE, HOME OK, DONE |

---

## Experiments tried and reverted (lessons)

| Experiment | Problem | Resolution |
|---|---|---|
| Multi-cell fused explore straights | Rammed walls by inertia; bad replans | **Removed** — `EXPLORE_CONTINUOUS = false` |
| Explore PH_CURVE arcs | Stuck at corners; disconnects | **Removed** — arcs fast-run only |
| No-brake FWD→FWD handoffs | Drift, IR centering fights | **Removed** for explore |
| IR centering on open side | Low L45 (~180) steered toward fake wall ref | Open-side guard + **disable in explore** |
| PH_ALIGN_FRONT before turns | 10–20 oscillations; infinite deadband stall | **Disabled** for explore; `preTurnAlignNeeded()` → false |
| Immediate return home at goal | Skipped unvisited sweep | **Fixed** — sweep then home |
| `buildContinuousPivotRoute()` | Complex, fragile | **Deleted** from Planner |

---

## Key source files (2026-06-24)

| File | Role |
|---|---|
| `include/Tuning.h` | All knobs; `EXPLORE_CONTINUOUS=false`, geometry, IR thresholds |
| `include/Planner.h` | `buildMoveScript`, `buildFastSmoothRoute`, wall sense, dead-end exit |
| `include/Pose.h` | Mode flags including `exploreFwdGoalSaved`, `g_smoothMode=false` default |
| `include/PinConfig.h` | IR cal constants |
| `include/MicromouseMaze.h` | `provenOpen`, `countUnvisited`, 6×3 sealing |
| `include/OLED.h` | Menu, run, explore status screens |
| `include/BLETelemetry.h` | NimBLE NUS server |
| `src/main.cpp` | State machine, RUN executor, explore-safe motion gates |

---

## Build & flash

```bash
~/.platformio/penv/bin/pio run -e main -t upload
~/.platformio/penv/bin/pio device monitor   # 115200
```

Verify boot line: `[FW] 2026-06-24 simple-safe explore maze6x3`.

---

## Tuning quick reference

| Knob | Value | Notes |
|---|---|---|
| `FWD_V_CRUISE_TPS` | 150 | Explore speed |
| `EXPLORE_CONTINUOUS` | **false** | Do not re-enable without front-stop hardening |
| `EXPLORE_RETURN_HOME` | true | Sweep then home |
| `CELL_TICKS` | 1400 | Hand-measured; do not derive from encoder math |
| `BASE_BREAKAWAY_PWM` | 110 | Master power knob |
| `g_smoothMode` default | false | Fast run opt-in only |

---

## Open / future work

- [ ] Re-enable faster explore only after per-cell stop + front IR proven stable on full 6×3
- [ ] Tune `CURVE_V_ARC_TPS` on bench before competition fast run
- [ ] Set `TELEMETRY=false` before competition (RUN loop speed)
- [ ] Optional: re-enable mild IR centering in explore only when **both** side walls present

---

## Related docs

- `CLAUDE.md` — agent module map + invariants (kept in sync)
- `OVERVIEW.md` — architecture summary
- `GEMINI.md` — deep architecture reference
- `tools/README.md` — BLE debug page usage
