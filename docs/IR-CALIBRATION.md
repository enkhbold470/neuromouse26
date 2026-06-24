# IR Sensor Calibration — Micromouse26

Last updated: **2026-06-24** (bench + user measurements).

Values live in `include/PinConfig.h` (factory defaults) and are refined per-run by `calibrateSideRefs()` in `include/Planner.h` when `SIDE_ADAPTIVE=true`.

---

## Sensor layout

| Index | Name | Role |
|---|---|---|
| `irVal[0]` | LF | Front-left (~10° from forward) |
| `irVal[1]` | L45 | Side-left (30° from forward axis) |
| `irVal[2]` | R45 | Side-right (30° from forward axis) |
| `irVal[3]` | RF | Front-right |

Front sensors: wall detect, `PH_ALIGN_FRONT`, proactive front stop during explore.

Side sensors: maze wall mapping, IR centering during **fast run only** (disabled in explore).

---

## Front calibration (LF / RF)

Captured with robot **centred in cell** at correct front-wall gap:

| Constant | Value |
|---|---|
| `IR_CAL_LF` | 570 |
| `IR_CAL_RF` | 570 |
| `ALIGN_LF_TARGET` | 570 |
| `ALIGN_RF_TARGET` | 570 |
| `ALIGN_TOL` | 40 |

Runtime thresholds (`Tuning.h`):

| Threshold | Value | Action |
|---|---|---|
| `WALL_FRONT_BRAKE_THRESH` | 480 | Begin slowing forward |
| `WALL_FRONT_STOP_THRESH` | 530 | Hard stop + bump recovery |
| `WALL_FRONT_THRESH` | 300 | Legacy front-wall detect in stall path |

Re-capture: env `sensor-cal-ble` or OLED menu **Cal IR** → paste into `PinConfig.h`.

---

## Side calibration (L45 / R45)

User bench measurements (2026-06-24):

| Pose | L45 | R45 |
|---|---|---|
| Cell centre (both side walls visible) | **522** | **690** |
| Touching **left** wall | 3600 | 270 |
| Touching **right** wall | 180 | 4000 |

Firmware constants:

| Constant | Value |
|---|---|
| `IR_CAL_L45` | 522 |
| `IR_CAL_R45` | 690 |
| `SIDE_OPEN_CEIL` | 350 |
| `SIDE_WALL_FRAC` | 0.45 |
| `SIDE_REF_MIN` | 300 |

### Open vs wall rules

1. **Open side:** raw read **< 350** (`SIDE_OPEN_CEIL`) → treated as open; no wall written; no centering weight.
2. **Wall present:** read **> 45%** of per-run reference (`sideRefL` / `sideRefR` from start-cell cal).
3. **Saturation:** emitter-off ambient **> 3500** → reading skipped (UNKNOWN), prior map kept.

### Per-run reference

At explore/fast-run start, robot in `(0,0)` facing North:

```
calibrateSideRefs() → sideRefL, sideRefR logged as [SIDECAL] on serial
```

West border guarantees left wall for reference capture.

---

## Front-wall contamination on side sensors

When a **front** wall is close, L45/R45 see it diagonally (~220 counts) and can write phantom side walls.

**Fix in `senseAndStoreWalls()`:** if front blocked, skip side re-sense; keep mid-cell side data from `senseSideWallsMidCell()` (~75% through forward leg into unvisited cell).

---

## IR centering (fast run only)

In `main.cpp` RUN loop — **not active during explore**.

Uses ref-relative `wallConf()`:

- Zero weight if `v < SIDE_OPEN_CEIL`
- Scales steering between 15% and 70% of reference
- `IR_CENTER_KP/KD`, max bias `IR_CENTER_MAX = 35`

---

## Recalibration procedure

1. Flash `sensor-cal-ble` env or use on-device **Cal IR** menu.
2. Place robot centred in cell with known wall geometry.
3. Copy serial output → `PinConfig.h` `IR_CAL_*`.
4. Re-flash `main` and verify `[SIDECAL]` at run start.
