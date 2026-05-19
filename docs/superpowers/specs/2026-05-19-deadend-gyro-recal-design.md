# Dead-End Front-Bump Re-Anchor + Gyro Re-Calibration

**Date:** 2026-05-19 (revised same day)
**Status:** Approved design; implementation pending.
**Branch:** `inky`

> **Revision:** This spec was rewritten to invert the 180° anchor strategy. Instead of `PH_REVERSE_TO_BACK` (blind reverse to back wall, depends on `backupOffsetMm`), the new flow drives forward into the front wall (real IR-sensed contact) and reverses exactly `startOffsetTicks` (45 mm) to land at cell-center. Gyro re-cal still happens during the post-reverse stillness window. The earlier "rear-bump + recal" version of this design is superseded.

---

## Problem

Two compounding issues at 180° dead-ends:

1. **Pose drift.** `PH_REVERSE_TO_BACK` samples front IR once at phase entry, computes a reverse tick target (`frontMm + backupOffsetMm`), and reverses open-loop until ticks hit the target. Errors stack: IR LUT noise, sample timing, `backupOffsetMm` is empirical and chassis-specific (currently 10 mm), and the robot's true distance to the back wall depends on chassis-front-to-axle vs chassis-back-to-axle asymmetry. End result: robot pose after the rear-bump can drift cell-to-cell.

2. **Gyro bias drift.** The MPU-6500 Z-axis bias `gyroBiasZ` is captured once at boot. Over a multi-second run the bias creeps (temperature, settling), and `yawDeg` integrates the creep directly. Yaw-hold during forward motion uses `yawDeg` as primary reference — drift directly degrades centering.

The dead-end is a natural pose-reset and stillness opportunity that the current flow under-uses.

## Goal

At every 180° turn:
1. Drive forward into the front wall using **real IR contact** (raw IR ≥ 3500 → robot is 1-3 cm from wall, matched against the `IR_DIST_TABLE` in `IRCalibration.h`).
2. Reverse exactly `T.startOffsetTicks` (= 351 ticks = 45 mm) — same offset used at boot, calibrated for this chassis to land robot center at cell-center.
3. Wait for stillness, re-zero `gyroBiasZ` via existing `calibrateGyroBias()`, zero `yawDeg` and `yawTargetDeg`.
4. Proceed with next FWD = full `cellTicks` (no `pendingOffsetTicks` carry, because robot is already at cell-center).

## Non-Goals

- Pose correction at non-180° events (cell-arrival pose drift is a separate problem — handled by IR centering during travel).
- IR cal refresh.
- Persisting `gyroBiasZ` to NVS.
- Removing `PH_REVERSE_TO_BACK` source code. Keep the definition + `onPhaseActivate()` hook for now; just stop calling it from `buildMoveScript`. Cleanup is a separate PR.

---

## Design

### New phases

```cpp
enum RunPhase {
    PH_FORWARD,
    PH_PIVOT,
    PH_SPOT,
    PH_FWD_TO_WALL,        // existing, mm-threshold based, kept for reuse
    PH_REVERSE_TO_BACK,    // existing, no longer called by buildMoveScript
    PH_FWD_TO_BUMP,        // NEW: slow forward, raw-IR threshold stop
    PH_DEADEND_RECAL       // NEW: motors off, wait stillness, re-cal gyro
};
```

**`PH_FWD_TO_BUMP`:** slow forward (uses `T.stictionPwm` like `PH_FWD_TO_WALL`) with full yaw hold against `yawTargetDeg`. Exits when `(irVal[0] + irVal[3]) / 2 >= T.wallBumpRaw` (default 3500) **or** when `tL + tR / 2 >= T.fwdToBumpMaxTicks` safety cap (default 1600 — same as existing `fwdToWallMaxTicks`). Raw IR is preferred over `estimateFrontDistMM()` because the LUT clamps to 10 mm minimum and is harder to read for "actually bumped".

**`PH_DEADEND_RECAL`:** identical to the prior design — motors coasted, stillness probe (reusing `GYRO_CAL` constants), `calibrateGyroBias()` on success, 500 ms timeout fallback, zero `yawDeg`/`yawTargetDeg` either way.

### `Tuning` struct additions

```cpp
// PH_FWD_TO_BUMP — front-wall bump detection by raw IR
int      wallBumpRaw           = 3500;   // (LF+RF)/2 threshold
long     fwdToBumpMaxTicks     = 1600;   // safety cap

// PH_DEADEND_RECAL — gyro re-cal during stillness window
uint32_t recalStillHoldMs      = 100;
long     recalStillTicks       = 1;
float    recalStillGz          = 1.0f;
int      recalSamples          = 300;
int      recalSampleDelayMs    = 2;
uint32_t recalTimeoutMs        = 500;
```

`T.backupOffsetMm` is unused by the new 180° flow but kept (for the legacy `PH_REVERSE_TO_BACK` path which still exists).

### New script-push helpers

```cpp
static void scriptPushFwdToBump() {
    scriptPush(PH_FWD_TO_BUMP, T.fwdToBumpMaxTicks, TURN_NONE);
}
static void scriptPushDeadendRecal() {
    scriptPush(PH_DEADEND_RECAL, 0, TURN_NONE);
}
// scriptPushFwd(-T.startOffsetTicks) is used for the reverse leg — no
// new helper, the existing PH_FORWARD path handles negative targets.
```

### `buildMoveScript()` 180° branch — new flow

```cpp
} else if (diff == 2) {
    // 180° front-bump re-anchor:
    //   1. SPOT 180         — rotate to face the back of the dead-end (now front).
    //   2. FWD_TO_BUMP      — slow forward until raw IR saturates (front wall).
    //   3. FWD(-startOff)   — reverse exactly 45 mm so robot center = cell center.
    //   4. DEADEND_RECAL    — re-zero gyro bias during the natural stillness.
    //   5. FWD(cellTicks)   — proceed to next cell, no pending offset.
    scriptPushSpot(TURN_RIGHT, 180.0f);
    scriptPushFwdToBump();
    scriptPushFwd(-T.startOffsetTicks);
    scriptPushDeadendRecal();
    pendingOffsetTicks = 0;          // robot now at cell-center, no carry
}

long fwd = T.ticksPerCell + pendingOffsetTicks;
pendingOffsetTicks = 0;
scriptPushFwd(fwd);
```

180° dead-end script grows from 3 steps (SPOT, REVERSE, FWD) to 5 steps (SPOT, FWD_TO_BUMP, FWD-reverse, RECAL, FWD). `MAX_SCRIPT = 8` covers it.

### `case RUN` dispatch — new branches

Mirrors the existing `PH_FWD_TO_WALL` pattern (early-return branch with its own `endNow()` advance).

**`PH_FWD_TO_BUMP` branch:**
```cpp
if (runPhase == PH_FWD_TO_BUMP) {
    sampleIR();
    int lfRaw    = irVal[0];
    int rfRaw    = irVal[3];
    int frontAvg = (lfRaw + rfRaw) / 2;
    long avgTicks = (tL + tR) / 2;

    auto endNow = [&](const char* reason) {
        stopMotors();
        Serial.printf("--- STEP END idx=%d/%d ph=FWD_TO_BUMP reason=%s avg=%ld frontRaw=%d tL=%ld tR=%ld ---\n",
                      scriptIdx + 1, scriptLen, reason, avgTicks, frontAvg, tL, tR);
        // ... same advance/transition logic as PH_FWD_TO_WALL endNow ...
    };

    if (frontAvg >= T.wallBumpRaw)         { endNow("BUMP");        break; }
    if (avgTicks >= runTarget)             { endNow("NO_BUMP_CAP"); break; }

    int throttle = T.stictionPwm;
    int yawBias  = (T.useImu && imuReady)
                    ? (int)(-T.yawHoldKp * (yawDeg - yawTargetDeg)) : 0;
    int pwmL = constrain(throttle - yawBias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    int pwmR = constrain(throttle + yawBias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    leftMotor.drive(pwmL);
    rightMotor.drive(pwmR);

    // (OLED + telemetry blocks omitted — mirror PH_FWD_TO_WALL)
    break;
}
```

**`PH_DEADEND_RECAL` branch:** identical to the prior design (motors off, stillness probe, cal + advance OR timeout + advance, zero yaw both paths).

### Pose math sanity check

- Cell pitch = 180 mm (`T.ticksPerCell = 1405` ticks ÷ 7.81 ticks/mm).
- Cell-center is 90 mm from each wall.
- `startOffsetTicks = 351` ≈ 45 mm.
- At boot: rear bumped against back wall → robot center is 45 mm forward of back wall → 45 mm short of cell-center (90 - 45 = 45). Forward 45 mm reaches center. **Currently calibrated.**
- After new front-bump: front IR plane within ~1-3 cm of front wall. By chassis symmetry (same physical wheelbase + same sensor offsets, but mirrored), robot center is 45 mm behind front wall when at the "bump" moment → 45 mm past cell-center. Reverse 45 mm lands at center.

If the chassis is asymmetric front-to-back (different sensor-to-axle vs back-to-axle distance), the `45 mm` figure on the front-bump side will need its own calibration. **Add a separate `T.frontBumpOffsetTicks` field if hardware testing shows the symmetric assumption is wrong.** Default to `startOffsetTicks` for now.

### Interactions

- **`yawTargetDeg` reset is still critical.** SPOT 180 accumulates 180° into `yawTargetDeg`. By the time RECAL fires, the new heading reference is established. RECAL zeroing both `yawDeg` and `yawTargetDeg` means subsequent forward motion holds the **post-RECAL** heading as straight. Correct.
- **`yawTargetDeg` between FWD_TO_BUMP and the reverse FWD.** FWD_TO_BUMP doesn't modify `yawTargetDeg` (no rotation). Neither does the reverse FWD. So both hold the post-180° heading. Good.
- **No effect on FAST mode.** Walls are fully known; if a dead-end appears, the same flow runs. Gyro re-cal during FAST is a bonus.
- **Legacy `PH_REVERSE_TO_BACK` stays compiled but unreachable.** Dead code — cleanup in a follow-up PR. Don't bundle the deletion with this change.

---

## Edge Cases

| Case | Behavior |
|------|----------|
| Front wall too far (e.g., mis-detected dead-end, actual cell is open ahead) | `PH_FWD_TO_BUMP` hits `fwdToBumpMaxTicks` safety cap → exits `NO_BUMP_CAP`. Reverse 45 mm still runs (now relative to wherever it stopped). Subsequent FWD may end up mis-aligned — this is graceful degradation, not a crash. |
| Front IR briefly saturates at long range (sensor anomaly) | `wallBumpRaw = 3500` corresponds to ≤ 3 cm per IR table; below this, raw values drop fast. False positive unlikely. If observed, tighten threshold (e.g., 3800 → ≤ 2 cm). |
| Robot already wedged against front wall when 180° completes | `PH_FWD_TO_BUMP` enters with IR already saturated → exits `BUMP` on first iteration. Effectively a no-op forward leg. Reverse 45 mm and RECAL still run. |
| IMU not ready during RECAL | `still` flag stays false → 500 ms timeout → yaw still zeroed, cal skipped. Same as prior spec. |
| Wobble during RECAL stillness window | `recal_stillStart` resets each iteration the threshold breaks. Same as prior spec. |
| Button-press anywhere in the sequence | Existing `buttonEdge()` check at top of `case RUN` aborts to IDLE. |

---

## Telemetry / Debug

```
--- STEP END idx=2/5 ph=FWD_TO_BUMP reason=BUMP avg=... frontRaw=3712 tL=... tR=... ---
--- STEP END idx=3/5 ph=FWD reason=SETTLED err=+0.50 tL=... tR=... yaw=...     (reverse leg)
[DEADEND] cal OK bias=-0.0123 deg/s (stillness held)
--- STEP END idx=4/5 ph=DEADEND_RECAL reason=DONE ---       (logged from RECAL endNow)
```

---

## Testing

1. **Build gate:** `pio run -e main` succeeds with no new warnings.
2. **Hardware single dead-end:** flash + place robot facing a 3-walled dead-end + run Explore. Watch for:
   - `--- STEP END ... ph=SPOT reason=SETTLED ...` (180°)
   - `--- STEP END ... ph=FWD_TO_BUMP reason=BUMP frontRaw=3500+ ...` (front wall hit)
   - `--- STEP END ... ph=FWD reason=SETTLED ...` with negative target (reverse leg)
   - `[DEADEND] cal OK bias=...`
   - Next FWD = single-cell ticks, holds heading straight (no 180° swing).
3. **Pose check:** mark the floor at cell-center before run. Verify robot center lands within ±5 mm of mark after the reverse leg. If consistently off in one direction, add `T.frontBumpOffsetTicks` and tune.
4. **No-bump fallback:** physically remove the dead-end front wall just before 180° fires. Verify `reason=NO_BUMP_CAP` log and robot continues without crash.
5. **Timeout fallback:** wobble robot during RECAL window. Verify `[DEADEND] cal SKIP (timeout)` log and motion continues.
6. **Multi-dead-end run:** maze with 2+ dead-ends. Each fires its own clean cycle.

---

## Out of Scope (Future Work)

- Removing the now-unused `PH_REVERSE_TO_BACK` source code (separate cleanup PR).
- Adding `T.frontBumpOffsetTicks` if hardware testing shows the symmetric-chassis assumption is wrong.
- Front-bump re-anchor at non-180° events (e.g., periodic re-anchor at known-blocked cells).
- IR cal refresh against the dead-end geometry.
- Persisting bias to NVS.

---

## Files Touched

- `src/main.cpp` — enum, `Tuning` struct, `buildMoveScript()`, two new script-push helpers, two new `case RUN` branches.

No new files. No header changes. No `platformio.ini` changes.
