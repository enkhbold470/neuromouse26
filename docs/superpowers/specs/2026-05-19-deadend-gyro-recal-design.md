# Dead-End Gyro Re-Calibration

**Date:** 2026-05-19
**Status:** Approved design; implementation pending.
**Branch:** `inky`

---

## Problem

The MPU-6500 Z-axis bias `gyroBiasZ` is captured once at boot via `calibrateGyroBias(300, 2)` (300 samples × 2 ms). During a maze run the bias drifts (temperature, vibration, settling). The yaw integration in `updateYaw()` accumulates this drift directly into `yawDeg`, which the position PID uses as its primary heading reference during `PH_FORWARD`. A drift of even 0.05 °/s over a 30 s run is ~1.5° of accumulated yaw error — enough to bias every centering/centering decision and degrade flood-fill execution.

The robot already passes through one momentary-stillness opportunity per 180° turn: after `PH_REVERSE_TO_BACK` settles, the robot is rear-bumped against a wall with motors coasted. This is a free re-cal window.

## Goal

At every 180° dead-end, re-zero the gyro bias using the natural stillness window after rear-bump settles. Same hardware/software path as the existing menu-driven `GYRO_CAL` state — reused, not forked.

## Non-Goals

- Position correction. Robot pose is **not** changed; rear-bumped pose is already well-defined and handled by `pendingOffsetTicks = startOffsetTicks`. This spec is gyro-only.
- Cal at every cell-arrival (would add 700 ms latency per cell). Only at 180° dead-ends, which are rare in the 6×3 flood-fill region.
- Persisting `gyroBiasZ` to NVS. Bias is per-power-cycle.

---

## Design

### New `RunPhase` variant

```cpp
enum RunPhase {
    PH_FORWARD, PH_PIVOT, PH_SPOT, PH_FWD_TO_WALL, PH_REVERSE_TO_BACK,
    PH_DEADEND_RECAL  // NEW
};
```

`PH_DEADEND_RECAL` has no encoder/yaw target. Its exit conditions are time-based:
- **Success:** stillness held continuously for `recalStillHoldMs` → run `calibrateGyroBias()` → advance.
- **Timeout:** `millis() - phaseStartMs >= recalTimeoutMs` without a successful hold → skip cal → advance.

In either case, both `yawDeg` and `yawTargetDeg` are zeroed before the next phase starts.

### Script insertion

In `buildMoveScript()`, diff==2 branch:

```cpp
// Before:
scriptPushSpot(TURN_RIGHT, 180.0f);
scriptPushReverseToBack();
pendingOffsetTicks = T.startOffsetTicks;

// After:
scriptPushSpot(TURN_RIGHT, 180.0f);
scriptPushReverseToBack();
scriptPushDeadendRecal();              // NEW
pendingOffsetTicks = T.startOffsetTicks;
```

180° dead-end script grows from 3 steps to 4 steps. `MAX_SCRIPT = 8` is unchanged and still adequate.

A new push helper mirrors the existing pattern:

```cpp
static void scriptPushDeadendRecal() {
    scriptPush(PH_DEADEND_RECAL, 0, TURN_NONE);  // target unused
}
```

### `Tuning` struct additions

```cpp
// Dead-end gyro re-cal
uint32_t recalStillHoldMs    = 100;   // continuous stillness required
long     recalStillTicks     = 1;     // max |Δticks| per sample window
float    recalStillGz        = 1.0f;  // max |gzFilt| dps for "still"
int      recalSamples        = 300;   // forwarded to calibrateGyroBias()
int      recalSampleDelayMs  = 2;     // forwarded to calibrateGyroBias()
uint32_t recalTimeoutMs      = 500;   // fallback: skip cal, zero yaw, advance
```

Values mirror the existing `GYRO_CAL` constants exactly. No re-tuning.

### `case RUN` dispatch branch

Mirrors the existing `PH_FWD_TO_WALL` branch (early-return-on-runPhase, then `endNow()`-style advance):

```cpp
if (runPhase == PH_DEADEND_RECAL) {
    static long     prevTL = 0, prevTR = 0;
    static uint32_t stillStart = 0;
    static uint32_t phaseStartMs = 0;
    static bool     firstEntry  = true;

    if (firstEntry) {
        stopMotors();
        phaseStartMs = millis();
        prevTL = leftEnc.getTicks();
        prevTR = rTicks();
        stillStart = 0;
        firstEntry = false;
    }

    // Stillness probe
    long tLnow = leftEnc.getTicks();
    long tRnow = rTicks();
    bool encStill = (labs(tLnow - prevTL) <= T.recalStillTicks)
                 && (labs(tRnow - prevTR) <= T.recalStillTicks);
    prevTL = tLnow; prevTR = tRnow;
    bool gzStill = fabsf(gzFilt) < T.recalStillGz;
    bool still   = encStill && gzStill && imuReady;

    if (still) { if (stillStart == 0) stillStart = millis(); }
    else       { stillStart = 0; }

    auto finish = [&](bool calOk, const char* reason) {
        if (calOk) {
            calibrateGyroBias(T.recalSamples, T.recalSampleDelayMs);
            Serial.printf("[DEADEND] cal OK bias=%.4f deg/s (%s)\n",
                          gyroBiasZ, reason);
        } else {
            Serial.printf("[DEADEND] cal SKIP (%s) — yaw zeroed anyway\n", reason);
        }
        yawDeg       = 0.0f;
        yawTargetDeg = 0.0f;
        firstEntry = true;     // reset locals for next entry
        // advance — same code as endPhase/endNow in other branches
        if (scriptIdx + 1 >= scriptLen) {
            robotRow = plannedRow; robotCol = plannedCol; robotHeading = plannedHeading;
            runTurnDir = TURN_NONE;
            Serial.printf("--- MOVE DONE pos=(%d,%d,%c) ---\n",
                          robotRow, robotCol, "NESW"[robotHeading]);
            if (exploreMode || fastRunMode) state = EXPLORE_THINK;
            else { menuEncRef = rightEnc.getTicks(); oledMenu(); state = IDLE; }
            return;
        }
        scriptIdx++;
        PhaseStep& next = script[scriptIdx];
        runPhase   = next.phase;
        runTarget  = next.target;
        runTurnDir = next.dir;
        pid.reset();
        phaseEnter();
    };

    if (stillStart && (millis() - stillStart) >= T.recalStillHoldMs) {
        finish(/*calOk=*/true, "stillness held");
        break;
    }
    if (millis() - phaseStartMs >= T.recalTimeoutMs) {
        finish(/*calOk=*/false, "timeout");
        break;
    }
    break;  // keep waiting
}
```

`firstEntry` is reset to `true` inside `finish()` so a subsequent dead-end during the same run starts fresh.

### Interactions

- **`yawTargetDeg` reset is critical.** After `SPOT 180`, `yawTargetDeg += −180` (or +180 for left). The forward yaw-hold (`-T.yawHoldKp * (yawDeg - yawTargetDeg)`) reads this offset. After cal sets `yawDeg = 0`, `yawTargetDeg` must also be 0 or the very next `PH_FORWARD` will try to swing the robot back 180°.
- **`pid.reset()` already happens** in `phaseEnter()` via the advance path. No PID state leaks from RECAL into FWD.
- **`phaseEnter()` runs for the new RECAL phase too.** It calls `onPhaseActivate()` — which is a no-op for `PH_DEADEND_RECAL` (only handles `PH_REVERSE_TO_BACK`). Safe.
- **No effect on FAST mode.** Fast run loads pre-explored walls and rarely encounters dead-ends. If it does, the cal still runs. If `fastRunMode` ever needs to skip cal, gate the script insertion on `!fastRunMode` later — not now.

---

## Edge Cases

| Case | Behavior |
|------|----------|
| IMU not ready (`imuReady == false`) | `still` flag stays false → timeout path → yaw still zeroed, cal skipped. Safe. |
| Robot wobbles within stillness window | `stillStart` resets each iteration the threshold breaks. Timer naturally re-arms. |
| Robot completely stuck (e.g., wedged) | After `recalTimeoutMs` (500 ms) the script advances regardless. RUN exit via button-press still works (handled at top of `case RUN`). |
| Stillness achieved instantly (≤ 1 ms after entry) | Hold timer starts → 100 ms later cal runs. Total ~705 ms phase. Expected case. |
| Multiple dead-ends in one run | `firstEntry` resets inside `finish()` — each entry initializes its own locals cleanly. |
| Button-press during recal | Existing `buttonEdge()` check at top of `case RUN` aborts → IDLE. Cal interrupted safely (no flash writes in progress). |

---

## Telemetry / Debug

One Serial line per dead-end event:
```
[DEADEND] cal OK bias=-0.0123 deg/s (stillness held)
[DEADEND] cal SKIP (timeout) — yaw zeroed anyway
```

No OLED update during recal — phase is brief (≤ 700 ms) and the prior `PH_REVERSE_TO_BACK` settle screen lingers. Adding OLED writes would just contend for the I2C bus during the cal window.

---

## Testing

1. **`wall-follow-pcnt` env** — not applicable (no 180° in that script).
2. **`main` env, EXPLORE mode** — induce a dead-end at boot by placing the robot in front of a known-bounded dead-end cell. Watch for:
   - `[BACKUP]` print after SPOT 180 completes
   - `[DEADEND] cal OK bias=...` print ~700 ms after the rear-bump settles
   - Next FWD leg holds heading correctly (no 180° swing)
3. **`main` env, IMU disabled at boot** — physically interrupt MPU-6500 startup (e.g., short cal time). Expect `[DEADEND] cal SKIP (timeout)` and forward leg still runs (yaw hold inactive when `!imuReady`).
4. **Multiple dead-ends in one run** — set up a maze with 2+ dead-ends. Verify each fires its own cal cleanly without stuck state.
5. **Hold robot in hand mid-recal** — manually wobble during the recal window. Verify timer resets, cal eventually completes when stilled, or times out cleanly if held in motion.

---

## Out of Scope (Future Work)

- Front-IR pose correction at the same window (option B from brainstorm — keep simple now).
- IR cal refresh against known dead-end geometry.
- Periodic re-cal at every cell-arrival (option not selected — too much latency).
- Persisting bias to NVS.
- OLED progress indicator during recal.

---

## Files Touched

- `src/main.cpp` — `RunPhase` enum, `Tuning` struct, `buildMoveScript()`, `scriptPushDeadendRecal()` helper, new `case RUN` branch for `PH_DEADEND_RECAL`.

No new files. No header changes. No `platformio.ini` changes.
