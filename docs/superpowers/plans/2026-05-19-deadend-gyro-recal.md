# Dead-End Front-Bump Re-Anchor + Gyro Re-Cal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the 180° rear-bump anchor with a front-bump anchor + 45 mm reverse + gyro re-cal. Robot lands at cell-center using real IR contact instead of blind reverse-to-back-wall.

**Architecture:** Two new `RunPhase` variants in `src/main.cpp`. `PH_FWD_TO_BUMP` drives slow-forward until raw IR `(LF+RF)/2 >= wallBumpRaw` (default 3500). The reverse leg reuses existing `PH_FORWARD` with target = `-startOffsetTicks`. `PH_DEADEND_RECAL` waits for stillness and re-zeros `gyroBiasZ` via existing `calibrateGyroBias()`. The legacy `PH_REVERSE_TO_BACK` is no longer called from `buildMoveScript`; its code stays compiled for now and will be cleaned up in a follow-up PR.

**Tech Stack:** ESP32-S3 + Arduino-2.x + PlatformIO. Reuses existing helpers (`scriptPush`, `scriptPushFwd`, `scriptPushSpot`, `calibrateGyroBias`, `sampleIR`, `phaseEnter`).

**Spec:** [docs/superpowers/specs/2026-05-19-deadend-gyro-recal-design.md](../specs/2026-05-19-deadend-gyro-recal-design.md)

**Note on testing:** No host-side unit test framework in this repo. Test gates are:
- **Build gate:** `pio run -e main` succeeds with no new warnings.
- **Hardware gate:** flash + run + observe Serial logs match expected output.

Commits use project emoji-prefix convention.

---

### Task 1: Add `Tuning` struct fields

**Files:**
- Modify: `src/main.cpp` — `Tuning` struct (around line 214)

- [ ] **Step 1: Locate the `Tuning` struct's `backupOffsetMm` line**

Run: `grep -n "backupOffsetMm" src/main.cpp`
Expected: `float backupOffsetMm = 10.0f;` near line 214.

- [ ] **Step 2: Add new fields after `fwdToWallMaxTicks`**

Find:
```cpp
    // Safety cap on PH_FWD_TO_WALL (unused by the 180° anchor now, kept as
    // primitive in case it's wanted later).
    float    wallTouchDistMm       = 35.0f;
    long     fwdToWallMaxTicks     = 1600;
```

Replace with:
```cpp
    // Safety cap on PH_FWD_TO_WALL (unused by the 180° anchor now, kept as
    // primitive in case it's wanted later).
    float    wallTouchDistMm       = 35.0f;
    long     fwdToWallMaxTicks     = 1600;
    // PH_FWD_TO_BUMP — front-wall bump detection by raw IR (LF+RF)/2.
    // 3500 ≈ ≤ 3 cm per IRCal::IR_DIST_FRONT_AVG. Tighten to ~3800 for ≤ 2 cm.
    int      wallBumpRaw           = 3500;
    long     fwdToBumpMaxTicks     = 1600;
    // PH_DEADEND_RECAL — gyro bias re-cal during stillness window after the
    // front-bump anchor lands the robot at cell-center.
    uint32_t recalStillHoldMs      = 100;
    long     recalStillTicks       = 1;
    float    recalStillGz          = 1.0f;
    int      recalSamples          = 300;
    int      recalSampleDelayMs    = 2;
    uint32_t recalTimeoutMs        = 500;
```

- [ ] **Step 3: Build**

Run: `~/.platformio/penv/bin/pio run -e main`
Expected: SUCCESS, no new warnings.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "🔧 Tuning: add wallBumpRaw + recal fields for front-bump anchor"
```

---

### Task 2: Add two new `RunPhase` variants

**Files:**
- Modify: `src/main.cpp` — `RunPhase` enum (line 344)

- [ ] **Step 1: Locate the enum**

Run: `grep -n "enum RunPhase" src/main.cpp`
Expected: line 344.

- [ ] **Step 2: Add `PH_FWD_TO_BUMP` and `PH_DEADEND_RECAL`**

Find:
```cpp
enum RunPhase  { PH_FORWARD, PH_PIVOT, PH_SPOT, PH_FWD_TO_WALL, PH_REVERSE_TO_BACK };
```

Replace with:
```cpp
enum RunPhase  { PH_FORWARD, PH_PIVOT, PH_SPOT, PH_FWD_TO_WALL, PH_REVERSE_TO_BACK,
                 PH_FWD_TO_BUMP, PH_DEADEND_RECAL };
```

- [ ] **Step 3: Build**

Run: `~/.platformio/penv/bin/pio run -e main`
Expected: SUCCESS.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "✨ RunPhase: add PH_FWD_TO_BUMP and PH_DEADEND_RECAL"
```

---

### Task 3: Add two script-push helpers

**Files:**
- Modify: `src/main.cpp` — after existing `scriptPushReverseToBack` (around line 378)

- [ ] **Step 1: Locate the push helpers**

Run: `grep -n "scriptPushReverseToBack" src/main.cpp`
Expected: definition near line 376.

- [ ] **Step 2: Add the two new helpers after `scriptPushReverseToBack`**

Find:
```cpp
// Reverse-to-back-wall. Target is set at phase activation (front IR sample).
static void scriptPushReverseToBack() {
    scriptPush(PH_REVERSE_TO_BACK, 0, TURN_NONE);
}
```

Replace with:
```cpp
// Reverse-to-back-wall. Target is set at phase activation (front IR sample).
// Legacy: no longer called by buildMoveScript. Kept compiled in case the
// blind-reverse strategy is wanted later.
static void scriptPushReverseToBack() {
    scriptPush(PH_REVERSE_TO_BACK, 0, TURN_NONE);
}
// Forward-to-front-wall via raw IR threshold. Used by the 180° anchor to
// land the robot at a known geometric reference (front wall).
static void scriptPushFwdToBump() {
    scriptPush(PH_FWD_TO_BUMP, T.fwdToBumpMaxTicks, TURN_NONE);
}
// Dead-end gyro re-cal. No target — exits on stillness-held or timeout.
static void scriptPushDeadendRecal() {
    scriptPush(PH_DEADEND_RECAL, 0, TURN_NONE);
}
```

- [ ] **Step 3: Build (helpers unused so far — possible unused-function warning)**

Run: `~/.platformio/penv/bin/pio run -e main`
Expected: SUCCESS. Warnings OK; they go away in Task 4.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "✨ Script: add scriptPushFwdToBump + scriptPushDeadendRecal helpers"
```

---

### Task 4: Rewrite the 180° branch of `buildMoveScript()`

**Files:**
- Modify: `src/main.cpp` — `buildMoveScript()` diff==2 branch (lines 636-643)

- [ ] **Step 1: Locate the 180° branch**

Run: `grep -n "scriptPushReverseToBack()" src/main.cpp`
Expected: call near line 641 inside `buildMoveScript`.

- [ ] **Step 2: Replace the 180° branch contents**

Find:
```cpp
    } else if (diff == 2) {
        // 180° re-anchor: spot, then reverse until rear bumps the wall now
        // behind. PH_REVERSE_TO_BACK measures front IR at activation time
        // and computes its own (negative) tick target = frontMm + 1.5 cm.
        scriptPushSpot(TURN_RIGHT, 180.0f);
        scriptPushReverseToBack();
        pendingOffsetTicks = T.startOffsetTicks;     // next fwd lands at next cell center
    }
```

Replace with:
```cpp
    } else if (diff == 2) {
        // 180° front-bump re-anchor:
        //   1. SPOT 180         — rotate to face what is now the front wall.
        //   2. FWD_TO_BUMP      — slow forward until raw IR saturates.
        //   3. FWD(-startOff)   — reverse exactly 45 mm so robot center
        //                          coincides with cell-center.
        //   4. DEADEND_RECAL    — re-zero gyro bias during the natural
        //                          stillness window after the reverse settles.
        //   5. FWD(cellTicks)   — proceed to next cell, no pending offset.
        scriptPushSpot(TURN_RIGHT, 180.0f);
        scriptPushFwdToBump();
        scriptPushFwd(-T.startOffsetTicks);
        scriptPushDeadendRecal();
        pendingOffsetTicks = 0;                       // robot at cell-center already
    }
```

- [ ] **Step 3: Build**

Run: `~/.platformio/penv/bin/pio run -e main`
Expected: SUCCESS. Unused-function warnings from Task 3 should now be gone except for `scriptPushReverseToBack` (intentionally unused).

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "✨ buildMoveScript: 180° uses front-bump anchor instead of rear-bump"
```

---

### Task 5: Add `case RUN` dispatch for `PH_FWD_TO_BUMP`

**Files:**
- Modify: `src/main.cpp` — `case RUN` block, directly after the existing `PH_FWD_TO_WALL` branch (around line 996)

- [ ] **Step 1: Locate the `PH_FWD_TO_WALL` branch closing**

Run: `grep -n "if (runPhase == PH_FWD_TO_WALL)" src/main.cpp`
Expected: line 939. Branch closes at `break;` near line 995, followed by `bool imuMode = ...`.

- [ ] **Step 2: Insert `PH_FWD_TO_BUMP` branch directly after the `PH_FWD_TO_WALL` branch**

Find (the closing `break; }` of `PH_FWD_TO_WALL` followed by `bool imuMode`):
```cpp
            break;
        }

        bool imuMode = T.useImu && imuReady && (runPhase != PH_FORWARD);
```

Replace with:
```cpp
            break;
        }

        // ── PH_FWD_TO_BUMP: slow forward, raw IR (LF+RF)/2 threshold stop. ──
        // Used by the 180° anchor to land the robot against the (now) front
        // wall. Uses raw IR instead of estimateFrontDistMM because the LUT
        // clamps to 10 mm minimum; raw values keep rising past that point.
        if (runPhase == PH_FWD_TO_BUMP) {
            sampleIR();
            int  lfRaw    = irVal[0];
            int  rfRaw    = irVal[3];
            int  frontAvg = (lfRaw + rfRaw) / 2;
            long avgTicks = (tL + tR) / 2;

            auto endNow = [&](const char* reason) {
                stopMotors();
                Serial.printf("--- STEP END idx=%d/%d ph=FWD_TO_BUMP reason=%s avg=%ld frontRaw=%d tL=%ld tR=%ld ---\n",
                              scriptIdx + 1, scriptLen, reason, avgTicks, frontAvg, tL, tR);
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

            if (frontAvg >= T.wallBumpRaw) { endNow("BUMP");        break; }
            if (avgTicks >= runTarget)     { endNow("NO_BUMP_CAP"); break; }

            int throttle = T.stictionPwm;
            int yawBias  = (T.useImu && imuReady)
                            ? (int)(-T.yawHoldKp * (yawDeg - yawTargetDeg)) : 0;
            int pwmL = constrain(throttle - yawBias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            int pwmR = constrain(throttle + yawBias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            leftMotor.drive(pwmL);
            rightMotor.drive(pwmR);

            static uint32_t lastOled = 0;
            if (millis() - lastOled > 150) {
                oledRun(avgTicks, runTarget, tL, tR);
                lastOled = millis();
            }
            if (T.telemetry) {
                static uint32_t lastTel = 0;
                if (millis() - lastTel > 80) {
                    Serial.printf("t=%lu ph=FWD2BUMP avg=%ld frontRaw=%d thr=%d yaw=%+.2f tL=%ld tR=%ld\n",
                                  (unsigned long)millis(), avgTicks, frontAvg, throttle, yawDeg, tL, tR);
                    lastTel = millis();
                }
            }
            break;
        }

        bool imuMode = T.useImu && imuReady && (runPhase != PH_FORWARD);
```

- [ ] **Step 3: Build**

Run: `~/.platformio/penv/bin/pio run -e main`
Expected: SUCCESS. Flash usage slightly up.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "✨ RUN: handle PH_FWD_TO_BUMP — slow forward to raw-IR threshold"
```

---

### Task 6: Add `case RUN` dispatch for `PH_DEADEND_RECAL`

**Files:**
- Modify: `src/main.cpp` — `case RUN` block, directly after the new `PH_FWD_TO_BUMP` branch from Task 5

- [ ] **Step 1: Verify the `PH_FWD_TO_BUMP` branch from Task 5 is present**

Run: `grep -n "PH_FWD_TO_BUMP" src/main.cpp`
Expected: enum line, helper, push call in buildMoveScript, **and** the new dispatch branch in case RUN.

- [ ] **Step 2: Insert `PH_DEADEND_RECAL` branch directly after the `PH_FWD_TO_BUMP` branch closing**

Find (closing of the FWD_TO_BUMP branch followed by `bool imuMode`):
```cpp
            break;
        }

        bool imuMode = T.useImu && imuReady && (runPhase != PH_FORWARD);
```

Replace with:
```cpp
            break;
        }

        // ── PH_DEADEND_RECAL: motors coast, wait for stillness, re-zero
        // gyro bias via calibrateGyroBias(). Falls through to next script
        // step on success OR on timeout (yaw zeroed either way so the next
        // PH_FORWARD has a fresh heading reference).
        if (runPhase == PH_DEADEND_RECAL) {
            static long     recal_prevTL      = 0;
            static long     recal_prevTR      = 0;
            static uint32_t recal_stillStart  = 0;
            static uint32_t recal_phaseStart  = 0;
            static bool     recal_firstEntry  = true;

            if (recal_firstEntry) {
                stopMotors();
                recal_phaseStart = millis();
                recal_prevTL     = leftEnc.getTicks();
                recal_prevTR     = rTicks();
                recal_stillStart = 0;
                recal_firstEntry = false;
            }

            long tLnow = leftEnc.getTicks();
            long tRnow = rTicks();
            bool encStill = (labs(tLnow - recal_prevTL) <= T.recalStillTicks)
                         && (labs(tRnow - recal_prevTR) <= T.recalStillTicks);
            recal_prevTL = tLnow; recal_prevTR = tRnow;
            bool gzStill = fabsf(gzFilt) < T.recalStillGz;
            bool still   = encStill && gzStill && imuReady;

            if (still) { if (recal_stillStart == 0) recal_stillStart = millis(); }
            else       { recal_stillStart = 0; }

            bool stillHeld = recal_stillStart
                          && (millis() - recal_stillStart) >= T.recalStillHoldMs;
            bool timedOut  = (millis() - recal_phaseStart) >= T.recalTimeoutMs;

            if (stillHeld || timedOut) {
                if (stillHeld) {
                    calibrateGyroBias(T.recalSamples, T.recalSampleDelayMs);
                    Serial.printf("[DEADEND] cal OK bias=%.4f deg/s (stillness held)\n",
                                  gyroBiasZ);
                } else {
                    Serial.printf("[DEADEND] cal SKIP (timeout) — yaw zeroed anyway\n");
                }
                yawDeg       = 0.0f;
                yawTargetDeg = 0.0f;
                recal_firstEntry = true;

                Serial.printf("--- STEP END idx=%d/%d ph=DEADEND_RECAL reason=%s ---\n",
                              scriptIdx + 1, scriptLen, stillHeld ? "CAL_OK" : "TIMEOUT");

                if (scriptIdx + 1 >= scriptLen) {
                    robotRow = plannedRow; robotCol = plannedCol; robotHeading = plannedHeading;
                    runTurnDir = TURN_NONE;
                    Serial.printf("--- MOVE DONE pos=(%d,%d,%c) ---\n",
                                  robotRow, robotCol, "NESW"[robotHeading]);
                    if (exploreMode || fastRunMode) state = EXPLORE_THINK;
                    else { menuEncRef = rightEnc.getTicks(); oledMenu(); state = IDLE; }
                    break;
                }
                scriptIdx++;
                PhaseStep& next = script[scriptIdx];
                runPhase   = next.phase;
                runTarget  = next.target;
                runTurnDir = next.dir;
                pid.reset();
                phaseEnter();
            }
            break;
        }

        bool imuMode = T.useImu && imuReady && (runPhase != PH_FORWARD);
```

- [ ] **Step 3: Build**

Run: `~/.platformio/penv/bin/pio run -e main`
Expected: SUCCESS.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "✨ RUN: handle PH_DEADEND_RECAL with stillness-detect + gyro re-zero"
```

---

### Task 7: Hardware verification — single dead-end happy path

**Files:**
- None — observation only.

- [ ] **Step 1: Flash + monitor**

Run: `~/.platformio/penv/bin/pio run -e main -t upload`
Then: `~/.platformio/penv/bin/pio device monitor`

If wrong port auto-detected (e.g., `/dev/cu.MindWaveMobile`), pass `--upload-port /dev/cu.usbmodem<XYZ>`.

- [ ] **Step 2: Place robot facing a 3-walled dead-end, start EXPLORE from menu**

Expected Serial sequence at the dead-end (5-step 180° script):
```
--- STEP END idx=1/5 ph=SPOT  reason=SETTLED ...           (180° rotate)
--- STEP END idx=2/5 ph=FWD_TO_BUMP reason=BUMP frontRaw=3500+ ...
--- STEP END idx=3/5 ph=FWD   reason=SETTLED err=...       (reverse leg, negative target)
[DEADEND] cal OK bias=-0.0XXX deg/s (stillness held)
--- STEP END idx=4/5 ph=DEADEND_RECAL reason=CAL_OK ---
--- STEP END idx=5/5 ph=FWD   reason=SETTLED ...           (next-cell forward, no offset)
```

The `[DEADEND]` line should appear ~100-200 ms after the reverse leg settles.

- [ ] **Step 3: Pose check**

Mark cell-center on the floor with tape before starting. After the reverse leg completes, robot center should land within ±5 mm of mark. Note the offset direction if biased.

If consistently off by more than 1 cm in one direction: the chassis-symmetry assumption is wrong. Add `T.frontBumpOffsetTicks` to `Tuning` (default = `startOffsetTicks`) and use it in `scriptPushFwd(-T.frontBumpOffsetTicks)` in `buildMoveScript`. Then re-measure.

- [ ] **Step 4: Heading check**

Forward leg after `[DEADEND]` must hold heading straight (no swing). If it rotates 180° on the spot, `yawTargetDeg` was not zeroed — re-check Task 6.

- [ ] **Step 5: No commit (verification only).**

---

### Task 8: Hardware verification — no-bump fallback

**Files:**
- None — observation only.

- [ ] **Step 1: Stage a no-bump scenario**

Either:
- Open the maze ahead of an expected 180° turn (so flood-fill commands a 180° but no front wall exists), OR
- Cover the front IR sensors so they read low.

- [ ] **Step 2: Run Explore and watch for `NO_BUMP_CAP`**

Expected:
```
--- STEP END idx=2/5 ph=FWD_TO_BUMP reason=NO_BUMP_CAP avg=1600 frontRaw=<low> ...
```

Then the rest of the script (reverse 45 mm + RECAL + next FWD) still runs. The pose may be off — graceful degradation only, not a crash.

- [ ] **Step 3: No commit.**

---

### Task 9: Hardware verification — RECAL timeout fallback

**Files:**
- None — observation only.

- [ ] **Step 1: Force the RECAL timeout path**

Reset robot. Start Explore. When the reverse leg of a 180° settles and you see motors coast, **gently wobble the chassis** through the 500 ms RECAL window so neither encoder nor gyro reads still.

Expected:
```
[DEADEND] cal SKIP (timeout) — yaw zeroed anyway
--- STEP END idx=4/5 ph=DEADEND_RECAL reason=TIMEOUT ---
```

- [ ] **Step 2: Verify next FWD still runs**

Forward leg after timeout must still execute. No hang, no IDLE drop.

- [ ] **Step 3: No commit.**

---

### Task 10: Hardware verification — multi-dead-end run

**Files:**
- None — observation only.

- [ ] **Step 1: Run a maze with ≥ 2 dead-ends**

Watch for one `[DEADEND] cal OK ...` per 180° event. Bias values may differ across events (this is the whole point — bias drift is being captured).

- [ ] **Step 2: Verify no stuck state between events**

Each event must fire cleanly. If a second event hangs, `recal_firstEntry` reset logic is broken — re-check Task 6.

- [ ] **Step 3: No commit.**

---

## Self-Review

**Spec coverage:**
- Two new `RunPhase` variants → Task 2 ✓
- `Tuning` field additions → Task 1 ✓
- Two new script-push helpers → Task 3 ✓
- `buildMoveScript()` 180° branch rewrite → Task 4 ✓
- `PH_FWD_TO_BUMP` dispatch + raw IR threshold + safety cap → Task 5 ✓
- `PH_DEADEND_RECAL` dispatch + stillness + cal + timeout fallback → Task 6 ✓
- `yawDeg`/`yawTargetDeg` reset on both success and timeout → Task 6 ✓
- Single dead-end hardware test → Task 7 ✓
- No-bump fallback hardware test → Task 8 ✓
- Timeout fallback hardware test → Task 9 ✓
- Multi-dead-end hardware test → Task 10 ✓
- Pose calibration check (with `frontBumpOffsetTicks` follow-up note) → Task 7 Step 3 ✓

**Placeholder scan:** No TBDs. Code blocks complete in every step.

**Type consistency:**
- `PH_FWD_TO_BUMP` and `PH_DEADEND_RECAL` spelled identically across Tasks 2/3/4/5/6 ✓
- `scriptPushFwdToBump` / `scriptPushDeadendRecal` match across Task 3 (definition) and Task 4 (calls) ✓
- All `T.recal*`, `T.wallBumpRaw`, `T.fwdToBumpMaxTicks` field names match across Tasks 1, 5, 6 ✓
- `calibrateGyroBias(T.recalSamples, T.recalSampleDelayMs)` signature matches existing `static void calibrateGyroBias(int N = 300, int sampleDelayMs = 2)` ✓
- `gzFilt`, `gyroBiasZ`, `yawDeg`, `yawTargetDeg`, `imuReady`, `leftEnc`, `rTicks()`, `stopMotors()`, `sampleIR()`, `irVal[]`, `pid.reset()`, `phaseEnter()`, `oledRun`, `oledMenu`, `menuEncRef`, `T.stictionPwm`, `T.yawHoldKp`, `MOTOR_PWM_MAX` all exist in current `main.cpp` with the signatures used ✓
- `scriptPushFwd(-T.startOffsetTicks)` works because existing `scriptPushFwd(long)` accepts signed `long`, and the standard `PH_FORWARD` PID handles negative targets per CLAUDE.md ✓
