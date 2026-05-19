# Dead-End Gyro Re-Cal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-zero gyro bias during the natural stillness window after `PH_REVERSE_TO_BACK` settles at every 180° turn.

**Architecture:** Add a new `PH_DEADEND_RECAL` phase to the script executor. After `SPOT 180 → REVERSE_TO_BACK` settles, robot is rear-bumped against a wall with motors coasted. RECAL phase: wait for stillness (reusing `GYRO_CAL` criteria), call existing `calibrateGyroBias()`, zero `yawDeg` and `yawTargetDeg`, advance to next phase. 500 ms timeout falls through (yaw still zeroed). All changes contained in `src/main.cpp`.

**Tech Stack:** ESP32-S3 + Arduino-2.x + PlatformIO. Reuses existing `calibrateGyroBias()`, `Tuning` struct, `RunPhase` enum, `case RUN` dispatch.

**Spec:** [docs/superpowers/specs/2026-05-19-deadend-gyro-recal-design.md](../specs/2026-05-19-deadend-gyro-recal-design.md)

**Note on testing:** This repo has no host-side unit test framework (no `unity`, no `Catch2`, no `pio test`). All `test/*.cpp` files are standalone Arduino sketches that flash to hardware. Verification gates are therefore:
- **Build gate:** `pio run -e main` succeeds with no new warnings.
- **Hardware gate:** flash + run + observe Serial logs match expected output.

Commits use the project's existing emoji-prefix convention (`✨`, `🔧`, `🐛`, `📄`).

---

### Task 1: Add `Tuning` struct fields for re-cal

**Files:**
- Modify: `src/main.cpp` — `Tuning` struct (around line 214, where `backupOffsetMm` lives)

- [ ] **Step 1: Locate the `Tuning` struct**

Run: `grep -n "backupOffsetMm" src/main.cpp`
Expected: line 214 contains `float backupOffsetMm = 5.0f;`

- [ ] **Step 2: Add the six new fields directly after `backupOffsetMm`**

Find:
```cpp
    float    backupOffsetMm        = 5.0f;
    // Safety cap on PH_FWD_TO_WALL (unused by the 180° anchor now, kept as
    // primitive in case it's wanted later).
```

Replace with:
```cpp
    float    backupOffsetMm        = 5.0f;
    // Dead-end gyro re-cal (PH_DEADEND_RECAL). Values mirror GYRO_CAL state.
    uint32_t recalStillHoldMs      = 100;
    long     recalStillTicks       = 1;
    float    recalStillGz          = 1.0f;
    int      recalSamples          = 300;
    int      recalSampleDelayMs    = 2;
    uint32_t recalTimeoutMs        = 500;
    // Safety cap on PH_FWD_TO_WALL (unused by the 180° anchor now, kept as
    // primitive in case it's wanted later).
```

- [ ] **Step 3: Build to confirm field additions compile**

Run: `~/.platformio/penv/bin/pio run -e main`
Expected: SUCCESS, no new warnings.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "🔧 Tuning: add recal fields for dead-end gyro re-cal"
```

---

### Task 2: Add `PH_DEADEND_RECAL` enum variant

**Files:**
- Modify: `src/main.cpp` — `RunPhase` enum (line 344)

- [ ] **Step 1: Locate the enum**

Run: `grep -n "enum RunPhase" src/main.cpp`
Expected: line 344 contains `enum RunPhase  { PH_FORWARD, PH_PIVOT, PH_SPOT, PH_FWD_TO_WALL, PH_REVERSE_TO_BACK };`

- [ ] **Step 2: Add the new variant**

Find:
```cpp
enum RunPhase  { PH_FORWARD, PH_PIVOT, PH_SPOT, PH_FWD_TO_WALL, PH_REVERSE_TO_BACK };
```

Replace with:
```cpp
enum RunPhase  { PH_FORWARD, PH_PIVOT, PH_SPOT, PH_FWD_TO_WALL, PH_REVERSE_TO_BACK,
                 PH_DEADEND_RECAL };
```

- [ ] **Step 3: Build to confirm enum compiles**

Run: `~/.platformio/penv/bin/pio run -e main`
Expected: SUCCESS.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "✨ RunPhase: add PH_DEADEND_RECAL variant"
```

---

### Task 3: Add `scriptPushDeadendRecal()` helper

**Files:**
- Modify: `src/main.cpp` — directly after `scriptPushReverseToBack()` (around line 378)

- [ ] **Step 1: Locate the existing push helper**

Run: `grep -n "scriptPushReverseToBack" src/main.cpp`
Expected: definition near line 376 and call near line 641.

- [ ] **Step 2: Add the new helper after `scriptPushReverseToBack`**

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
static void scriptPushReverseToBack() {
    scriptPush(PH_REVERSE_TO_BACK, 0, TURN_NONE);
}
// Dead-end gyro re-cal. No target — exit on stillness-held or timeout.
static void scriptPushDeadendRecal() {
    scriptPush(PH_DEADEND_RECAL, 0, TURN_NONE);
}
```

- [ ] **Step 3: Build (helper unused so far — expect a single "unused function" warning at worst)**

Run: `~/.platformio/penv/bin/pio run -e main`
Expected: SUCCESS. Warning OK if it appears (will go away in Task 4 when the function is called).

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "✨ Script: add scriptPushDeadendRecal helper"
```

---

### Task 4: Insert RECAL step into 180° branch of `buildMoveScript()`

**Files:**
- Modify: `src/main.cpp` — `buildMoveScript()` diff==2 branch (around line 636-643)

- [ ] **Step 1: Locate the 180° branch**

Run: `grep -n "scriptPushReverseToBack()" src/main.cpp`
Expected: call appears near line 641 inside `buildMoveScript`.

- [ ] **Step 2: Insert the RECAL push between REVERSE and `pendingOffsetTicks`**

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
        // 180° re-anchor: spot, then reverse until rear bumps the wall now
        // behind. PH_REVERSE_TO_BACK measures front IR at activation time
        // and computes its own (negative) tick target = frontMm + 1.5 cm.
        // After rear-bump settles, PH_DEADEND_RECAL re-zeroes the gyro bias
        // using the natural stillness window.
        scriptPushSpot(TURN_RIGHT, 180.0f);
        scriptPushReverseToBack();
        scriptPushDeadendRecal();
        pendingOffsetTicks = T.startOffsetTicks;     // next fwd lands at next cell center
    }
```

- [ ] **Step 3: Build**

Run: `~/.platformio/penv/bin/pio run -e main`
Expected: SUCCESS. The "unused function" warning from Task 3 should be gone.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "✨ buildMoveScript: insert PH_DEADEND_RECAL after 180° reverse"
```

---

### Task 5: Add `case RUN` dispatch branch for `PH_DEADEND_RECAL`

**Files:**
- Modify: `src/main.cpp` — `case RUN` block, directly after the `PH_FWD_TO_WALL` branch (around line 996)

- [ ] **Step 1: Locate the `PH_FWD_TO_WALL` branch closing**

Run: `grep -n "if (runPhase == PH_FWD_TO_WALL)" src/main.cpp`
Expected: line 939. The branch ends with a `break;` near line 995.

- [ ] **Step 2: Insert the RECAL branch immediately after the `PH_FWD_TO_WALL` branch closes**

Find (look for the closing `break;` of the FWD_TO_WALL branch followed by `bool imuMode = ...`):
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
        // gyro bias using calibrateGyroBias(). Falls through to next script
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
                recal_firstEntry = true;     // reset locals for next dead-end

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
Expected: SUCCESS. Flash usage roughly unchanged (small addition).

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "✨ RUN: handle PH_DEADEND_RECAL with stillness-detect + cal"
```

---

### Task 6: Hardware verification — single dead-end

**Files:**
- None — observation only.

- [ ] **Step 1: Flash production firmware**

Run: `~/.platformio/penv/bin/pio run -e main -t upload`
Expected: upload succeeds, post-upload chime plays via `tools/notify_upload.py`.

If upload picks the wrong serial port (e.g., `/dev/cu.MindWaveMobile`), pass `--upload-port`:
```bash
~/.platformio/penv/bin/pio run -e main -t upload --upload-port /dev/cu.usbmodem<XYZ>
```

- [ ] **Step 2: Open Serial monitor**

Run: `~/.platformio/penv/bin/pio device monitor`
Expected: 115200 baud, banner prints.

- [ ] **Step 3: Set up a known dead-end and start EXPLORE mode**

Physically place the robot facing a 3-walled dead-end. From the menu, select **Explore**. Watch Serial output.

Expected sequence near the dead-end:
```
--- STEP END idx=1/4 ph=SPOT reason=SETTLED err=... yaw=...
[BACKUP] frontMm=... offsetMm=5.0 → target=-... ticks (reverse)
--- STEP END idx=2/4 ph=FWD reason=SETTLED err=... yaw=...
[DEADEND] cal OK bias=-0.0XXX deg/s (stillness held)
--- STEP END idx=3/4 ph=...     (FWD that follows RECAL)
```

The `[DEADEND]` line should appear within ~200 ms of the rear-bump settle line.

- [ ] **Step 4: Verify next FWD does not swing 180°**

The forward leg immediately after `[DEADEND]` must hold heading straight. If the robot rotates ~180° instead of driving forward, `yawTargetDeg` was not zeroed correctly — revisit Task 5.

- [ ] **Step 5: Document the observed bias value in the spec (optional)**

If the bias value is interesting (e.g., very different from boot bias), note it in the spec's "Testing" section. Skip otherwise.

- [ ] **Step 6: Commit (only if Step 5 ran)**

```bash
git add docs/superpowers/specs/2026-05-19-deadend-gyro-recal-design.md
git commit -m "📄 spec: note observed dead-end bias from hardware run"
```

---

### Task 7: Hardware verification — timeout fallback

**Files:**
- None — observation only.

- [ ] **Step 1: Force the timeout path**

Reset the robot. Start an Explore run. When the robot reaches the dead-end and you hear/see the rear-bump settle, **gently nudge the chassis** so neither encoder nor gyro is still. Hold the wobble through the 500 ms window.

Expected Serial output:
```
[DEADEND] cal SKIP (timeout) — yaw zeroed anyway
```

- [ ] **Step 2: Verify next FWD still runs**

The forward leg after the timeout must still execute. Heading hold may be slightly off (since cal didn't run), but motion must continue — no hang, no IDLE drop.

- [ ] **Step 3: No commit required.**

---

### Task 8: Hardware verification — multiple dead-ends per run

**Files:**
- None — observation only.

- [ ] **Step 1: Run a maze that contains 2+ dead-ends**

Either rearrange physical walls or run an Explore that's known to hit multiple 180° turns. Watch for one `[DEADEND] cal OK ...` line per 180° event.

Expected: each event fires cleanly; bias values may differ slightly between events (this is the whole point — bias drift over the run).

- [ ] **Step 2: If any event hangs or skips, file a bug and stop here.**

- [ ] **Step 3: No commit required.**

---

## Self-Review

**Spec coverage:**
- New `RunPhase` variant → Task 2 ✓
- `Tuning` struct additions → Task 1 ✓
- `scriptPushDeadendRecal()` helper → Task 3 ✓
- `buildMoveScript()` insertion → Task 4 ✓
- `case RUN` dispatch branch → Task 5 ✓
- Stillness detection + 100 ms hold → Task 5 ✓
- `calibrateGyroBias(300, 2)` invocation → Task 5 ✓
- `yawDeg=0` AND `yawTargetDeg=0` reset on both success and timeout paths → Task 5 ✓
- 500 ms timeout fallback → Task 5 ✓
- Serial telemetry lines → Task 5 ✓
- Single dead-end hardware test → Task 6 ✓
- Timeout fallback hardware test → Task 7 ✓
- Multi-dead-end hardware test → Task 8 ✓

No spec requirement is left unimplemented.

**Placeholder scan:** No TBDs, no "implement later", no "similar to Task N", no missing code blocks. Optional Step 5 in Task 6 is gated on observation and explicitly says "skip otherwise" — not a placeholder.

**Type consistency:**
- `PH_DEADEND_RECAL` spelled identically across Tasks 2/3/4/5 ✓
- `scriptPushDeadendRecal` spelled identically in Task 3 (definition) and Task 4 (call) ✓
- `Tuning` field names (`recalStillHoldMs`, `recalStillTicks`, `recalStillGz`, `recalSamples`, `recalSampleDelayMs`, `recalTimeoutMs`) match across Tasks 1 and 5 ✓
- `calibrateGyroBias(T.recalSamples, T.recalSampleDelayMs)` signature matches existing `static void calibrateGyroBias(int N = 300, int sampleDelayMs = 2)` at line 96 ✓
- `gzFilt`, `gyroBiasZ`, `yawDeg`, `yawTargetDeg`, `imuReady`, `leftEnc`, `rTicks()`, `stopMotors()`, `pid.reset()`, `phaseEnter()` all exist in `main.cpp` and are used with their existing signatures ✓
