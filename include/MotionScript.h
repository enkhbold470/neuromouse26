// include/MotionScript.h — phase-script data types + simple pushers.
//
// A "move" is encoded as a short list of `PhaseStep`s (max 8). The RUN
// executor in main.cpp walks the list step-by-step. Stateful hooks
// (`phaseEnter`, `onPhaseActivate`, `scriptKick`) live in main.cpp because
// they reach into hardware (encoders, IR, pid) — this header is data-only.

#ifndef MM26_MOTION_SCRIPT_H
#define MM26_MOTION_SCRIPT_H

#include <Arduino.h>
#include "Tuning.h"

enum TurnDir { TURN_NONE, TURN_RIGHT, TURN_LEFT };

// PH_FORWARD          — tick-target forward (signed; negative = reverse).
//                       IR centering + IMU yaw hold + encoder balance bias.
// PH_CURVE            — constant-curvature FORWARD-ONLY arc (smooth turn).
//                       Target = degrees (90). Both wheels roll forward; the
//                       inner wheel only slows, never reverses. Gyro-closed.
//                       Only emitted when g_smoothMode && CURVE_ENABLE.
// PH_PIVOT            — single-wheel pivot. Inner wheel braked, outer drives.
//                       Target in degrees (IMU mode) or ticks (fallback).
// PH_SPOT             — both wheels opposite, rotates about chassis centre.
//                       Target = degrees (45 / 90 / 180 all supported).
// PH_REVERSE_TO_BACK  — at activation, samples front IR and computes a reverse
//                       tick target then reclassifies itself to PH_FORWARD so
//                       the standard PID drives it.
// PH_ALIGN_FRONT      — creeps fwd/rev until front IR LF/RF read the target
//                       row (ALIGN_LF_TARGET / ALIGN_RF_TARGET).
enum RunPhase { PH_FORWARD, PH_CURVE, PH_PIVOT, PH_SPOT,
                PH_REVERSE_TO_BACK, PH_ALIGN_FRONT };

struct PhaseStep {
    RunPhase phase;
    long     target;
    TurnDir  dir;
};

// 64 lets a full continuous fast-run route (FWD CURVE FWD CURVE …) fit in one
// script so the mouse flows start→goal without an inter-move brake. A route
// that exceeds this just brakes once at the seam and re-plans. (≥4 preserved.)
constexpr int MAX_SCRIPT = 64;

static PhaseStep script[MAX_SCRIPT];
static int       scriptLen = 0;
static int       scriptIdx = 0;

static TurnDir  runTurnDir = TURN_NONE;
static RunPhase runPhase   = PH_FORWARD;
static long     runTarget  = 0;

// Per-phase encoder + time snapshots. Captured by `phaseEnter()` in main.cpp.
static long     phaseStartTL = 0;
static long     phaseStartTR = 0;
static uint32_t phaseStartUs = 0;

static void scriptReset() { scriptLen = 0; scriptIdx = 0; }

static void scriptPush(RunPhase ph, long target, TurnDir d = TURN_NONE) {
    if (scriptLen < MAX_SCRIPT) script[scriptLen++] = { ph, target, d };
}

static void scriptPushFwd(long ticks) {
    scriptPush(PH_FORWARD, ticks + (long)POS_STOP_BIAS, TURN_NONE);
}

static void scriptPushSpot(TurnDir d, float deg) {
    long target = USE_IMU ? (long)(deg + 0.5f) : SPOT180_TICKS_FALLBACK;
    scriptPush(PH_SPOT, target, d);
}

// Smooth 90° arc. Target carried in degrees (like SPOT). Executed by the
// PH_CURVE branch in main.cpp; only pushed when g_smoothMode && CURVE_ENABLE.
static void scriptPushCurve(TurnDir d) {
    scriptPush(PH_CURVE, (long)(PIVOT_90_DEG + 0.5f), d);
}

static void scriptPushPivot(TurnDir d, float deg) {
    long target = USE_IMU ? (long)(deg + 0.5f) : PIVOT_TICKS_FALLBACK;
    scriptPush(PH_PIVOT, target, d);
}

static void scriptPushReverseToBack() { scriptPush(PH_REVERSE_TO_BACK, 0, TURN_NONE); }
static void scriptPushAlignFront()    { scriptPush(PH_ALIGN_FRONT,    0, TURN_NONE); }

#endif
