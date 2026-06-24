// include/Tuning.h — every tunable knob for Micromouse26 lives in this file.
//
// If a value affects motion, speed, power, alignment, or geometry, it should
// be here. Sections [A]–[F], [H] grouped by purpose. (Section [G] FWD_TO_WALL
// was removed when the legacy primitive was retired.)

#ifndef MM26_TUNING_H
#define MM26_TUNING_H

#include <Arduino.h>
#include "PinConfig.h"

// ── [A] ROBOT + MAZE GEOMETRY (mm) ──────────────────────────────────────────
// Body 100×88 mm, wheel track 80 mm, axle→front 67 mm, axle→rear 33 mm.
// Cell pitch 180 mm centre-to-centre, wall opening 170 mm.
constexpr float ROBOT_LEN_MM       =  100.0f;
constexpr float ROBOT_WIDTH_MM     =  88.0f;
constexpr float WHEELBASE_MM       =  80.0f;
constexpr float WHEEL_FRONT_OFF_MM =  67.0f;
constexpr float WHEEL_REAR_OFF_MM  =  33.0f;
constexpr float CELL_PITCH_MM      = 180.0f;
constexpr float CELL_INNER_MM      = 170.0f;
constexpr float CELL_SIDE_GAP_MM   =  45.0f;

// Active physical maze region inside the 16×16 flood grid. setupMaze() seals
// the border around this rectangle so flood-fill cannot route through the
// unused tail of the array (cols 3–15, rows 6–15 would otherwise look open).
constexpr uint8_t MAZE_ROWS = 6;
constexpr uint8_t MAZE_COLS = 3;
constexpr uint8_t START_ROW = 0;
constexpr uint8_t START_COL = 0;
// Single goal cell (5,2) — top-right of the 6×3 test maze.
constexpr uint8_t GOAL_ROW  = 5;
constexpr uint8_t GOAL_COL  = 2;
// true = (5,2) → save NVS → sweep all unvisited → go home (0,0) → GOAL blink.
constexpr bool  EXPLORE_RETURN_HOME = true;
// Added to flood[visited neighbour] during explore so backtracking loses to
// any path toward unvisited cells (junction after a dead end).
constexpr uint8_t EXPLORE_VISITED_FLOOD_PENALTY = 64;

// ── [B0] ★★★ MAIN POWER KNOB ★★★ ────────────────────────────────────────────
// Single source-of-truth for motor breakaway PWM at the current
// MOTOR_PWM_FREQ_HZ. Every stiction floor and PWM cap below is derived from
// this as a fixed ratio. Bump this ONE number and the robot gets stronger
// everywhere proportionally.
//
// NOT derived from this (independent physics, leave alone):
//   POS_KP / POS_KD / YAW_KP / YAW_KD     — closed-loop PID gains
//   FWD_KV_SLOPE                          — velocity-FF slope (PWM per tps)
//   FWD_V_CRUISE_TPS / FWD_ACCEL_TPS2     — trapezoidal profile
//
// Tuning recipe:
//   1. Start with a low FAST_RUN_CRUISE_TPS (≈ 600).
//   2. Watch wheels cold-start. Buzz w/o rotation → BASE too low.
//      Wheels jerk hard / overshoot     → BASE too high.
//   3. Add ~20 % headroom over the threshold so dirty wheels still go.
//   4. Changing MOTOR_PWM_FREQ_HZ invalidates BASE — re-tune.
constexpr int BASE_BREAKAWAY_PWM = 110;

// Derived ratios. `(BASE × N) / 10` is an integer-constexpr way to write N/10×.
constexpr int FWD_STICTION_FF  = (BASE_BREAKAWAY_PWM * 10) / 10;   // 1.0×
constexpr int ALIGN_PWM        = (BASE_BREAKAWAY_PWM * 18) / 10;   // 1.8× slow-creep needs breakaway headroom
constexpr int POS_STICTION_PWM = (BASE_BREAKAWAY_PWM * 12) / 10;   // 1.2× non-FWD pos floors
constexpr int YAW_STICTION_PWM = (BASE_BREAKAWAY_PWM * 15) / 10;   // 1.5× turns drag more
constexpr int POS_MAX_PWM      = MOTOR_PWM_MAX;                    // hard cap; PH_FORWARD limits via dynMax
constexpr int YAW_MAX_PWM      = ((BASE_BREAKAWAY_PWM * 16) / 10 < MOTOR_PWM_MAX)
                                  ? ((BASE_BREAKAWAY_PWM * 16) / 10) : MOTOR_PWM_MAX;  // 1.6× cap

// ── [B] FORWARD MOTION ──────────────────────────────────────────────────────
// Trapezoidal velocity profile drives a smoothly-moving position setpoint.
// Position-PID tracks it; velocity FF supplies the steady-state PWM so the PID
// only handles tracking error, not driving force.
//   - Raise FWD_V_CRUISE_TPS for higher top speed.
//   - Raise FWD_ACCEL_TPS2 if accel feels sluggish (lower if it skids).
//   - FWD_KV_SLOPE is PWM-per-(ticks/s) FF slope.
//   - Stiction-related PWMs are DERIVED from BASE_BREAKAWAY_PWM in [B0].
constexpr float    FWD_V_CRUISE_TPS = 150.0f;  // classic explore (stop-pivot)

// Explore always uses FWD_V_CRUISE_TPS — one cell, full stop, SPOT turns only.

// FAST RUN cruise speed is RUNTIME-ADJUSTABLE via the "Fast Speed" menu and
// persisted to NVS (key "fspeed"). Default == FWD_V_CRUISE_TPS. EXPLORE
// mode ignores this knob.
constexpr float    FAST_RUN_CRUISE_TPS_DEFAULT = FWD_V_CRUISE_TPS;
constexpr float    FAST_RUN_CRUISE_TPS_MIN     =  400.0f;
constexpr float    FAST_RUN_CRUISE_TPS_MAX     = 3000.0f;
constexpr float    FAST_RUN_CRUISE_TPS_STEP    =  100.0f;

constexpr float    FWD_ACCEL_TPS2    = 500.0f;
constexpr float    FWD_DECEL_TPS2    = 500.0f;
constexpr float    FWD_KV_SLOPE      =   0.12f;
constexpr float    POS_KP            =   0.80f;
constexpr float    POS_KD            =   0.10f;
constexpr int      POS_FRICTION_ZONE =  10;
constexpr int      POS_STK_SOFT_BAND =  30;
constexpr int      POS_HOLD_BAND     =  20;   // |posErr| < this → settle candidate
constexpr uint32_t POS_SETTLE_MS     =  80;
constexpr int      POS_STOP_BIAS     =   0;
constexpr float    POS_STALL_VEL     =  30.0f;
constexpr uint32_t POS_STALL_MS      = 200;
constexpr int      POS_STALL_ERR_MAX =  40;
constexpr float    BALANCE_KP        =   0.03f;  // (tL−tR) × this added as bias

// ── [C] PIVOT / SPOT TURNS (yaw-IMU controller) ─────────────────────────────
// YAW_STICTION_PWM / YAW_MAX_PWM are DERIVED from BASE_BREAKAWAY_PWM in [B0].
// INVARIANT: YAW_FRICTION_ZONE ≤ YAW_HOLD_BAND (else dead-zone).
constexpr float    YAW_KP            =   6.0f;
constexpr float    YAW_KD            =   0.3f;
constexpr float    YAW_FRICTION_ZONE =   3.0f;
constexpr float    YAW_STK_SOFT_BAND =   2.0f;
constexpr float    YAW_HOLD_BAND     =   1.5f;
constexpr uint32_t YAW_SETTLE_MS     =  80;
constexpr float    YAW_STALL_VEL     =   5.0f;
constexpr uint32_t YAW_STALL_MS      = 250;
constexpr float    YAW_STALL_ERR_MAX =   4.0f;
constexpr float    YAW_HOLD_KP       =   5.0f;   // forward-leg heading hold
constexpr float    YAW_HOLD_KD       =   0.8f;   // damps yaw overshoot during forward (PWM per °/s)
constexpr float    PIVOT_90_DEG      =  90.0f;
constexpr float    SPOT_180_DEG      = 180.0f;

// ── [D] IR CENTERING (forward phase only) ───────────────────────────────────
// Trims L/R PWM bias to keep robot mid-corridor. wallConf in main.cpp scales
// to per-run sideRef; IR_CENTER_MAX scales with vAbsCmd at higher cruise.
constexpr float    IR_CENTER_KP  =   2.0f;
constexpr float    IR_CENTER_KI  =   0.0f;
constexpr float    IR_CENTER_KD  =   1.0f;
constexpr int      IR_CENTER_MAX =  35;

// Front IR proactive stop during explore (ALIGN target 570 @ cell center).
constexpr int      WALL_FRONT_BRAKE_THRESH = 480;  // begin slowing (~84% of align)
constexpr int      WALL_FRONT_STOP_THRESH  = 530;  // ALIGN_LF − ALIGN_TOL; hard stop
constexpr uint32_t POS_BUMP_MS              =   80;
constexpr int      POS_BUMP_BACKUP_PWM      =  130;
constexpr uint32_t POS_BUMP_BACKUP_MS       =  250;
constexpr uint32_t POS_BUMP_BACKUP_TIMEOUT_MS = 3000;
constexpr uint32_t POS_HARD_STALL_MS        = 4000;
// Front-IR distance (mm) when centered in cell; ~45 mm matches LF/RF≈570 via IRCal.
constexpr float    CELL_CENTER_FRONT_MM     =  45.0f;

// ── [E] DEAD-END + PH_ALIGN_FRONT ───────────────────────────────────────────
// Dead-end exit: ALIGN → SPOT 180° → reverse DEADEND_REVERSE_MM → forward
// DEADEND_FWD_MM. Pre-turn 90°: optional ALIGN when front wall present.
// ALIGN_* captured 2026-06-24: robot centered in cell, correct front gap → LF=570 RF=570.
constexpr int      ALIGN_LF_TARGET   =  570;
constexpr int      ALIGN_RF_TARGET   =  570;
constexpr int      ALIGN_TOL         =   40;
constexpr int      ALIGN_MOVE_DEADBAND = 25;   // |errMean| below → hold still (anti-hunt)
constexpr float    ALIGN_ERR_KP      = 0.012f; // proportional creep PWM per count
constexpr long     ALIGN_MAX_TICKS   =  500;   // cap align travel (was 800 → fewer oscillations)
constexpr uint32_t ALIGN_SETTLE_MS   =  120;
constexpr float    DEADEND_REVERSE_MM =  20.0f;
constexpr float    DEADEND_FWD_MM     = 180.0f;

// ── [F] SCRIPT GEOMETRY ─────────────────────────────────────────────────────
// CELL_TICKS = encoder ticks to traverse one 180 mm cell pitch.
// Hand-measured 2026-05-19, both wheels. Do NOT derive from `TICKS_PER_REV`.
// START_OFFSET_TICKS = extra ticks on the first forward leg because the robot
// starts pressed against the back wall of cell (0,0) — its center sits
// ~41 mm behind the cell-(0,0) centre.
constexpr long  CELL_TICKS             = 1400;
constexpr long  START_OFFSET_TICKS     =  322;
constexpr long  PIVOT_TICKS_FALLBACK   =  900;   // used only if USE_IMU=false
constexpr long  SPOT180_TICKS_FALLBACK =  906;   // used only if USE_IMU=false
constexpr float BACKUP_OFFSET_MM       =   0.0f; // PH_REVERSE_TO_BACK

// ── [I] SMOOTH-CURVE ARC (continuous motion / fast run) ──────────────────────
// A PH_CURVE is a constant-curvature, FORWARD-ONLY arc that replaces a
// stop-and-pivot SPOT 90° turn. Both wheels keep rolling forward (the inner
// wheel only *slows*, never reverses → the old "fastFwdRoll breaks R-turns"
// failure is impossible by construction). The arc is closed on the gyro
// (heading-vs-distance-progress PD) and feed-forwarded in TICK-SPACE so it
// reuses the proven FWD feed-forward (FWD_STICTION_FF + FWD_KV_SLOPE), NOT the
// mm/s KV_L/KV_R domain (which is a different, separately-calibrated unit).
//
// TICK↔mm: use TICKS_PER_MM below (derived from hand-measured CELL_TICKS).
// Do NOT use PinConfig::MM_PER_TICK for continuous-motion math — it is the
// legacy single-channel value and is ~4× off against the 4× PCNT CELL_TICKS.
//
// CURVE_RADIUS_MM = CELL_PITCH/2 (90 mm centerline), NOT from PCB netlist.
// Body 100×88 mm, track 80 mm, axle→front 67 mm — fast-run arcs only (explore
// uses SPOT pivots). WALL-CLIP MITIGATION: lower CURVE_V_ARC first → nudge
// CURVE_PRE/POST → NEVER raise R to fix an inner clip.
//
// ALL of this is gated at runtime by g_smoothMode (OLED Smooth/Classic toggle);
// with the toggle OFF the robot is byte-for-byte the legacy stop-pivot firmware.
constexpr bool  CURVE_ENABLE      = true;     // master compile gate; false → no arc code path ever
constexpr bool  EXPLORE_CONTINUOUS = false;   // false = simple safe explore (1 cell, stop, sense, turn)

constexpr float TICKS_PER_MM      = (float)CELL_TICKS / CELL_PITCH_MM;   // 7.778 (authoritative)
constexpr float CURVE_RADIUS_MM   = 90.0f;    // = half cell pitch, centerline-to-centerline
constexpr long  CURVE_ARC_TICKS   = (long)(CURVE_RADIUS_MM * 1.5707963f * TICKS_PER_MM + 0.5f); // R·π/2 ≈ 1100
constexpr long  CURVE_PRE_TICKS   = (long)(CURVE_RADIUS_MM * TICKS_PER_MM + 0.5f);  // shorten approach straight ≈ 700
constexpr long  CURVE_POST_TICKS  = (long)(CURVE_RADIUS_MM * TICKS_PER_MM + 0.5f);  // shorten exit straight ≈ 700

// Body speeds in TPS (match the FWD trapezoid units). Conversion in THIS
// codebase: mm/s = tps / TICKS_PER_MM = tps × 180 / 1400 (so 150 tps ≈ 19 mm/s,
// matching the shipping explore cruise). START LOW — bench-verify wall
// clearance before raising (stage F4). 1000 tps ≈ 129 mm/s; a_lat = v²/R ≈
// 0.18 m/s² at R=90 mm (very gentle, safe first arc).
constexpr float CURVE_V_ARC_TPS   = 1000.0f;  // arc body cruise (≈129 mm/s)
constexpr float CURVE_V_ENTRY_TPS = 1000.0f;  // FWD decel terminal speed into the arc (= vArc)
constexpr float CURVE_V_EXIT_TPS  =  700.0f;  // body speed at the end of the taper (≈90 mm/s)
constexpr float CURVE_TAPER_DEG   = 20.0f;    // taper body speed over the last N° of the sweep

// Heading closed loop on the arc (PWM domain, like YAW_HOLD_*).
constexpr float CURVE_HEAD_KP       = 8.0f;   // PWM per ° of heading-vs-progress error
constexpr float CURVE_HEAD_KD       = 0.8f;   // PWM per (°/s) of turn-rate deviation
constexpr float CURVE_HEAD_DEADBAND = 2.0f;   // ° remaining → sweep complete (settle)
constexpr long  CURVE_TICK_MARGIN   = 250;    // distance backstop: force-end if avg ticks exceed arc + this
// Min approach-straight ticks to reach arc-entry speed from rest (d = v²/2a).
// A first turn from a standstill with less room than this stays a SPOT.
constexpr long  CURVE_MIN_ENTRY_TICKS =
    (long)((CURVE_V_ENTRY_TPS * CURVE_V_ENTRY_TPS) / (2.0f * FWD_ACCEL_TPS2));


// ── [J] ADAPTIVE SIDE-WALL SENSING ──────────────────────────────────────────
// Side sensors (L45/R45) are at 30° from the robot's forward axis. This causes
// two problems:
//   1. WEAK SIGNAL: side wall at 42.5 mm lateral sits at 85 mm from sensor
//      (vs 42.5 mm for a 90° perpendicular sensor) → only ~330-400 raw cts.
//   2. FRONT CONTAMINATION: a front wall at 90 mm ahead appears at
//      90/cos(30°) ≈ 104 mm diagonal to the 30° sensor → ~220 cts, above
//      the threshold → phantom side wall. Fix: Planner.h senseAndStoreWalls()
//      skips side sensing when front wall is confirmed (mid-cell sensing at
//      ~50% already ran with front wall 208 mm away = no contamination).
// Additional issues: fixed absolute threshold (WALL_SIDE_THRESH) gives phantom
// walls when room light shifts. Pros: per-RUN calibration (room light divides
// out) + fractional threshold + saturation skip + median sampling.
//   Per-run reference: at start (0,0) facing North WEST border wall guaranteed
//   on LEFT → captured live in calibrateSideRefs(). SIDE_ADAPTIVE=false →
//   exact legacy fixed-threshold behavior.
constexpr bool  SIDE_ADAPTIVE  = true;
constexpr float SIDE_WALL_FRAC = 0.45f;  // wall present if median read > frac × per-run reference
constexpr int   SIDE_OPEN_CEIL = 350;    // below = open side (user cal: L=180 R=270 opposite touch)
constexpr int   SIDE_SAT_AMB   = 3500;   // emitter-off ambient (of 4095) above this → read UNKNOWN (skip)
constexpr int   SIDE_SAMPLES   = 5;      // samples per side decision; median used (odd ≥3)
constexpr int   SIDE_REF_MIN   = 300;    // captured start ref below this = implausible → keep factory IR_CAL

// ── [H] DEBUG FLAGS ─────────────────────────────────────────────────────────
constexpr bool USE_IMU   = true;   // false = encoder-tick turns
constexpr bool USE_IR    = true;   // false = no IR centering
constexpr bool TELEMETRY = true;   // Serial.printf control prints

#endif
