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
// Chassis 95×85, wheel center 55 mm from front edge. Cell pitch 180 mm
// centre-to-centre, wall-to-wall opening 170 mm. Robot 85 mm wide in 170 mm
// cell → 42.5 mm side clearance each side.
constexpr float ROBOT_LEN_MM       =  100.0f;
constexpr float ROBOT_WIDTH_MM     =  80.0f;
constexpr float WHEEL_FRONT_OFF_MM =  65.0f;
constexpr float CELL_PITCH_MM      = 180.0f;
constexpr float CELL_INNER_MM      = 170.0f;
constexpr float CELL_SIDE_GAP_MM   =  45.0f;

// Full IEEE 16×16 maze (MAZE_SIZE in PinConfig.h). Start SW corner (0,0) facing
// North; goal is centre 2×2 cells (7,7)–(8,8) via setGoalCentre4() in reset().
constexpr uint8_t MAZE_ROWS = MAZE_SIZE;
constexpr uint8_t MAZE_COLS = MAZE_SIZE;
constexpr uint8_t START_ROW = 0;
constexpr uint8_t START_COL = 0;
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
constexpr float    FWD_V_CRUISE_TPS = 150.0f;  // EXPLORE cruise (slow for sense reliability)

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
// Trims L/R PWM bias to keep robot mid-corridor matching L/R raw counts to
// IR_CAL_L / IR_CAL_R from PinConfig.h.
constexpr float    IR_CENTER_KP  =   1.0f;
constexpr float    IR_CENTER_KI  =   0.0f;
constexpr float    IR_CENTER_KD  =   2.0f;
constexpr int      IR_CENTER_MAX =  15;

// ── [E] PH_ALIGN_FRONT (optional creep primitive; not used in explore script) ─
// Creep until LF≈ALIGN_LF_TARGET / RF≈ALIGN_RF_TARGET. Dead ends use SPOT 180 only.
constexpr int      ALIGN_LF_TARGET   = 3660;
constexpr int      ALIGN_RF_TARGET   = 2940;
constexpr int      ALIGN_TOL         =  150;
constexpr long     ALIGN_MAX_TICKS   =  800;
constexpr uint32_t ALIGN_SETTLE_MS   =   80;
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


// ── [H] DEBUG FLAGS ─────────────────────────────────────────────────────────
constexpr bool USE_IMU   = true;   // false = encoder-tick turns
constexpr bool USE_IR    = true;   // false = no IR centering
constexpr bool TELEMETRY = true;   // Serial.printf control prints

#endif
