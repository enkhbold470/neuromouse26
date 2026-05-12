#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <Arduino.h>

// ── Pins ──────────────────────────────────────────────────────────────────────
constexpr uint8_t MOTOR_L_IN1      = 15;
constexpr uint8_t MOTOR_L_IN2      = 16;
constexpr uint8_t MOTOR_R_IN3      = 18;
constexpr uint8_t MOTOR_R_IN4      = 17;
constexpr uint8_t DRV_SLEEP_PIN    = 41;

constexpr uint8_t ENC_L_A          = 38;
constexpr uint8_t ENC_L_B          = 39;
constexpr uint8_t ENC_R_A          = 21;
constexpr uint8_t ENC_R_B          = 14;

constexpr uint8_t RX_L            = 10;
constexpr uint8_t RX_LF            = 4;
constexpr uint8_t RX_RF            = 1;
constexpr uint8_t RX_R            = 7;

constexpr uint8_t EMIT_L          = 47;
constexpr uint8_t EMIT_LF          = 13;
constexpr uint8_t EMIT_RF          = 46;
constexpr uint8_t EMIT_R          = 11;

constexpr uint8_t BUTTON_1         = 42;
constexpr uint8_t BUZZER_PIN       = 40;
constexpr uint8_t WS2812_DATA      = 3;
constexpr uint8_t BAT_V_SENSE      = 5;

constexpr int     BUZZER_FREQ      = 4000;

// ── Motor PWM (LEDC) ─────────────────────────────────────────────────────────
// 500 Hz: audible whine but maximum torque — DRV8833 slower switching = more avg current.
// 4 kHz: silent, smoother but weaker at same duty cycle.
// 20 kHz: silent but lowest torque at low duty. Recalibrate DRIVE_PWM/TURN_PWM after changing.
constexpr int     MOTOR_PWM_FREQ_HZ = 500;
constexpr int     MOTOR_PWM_BITS    = 10;

// ── Motor polarity ────────────────────────────────────────────────────────────
constexpr bool    MOTOR_L_INV      = false;
constexpr bool    MOTOR_R_INV      = true;   // back-to-back mount requires inversion

// ── Wheel / encoder physics ───────────────────────────────────────────────────
// N20 1:30 500RPM @ 6V, running on 2S LiPo (7.4V).
// PCNT quadrature: 14 PPR motor shaft × 4 edges × 30 gear = 1680 ticks/output-rev.
// Verify: spin output shaft exactly one full revolution → should read ~1680.
constexpr float   WHEEL_DIAMETER   = 33.4f;   // mm, measured
constexpr float   TICKS_PER_REV    = 1680.0f; // PCNT 4× quadrature: 14 PPR × 4 × 30
constexpr float   WHEEL_TRACK_MM   = 74.0f;   // mm center-to-center, measured
constexpr float   BAT_VDIV_MULT    = 3.1197f;

// Recalibrate after PCNT switch: spin each wheel one full rev, record counts L and R.
constexpr float   RIGHT_ENC_SCALE  = 3439.0f / 3478.0f;  // TODO: recalibrate

// ── Cell / turn geometry (derived) ───────────────────────────────────────────
constexpr float   CELL_MM          = 180.0f;  // standard half-size micromouse cell
constexpr float   MM_PER_TICK      = 3.14159265f * WHEEL_DIAMETER / TICKS_PER_REV; // ~0.0624 mm
constexpr long    TICKS_PER_CELL   = (long)(CELL_MM / MM_PER_TICK);  // ≈ 2886
constexpr long    TICKS_PER_90     = (long)(WHEEL_TRACK_MM * TICKS_PER_REV / (4.0f * WHEEL_DIAMETER)); // ≈ 931
// PCNT 4× resolution: starting point ≈ 8× old ISR values (old 100 → ~800).
// Tune: each ~40 ticks ≈ 10°. Left/right calibrate independently.
constexpr long    TURN_TICKS_90_L  = 420;   // measured: 800 = 180°, halved for 90°
constexpr long    TURN_TICKS_90_R  = 410;   // measured: 800 = 180°, halved for 90°

// ── Drive tuning ──────────────────────────────────────────────────────────────
// Uses BRAKE stop (both INs HIGH). Recalibrate TURN_TICKS_90 if TURN_PWM changes.
constexpr int     MOTOR_PWM_MAX    = 1023;
constexpr int     DRIVE_PWM        = 250;                        // ← master cruise speed (0–1023)
constexpr int     TURN_PWM         = (int)(DRIVE_PWM * 0.80f);  // 70% of cruise for pivots
constexpr int     DRIVE_PWM_MIN    = 100;                        // dynamic stall floor; 150 is static-only, motor won't stall mid-ramp
// Coast comp: MIN=100 (dynamic) → shorter brake distance than static-stall MIN=150.
// Estimate 115mm; COAST_COMP(115) + DECEL(60) = 175mm < 180mm cell.
// Re-run moveCells(1): stops short → decrease COAST_COMP_MM; overshoots → increase.
constexpr float   COAST_COMP_MM    = 100.0f;                     // measured: 3.5cm overshoot at 80mm → +35mm; tune empirically
constexpr int     COAST_COMP_TICKS = (int)(COAST_COMP_MM / MM_PER_TICK); // ~1842 ticks
constexpr float   DECEL_MM         = 60.0f;                      // trimmed: COAST(115)+DECEL(60)=175mm < 180mm cell
constexpr int     DECEL_TICKS      = (int)(DECEL_MM / MM_PER_TICK); // ~961 ticks
constexpr float   BALANCE_KP       = 0.4f;                      // PWM per tick L-R error (scaled for PCNT 8× resolution)
constexpr int     TIMEOUT_MS       = 5000;                       // per-cell abort timeout ms
constexpr int     CELL_PAUSE_MS    = 40;
constexpr int     BASE_PWM         = DRIVE_PWM;

// ── IR thresholds (calibrated 2026-05-07, dead-end centered, all 4 walls) ─────
// irRead() is ambient-subtracted: no-wall ~0, wall ~400–550. Threshold=50 is safe.
constexpr int     L_CENTER         = 421;
constexpr int     R_CENTER         = 504;
constexpr int     L_THRESH         = 450;
constexpr int     R_THRESH         = 450;
constexpr int     LF_THRESH        = 450;
constexpr int     RF_THRESH        = 450;

// ── Wall-centering PID ────────────────────────────────────────────────────────
constexpr float   WALL_KP          = 0.25f;
constexpr float   WALL_KI          = 0.00f;
constexpr float   WALL_KD          = 0.02f;
constexpr int     WALL_MAX_CORR    = 200;

// ── Encoder-balance PID ───────────────────────────────────────────────────────
constexpr float   ENC_KP           = 6.0f;
constexpr float   ENC_KI           = 0.00f;
constexpr float   ENC_KD           = 0.2f;
constexpr int     ENC_MAX_CORR     = 120;

// ── Maze constants ────────────────────────────────────────────────────────────
constexpr uint8_t MAZE_SIZE        = 16;
constexpr uint16_t MAZE_CELLS      = MAZE_SIZE * MAZE_SIZE;
constexpr uint8_t WALL_NORTH       = 0x01;
constexpr uint8_t WALL_EAST        = 0x02;
constexpr uint8_t WALL_SOUTH       = 0x04;
constexpr uint8_t WALL_WEST        = 0x08;
constexpr uint8_t FLOOD_INFINITY   = 255;

#endif
