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

constexpr uint8_t RX_LF            = 4;
constexpr uint8_t RX_L45           = 6;
constexpr uint8_t RX_R45           = 2;
constexpr uint8_t RX_RF            = 1;

constexpr uint8_t EMIT_LF          = 13;
constexpr uint8_t EMIT_L45         = 45;
constexpr uint8_t EMIT_R45         = 12;
constexpr uint8_t EMIT_RF          = 11;

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
// Encoder: ~14 PPR motor shaft × 30 = ~420 raw ticks/rev.
// ISR rising-edge only + 200µs noise filter → effective 210 ticks/rev at speed.
// Do NOT change to 420 — empirically validated at running speed.
constexpr float   WHEEL_DIAMETER   = 33.4f;   // mm, measured
constexpr float   TICKS_PER_REV    = 210.0f;  // empirical at running speed
constexpr float   WHEEL_TRACK_MM   = 74.0f;   // mm center-to-center, measured
constexpr float   BAT_VDIV_MULT    = 3.1197f;

// Right encoder reads more ticks than left over same distance.
// Calibrated 5-cell run: L=3439, R=3478. Scale brings R down to match L.
constexpr float   RIGHT_ENC_SCALE  = 3439.0f / 3478.0f;

// ── Cell / turn geometry (derived) ───────────────────────────────────────────
constexpr float   CELL_MM          = 180.0f;  // standard half-size micromouse cell
constexpr float   MM_PER_TICK      = 3.14159265f * WHEEL_DIAMETER / TICKS_PER_REV; // ~0.4997 mm
constexpr long    TICKS_PER_CELL   = (long)(CELL_MM / MM_PER_TICK);  // ≈ 360
constexpr long    TICKS_PER_90     = (long)(WHEEL_TRACK_MM * TICKS_PER_REV / (4.0f * WHEEL_DIAMETER)); // ≈ 116 (formula, based on TICKS_PER_REV at drive speed)
// TICKS_PER_REV=210 is calibrated at DRIVE_PWM speed. At TURN_PWM the encoder
// counts differently → robot turned 135° with formula value. Calibrated to 77 (=116×90/135).
// Tune: each ~8 ticks ≈ 10°. Too far → lower; too short → raise.
// Left/right differ due to mechanical asymmetry — calibrate independently.
// With brake stop, no coast compensation — these are the actual target tick counts for 90°.
// Tune: each ~5 ticks ≈ 10°. Too far → lower; too short → raise.
constexpr long    TURN_TICKS_90_L  = 100;   // starting point for brake stop at TURN_PWM=150/500Hz
constexpr long    TURN_TICKS_90_R  = 100;   // starting point for brake stop at TURN_PWM=150/500Hz

// ── Drive tuning — ONE knob: change DRIVE_PWM, rest scale automatically ───────
// Uses BRAKE stop (both INs HIGH). Coast comp near-zero — brake stops in <5mm.
// Recalibrate TURN_TICKS_90 if TURN_PWM changes (empirical, not derived).
constexpr int     MOTOR_PWM_MAX    = 1023;
constexpr int     DRIVE_PWM        = 250;                        // ← master cruise speed (0–1023)
constexpr int     TURN_PWM         = (int)(DRIVE_PWM * 0.70f);  // 60% of cruise for pivots
constexpr int     DRIVE_PWM_MIN    = (int)(DRIVE_PWM * 0.32f);  // decel floor (~32%); below 70 risks stall
constexpr int     COAST_COMP_TICKS = (int)(DRIVE_PWM * 0.25f);  // brake carries ~62 ticks at 250; 2.5cm overshoot calibrated
constexpr int     DECEL_TICKS      = 200;                        // fixed distance ramp (~100mm)
constexpr int     BALANCE_KP       = 3;                          // PWM per tick of L-R encoder error
constexpr int     TIMEOUT_MS       = 5000;                       // per-cell abort timeout ms
constexpr int     CELL_PAUSE_MS    = 40;
constexpr int     BASE_PWM         = DRIVE_PWM;

// ── IR thresholds (calibrated 2026-05-07, dead-end centered, all 4 walls) ─────
// irRead() is ambient-subtracted: no-wall ~0, wall ~400–550. Threshold=50 is safe.
constexpr int     L45_CENTER       = 421;
constexpr int     R45_CENTER       = 504;
constexpr int     L45_THRESH       = 450;
constexpr int     R45_THRESH       = 450;
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
