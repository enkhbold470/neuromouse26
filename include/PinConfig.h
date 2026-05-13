#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <Arduino.h>

// ── Pins ──────────────────────────────────────────────────────────────────────
constexpr uint8_t MOTOR_L_IN1      = 15;
constexpr uint8_t MOTOR_L_IN2      = 16;
constexpr uint8_t MOTOR_R_IN3      = 18;
constexpr uint8_t MOTOR_R_IN4      = 17;
// DRV8833 nSLEEP tied to VCC permanently — no pin needed.

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

// ── I2C OLED (SSD1306 0.96" 128x64, addr 0x78 8-bit = 0x3C 7-bit) ────────────
constexpr uint8_t OLED_SDA         = 8;
constexpr uint8_t OLED_SCL         = 9;
constexpr uint8_t OLED_ADDR        = 0x3C;

constexpr int     BUZZER_FREQ      = 4000;

// Mechanical keyswitch debounce/hold time. Press must stay LOW this long
// to register one event. 50 ms = snappy but bounce-safe.
constexpr unsigned long BUTTON_HOLD_MS = 50;


// some importnat physical information,
// robot weight is 161gramms with batteries,
// we have 2s battery
// 500rpm 1:30 n20 motor
// we have side wall detecting sensors 50mm from axle wheel 
// we have 40mm from wheel axle to rear bumper,

// ── Motor PWM (LEDC) ─────────────────────────────────────────────────────────
// 500 Hz: audible whine but maximum torque — DRV8833 slower switching = more avg current.
// 4 kHz: silent, smoother but weaker at same duty cycle.
// 20 kHz: silent but lowest torque at low duty. Recalibrate DRIVE_PWM/TURN_PWM after changing.
constexpr int     MOTOR_PWM_FREQ_HZ = 1000;
constexpr int     MOTOR_PWM_BITS    = 10;

// ── Motor polarity ────────────────────────────────────────────────────────────
// Both motors mounted back-to-back / wired reversed → invert PWM polarity.
// Encoders must also be inverted to keep getTicks() positive for forward motion
// (see MicromouseEncoder constructor in main.cpp).
constexpr bool    MOTOR_L_INV      = true;
constexpr bool    MOTOR_R_INV      = true;

// ── Wheel / encoder physics ───────────────────────────────────────────────────
// N20 1:30 500RPM @ 6V, running on 2S LiPo (7.4V).
// ISR rising-edge on channel A: 14 PPR motor shaft × 30 gear = 420 ticks/output-rev.
// Verify on hardware: TEST_ENC menu, spin output shaft one full revolution → ~420.
constexpr float   WHEEL_DIAMETER   = 33.4f;   // mm, measured
constexpr float   TICKS_PER_REV    = 205.0f;  // empirical: motor-driven half-quad ~720/cell → 420/rev
constexpr float   WHEEL_TRACK_MM   = 74.0f;   // mm center-to-center, measured
constexpr float   BAT_VDIV_MULT    = 3.1197f;

// Recalibrate empirically: spin each wheel one full rev, record L and R counts.
constexpr float   RIGHT_ENC_SCALE  = 1.0f;    // empirical: L=R=360 per cell

// ── Cell / turn geometry ─────────────────────────────────────────────────────
constexpr float   CELL_MM          = 180.0f;
constexpr float   MM_PER_TICK      = 3.14159265f * WHEEL_DIAMETER / TICKS_PER_REV;
constexpr long    TICKS_PER_90     = (long)(WHEEL_TRACK_MM * TICKS_PER_REV / (4.0f * WHEEL_DIAMETER));
constexpr long    TURN_TICKS_90_L  = 420;
constexpr long    TURN_TICKS_90_R  = 410;

// ── Drive tuning ─────────────────────────────────────────────────────────────
// Change DRIVE_PWM only — ramp zones and TICKS_PER_CELL auto-scale.
constexpr int     MOTOR_PWM_MAX    = 1023;
constexpr int     DRIVE_PWM        = 250;   // ← only change this (0–1023)
constexpr int     TURN_PWM         = (int)(DRIVE_PWM * 0.7f);
constexpr int     DRIVE_PWM_MIN    = 100;
constexpr float   TURN_VERIFY_THRESH = 3.0f;  // degrees tolerance after pivot settle
constexpr int     TURN_CORRECT_PWM   = DRIVE_PWM_MIN;  // nudge PWM for post-turn correction
constexpr float   BALANCE_KP       = 0.4f;
constexpr int     TIMEOUT_MS       = 5000;
constexpr int     CELL_PAUSE_MS    = 40;
constexpr int     BASE_PWM         = DRIVE_PWM;
constexpr int     STALL_TIME_MS    = 150;   // ms without encoder advance → stall
constexpr int     STALL_BOOST_PWM  = (int)(DRIVE_PWM * 1.8f);  // capped to MOTOR_PWM_MAX at use

// Trapezoidal velocity profile (GreenYe pattern).
// ACC_RATE / DEC_RATE: PWM units per encoder tick.
// Larger value = shorter ramp zone. Recalibrate if robot jerks or overshoots.
constexpr int     ACC_RATE         = 2;    // PWM per tick, ramp up
constexpr int     DEC_RATE         = 2;    // PWM per tick, ramp down
constexpr int     ACCEL_TICKS      = (DRIVE_PWM - DRIVE_PWM_MIN) / ACC_RATE;
constexpr float   ACCEL_MM         = ACCEL_TICKS * MM_PER_TICK;
constexpr int     DECEL_TICKS      = (DRIVE_PWM - DRIVE_PWM_MIN) / DEC_RATE;
constexpr float   DECEL_MM         = DECEL_TICKS * MM_PER_TICK;

// Coast from DRIVE_PWM_MIN (end of decel ramp). v² model calibrated: 26mm at PWM=180.
// DRIVE_PWM_MIN is fixed → coast is constant regardless of cruise speed.
constexpr float   COAST_COMP_MM    = 26.0f * ((float)DRIVE_PWM_MIN / 180.0f) * ((float)DRIVE_PWM_MIN / 180.0f);
constexpr int     COAST_COMP_TICKS = (int)(COAST_COMP_MM / MM_PER_TICK);

// Brake this many ticks early; coast at DRIVE_PWM_MIN lands on cell boundary.
// At DRIVE_PWM_MIN=100: coast≈8mm (≈15 ticks). TICKS_PER_CELL ≈ 352−15 = 337.
// NOTE: if stopping short/long after adding decel ramp, adjust COAST_COMP_MM.
constexpr long    TICKS_PER_CELL   = (long)(CELL_MM / MM_PER_TICK) - COAST_COMP_TICKS;

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
