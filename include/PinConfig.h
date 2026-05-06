#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <Arduino.h>

// ── Pin Config ────────────────────────────────────────────────────────────────
#define MOTOR_L_IN1     15
#define MOTOR_L_IN2     16
#define MOTOR_R_IN3     18
#define MOTOR_R_IN4     17
#define DRV_SLEEP_PIN   41

#define ENC_L_A         21
#define ENC_L_B         14
#define ENC_R_A         38
#define ENC_R_B         39

#define RX_LF           4
#define RX_L45          6
#define RX_R45          2
#define RX_RF           1

#define EMIT_LF         13
#define EMIT_L45        45
#define EMIT_R45        12
#define EMIT_RF         11

#define BUTTON_1        42
#define BUZZER_PIN      40
#define BUZZER_FREQ     4000
#define WS2812_DATA     3

#define BAT_V_SENSE     5      // Battery voltage divider pin
#define BAT_VDIV_MULT   3.1197f // Calibration multiplier

// ── Physics ───────────────────────────────────────────────────────────────────
#define WHEEL_DIAMETER  33.4f
#define TICKS_PER_REV   210.0f

// ── Motor Polarity (Physics: Back-to-back motors require inversion) ───────────
#define MOTOR_L_INV     false
#define MOTOR_R_INV     true   // Usually right is inverted

// ── IR thresholds (Calibrated 2026-05-06) ──────────────────────────────────
#define L45_CENTER      1010
#define R45_CENTER      785
#define L45_THRESH      500   // Wall detected if > 50% of center
#define R45_THRESH      400   
#define LF_THRESH       600   // Front wall detection
#define RF_THRESH       400   

// ── Drive tuning (Scaled for Universal 8-bit PWM: 0-255) ──────────────────────
#define BASE_PWM        60   // Safe starting speed

#define WALL_KP         0.5f  // Much lower gain for smooth proportional steering
#define WALL_KI         0.01f 
#define WALL_KD         0.1f
#define WALL_MAX_CORR   50    // Max steering deviation

#define ENC_KP          2.0f
#define ENC_KI          0.0f
#define ENC_KD          0.0f
#define ENC_MAX_CORR    30

// ── Cell math ─────────────────────────────────────────────────────────────────
#define TICKS_PER_CELL  360L
#define CELL_PAUSE_MS   40

// ── 90° pivot turn ────────────────────────────────────────────────────────────
#define WHEEL_TRACK_MM  74.0f
#define TICKS_PER_90    (long)(WHEEL_TRACK_MM * TICKS_PER_REV / (4.0f * WHEEL_DIAMETER))
#define TURN_PWM        50

// ── Maze constants ────────────────────────────────────────────────────────────
#define MAZE_SIZE       16
#define MAZE_CELLS      (MAZE_SIZE * MAZE_SIZE)

#define WALL_NORTH      0x01
#define WALL_EAST       0x02
#define WALL_SOUTH      0x04
#define WALL_WEST       0x08

#define FLOOD_INFINITY  255

#endif
