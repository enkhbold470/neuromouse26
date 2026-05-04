// --- PinConfig.h ---
#pragma once

// 1. Motor Control (DRV8833 PWM)
#define MOTOR_L_IN1    15 // IN1
#define MOTOR_L_IN2    16 // IN2
#define MOTOR_R_IN3    18 // IN3 (swapped: right motor FWD was physically REV)
#define MOTOR_R_IN4    17 // IN4
#define DRV_SLEEP_PIN  41 // Driver Wake/Sleep logic

// 2. Motor Encoders
#define ENC_L_A        21  // swapped: left encoder counted negative on FWD
#define ENC_L_B        14 
#define ENC_R_A        38 // MOT34-ENA
#define ENC_R_B        39 // MOT34-ENB

// 3. IR Receiver Array (Analog Inputs)
#define RX_LF          4  // Left-Front
#define RX_L45         6  // Left-45
#define RX_L           10 // Left-Side
#define RX_R           7  // Right-Side
#define RX_R45         2  // Right-45
#define RX_RF          1  // Right-Front

// 4. IR Emitter Array (Digital Drive)
#define EMIT_LF        13
#define EMIT_L45       45
#define EMIT_L         47
#define EMIT_R         46
#define EMIT_R45       12
#define EMIT_RF        11

// 5. IMU (MPU6500) I2C
#define IMU_SDA        8
#define IMU_SCL        9
#define IMU_INT        48

// 6. User Interface & Indicators
#define BUTTON_1       42 // Main User Button
#define BOOT_BUTTON    0  // Pin 27/GPIO0
#define BUZZER_PIN     40
#define BUZZER_LEDC_CH 4      // LEDC channel (0-3 used by motors)
#define BUZZER_FREQ    4000   // SMD5020 resonant frequency Hz
#define BUZZER_RES     8      // 8-bit — NEVER use duty 255 (DC = coil burn)
#define BUZZER_DUTY    25     // 25/255 = 10% — hard cap, DO NOT EXCEED 50
#define WS2812_DATA    3  // LED_WS2812B
#define BAT_V_SENSE       5      // BAT-TEST
#define BAT_VDIV_MULT     3.1197f // calibrated: 100k+50k divider, corrected vs multimeter

// 7. Competition Physics Constants
// Motor: N20 1:30 gear ratio, 6V, 500RPM
// Encoder: 7 pulses/rev (motor shaft), single-channel RISING only
// Effective ticks/wheel-rev = 7 * 30 = 210
// If using full quadrature (both edges, both channels) = 7 * 4 * 30 = 840
#define WHEEL_DIAMETER    33.4f  // mm — measured physically
#define TICKS_PER_REV     210.0f // Single-channel RISING: 7 PPR * 30 gear ratio
                                 // Change to 840.0 if using full quadrature decoding
#define MOTOR_PWM_FREQ    20000  // 20kHz silent frequency
#define MOTOR_PWM_RES     10     // 10-bit resolution (0–1023)

// Robot geometry (measured)
#define ROBOT_LENGTH_MM   100.0f // front to back
#define WHEEL_AXLE_TO_BACK_MM 33.0f // wheel axle to rear of robot
#define ROBOT_WEIGHT_G    85.0f  // grams with battery

// Brake compensation: stop this many ticks before target to account for
// mechanical coasting after brake(). Robot at ~85g coasts ~5-8 ticks.
// Tune: if robot overshoots → increase. undershoots → decrease.
#define BRAKE_COMP_TICKS  8      // ticks to stop early
