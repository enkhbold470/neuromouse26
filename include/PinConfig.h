#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <Arduino.h>

// rgb led
constexpr uint8_t rgb_pin      = 48;

// ── Pins ──────────────────────────────────────────────────────────────────────
// IN1/IN2 (GPIO15/16) → OUT1/OUT2 → left motor.
// IN3/IN4 (GPIO17/18) → OUT3/OUT4 → right motor.
constexpr uint8_t MOTOR_L_IN1      = 15;
constexpr uint8_t MOTOR_L_IN2      = 16;
constexpr uint8_t MOTOR_R_IN3      = 17;
constexpr uint8_t MOTOR_R_IN4      = 18;
// DRV8833 nSLEEP wired to GPIO41 — must be driven HIGH at boot to enable drivers.
constexpr uint8_t MOTOR_SLEEP      = 41;

constexpr uint8_t ENC_L_A          = 21;
constexpr uint8_t ENC_L_B          = 14;
constexpr uint8_t ENC_R_A          = 38;
constexpr uint8_t ENC_R_B          = 39;

// ── IR sensors ────────────────────────────────────────────────────────────────
// Sensor geometry (as-built, confirmed 2026-06-23):
//   LF, RF : aimed ~10° from straight-forward (nearly perpendicular to front
//            wall). Used for front-wall detection and PH_ALIGN_FRONT.
//   L45, R45 (labelled "45" in netlist, actually 30° from robot axis):
//            aimed 30° outward from the robot's forward axis. These are the
//            SIDE sensors. At 30°, a 42.5 mm lateral wall sits at
//            42.5/sin(30°) = 85 mm sensor distance — twice as far as a
//            true 90° side sensor, so cal values (330-400 cts) are weak.
//            IMPORTANT: a front wall at 90 mm ahead is seen at 90/cos(30°)
//            ≈ 104 mm diagonal by these sensors → ~220 raw counts → above
//            the side-wall threshold → phantom side wall. Fix in Planner.h:
//            skip side re-sensing when front wall is confirmed.
//   L, R (true 90° perpendicular sensors): wired on PCB (netlist signals
//            L_EMIT/L_RECEIVER, R_EMIT/R_RECEIVER) but NOT in PAIRS[] — the
//            L45/R45 pair is currently used as the sole side sensor.
constexpr uint8_t RX_L45             = 6;   //good
constexpr uint8_t RX_LF            = 4; //good
constexpr uint8_t RX_RF            = 1; //good
constexpr uint8_t RX_R45             = 2; //good

constexpr uint8_t EMIT_L45           = 45; //good
constexpr uint8_t EMIT_LF          = 13; //good
constexpr uint8_t EMIT_RF          = 11;  //good
constexpr uint8_t EMIT_R45           = 12; //good

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

// ── Hardware physical info (reference, not used as constants) ─────────────────
// Robot weight ~161 g w/ batteries. 2S LiPo (7.4 V nominal).
// N20 1:30 motor, 500 RPM @ 6 V.
// Side IR sensor optical axis = 50 mm fwd of wheel axle.
// Rear bumper = 40 mm aft of wheel axle.

// ── Motor PWM (LEDC) ─────────────────────────────────────────────────────────
// 200 Hz: empirically best torque on this DRV8833+N20+heavy chassis combo.
// Audible whine but maximum stiction-breaking torque from each cycle's
// peak current spike. Higher freq (10–20 kHz) produced weaker low-PWM
// response. If you change this, re-run Calibrate on-device.
constexpr int     MOTOR_PWM_FREQ_HZ = 200;
constexpr int     MOTOR_PWM_BITS    = 10;
constexpr int     MOTOR_PWM_MAX     = 1023;

// ── Motor polarity ────────────────────────────────────────────────────────────
constexpr bool    MOTOR_L_INV      = false;
constexpr bool    MOTOR_R_INV      = true;

// ── Wheel / encoder physics ───────────────────────────────────────────────────
// N20 1:30 500RPM @ 6V on 2S LiPo (7.4 V).
// Encoder: single-channel rising-edge ISR with 200 µs filter.
//   physical PPR ≈ 14 motor × 30 gear = 420/rev raw
//   effective at running speed ≈ 205 (empirical)
constexpr float   WHEEL_DIAMETER   = 33.4f;   // mm
constexpr float   TICKS_PER_REV    = 205.0f;
constexpr float   WHEEL_TRACK_MM   = 80.0f;   // outer-tire to outer-tire (true axle pitch is slightly less by tire width)
// Battery voltage divider: V_bat → 100k → ADC → 39k → GND.
// Theoretical multiplier: 1 / (39 / (39+100)) = 3.564.
// Empirically calibrated 2026-05-17: actual 7.96V → raw read 6.62V with
// old mult 3.1197 → new mult = 3.1197 × (7.96/6.62) = 3.751.
// Delta from theoretical accounts for ADC ref offset + resistor tolerance.
constexpr float   BAT_VDIV_MULT    = 3.751f;
constexpr float   NOMINAL_VBAT     = 7.4f;    // 2S full-charge target for comp

// ── Encoder L/R equalization ─────────────────────────────────────────────────
// Calibrated 2026-05-17 via on-device auto-calibration.
// Equalizes accumulated rTicks() against leftEnc.getTicks() for equal travel.
constexpr float   RIGHT_ENC_SCALE  = 1.0135f;

// ── Cell / turn geometry ─────────────────────────────────────────────────────
constexpr float   CELL_MM          = 180.0f;
constexpr float   MM_PER_TICK      = 3.14159265f * WHEEL_DIAMETER / TICKS_PER_REV;
constexpr long    TICKS_PER_CELL   = 350;     // empirical, validated by tape

// ── Drive base values ────────────────────────────────────────────────────────
// DRIVE_PWM and TURN_PWM are saturation caps + diagnostic fallbacks; the
// velocity PID computes per-loop PWM from target mm/s, kV, and FF — actual
// running duty is determined by the controller, not these constants.
constexpr int     DRIVE_PWM        = 250;
constexpr int     TURN_PWM         = 200;
constexpr int     DRIVE_PWM_MIN    = 100;     // 200 Hz motors reliably start
                                              // around 10 % duty

// ── Cascaded velocity PID — SOTA, calibrated 2026-05-17 ──────────────────────
// Inner loop runs at 1 / VPID_LOOP_US Hz. Cascade is:
//   speed   PI on (vL+vR)/2 → target  → pidSpeed
//   straight PI on (curL-curR) → 0    → pidStraight   (P on integral of dvel)
//   pwmL = ffShared + pidSpeed − pidStraight + L_PWM_BIAS
//   pwmR = ffShared + pidSpeed + pidStraight + R_PWM_BIAS
// FF: pwm = target/kV_avg + off_avg, scaled by NOMINAL_VBAT / Vbat_measured.
constexpr unsigned long VPID_LOOP_US = 5000;  // 200 Hz, matches one PWM period

constexpr float   VPID_LOOP_KP     = 1.00f;   // pwm per (mm/s) error
constexpr float   VPID_LOOP_KI     = 1.50f;   // pwm per (mm·s)
constexpr float   VPID_INTEG_LIM   = 800.0f;  // anti-windup
constexpr float   VPID_EMA_ALPHA   = 0.50f;   // velocity low-pass

constexpr float   VPID_STRAIGHT_KP = 6.00f;   // pwm per tick of L-R mismatch
constexpr int     VPID_STRAIGHT_MAX = 500;

constexpr int     L_PWM_BIAS       = 0;
constexpr int     R_PWM_BIAS       = 0;

// Calibrated FF (mm/s per PWM unit) and dead-band offset, per wheel.
constexpr float   KV_L             = 2.961f;
constexpr float   KV_R             = 2.749f;
constexpr float   OFF_L            = 0.0f;
constexpr float   OFF_R            = 0.0f;

// Default cell-cruise target mm/s. Recalibrate motors before raising.
constexpr int     CELL_TARGET_MMS  = 250;

// ── Gyro turn (trapezoidal ω + PID on integrated yaw) ────────────────────────
// Surface-independent. Peak/accel scaled for TURN_PWM cap above.
// Locked to test/mpu6500.cpp working values — that test runs the same
// trapezoid+PID and converges cleanly. Do not lower peak/accel; ramp must
// be aggressive enough that profile PWM exceeds motor stiction without
// help from a constant min-PWM floor (which fights the ramp).
constexpr float   TURN_PEAK_OMEGA_DPS  = 360.0f;
constexpr float   TURN_ACCEL_DPS2      = 1800.0f;
constexpr float   TURN_KFF_PWM_PER_DPS = 1.0f;
constexpr float   TURN_KP_PWM_PER_DEG  = 12.0f;
constexpr float   TURN_KD_PWM_PER_DPS  = 0.4f;
constexpr int     TURN_MIN_HOLD_PWM    = 260; // hold-phase floor only
constexpr float   TURN_DEADBAND_DEG    = 1.5f;
constexpr unsigned long TURN_HOLD_MS    = 350;
constexpr unsigned long TURN_TIMEOUT_MS = 4000;
constexpr unsigned long TURN_SETTLE_MS  = 200;

// ── Cell-boundary brake thresholds (derived from cal at runtime) ─────────────
constexpr float   MID_BRAKE_FRAC   = 0.45f;
constexpr float   TURN_CLEAR_FRAC  = 1.15f;

// ── IR calibration defaults ──────────────────────────────────────────────────
// Captured 2026-05-17 in dead-end (centered, all 4 walls present) via
// test/sensor-cal-ble.cpp `de` command, 32-sample mean. Re-capture via BLE
// `ircal` or on-device "Cal IR" menu after a sensor swap, mount-angle
// change, or surface-reflectivity change. Each value = differential
// ambient-subtracted reading (no-wall ~0, wall present 1500–3500).
constexpr int     IR_CAL_LF        = 3483;
// L45/R45 are the 30°-from-forward side sensors. At 42.5 mm lateral wall
// the sensor-to-wall path is 42.5/sin(30°) = 85 mm → weak signal.
// Values captured 2026-05-22 at cell center with side wall present.
constexpr int     IR_CAL_L45       = 330;    // 30° side-left, centered in cell
constexpr int     IR_CAL_R45       = 403;    // 30° side-right, centered in cell
constexpr int     IR_CAL_RF        = 2702;

// Wall-presence thresholds (legacy fixed; overridden by SIDE_ADAPTIVE in Tuning [J]).
// Side (SIDE_ADAPTIVE=false only): open~0-80, wall~300-400; <150 = open.
// Front: open~12-80, wall~500+; 300 gives detection at ~8 cm.
constexpr int     WALL_SIDE_THRESH  = 150;   // legacy only — SIDE_ADAPTIVE replaces this
constexpr int     WALL_FRONT_THRESH = 300;   // lit-amb threshold for front-wall detect

// ── Drive loop misc ──────────────────────────────────────────────────────────
constexpr int     TIMEOUT_MS       = 5000;    // per-cell drive timeout

// Pause after the cell-boundary stopMotors() before pose update / sensing.
// 0 = no pause (max speed). >0 helps if motors coast past the boundary or
// the chassis rocks at brake. Original value before tuning was 80 ms.
constexpr unsigned long CELL_STOP_DELAY_MS = 0;

// ── Maze constants ────────────────────────────────────────────────────────────
constexpr uint8_t MAZE_SIZE        = 16;
constexpr uint16_t MAZE_CELLS      = MAZE_SIZE * MAZE_SIZE;
constexpr uint8_t WALL_NORTH       = 0x01;
constexpr uint8_t WALL_EAST        = 0x02;
constexpr uint8_t WALL_SOUTH       = 0x04;
constexpr uint8_t WALL_WEST        = 0x08;
constexpr uint8_t FLOOD_INFINITY   = 255;

#endif
