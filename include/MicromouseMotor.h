// =============================================================================
// MicromouseMotor.h
// DRV8833 dual H-bridge motor driver wrapper for one motor.
// Uses two LEDC channels (ESP32-S3 Arduino 2.x API) for PWM.
//
// DRV8833 truth table (IN1/IN2 → motor action):
//   PWM / 0  → forward
//   0 / PWM  → reverse
//   0 / 0    → coast (fast decay)
//   1 / 1    → brake (slow decay)
//
// Channel allocation (avoid overlapping with other LEDC users):
//   leftMotor  → ch 0, 1
//   rightMotor → ch 2, 3
// =============================================================================
#pragma once
#include <Arduino.h>

class MicromouseMotor {
private:
    uint8_t pinIN1;
    uint8_t pinIN2;
    uint8_t ch1;   // LEDC channel for IN1
    uint8_t ch2;   // LEDC channel for IN2

    // N20 + DRV8833 + ESP32-S3 Constants
    const uint32_t PWM_FREQ = 20000; // 20 kHz (above human hearing — silent operation)
    const uint8_t  PWM_RES  = 10;    // 10-bit → 0–1023
    const int      MAX_PWM  = 1023;

    // MIN_POWER: minimum PWM that physically overcomes DRV8833 FET drop + motor stiction.
    // DRV8833 has ~0.6V drop per FET path. At 6V supply:
    // Effective threshold ≈ 150/1023 * 6V ≈ 0.88V effective.
    // Lower if the motor stutters at very low speeds; raise if it buzzes without moving.
    const int MIN_POWER = 150;

    // Human-readable label for verbose logging
    const char* label;

public:
    // ch1Num / ch2Num: unique LEDC channel numbers (0–7) per motor instance.
    // label: short string shown in Serial messages (e.g. "LEFT", "RIGHT")
    MicromouseMotor(uint8_t in1, uint8_t in2, uint8_t ch1Num, uint8_t ch2Num,
                    const char* motorLabel = "MOTOR")
        : pinIN1(in1), pinIN2(in2), ch1(ch1Num), ch2(ch2Num), label(motorLabel) {}

    // --------------------------------------------------------------------------
    // begin() — attach LEDC channels to pins and set motor to coast
    // --------------------------------------------------------------------------
    void begin() {
        Serial.printf("[MOTOR:%s] begin() — IN1=GPIO%d ch%d  IN2=GPIO%d ch%d  freq=%luHz  res=%dbit\n",
                      label, pinIN1, ch1, pinIN2, ch2, (unsigned long)PWM_FREQ, (int)PWM_RES);

        // Arduino-ESP32 2.x LEDC API: setup channel first, then attach pin
        ledcSetup(ch1, PWM_FREQ, PWM_RES);
        ledcAttachPin(pinIN1, ch1);
        Serial.printf("[MOTOR:%s] begin() — ch%d attached to GPIO%d\n", label, ch1, pinIN1);

        ledcSetup(ch2, PWM_FREQ, PWM_RES);
        ledcAttachPin(pinIN2, ch2);
        Serial.printf("[MOTOR:%s] begin() — ch%d attached to GPIO%d\n", label, ch2, pinIN2);

        coast();
        Serial.printf("[MOTOR:%s] begin() done — motor in COAST state\n", label);
    }

    // --------------------------------------------------------------------------
    // drive() — accepts signed PID output: negative=reverse, positive=forward
    // Maps PID range [1, MAX_PWM] → real motor range [MIN_POWER, MAX_PWM]
    // --------------------------------------------------------------------------
    void drive(int speed) {
        int original = speed;
        speed = constrain(speed, -MAX_PWM, MAX_PWM);

        if (speed == 0) {
            Serial.printf("[MOTOR:%s] drive(%d) → COAST\n", label, original);
            coast();
            return;
        }

        if (speed > 0) {
            int pwm = map(speed, 1, MAX_PWM, MIN_POWER, MAX_PWM);
            ledcWrite(ch1, pwm);
            ledcWrite(ch2, 0);
            Serial.printf("[MOTOR:%s] drive(%d) → FORWARD  pwm=%d\n",
                          label, original, pwm);
        } else {
            int pwm = map(-speed, 1, MAX_PWM, MIN_POWER, MAX_PWM);
            ledcWrite(ch1, 0);
            ledcWrite(ch2, pwm);
            Serial.printf("[MOTOR:%s] drive(%d) → REVERSE  pwm=%d\n",
                          label, original, pwm);
        }
    }

    // --------------------------------------------------------------------------
    // brake() — active braking: both pins HIGH → slow decay
    // --------------------------------------------------------------------------
    void brake() {
        Serial.printf("[MOTOR:%s] brake() — both pins HIGH (slow decay)\n", label);
        ledcWrite(ch1, MAX_PWM);
        ledcWrite(ch2, MAX_PWM);
    }

    // --------------------------------------------------------------------------
    // coast() — free-wheel: both pins LOW → fast decay
    // --------------------------------------------------------------------------
    void coast() {
        Serial.printf("[MOTOR:%s] coast() — both pins LOW (fast decay)\n", label);
        ledcWrite(ch1, 0);
        ledcWrite(ch2, 0);
    }
};
