// --- MicromouseMotor.h ---
#pragma once
#include <Arduino.h>

class MicromouseMotor {
private:
    uint8_t pinIN1;
    uint8_t pinIN2;

    // N20 + DRV8833 + ESP32-S3 Constants
    const uint32_t PWM_FREQ = 20000; // 20 kHz (silent operation)
    const uint8_t  PWM_RES  = 10;    // 10-bit → 0–1023
    const int      MAX_PWM  = 1023;

    // MIN_POWER: minimum PWM that physically overcomes DRV8833 FET drop + motor stiction.
    // DRV8833 has ~0.6V drop per FET path. At 6V supply:
    // Effective voltage at MIN_POWER should be ~1.5V → MIN_POWER ≈ 150/1023 * 6V ≈ 0.88V
    // Start at 150, lower it if the motor stutters at very low speeds.
    const int MIN_POWER = 150;

public:
    MicromouseMotor(uint8_t in1, uint8_t in2) : pinIN1(in1), pinIN2(in2) {}

    void begin() {
        ledcAttach(pinIN1, PWM_FREQ, PWM_RES);
        ledcAttach(pinIN2, PWM_FREQ, PWM_RES);
        coast();
    }

    // drive(): accepts -1023 to +1023 from PID
    void drive(int speed) {
        speed = constrain(speed, -MAX_PWM, MAX_PWM);

        if (speed == 0) {
            coast();
            return;
        }

        if (speed > 0) {
            // Map PID output range into real motor-usable range [MIN_POWER, MAX_PWM]
            int pwm = map(speed, 1, MAX_PWM, MIN_POWER, MAX_PWM);
            ledcWrite(pinIN1, pwm);
            ledcWrite(pinIN2, 0);
        } else {
            int pwm = map(-speed, 1, MAX_PWM, MIN_POWER, MAX_PWM);
            ledcWrite(pinIN1, 0);
            ledcWrite(pinIN2, pwm);
        }
    }

    // brake(): active braking (slow decay — both pins HIGH)
    void brake() {
        ledcWrite(pinIN1, MAX_PWM);
        ledcWrite(pinIN2, MAX_PWM);
    }

    // coast(): free-wheel (fast decay — both pins LOW)
    void coast() {
        ledcWrite(pinIN1, 0);
        ledcWrite(pinIN2, 0);
    }
};
