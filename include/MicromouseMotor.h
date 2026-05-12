#ifndef MICROMOUSE_MOTOR_H
#define MICROMOUSE_MOTOR_H

#include <Arduino.h>
#include "PinConfig.h"

class MicromouseMotor {
    uint8_t pinIN1, pinIN2;
    uint8_t ch1, ch2;
    bool    inverted;

    void ledcSet(uint8_t ch, int duty) {
        ledcWrite(ch, (uint32_t)constrain(duty, 0, MOTOR_PWM_MAX));
    }

public:
    MicromouseMotor(uint8_t in1, uint8_t in2, uint8_t c1, uint8_t c2, bool inv = false)
        : pinIN1(in1), pinIN2(in2), ch1(c1), ch2(c2), inverted(inv) {}

    void begin() {
        // Hold IN pins LOW as plain GPIO first → no float, no kick.
        pinMode(pinIN1, OUTPUT); digitalWrite(pinIN1, LOW);
        pinMode(pinIN2, OUTPUT); digitalWrite(pinIN2, LOW);
        // Configure LEDC channels and force duty 0 BEFORE attaching to pins,
        // so the moment pin switches to LEDC it stays LOW (no stale duty kick).
        ledcSetup(ch1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
        ledcSetup(ch2, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
        ledcWrite(ch1, 0);
        ledcWrite(ch2, 0);
        ledcAttachPin(pinIN1, ch1);
        ledcAttachPin(pinIN2, ch2);
    }

    void drive(int speed) {
        if (inverted) speed = -speed;
        speed = constrain(speed, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
        if (speed == 0) { coast(); return; }
        if (speed > 0) { ledcSet(ch1, speed); ledcSet(ch2, 0);      }
        else           { ledcSet(ch1, 0);     ledcSet(ch2, -speed); }
    }

    void brake() { ledcSet(ch1, DRIVE_PWM); ledcSet(ch2, DRIVE_PWM); }
    void coast() { ledcSet(ch1, 0);            ledcSet(ch2, 0);            }
};

#endif
