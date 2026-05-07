#ifndef MICROMOUSE_MOTOR_H
#define MICROMOUSE_MOTOR_H

#include <Arduino.h>
#include "PinConfig.h"

// N20 6V + DRV8833: 20 kHz keeps motor silent, reduces switching losses.
// 8-bit resolution (0-255) matches existing PWM scale throughout codebase.
#define MOTOR_PWM_FREQ_HZ  20000
#define MOTOR_PWM_BITS     8

class MicromouseMotor {
    uint8_t pinIN1, pinIN2;
    uint8_t ch1, ch2;
    bool    inverted;

    void ledcSet(uint8_t ch, int duty) {
        ledcWrite(ch, (uint32_t)constrain(duty, 0, 255));
    }

public:
    MicromouseMotor(uint8_t in1, uint8_t in2, uint8_t c1, uint8_t c2, bool inv = false)
        : pinIN1(in1), pinIN2(in2), ch1(c1), ch2(c2), inverted(inv) {}

    void begin() {
        ledcSetup(ch1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
        ledcSetup(ch2, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
        ledcAttachPin(pinIN1, ch1);
        ledcAttachPin(pinIN2, ch2);
        coast();
    }

    void drive(int speed) {
        if (inverted) speed = -speed;
        speed = constrain(speed, -255, 255);
        if (speed == 0) { coast(); return; }
        if (speed > 0) { ledcSet(ch1, speed); ledcSet(ch2, 0);      }
        else           { ledcSet(ch1, 0);     ledcSet(ch2, -speed); }
    }

    void brake() { ledcSet(ch1, 255); ledcSet(ch2, 255); }
    void coast() { ledcSet(ch1, 0);   ledcSet(ch2, 0);   }
};

#endif
