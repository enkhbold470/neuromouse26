#ifndef MICROMOUSE_MOTOR_H
#define MICROMOUSE_MOTOR_H

#include <Arduino.h>
#include "PinConfig.h"

class MicromouseMotor {
    uint8_t pinIN1, pinIN2;
public:
    MicromouseMotor(uint8_t in1, uint8_t in2, uint8_t c1, uint8_t c2)
        : pinIN1(in1), pinIN2(in2) {} // c1/c2 ignored for ultimate portability, kept for signature compatibility

    void begin() {
        pinMode(pinIN1, OUTPUT);
        pinMode(pinIN2, OUTPUT);
        coast();
    }

    void drive(int speed) {
        speed = constrain(speed, -255, 255); // Universal 8-bit Arduino PWM
        if (speed == 0) { coast(); return; }
        if (speed > 0) {
            analogWrite(pinIN1, speed);
            analogWrite(pinIN2, 0);
        } else {
            analogWrite(pinIN1, 0);
            analogWrite(pinIN2, -speed);
        }
    }

    void brake() { analogWrite(pinIN1, 255); analogWrite(pinIN2, 255); }
    void coast() { analogWrite(pinIN1, 0);   analogWrite(pinIN2, 0);   }
};

#endif
