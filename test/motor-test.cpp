#include <Arduino.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"

// MOTOR_L_INV and MOTOR_R_INV intentionally both false here —
// we want to see raw direction without any inversion so we can
// confirm which physical direction +/- maps to.
MicromouseMotor leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, false);
MicromouseMotor rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, false);

struct Step { const char* label; int l; int r; };
static const Step STEPS[] = {
    {"L+400  R 0  ", +400,    0},
    {"L-400  R 0  ", -400,    0},
    {"L 0   R+400 ",    0, +400},
    {"L 0   R-400 ",    0, -400},
};

void setup() {
    Serial.begin(115200);
    pinMode(MOTOR_SLEEP, OUTPUT);
    digitalWrite(MOTOR_SLEEP, HIGH);
    leftMotor.begin();
    rightMotor.begin();
    Serial.println("Motor direction test (no INV flags)");
}

void loop() {
    for (auto& s : STEPS) {
        Serial.println(s.label);
        leftMotor.drive(s.l);
        rightMotor.drive(s.r);
        delay(1200);
        leftMotor.brake(); rightMotor.brake();
        delay(400);
    }
}
