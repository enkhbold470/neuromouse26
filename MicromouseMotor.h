// --- MicromouseMotor.h ---
#pragma once
#include <Arduino.h>

class MicromouseMotor {
private:
    uint8_t pinIN1;
    uint8_t pinIN2;
    
    // N20 & ESP32-S3 Constants
    const uint32_t PWM_FREQ = 20000; // 20 kHz (Silent & Efficient)
    const uint8_t PWM_RES = 10;      // 10-bit (0-1023 granularity)
    const int MAX_PWM = 1023;
    
    // Friction Feedforward: Minimum PWM required to physically move the motor.
    // Tune this value! It prevents the PID from "stalling" at low speeds.
    const int MIN_POWER = 40; 

public:
    // Constructor
    MicromouseMotor(uint8_t in1, uint8_t in2) {
        pinIN1 = in1;
        pinIN2 = in2;
    }

    // Initialize hardware timers
    void begin() {
        // Modern ESP32 Core v3.x syntax for hardware PWM
        ledcAttach(pinIN1, PWM_FREQ, PWM_RES);
        ledcAttach(pinIN2, PWM_FREQ, PWM_RES);
        coast(); // Ensure motors start off
    }

    // The Ultimate PID-Ready Drive Function
    // Accepts values from -1023 to +1023
    void drive(int speed) {
        // Constrain to prevent overflow
        speed = constrain(speed, -MAX_PWM, MAX_PWM);

        if (speed == 0) {
            coast();
            return;
        }

        // Apply Friction Feedforward (removes deadband)
        if (speed > 0) {
            speed = map(speed, 1, MAX_PWM, MIN_POWER, MAX_PWM);
            // Fast Decay Forward: IN1 = PWM, IN2 = LOW
            ledcWrite(pinIN1, speed);
            ledcWrite(pinIN2, 0);
        } else {
            speed = map(speed, -1, -MAX_PWM, -MIN_POWER, -MAX_PWM);
            // Fast Decay Reverse: IN1 = LOW, IN2 = PWM
            ledcWrite(pinIN1, 0);
            ledcWrite(pinIN2, abs(speed));
        }
    }

    // Active Braking (Slow Decay)
    void brake() {
        ledcWrite(pinIN1, MAX_PWM);
        ledcWrite(pinIN2, MAX_PWM);
    }

    // Coasting (Fast Decay Zero)
    void coast() {
        ledcWrite(pinIN1, 0);
        ledcWrite(pinIN2, 0);
    }
};