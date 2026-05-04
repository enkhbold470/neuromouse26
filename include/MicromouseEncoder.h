// =============================================================================
// MicromouseEncoder.h
// Quadrature encoder reader for N20 motors.
//
// Hardware note:
//   ENC_x_A is the primary interrupt pin (RISING edge = one tick).
//   ENC_x_B is sampled at the same moment to determine direction:
//     B HIGH when A rises → forward (count++)
//     B LOW  when A rises → reverse (count--)
//   Single-channel RISING-only: 7 PPR × 30 gear ratio = 210 ticks/wheel-rev.
// =============================================================================
#pragma once
#include <Arduino.h>

class MicromouseEncoder {
private:
    uint8_t       pinA, pinB;
    volatile long count;

public:
    MicromouseEncoder(uint8_t a, uint8_t b)
        : pinA(a), pinB(b), count(0) {}

    void begin(void (*isr)()) {
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pinA), isr, RISING);
    }

    // Called from ISR — defined in MicromouseEncoder.cpp (IRAM_ATTR)
    void IRAM_ATTR handleInterrupt();

    long getTicks() {
        noInterrupts();
        long v = count;
        interrupts();
        return v;
    }

    void reset() {
        noInterrupts();
        count = 0;
        interrupts();
    }
};
