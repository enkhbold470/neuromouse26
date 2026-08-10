// include/MicromouseEncoder.h — LEGACY ISR half-quadrature encoder.
//
// Unused by [env:main]. Prefer MicromouseEncoderPCNT.h (hardware 4× decode).
// Kept for a few older test/ sketches (encoder-test, mpu6500, ir-turn-test,
// motor-freq-config). Do not add new main-firmware dependencies on this file.

#ifndef MICROMOUSE_ENCODER_H
#define MICROMOUSE_ENCODER_H

#include <Arduino.h>

// Half-quadrature ISR encoder. On rising edge of A, sample B level → ±1 tick.
// CW vs CCW (hand-rolled or motor-driven) give opposite signs. `inverted`
// flips final sign if wiring polarity is backward.
//
// Up to 4 instances. Flat ISRs (no `this` ptr) — avoids Xtensa l32r literal
// issues. ISRs live in flash (no IRAM_ATTR): safe because firmware never
// writes flash at runtime.

namespace _mmenc {
    static volatile long c0 = 0, c1 = 0, c2 = 0, c3 = 0;
    static uint8_t pinB0 = 0xFF, pinB1 = 0xFF, pinB2 = 0xFF, pinB3 = 0xFF;

    inline void isr0() { c0 += (pinB0 != 0xFF && digitalRead(pinB0)) ? -1 : 1; }
    inline void isr1() { c1 += (pinB1 != 0xFF && digitalRead(pinB1)) ? -1 : 1; }
    inline void isr2() { c2 += (pinB2 != 0xFF && digitalRead(pinB2)) ? -1 : 1; }
    inline void isr3() { c3 += (pinB3 != 0xFF && digitalRead(pinB3)) ? -1 : 1; }
}

class MicromouseEncoder {
    uint8_t pinA;
    uint8_t pinB;
    bool    inv;
    int     id;
    static int nextId;

public:
    MicromouseEncoder(uint8_t a, uint8_t b = 0xFF, bool inverted = false)
        : pinA(a), pinB(b), inv(inverted), id(nextId++) {}

    void begin() {
        pinMode(pinA, INPUT);
        if (pinB != 0xFF) pinMode(pinB, INPUT);
        void (*table[4])() = { _mmenc::isr0, _mmenc::isr1, _mmenc::isr2, _mmenc::isr3 };
        switch (id) {
            case 0: _mmenc::pinB0 = pinB; break;
            case 1: _mmenc::pinB1 = pinB; break;
            case 2: _mmenc::pinB2 = pinB; break;
            case 3: _mmenc::pinB3 = pinB; break;
        }
        if (id >= 0 && id < 4) {
            attachInterrupt(digitalPinToInterrupt(pinA), table[id], RISING);
        }
    }

    long getTicks() const {
        long v = 0;
        noInterrupts();
        switch (id) {
            case 0: v = _mmenc::c0; break;
            case 1: v = _mmenc::c1; break;
            case 2: v = _mmenc::c2; break;
            case 3: v = _mmenc::c3; break;
        }
        interrupts();
        return inv ? -v : v;
    }

    void reset() {
        noInterrupts();
        switch (id) {
            case 0: _mmenc::c0 = 0; break;
            case 1: _mmenc::c1 = 0; break;
            case 2: _mmenc::c2 = 0; break;
            case 3: _mmenc::c3 = 0; break;
        }
        interrupts();
    }
};

int MicromouseEncoder::nextId = 0;

#endif
