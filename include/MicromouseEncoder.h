#ifndef MICROMOUSE_ENCODER_H
#define MICROMOUSE_ENCODER_H

#include <Arduino.h>

// Single-channel ISR-based encoder counter.
// Counts rising edges of pinA. pinB is accepted for API compat but unused —
// we deliberately do NOT do quadrature. Direction comes only from `inverted`.
//
// Up to 4 encoder instances. ISRs are flat IRAM functions that increment a
// fixed counter slot — no `this` indirection (avoids Xtensa l32r literal
// placement issues).

namespace _mmenc {
    static volatile long c0 = 0, c1 = 0, c2 = 0, c3 = 0;
    // No IRAM_ATTR: ISRs live in flash. Acceptable here because the firmware
    // never writes flash at runtime. Flat ISRs avoid Xtensa l32r literal issues.
    inline void isr0() { c0++; }
    inline void isr1() { c1++; }
    inline void isr2() { c2++; }
    inline void isr3() { c3++; }
}

class MicromouseEncoder {
    uint8_t pinA;
    uint8_t pinB;        // accepted but unused
    bool    inv;
    int     id;
    static int nextId;

public:
    MicromouseEncoder(uint8_t a, uint8_t b = 0xFF, bool inverted = false)
        : pinA(a), pinB(b), inv(inverted), id(nextId++) {}

    void begin() {
        pinMode(pinA, INPUT);
        void (*table[4])() = { _mmenc::isr0, _mmenc::isr1, _mmenc::isr2, _mmenc::isr3 };
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
