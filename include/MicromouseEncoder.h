#ifndef MICROMOUSE_ENCODER_H
#define MICROMOUSE_ENCODER_H

#include <Arduino.h>

class MicromouseEncoder {
    uint8_t       pinA, pinB;
    volatile long count;
public:
    MicromouseEncoder(uint8_t a, uint8_t b) : pinA(a), pinB(b), count(0) {}

    void begin(void (*isr)()) {
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pinA), isr, RISING);
    }

    void handleInterrupt();

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

#endif
