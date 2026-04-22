#pragma once
#include <Arduino.h>

class MicromouseEncoder {
private:
    uint8_t pinA, pinB;
    volatile long count;

public:
    MicromouseEncoder(uint8_t a, uint8_t b) : pinA(a), pinB(b), count(0) {}

    void begin(void (*ISR_callback)()) {
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pinA), ISR_callback, RISING);
    }

    // Logic for the ISR
    void IRAM_ATTR handleInterrupt() {
        digitalRead(pinB) ? count++ : count--;
    }

    long getTicks() { return count; }
    void reset() { count = 0; }
};