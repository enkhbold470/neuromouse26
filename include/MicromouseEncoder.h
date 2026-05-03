// =============================================================================
// MicromouseEncoder.h
// Quadrature encoder reader for N20 motors.
//
// Hardware note:
//   ENC_x_A is the primary interrupt pin (RISING edge = one tick).
//   ENC_x_B is sampled at the same moment to determine direction:
//     B HIGH when A rises → counter-clockwise rotation → count++
//     B LOW  when A rises → clockwise rotation         → count--
//   This is single-channel RISING-only decoding: 7 pulses/motor-rev × 30 gear
//   ratio = 210 effective ticks/wheel-rev (TICKS_PER_REV in PinConfig.h).
//
// To upgrade to full quadrature (4× resolution = 840 ticks/rev):
//   1. Attach a second interrupt on pinB (CHANGE)
//   2. Decode both A and B states in that ISR
//   3. Update TICKS_PER_REV to 840
// =============================================================================
#pragma once
#include <Arduino.h>

class MicromouseEncoder {
private:
    uint8_t         pinA, pinB;
    volatile long   count;
    const char*     label;

public:
    MicromouseEncoder(uint8_t a, uint8_t b, const char* encLabel = "ENC")
        : pinA(a), pinB(b), count(0), label(encLabel) {}

    // --------------------------------------------------------------------------
    // begin() — set pin modes and attach interrupt
    // --------------------------------------------------------------------------
    void begin(void (*ISR_callback)()) {
        Serial.printf("[ENC:%s] begin() — pinA=GPIO%d (interrupt, RISING)  pinB=GPIO%d (direction)\n",
                      label, pinA, pinB);

        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pinA), ISR_callback, RISING);

        Serial.printf("[ENC:%s] begin() done — interrupt attached, count reset to 0\n", label);
    }

    // --------------------------------------------------------------------------
    // handleInterrupt() — called from ISR; must be in IRAM.
    // Definition lives in src/MicromouseEncoder.cpp (not inline — inline
    // IRAM_ATTR methods cause "literal placed after use" linker errors on
    // Xtensa ESP32-S3).
    // --------------------------------------------------------------------------
    void IRAM_ATTR handleInterrupt();

    // --------------------------------------------------------------------------
    // getTicks() — return current accumulated tick count (thread-safe read)
    // --------------------------------------------------------------------------
    long getTicks() {
        noInterrupts();
        long val = count;
        interrupts();
        return val;
    }

    // --------------------------------------------------------------------------
    // reset() — zero the tick counter
    // --------------------------------------------------------------------------
    void reset() {
        noInterrupts();
        count = 0;
        interrupts();
        Serial.printf("[ENC:%s] reset() — tick counter zeroed\n", label);
    }

    // --------------------------------------------------------------------------
    // printStatus() — dump current state to Serial
    // --------------------------------------------------------------------------
    void printStatus() {
        long val = getTicks();
        Serial.printf("[ENC:%s] ticks=%ld  pinA=GPIO%d  pinB=GPIO%d\n",
                      label, val, pinA, pinB);
    }
};
