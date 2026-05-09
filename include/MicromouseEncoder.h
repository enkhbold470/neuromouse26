#ifndef MICROMOUSE_ENCODER_H
#define MICROMOUSE_ENCODER_H

#include <Arduino.h>
#include "driver/pcnt.h"

// PCNT quadrature decoder — 4× resolution vs single-channel ISR.
// Both A and B channels counted on both edges → TICKS_PER_REV = 14 PPR × 4 × 30 gear = 1680.
// Hardware glitch filter (1µs) replaces the old 200µs software noise filter.
// Max count ±32767 — safe for 22 cells at TICKS_PER_CELL≈1440 before overflow.
// Reset before each move to stay well within range.
// If forward drive gives negative ticks, pass inv=true to constructor.
class MicromouseEncoder {
    uint8_t     pinA, pinB;
    bool        inv;
    pcnt_unit_t unit;

    static int nextUnit;  // auto-assigns PCNT_UNIT_0, _1, ... in construction order

public:
    MicromouseEncoder(uint8_t a, uint8_t b, bool inverted = false)
        : pinA(a), pinB(b), inv(inverted), unit((pcnt_unit_t)(nextUnit++)) {}

    void begin() {
        // Channel 0: pulse = A, direction from B level
        // Forward (A leads B): A rises at B=0, A falls at B=1 → both count UP
        pcnt_config_t ca = {};
        ca.pulse_gpio_num = (int)pinA;
        ca.ctrl_gpio_num  = (int)pinB;
        ca.pos_mode       = PCNT_COUNT_INC;
        ca.neg_mode       = PCNT_COUNT_DEC;
        ca.lctrl_mode     = PCNT_MODE_KEEP;    // B=0: keep direction
        ca.hctrl_mode     = PCNT_MODE_REVERSE; // B=1: reverse direction
        ca.counter_h_lim  = 32767;
        ca.counter_l_lim  = -32768;
        ca.unit           = unit;
        ca.channel        = PCNT_CHANNEL_0;
        pcnt_unit_config(&ca);

        // Channel 1: pulse = B, direction from A level
        // Forward: B rises at A=1, B falls at A=0 → both count UP
        pcnt_config_t cb = {};
        cb.pulse_gpio_num = (int)pinB;
        cb.ctrl_gpio_num  = (int)pinA;
        cb.pos_mode       = PCNT_COUNT_INC;
        cb.neg_mode       = PCNT_COUNT_DEC;
        cb.lctrl_mode     = PCNT_MODE_REVERSE; // A=0: reverse direction
        cb.hctrl_mode     = PCNT_MODE_KEEP;    // A=1: keep direction
        cb.counter_h_lim  = 32767;
        cb.counter_l_lim  = -32768;
        cb.unit           = unit;
        cb.channel        = PCNT_CHANNEL_1;
        pcnt_unit_config(&cb);

        // 1µs glitch filter (80 APB clocks @ 80 MHz) — removes electrical noise
        pcnt_set_filter_value(unit, 80);
        pcnt_filter_enable(unit);

        pcnt_counter_pause(unit);
        pcnt_counter_clear(unit);
        pcnt_counter_resume(unit);
    }

    long getTicks() {
        int16_t v = 0;
        pcnt_get_counter_value(unit, &v);
        return inv ? -(long)v : (long)v;
    }

    void reset() {
        pcnt_counter_pause(unit);
        pcnt_counter_clear(unit);
        pcnt_counter_resume(unit);
    }
};

int MicromouseEncoder::nextUnit = 0;

#endif
