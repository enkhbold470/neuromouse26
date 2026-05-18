#ifndef MICROMOUSE_ENCODER_PCNT_H
#define MICROMOUSE_ENCODER_PCNT_H

// Full 4x quadrature encoder via ESP32-S3 PCNT peripheral. Zero CPU/ISR cost
// at running speed — pulse counter is silicon. Compare to MicromouseEncoder.h
// (rising-edge ISR, half-quadrature, ~205 ticks/rev effective).
//
// Layout per PCNT unit:
//   ch0: pulse=A, ctrl=B → +1 on A-rising  when B=0, −1 when B=1
//                          −1 on A-falling when B=0, +1 when B=1
//   ch1: pulse=B, ctrl=A → mirrored, contributes the other 2 edges
// → 4 counts per full A·B cycle = 4x decode.
//
// PCNT 16-bit counter (±32767) → we periodically drain into a long accumulator
// on every getTicks() call. As long as the consumer polls more often than the
// motor can spin ~32k counts, no overflow. At 500 RPM × ~420 raw/rev × 4 =
// ~14000 counts/sec — polling at 200 Hz drains ~70 counts/call, safe.
//
// Assumes ESP-IDF 4.x legacy driver/pcnt.h (Arduino-2.x espressif32 stock).

#include <Arduino.h>
#include "driver/pcnt.h"

class MicromouseEncoderPCNT {
public:
    MicromouseEncoderPCNT(pcnt_unit_t unit,
                          int pinA,
                          int pinB,
                          bool inverted = false,
                          uint16_t filterAPBCycles = 250)
        : _unit(unit), _a(pinA), _b(pinB),
          _inv(inverted), _filt(filterAPBCycles) {}

    void begin() {
        pinMode(_a, INPUT);
        pinMode(_b, INPUT);

        pcnt_config_t c0 = {};
        c0.pulse_gpio_num = _a;
        c0.ctrl_gpio_num  = _b;
        c0.unit           = _unit;
        c0.channel        = PCNT_CHANNEL_0;
        c0.pos_mode       = PCNT_COUNT_INC;
        c0.neg_mode       = PCNT_COUNT_DEC;
        c0.hctrl_mode     = PCNT_MODE_KEEP;
        c0.lctrl_mode     = PCNT_MODE_REVERSE;
        c0.counter_h_lim  =  H_LIM;
        c0.counter_l_lim  = -H_LIM;
        pcnt_unit_config(&c0);

        pcnt_config_t c1 = {};
        c1.pulse_gpio_num = _b;
        c1.ctrl_gpio_num  = _a;
        c1.unit           = _unit;
        c1.channel        = PCNT_CHANNEL_1;
        c1.pos_mode       = PCNT_COUNT_DEC;
        c1.neg_mode       = PCNT_COUNT_INC;
        c1.hctrl_mode     = PCNT_MODE_KEEP;
        c1.lctrl_mode     = PCNT_MODE_REVERSE;
        c1.counter_h_lim  =  H_LIM;
        c1.counter_l_lim  = -H_LIM;
        pcnt_unit_config(&c1);

        if (_filt > 0) {
            pcnt_set_filter_value(_unit, _filt > 1023 ? 1023 : _filt);
            pcnt_filter_enable(_unit);
        } else {
            pcnt_filter_disable(_unit);
        }

        pcnt_counter_pause(_unit);
        pcnt_counter_clear(_unit);
        pcnt_counter_resume(_unit);
        _acc = 0;
        _lastRaw = 0;
    }

    // Drain hardware counter into accumulator and return signed total.
    long getTicks() {
        int16_t cur = 0;
        pcnt_get_counter_value(_unit, &cur);
        _acc += (long)(cur - _lastRaw);
        _lastRaw = cur;
        if (cur > (H_LIM - 1024) || cur < -(H_LIM - 1024)) {
            pcnt_counter_clear(_unit);
            _lastRaw = 0;
        }
        return _inv ? -_acc : _acc;
    }

    void reset() {
        pcnt_counter_pause(_unit);
        pcnt_counter_clear(_unit);
        pcnt_counter_resume(_unit);
        _acc = 0;
        _lastRaw = 0;
    }

private:
    static constexpr int16_t H_LIM = 30000;
    pcnt_unit_t _unit;
    int _a, _b;
    bool _inv;
    uint16_t _filt;
    long _acc = 0;
    int16_t _lastRaw = 0;
};

#endif
