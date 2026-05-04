// =============================================================================
// MicromouseIR.h
// 4-sensor IR array: Left-Front, Left-45, Right-45, Right-Front
//
// Calibration:
//   IrCal struct stores noWall/wall ADC values measured at runtime.
//   normalize() maps raw ADC → 0.0 (open) … 1.0 (wall present).
//   Threshold = midpoint between noWall and wall.
//
// Usage:
//   ir.begin();
//   ir.update();               // ~0.4ms — no Serial prints
//   float n = irCal[1].normalize(ir.raw[IR_LEFT_45]);
//   bool  w = ir.wallFront(irCal);
// =============================================================================
#pragma once
#include <Arduino.h>
#include "PinConfig.h"

// ── Sensor indices ────────────────────────────────────────────────────────────
enum IRSensor : uint8_t {
    IR_LEFT_FRONT  = 0,
    IR_LEFT_45     = 1,
    IR_RIGHT_45    = 2,
    IR_RIGHT_FRONT = 3,
    IR_COUNT       = 4
};

// ── Calibration struct ────────────────────────────────────────────────────────
struct IrCal {
    int  noWall    = 0;
    int  wall      = 4095;
    int  threshold = 500;
    bool calibrated = false;  // if false: isWall()→false, normalize()→0 (assume open)

    // 0.0 = fully open, 1.0 = at wall distance used during calibration
    float normalize(int raw) const {
        if (!calibrated) return 0.0f;
        if (wall <= noWall) return 0.0f;
        return constrain((float)(raw - noWall) / (float)(wall - noWall), 0.0f, 2.0f);
    }

    bool isWall(int raw) const {
        if (!calibrated) return false;
        return raw > threshold;
    }
};

// ── MicromouseIR ─────────────────────────────────────────────────────────────
class MicromouseIR {
public:
    int raw[IR_COUNT] = {0, 0, 0, 0};

    static const uint8_t EMIT_PINS[IR_COUNT];
    static const uint8_t RX_PINS[IR_COUNT];

    void begin() {
        pinMode(EMIT_LF,  OUTPUT); digitalWrite(EMIT_LF,  LOW);
        pinMode(EMIT_L45, OUTPUT); digitalWrite(EMIT_L45, LOW);
        pinMode(EMIT_R45, OUTPUT); digitalWrite(EMIT_R45, LOW);
        pinMode(EMIT_RF,  OUTPUT); digitalWrite(EMIT_RF,  LOW);
        pinMode(RX_LF,  INPUT);
        pinMode(RX_L45, INPUT);
        pinMode(RX_R45, INPUT);
        pinMode(RX_RF,  INPUT);
    }

    // Differential read — no Serial prints, safe to call at 500Hz+
    void update() {
        for (int i = 0; i < IR_COUNT; i++) {
            int ambient = analogRead(RX_PINS[i]);
            digitalWrite(EMIT_PINS[i], HIGH);
            delayMicroseconds(50);
            int lit = analogRead(RX_PINS[i]);
            digitalWrite(EMIT_PINS[i], LOW);
            raw[i] = max(0, lit - ambient);
        }
    }

    // Wall detection using calibrated thresholds
    bool wallFront(const IrCal cal[IR_COUNT]) const {
        return cal[IR_LEFT_FRONT].isWall(raw[IR_LEFT_FRONT]) ||
               cal[IR_RIGHT_FRONT].isWall(raw[IR_RIGHT_FRONT]);
    }
    bool wallLeft(const IrCal cal[IR_COUNT]) const {
        return cal[IR_LEFT_45].normalize(raw[IR_LEFT_45]) > 0.3f;
    }
    bool wallRight(const IrCal cal[IR_COUNT]) const {
        return cal[IR_RIGHT_45].normalize(raw[IR_RIGHT_45]) > 0.3f;
    }

    // Sample N readings and return average for one sensor (for calibration)
    int sampleAvg(uint8_t sensorIdx, int n = 64) {
        long sum = 0;
        for (int i = 0; i < n; i++) {
            int ambient = analogRead(RX_PINS[sensorIdx]);
            digitalWrite(EMIT_PINS[sensorIdx], HIGH);
            delayMicroseconds(50);
            int lit = analogRead(RX_PINS[sensorIdx]);
            digitalWrite(EMIT_PINS[sensorIdx], LOW);
            sum += max(0, lit - ambient);
            delay(5);
        }
        return (int)(sum / n);
    }
};

// Pin arrays defined here (header-only, used in .cpp via extern or inline)
inline const uint8_t MicromouseIR::EMIT_PINS[IR_COUNT] = {
    EMIT_LF, EMIT_L45, EMIT_R45, EMIT_RF
};
inline const uint8_t MicromouseIR::RX_PINS[IR_COUNT] = {
    RX_LF, RX_L45, RX_R45, RX_RF
};
