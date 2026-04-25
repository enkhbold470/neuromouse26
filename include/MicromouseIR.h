// =============================================================================
// MicromouseIR.h
// Manages the 6-sensor IR array: Left-Front, Left-45, Left-Side,
//                                 Right-Side, Right-45, Right-Front
//
// Hardware:
//   Each sensor has a dedicated emitter LED and a photodiode/phototransistor
//   receiver. The emitter is driven HIGH to emit IR, then the analog value on
//   the receiver pin is read. We subtract an ambient reading (emitter OFF) to
//   cancel background IR noise — called "differential reading".
//
// Usage:
//   ir.begin();          // set pin modes
//   ir.update();         // take one full differential reading cycle (~6 pairs)
//   int v = ir.get(IR_FRONT_LEFT);   // raw differential ADC value (0–4095)
//   bool w = ir.wallLeft();          // true if a wall is detected on the left
// =============================================================================
#pragma once
#include <Arduino.h>
#include "PinConfig.h"

// --------------------------------------------------------------------------
// Sensor index constants — use these instead of raw numbers
// --------------------------------------------------------------------------
enum IRSensor : uint8_t {
    IR_LEFT_FRONT  = 0,
    IR_LEFT_45     = 1,
    IR_LEFT_SIDE   = 2,
    IR_RIGHT_SIDE  = 3,
    IR_RIGHT_45    = 4,
    IR_RIGHT_FRONT = 5,
    IR_COUNT       = 6
};

// --------------------------------------------------------------------------
// Wall-detection thresholds (ADC counts, 12-bit = 0–4095)
// These MUST be calibrated on your actual maze walls.
// Increase if phantom walls appear; decrease if real walls are missed.
// --------------------------------------------------------------------------
#define IR_THRESH_FRONT   800   // LF / RF front sensors
#define IR_THRESH_DIAG    600   // L45 / R45 diagonal sensors
#define IR_THRESH_SIDE    500   // L / R side sensors

// --------------------------------------------------------------------------
// MicromouseIR class
// --------------------------------------------------------------------------
class MicromouseIR {
public:
    // Raw differential ADC readings (emitter ON minus emitter OFF)
    int raw[IR_COUNT];

    // --------------------------------------------------------------------------
    // begin() — configure all emitter and receiver pins
    // --------------------------------------------------------------------------
    void begin() {
        Serial.println(F("[IR] begin() — configuring emitter pins as OUTPUT, receiver pins as INPUT"));

        // Emitter pins: digital output
        pinMode(EMIT_LF,  OUTPUT); digitalWrite(EMIT_LF,  LOW);
        pinMode(EMIT_L45, OUTPUT); digitalWrite(EMIT_L45, LOW);
        pinMode(EMIT_L,   OUTPUT); digitalWrite(EMIT_L,   LOW);
        pinMode(EMIT_R,   OUTPUT); digitalWrite(EMIT_R,   LOW);
        pinMode(EMIT_R45, OUTPUT); digitalWrite(EMIT_R45, LOW);
        pinMode(EMIT_RF,  OUTPUT); digitalWrite(EMIT_RF,  LOW);

        // Receiver pins: analog input (no pull-up — they are driven by phototransistor)
        pinMode(RX_LF,  INPUT);
        pinMode(RX_L45, INPUT);
        pinMode(RX_L,   INPUT);
        pinMode(RX_R,   INPUT);
        pinMode(RX_R45, INPUT);
        pinMode(RX_RF,  INPUT);

        // Zero readings
        for (int i = 0; i < IR_COUNT; i++) raw[i] = 0;

        Serial.println(F("[IR] begin() done"));
    }

    // --------------------------------------------------------------------------
    // update() — fire each emitter, read receiver, subtract ambient
    // Runs in ~12 analogRead cycles (6 pairs). Each analogRead ≈ ~20–50 µs
    // on ESP32, so total ≈ 0.2–0.6 ms.
    // --------------------------------------------------------------------------
    void update() {
        Serial.println(F("[IR] update() — starting differential reading cycle"));

        // Sensor pairing: {emitter pin, receiver pin, sensor index}
        static const uint8_t EMIT_PINS[IR_COUNT] = {
            EMIT_LF, EMIT_L45, EMIT_L, EMIT_R, EMIT_R45, EMIT_RF
        };
        static const uint8_t RX_PINS[IR_COUNT] = {
            RX_LF, RX_L45, RX_L, RX_R, RX_R45, RX_RF
        };
        static const char* NAMES[IR_COUNT] = {
            "LF", "L45", "L", "R", "R45", "RF"
        };

        for (int i = 0; i < IR_COUNT; i++) {
            // 1. Read ambient (emitter OFF)
            int ambient = analogRead(RX_PINS[i]);

            // 2. Fire emitter
            digitalWrite(EMIT_PINS[i], HIGH);
            delayMicroseconds(50);  // allow photodetector to settle

            // 3. Read lit value
            int lit = analogRead(RX_PINS[i]);

            // 4. Turn emitter off immediately
            digitalWrite(EMIT_PINS[i], LOW);

            // 5. Differential value (clamp at 0)
            raw[i] = lit - ambient;
            if (raw[i] < 0) raw[i] = 0;

            Serial.printf("[IR] %s: ambient=%4d  lit=%4d  diff=%4d\n",
                          NAMES[i], ambient, lit, raw[i]);
        }

        Serial.println(F("[IR] update() done"));
    }

    // --------------------------------------------------------------------------
    // get() — return raw differential value for a given sensor
    // --------------------------------------------------------------------------
    int get(IRSensor s) const {
        return raw[(int)s];
    }

    // --------------------------------------------------------------------------
    // Wall detection helpers — return true if a wall is detected
    // --------------------------------------------------------------------------
    bool wallFrontLeft()  const { return raw[IR_LEFT_FRONT]  > IR_THRESH_FRONT; }
    bool wallFrontRight() const { return raw[IR_RIGHT_FRONT] > IR_THRESH_FRONT; }
    bool wallLeft()       const { return raw[IR_LEFT_SIDE]   > IR_THRESH_SIDE;  }
    bool wallRight()      const { return raw[IR_RIGHT_SIDE]  > IR_THRESH_SIDE;  }
    bool wallDiagLeft()   const { return raw[IR_LEFT_45]     > IR_THRESH_DIAG;  }
    bool wallDiagRight()  const { return raw[IR_RIGHT_45]    > IR_THRESH_DIAG;  }

    // wallFront() — true if BOTH front sensors see a wall
    // Using AND prevents a partial-angle approach from triggering a false front wall.
    bool wallFront() const {
        return wallFrontLeft() && wallFrontRight();
    }

    // --------------------------------------------------------------------------
    // printStatus() — human-readable wall status for all sensors
    // --------------------------------------------------------------------------
    void printStatus() const {
        Serial.println(F("[IR] === Sensor status ==="));
        Serial.printf( "[IR]   Left-Front  (LF)  raw=%4d  wall=%s\n",
                       raw[IR_LEFT_FRONT],  wallFrontLeft()  ? "YES" : "no");
        Serial.printf( "[IR]   Left-45     (L45) raw=%4d  wall=%s\n",
                       raw[IR_LEFT_45],     wallDiagLeft()   ? "YES" : "no");
        Serial.printf( "[IR]   Left-Side   (L)   raw=%4d  wall=%s\n",
                       raw[IR_LEFT_SIDE],   wallLeft()       ? "YES" : "no");
        Serial.printf( "[IR]   Right-Side  (R)   raw=%4d  wall=%s\n",
                       raw[IR_RIGHT_SIDE],  wallRight()      ? "YES" : "no");
        Serial.printf( "[IR]   Right-45    (R45) raw=%4d  wall=%s\n",
                       raw[IR_RIGHT_45],    wallDiagRight()  ? "YES" : "no");
        Serial.printf( "[IR]   Right-Front (RF)  raw=%4d  wall=%s\n",
                       raw[IR_RIGHT_FRONT], wallFrontRight() ? "YES" : "no");
        Serial.printf( "[IR]   Combined: FRONT=%s  LEFT=%s  RIGHT=%s\n",
                       wallFront() ? "WALL" : "open",
                       wallLeft()  ? "WALL" : "open",
                       wallRight() ? "WALL" : "open");
        Serial.println(F("[IR] ========================="));
    }
};
