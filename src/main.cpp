// main.cpp — IR sensor array test (minimal)
// Hardware: SFH4545 emitter + TEFT4300 phototransistor receiver, 6 pairs
// Method: differential reading — emitter OFF (ambient) then ON (lit), subtract
// Only LF pair soldered initially — unsoldered sensors will read 0 diff, that is expected
#include <Arduino.h>
#include "PinConfig.h"

// Sensor table — name, emitter pin, receiver pin
struct IRPair {
    const char* name;
    uint8_t     emitPin;
    uint8_t     rxPin;
};

static const IRPair SENSORS[] = {
    { "LF  (Left-Front) ", EMIT_LF,  RX_LF  },
    { "L45 (Left-45)    ", EMIT_L45, RX_L45 },
    { "L   (Left-Side)  ", EMIT_L,   RX_L   },
    { "R   (Right-Side) ", EMIT_R,   RX_R   },
    { "R45 (Right-45)   ", EMIT_R45, RX_R45 },
    { "RF  (Right-Front)", EMIT_RF,  RX_RF  },
};
static const int N_SENSORS = sizeof(SENSORS) / sizeof(SENSORS[0]);

// Read one differential value for a sensor
int readDiff(const IRPair& s) {
    // Ambient (emitter off)
    int ambient = analogRead(s.rxPin);

    // Fire emitter
    digitalWrite(s.emitPin, HIGH);
    delayMicroseconds(100);  // TEFT4300 rise time ~100µs

    // Lit reading
    int lit = analogRead(s.rxPin);

    // Emitter off
    digitalWrite(s.emitPin, LOW);

    int diff = lit - ambient;
    return diff < 0 ? 0 : diff;
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    // Emitter pins: output, default off
    for (int i = 0; i < N_SENSORS; i++) {
        pinMode(SENSORS[i].emitPin, OUTPUT);
        digitalWrite(SENSORS[i].emitPin, LOW);
    }
    // Receiver pins: analog input
    for (int i = 0; i < N_SENSORS; i++) {
        pinMode(SENSORS[i].rxPin, INPUT);
    }

    Serial.println("\n[IR-TEST] ==============================");
    Serial.println("[IR-TEST] IR sensor array test");
    Serial.println("[IR-TEST] SFH4545 emitter + TEFT4300 receiver");
    Serial.println("[IR-TEST] Unsoldered sensors will read ~0 — expected");
    Serial.println("[IR-TEST] ==============================\n");
    Serial.println("[IR-TEST] Point sensors at a white wall ~5cm away for best result\n");
}

void loop() {
    Serial.println("[IR] --------");
    for (int i = 0; i < N_SENSORS; i++) {
        int diff = readDiff(SENSORS[i]);

        // Simple wall threshold indicator
        const char* status;
        if      (diff == 0)    status = "no signal (not soldered?)";
        else if (diff < 200)   status = "open / far";
        else if (diff < 600)   status = "object nearby";
        else                   status = "WALL detected";

        Serial.printf("[IR] %s  diff=%4d  %s\n",
                      SENSORS[i].name, diff, status);
    }
    delay(200);
}
