// test/ir-test.cpp — 4-sensor IR diagnostic, Serial only
// Order: L, LF, RF, R. Prints delta = lit - ambient.

#include <Arduino.h>
#include "PinConfig.h"

struct IRPair { const char* name; uint8_t emit, rx; };

IRPair PAIRS[4] = {
    { "L ", EMIT_L,  RX_L  },
    { "LF", EMIT_LF, RX_LF },
    { "RF", EMIT_RF, RX_RF },
    { "R ", EMIT_R,  RX_R  },
};

int readDelta(const IRPair& p) {
    digitalWrite(p.emit, LOW);
    delayMicroseconds(500);
    int amb = analogRead(p.rx);
    digitalWrite(p.emit, HIGH);
    delayMicroseconds(500);
    int lit = analogRead(p.rx);
    digitalWrite(p.emit, LOW);
    return max(0, amb - lit);  // photodiode pulls LOW when lit → invert
}

void setup() {
    Serial.begin(115200);
    delay(500);
    for (auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT);
        digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }
    analogReadResolution(12);
    Serial.println("[IR] L LF RF R");
}

void loop() {
    int d[4];
    for (int i = 0; i < 4; i++) d[i] = readDelta(PAIRS[i]);
    Serial.printf("L:%4d LF:%4d RF:%4d R:%4d\n", d[0], d[1], d[2], d[3]);
    delay(100);
}
