// main.cpp — LF IR pair test (SFH4545 + TEFT4300)
// Circuit behavior (collector/emitter swapped on TEFT4300):
//   No IR  → ADC low  (~100)
//   IR hit → ADC high (~2000+)
//   delta  = lit - ambient  (positive = IR reflected = wall)
#include <Arduino.h>
#include "PinConfig.h"

void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(EMIT_LF, OUTPUT);
    digitalWrite(EMIT_LF, LOW);
    pinMode(RX_LF, INPUT);

    Serial.println("\n[LF-TEST] SFH4545 + TEFT4300 LF pair test");
    Serial.printf( "[LF-TEST] emit=GPIO%d  rx=GPIO%d\n", EMIT_LF, RX_LF);
    Serial.println("[LF-TEST] Low ADC = no IR, High ADC = IR detected");
    Serial.println("[LF-TEST] Put hand/wall ~4cm in front, watch delta rise\n");
}

void loop() {
    // Ambient — emitter off
    int ambient = analogRead(RX_LF);

    // Fire emitter
    digitalWrite(EMIT_LF, HIGH);
    delayMicroseconds(100);

    // Lit
    int lit = analogRead(RX_LF);
    digitalWrite(EMIT_LF, LOW);

    int delta = lit - ambient;  // positive = IR reflected back

    Serial.printf("ambient=%4d  lit=%4d  delta=%4d  → %s\n",
                  ambient, lit, delta,
                  delta < 50   ? "no reflection / open air" :
                  delta < 300  ? "weak / far"               :
                  delta < 800  ? "object nearby"            :
                                 "WALL");
    delay(100);
}
