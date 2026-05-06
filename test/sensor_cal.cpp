#include <Arduino.h>
#include "../include/PinConfig.h"

// Physics: We must subtract ambient light to see the "true" wall signal.
int getTrueSignal(int rxPin, int emitPin) {
    int ambient = analogRead(rxPin);
    digitalWrite(emitPin, HIGH);
    delayMicroseconds(50); // Physics: LED rise time
    int lit = analogRead(rxPin);
    digitalWrite(emitPin, LOW);
    return max(0, lit - ambient);
}

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for USB
    
    // Init IR hardware
    pinMode(EMIT_LF, OUTPUT);
    pinMode(EMIT_L45, OUTPUT);
    pinMode(EMIT_R45, OUTPUT);
    pinMode(EMIT_RF, OUTPUT);
    
    Serial.println("\n--- MICROMOUSE SENSOR CALIBRATION TOOL ---");
    Serial.println("Physics Law: Values are Differential (Lit - Ambient)");
    Serial.println("1. Place robot in CENTER of cell for L45/R45 centers.");
    Serial.println("2. Place robot nose 1cm from wall for LF/RF thresholds.");
    Serial.println("-------------------------------------------\n");
    delay(2000);
}

void loop() {
    int lf  = getTrueSignal(RX_LF,  EMIT_LF);
    int l45 = getTrueSignal(RX_L45, EMIT_L45);
    int r45 = getTrueSignal(RX_R45, EMIT_R45);
    int rf  = getTrueSignal(RX_RF,  EMIT_RF);

    // Print values in a format easy to copy into PinConfig.h
    Serial.printf("LF:%-4d  L45:%-4d  R45:%-4d  RF:%-4d\n", lf, l45, r45, rf);
    
    // Physics Check: If any value is < 10, the sensor might be unplugged or LED is dead
    if (lf < 10)  Serial.print("[WARN: LF LOW] ");
    if (l45 < 10) Serial.print("[WARN: L45 LOW] ");
    if (r45 < 10) Serial.print("[WARN: R45 LOW] ");
    if (rf < 10)  Serial.print("[WARN: RF LOW] ");
    
    if (lf < 10 || l45 < 10 || r45 < 10 || rf < 10) Serial.println();

    delay(200);
}
