// test/encoder-test.cpp
//
// Bare encoder diagnostic. No motors, no BLE.
// Prints tick counts every 500ms to Serial (115200).
//
// Serial commands:
//   r  → reset both encoders
//   ?  → one-shot status print

#include <Arduino.h>
#include "PinConfig.h"
#include "MicromouseEncoder.h"

MicromouseEncoder leftEnc (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc(ENC_R_A, ENC_R_B);

void setup() {
    Serial.begin(115200);
    delay(1500);

    leftEnc.begin();
    rightEnc.begin();

    Serial.println("[ENC] ready — spin wheels by hand, watch counts (PCNT quadrature)");
    Serial.printf("[ENC] L=GPIO%d/%d  R=GPIO%d/%d\n", ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B);
}

void loop() {
    static unsigned long lastPrint = 0;

    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r') {
            leftEnc.reset(); rightEnc.reset();
            Serial.println("[ENC] reset");
        } else if (c == '?') {
            Serial.printf("[ENC] L=%ld  R=%ld\n",
                          leftEnc.getTicks(), rightEnc.getTicks());
        }
    }

    if (millis() - lastPrint >= 500) {
        lastPrint = millis();
        long tL = leftEnc.getTicks();
        long tR = rightEnc.getTicks();
        Serial.printf("L=%6ld  R=%6ld\n", tL, tR);
    }
}
