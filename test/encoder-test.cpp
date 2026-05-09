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

void IRAM_ATTR MicromouseEncoder::handleInterrupt() {
    count++;
}

void IRAM_ATTR leftISR()  { leftEnc.handleInterrupt(); }
void IRAM_ATTR rightISR() { rightEnc.handleInterrupt(); }

void setup() {
    Serial.begin(115200);
    delay(1500);

    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, CHANGE);

    Serial.println("[ENC] ready — spin wheels by hand, watch counts");
    Serial.printf("[ENC] L=GPIO%d  R=GPIO%d\n", ENC_L_A, ENC_R_A);
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
