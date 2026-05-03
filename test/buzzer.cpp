// main.cpp — Button + Buzzer test (minimal)
// SMD5020 magnetic buzzer @ 4kHz resonant freq, 10% duty (quiet)
// LEDC ch4, 8-bit res → duty 25/255 ≈ 10%
// Press BUTTON_1 → 1 short beep
// Hold BUTTON_1 3s → long tone
#include <Arduino.h>
#include "PinConfig.h"

#define BUZZ_CH     4
#define BUZZ_FREQ   4000
#define BUZZ_RES    8
#define BUZZ_DUTY   100    // 25/255 ≈ 10% volume

void beep(int ms) {
    ledcWrite(BUZZ_CH, BUZZ_DUTY);
    delay(ms);
    ledcWrite(BUZZ_CH, 0);
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(BUTTON_1, INPUT_PULLUP);

    ledcSetup(BUZZ_CH, BUZZ_FREQ, BUZZ_RES);
    ledcAttachPin(BUZZER_PIN, BUZZ_CH);
    ledcWrite(BUZZ_CH, 0);

    Serial.println("\n[BTN-BUZZ] Button + Buzzer test");
    Serial.printf("[BTN-BUZZ] Button=GPIO%d  Buzzer=GPIO%d  freq=%dHz  duty=%d/255\n",
                  BUTTON_1, BUZZER_PIN, BUZZ_FREQ, BUZZ_DUTY);
    Serial.println("[BTN-BUZZ] Short press → 1 beep");
    Serial.println("[BTN-BUZZ] Hold 3s     → long tone\n");

    // Startup beep — confirms buzzer alive
    beep(100);
}

void loop() {
    static bool     lastBtn    = HIGH;
    static unsigned long pressStart = 0;
    static bool     longFired  = false;

    bool btn = digitalRead(BUTTON_1);

    // Falling edge — button down
    if (lastBtn == HIGH && btn == LOW) {
        pressStart = millis();
        longFired  = false;
        Serial.println("[BTN] pressed");
    }

    // Held — check for long press threshold
    if (btn == LOW && !longFired) {
        unsigned long held = millis() - pressStart;
        if (held >= 3000) {
            longFired = true;
            Serial.printf("[BTN] long hold (%lums) → long tone\n", held);
            beep(800);
        }
    }

    // Rising edge — button released
    if (lastBtn == LOW && btn == HIGH) {
        unsigned long held = millis() - pressStart;
        Serial.printf("[BTN] released  held=%lums\n", held);
        if (!longFired) {
            Serial.println("[BTN] short press → beep");
            beep(100);
        }
    }

    lastBtn = btn;
    delay(10);
}
