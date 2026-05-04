# Button + Buzzer Test

## Hardware notes
- Buzzer: SMD5020 magnetic, 4kHz resonant frequency
- Circuit: GPIO40 → 330Ω → buzzer pin 3, pin 1=5V, pin 2=GND
- Drive: LEDC PWM at 4kHz, 10% duty (25/255) — NEVER use duty=255 (DC = coil burn)
- 3.3V signal through 330Ω is sufficient at 10% duty

## main.cpp

```cpp
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
#define BUZZ_DUTY   25    // 25/255 ≈ 10% volume (quiet)

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

    beep(100);
}

void loop() {
    static bool          lastBtn   = HIGH;
    static unsigned long pressStart = 0;
    static bool          longFired  = false;

    bool btn = digitalRead(BUTTON_1);

    if (lastBtn == HIGH && btn == LOW) {
        pressStart = millis();
        longFired  = false;
        Serial.println("[BTN] pressed");
    }

    if (btn == LOW && !longFired) {
        if (millis() - pressStart >= 3000) {
            longFired = true;
            Serial.printf("[BTN] long hold (%lums) → long tone\n", millis() - pressStart);
            beep(800);
        }
    }

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
```

## Pass criteria
- Boot → 1 short beep
- Short press → 1 × 100ms beep
- Hold 3s → 1 × 800ms tone at 3s mark
- Serial logs every edge + hold duration

## WARNING
**NEVER set BUZZER_DUTY to 255.** DC signal on magnetic buzzer = coil stays energized = heat = burn.
Max safe duty: 50 (20%). Normal use: 25 (10%).
