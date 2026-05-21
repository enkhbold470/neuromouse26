// test/goal-blink.cpp — WS2812B "goal reached" celebration pattern.
//
// Standalone test: validates the blink behavior to drop into main.cpp's GOAL
// state. 8 LEDs on WS2812_DATA (GPIO3). Non-blocking pattern so the same
// state machine can be inlined into the main loop without delay()s.
//
// Pattern (4 s total, then repeats):
//   0.0–2.0 s : 5x green flash (200 ms ON, 200 ms OFF)
//   2.0–3.6 s : rainbow chase (8 steps × 200 ms)
//   3.6–4.0 s : OFF
//
// Flash: pio run -e goal-blink -t upload
#include <Arduino.h>
#include <FastLED.h>
#include "PinConfig.h"

// Third-party ESP32-S3 module: single onboard RGB LED on `rgb_pin` (GPIO48,
// confirmed 2026-05-21).
constexpr int NUM_LEDS    = 1;
constexpr uint8_t BRIGHT  = 20;

static CRGB leds[NUM_LEDS];

static void allOff() {
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB::Black;
}

static void allColor(const CRGB& c) {
    for (int i = 0; i < NUM_LEDS; i++) leds[i] = c;
}

// One step of the celebration. Returns true while the pattern is still running,
// false once it has completed one full cycle (so the main code can decide to
// stop or loop again).
static bool goalBlinkStep(uint32_t elapsedMs) {
    constexpr uint32_t FLASH_PERIOD_MS = 400;   // 200 on + 200 off
    constexpr uint32_t FLASH_PHASE_MS  = 2000;  // 5 flashes
    constexpr uint32_t RAIN_STEP_MS    = 200;
    constexpr uint32_t RAIN_PHASE_MS   = 1600;  // 8 steps
    constexpr uint32_t TOTAL_MS        = FLASH_PHASE_MS + RAIN_PHASE_MS + 400;

    if (elapsedMs < FLASH_PHASE_MS) {
        bool on = (elapsedMs % FLASH_PERIOD_MS) < (FLASH_PERIOD_MS / 2);
        allColor(on ? CRGB::Green : CRGB::Black);
    } else if (elapsedMs < FLASH_PHASE_MS + RAIN_PHASE_MS) {
        uint32_t t = elapsedMs - FLASH_PHASE_MS;
        uint8_t  step = t / RAIN_STEP_MS;
        for (int i = 0; i < NUM_LEDS; i++) {
            uint8_t hue = (step * 32 + i * 32) & 0xFF;
            leds[i] = CHSV(hue, 255, 255);
        }
    } else {
        allOff();
    }
    FastLED.show();
    return elapsedMs < TOTAL_MS;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[GOAL-BLINK] start");

    FastLED.addLeds<WS2812B, rgb_pin, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHT);
    allOff();
    FastLED.show();
}

void loop() {
    static uint32_t t0 = millis();
    uint32_t elapsed = millis() - t0;

    if (!goalBlinkStep(elapsed)) {
        Serial.println("[GOAL-BLINK] cycle done, restarting in 1 s");
        delay(1000);
        t0 = millis();
    }
    delay(20);
}
