// WS2812B onboard status LED test — matches main.cpp (1 LED on rgb_pin).
#include <Arduino.h>
#include <FastLED.h>
#include "PinConfig.h"

static constexpr int NUM_LEDS = 1;
CRGB leds[NUM_LEDS];

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("[LED] WS2812B test start");

    FastLED.addLeds<WS2812B, rgb_pin, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(20);
    leds[0] = CRGB::Black;
    FastLED.show();
    Serial.printf("[LED] FastLED init done (GPIO %u)\n", rgb_pin);
}

void loop() {
    static const struct { CRGB colour; const char* name; } steps[] = {
        { CRGB::Red,   "RED"   },
        { CRGB::Green, "GREEN" },
        { CRGB::Blue,  "BLUE"  },
        { CRGB::White, "WHITE" },
        { CRGB::Black, "OFF"   },
    };
    static uint8_t idx = 0;

    leds[0] = steps[idx].colour;
    FastLED.show();
    Serial.printf("[LED] colour=%s\n", steps[idx].name);

    idx = (uint8_t)((idx + 1) % 5);
    delay(1000);
}
