// main.cpp — WS2812B LED test (minimal)
#include <Arduino.h>
#include <FastLED.h>
#include "PinConfig.h"

#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("[LED] WS2812B test start");

    FastLED.addLeds<WS2812B, WS2812_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(80);
    leds[0] = CRGB::Black;
    FastLED.show();
    Serial.println("[LED] FastLED init done");
}

void loop() {
    // Cycle through red → green → blue → white → off, 1 s each
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

    idx = (idx + 1) % 5;
    delay(1000);
}
