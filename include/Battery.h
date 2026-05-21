// include/Battery.h — battery voltage + state-of-charge.
//
// 2S LiPo via resistor divider on BAT_V_SENSE (PinConfig.h). 8.4 V = 100 %,
// 6.4 V = 0 %, linear in between.

#ifndef MM26_BATTERY_H
#define MM26_BATTERY_H

#include <Arduino.h>
#include "PinConfig.h"

static float readVbat() {
    int raw = analogRead(BAT_V_SENSE);
    return (raw / 4095.0f) * 3.3f * BAT_VDIV_MULT;
}

static int batPct() {
    float v = readVbat();
    if (v < 6.4f) return 0;
    if (v > 8.4f) return 100;
    return (int)((v - 6.4f) * 50.0f + 0.5f);  // (v − 6.4)/2.0 × 100
}

#endif
