// main.cpp — Battery voltage test (minimal)
// Divider: 100k (VCC) + 50k (GND) → nominal ratio 1/3
// Calibrated multiplier 3.1197 (nominal 3.0 * 1.0399 correction factor)
// Measured 7.82V on multimeter vs 7.52V raw → factor = 7.82/7.52
#include <Arduino.h>
#include "PinConfig.h"

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n[BATT-TEST] Battery voltage monitor");
    Serial.printf("[BATT-TEST] GPIO%d  divider=1/3  (100k+50k)\n\n", BAT_V_SENSE);
}

void loop() {
    int   raw  = analogRead(BAT_V_SENSE);
    float vADC = (raw / 4095.0f) * 3.3f;
    float vBat = vADC * BAT_VDIV_MULT;

    Serial.printf("raw=%4d  V_adc=%.3fV  V_bat=%.2fV  %s\n",
                  raw, vADC, vBat,
                  vBat < 6.8f ? "WARN: LOW" :
                  vBat < 7.4f ? "OK: discharging" :
                  vBat < 8.4f ? "OK: good" :
                                "OK: full");
    delay(500);
}
