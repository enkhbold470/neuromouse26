# Motor + Encoder Complete Test

## Pin fixes applied (in PinConfig.h)

| Issue | Fix |
|-------|-----|
| Left encoder counted negative on FWD | Swapped `ENC_L_A` ↔ `ENC_L_B` (GPIO14 ↔ GPIO21) |
| Right motor FWD = physical reverse | Swapped `MOTOR_R_IN3` ↔ `MOTOR_R_IN4` (GPIO17 ↔ GPIO18) |

## main.cpp

```cpp
// main.cpp — Motor + Encoder complete test (minimal)
#include <Arduino.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "LEFT");
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "RIGHT");
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B, "LEFT");
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B, "RIGHT");

void IRAM_ATTR leftISR()  { leftEnc.handleInterrupt();  }
void IRAM_ATTR rightISR() { rightEnc.handleInterrupt(); }

void runTest(MicromouseMotor& mot, MicromouseEncoder& enc,
             const char* side, int pwm, unsigned long ms) {
    enc.reset();
    Serial.printf("[TEST] %s  pwm=%4d  dur=%lums ... ", side, pwm, ms);
    mot.drive(pwm);
    delay(ms);
    mot.brake(); delay(60); mot.coast();
    long ticks = enc.getTicks();
    Serial.printf("ticks=%ld  %s\n", ticks,
                  (ticks == 0) ? "WARN: no ticks — check wiring" :
                  (ticks < 0)  ? "NOTE: negative (reverse counted as fwd?)" : "OK");
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n[MOTOR-TEST] ==============================");
    Serial.println("[MOTOR-TEST] Motor + Encoder complete test");
    Serial.println("[MOTOR-TEST] ==============================");
    Serial.println("[MOTOR-TEST] !!! LIFT ROBOT OFF GROUND !!!\n");

    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);
    Serial.println("[INIT] DRV8833 awake");

    leftMotor.begin();
    rightMotor.begin();

    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, RISING);
    Serial.println("[INIT] Encoders attached\n");

    // LEFT motor tests
    Serial.println("[TEST] === LEFT MOTOR ===");
    runTest(leftMotor, leftEnc, "LEFT  FWD  30%", +307,  500);  delay(300);
    runTest(leftMotor, leftEnc, "LEFT  FWD  60%", +614,  500);  delay(300);
    runTest(leftMotor, leftEnc, "LEFT  FWD 100%", +1023, 500);  delay(300);
    runTest(leftMotor, leftEnc, "LEFT  REV  50%", -512,  500);  delay(500);

    // RIGHT motor tests
    Serial.println("[TEST] === RIGHT MOTOR ===");
    runTest(rightMotor, rightEnc, "RIGHT FWD  30%", +307,  500);  delay(300);
    runTest(rightMotor, rightEnc, "RIGHT FWD  60%", +614,  500);  delay(300);
    runTest(rightMotor, rightEnc, "RIGHT FWD 100%", +1023, 500);  delay(300);
    runTest(rightMotor, rightEnc, "RIGHT REV  50%", -512,  500);  delay(500);

    // Both together
    Serial.println("[TEST] === BOTH MOTORS TOGETHER ===");
    leftEnc.reset(); rightEnc.reset();
    Serial.print("[TEST] BOTH FWD 50% for 1000ms ... ");
    leftMotor.drive(512); rightMotor.drive(512);
    delay(1000);
    leftMotor.brake(); rightMotor.brake(); delay(60);
    leftMotor.coast(); rightMotor.coast();
    Serial.printf("L=%ld  R=%ld ticks\n", leftEnc.getTicks(), rightEnc.getTicks());
    delay(500);

    // Live encoder stream
    Serial.println("\n[TEST] === LIVE ENCODER STREAM ===");
    Serial.println("[TEST] Spin wheels by hand — watch ticks change");
    Serial.println("[TEST] Format: L_ticks  R_ticks  (printed every 200ms)\n");
    leftEnc.reset(); rightEnc.reset();
}

void loop() {
    static unsigned long last = 0;
    if (millis() - last >= 200) {
        last = millis();
        Serial.printf("L=%6ld  R=%6ld\n", leftEnc.getTicks(), rightEnc.getTicks());
    }
}
```

## Test sequence

| Phase | PWM | Duration | Pass condition |
|-------|-----|----------|----------------|
| LEFT FWD 30% | 307 | 500ms | ticks > 0, increasing |
| LEFT FWD 60% | 614 | 500ms | more ticks than 30% |
| LEFT FWD 100% | 1023 | 500ms | most ticks |
| LEFT REV 50% | -512 | 500ms | ticks negative |
| RIGHT (same) | — | — | mirror of left |
| BOTH FWD 50% | 512 | 1000ms | L and R ticks similar |
| Live stream | — | — | hand-spin changes count |

## Notes
- Ticks = 0 → motor or encoder wiring dead
- Ticks negative on FWD → A/B still swapped, recheck
- L vs R ticks very different at same PWM → motor mismatch, adjust PID gains later
- `MicromouseEncoder.cpp` required in `src/` — `handleInterrupt()` must not be inline in header (Xtensa IRAM literal pool issue)
