// test/motor-pid-straight.cpp
//
// STEP 1 — verify motors physically spin at full power, encoders count.
//          If ticks=0 after STEP 1, stop here and fix wiring/DRV before PID.
//
// STEP 2 — PID straight-line drive: both motors track a shared tick target.
//          Left encoder is the reference. Right motor corrects to match left.
//          Press BUTTON_1 to trigger one cell (180 mm) move.
//
// Serial output every 20 ms during move:
//   [PID] tL=xxx tR=xxx err=xx pwmL=xxx pwmR=xxx
//
// Tune:
//   DRIVE_PWM   — base speed. Start at 600, lower if it overshoots badly.
//   Kp          — raise until straight, back off if it oscillates side-to-side.
//   No Ki/Kd yet — add only after Kp gives stable straight line.

#include <Arduino.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── Tuning knobs ─────────────────────────────────────────────────────────────
#define DRIVE_PWM       700    // base PWM — high enough to guarantee motion
#define Kp              2.0f   // proportional gain for L/R tick error
#define CELL_MM         180.0f
#define MM_PER_TICK     ((float)(M_PI * WHEEL_DIAMETER) / TICKS_PER_REV)
#define TICKS_PER_CELL  ((long)(CELL_MM / MM_PER_TICK))
#define RAMP_DOWN_AT    0.80f  // fraction of target → drop to DRIVE_PWM/2
#define TIMEOUT_MS      6000   // safety cutoff

// ── Objects ───────────────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "LEFT");
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "RIGHT");
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B, "LEFT");
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B, "RIGHT");

void IRAM_ATTR leftISR()  { leftEnc.handleInterrupt();  }
void IRAM_ATTR rightISR() { rightEnc.handleInterrupt(); }

// ── Helpers ───────────────────────────────────────────────────────────────────
void stopMotors() {
    leftMotor.brake();
    rightMotor.brake();
    delay(80);
    leftMotor.coast();
    rightMotor.coast();
}

// ── STEP 1: spin-up verify ────────────────────────────────────────────────────
// Runs BOTH motors at full PWM for 400 ms.
// Prints ticks. If either is 0 → encoder or motor wiring problem.
bool verifyMotorsMove() {
    Serial.println("\n[VERIFY] Full-power spin 400ms — if ticks=0 check wiring");
    leftEnc.reset();
    rightEnc.reset();

    leftMotor.drive(1023);
    rightMotor.drive(1023);
    delay(400);
    stopMotors();

    long tL = leftEnc.getTicks();
    long tR = rightEnc.getTicks();
    Serial.printf("[VERIFY] L=%ld ticks  R=%ld ticks\n", tL, tR);

    bool ok = true;
    if (tL == 0) { Serial.println("[VERIFY] FAIL LEFT  — 0 ticks: check ENC_L wiring or motor"); ok = false; }
    if (tR == 0) { Serial.println("[VERIFY] FAIL RIGHT — 0 ticks: check ENC_R wiring or motor"); ok = false; }
    if (tL < 0)  { Serial.println("[VERIFY] WARN LEFT  — negative ticks: A/B wires swapped"); }
    if (tR < 0)  { Serial.println("[VERIFY] WARN RIGHT — negative ticks: A/B wires swapped"); }
    if (ok)      { Serial.println("[VERIFY] PASS — both motors spinning, encoders counting"); }
    return ok;
}

// ── STEP 2: PID straight-line move ───────────────────────────────────────────
// Left motor runs at base PWM as reference speed.
// Right motor corrects: if right is behind left → add correction, and vice versa.
// Both stop when left encoder reaches TICKS_PER_CELL.
void moveOneCell() {
    leftEnc.reset();
    rightEnc.reset();

    const long rampAt = (long)(TICKS_PER_CELL * RAMP_DOWN_AT);
    bool ramped = false;
    int  baseL  = DRIVE_PWM;
    int  baseR  = DRIVE_PWM;

    Serial.printf("\n[MOVE] target=%ld ticks (%.1fmm)  Kp=%.1f  base=%d\n",
                  TICKS_PER_CELL, (float)TICKS_PER_CELL * MM_PER_TICK, Kp, DRIVE_PWM);

    leftMotor.drive(baseL);
    rightMotor.drive(baseR);

    unsigned long startMs = millis();
    unsigned long lastLog = 0;

    while (true) {
        long tL = leftEnc.getTicks();
        long tR = rightEnc.getTicks();

        // Ramp-down at 80%
        if (!ramped && tL >= rampAt) {
            ramped = true;
            baseL  = DRIVE_PWM / 2;
            baseR  = DRIVE_PWM / 2;
            Serial.printf("[MOVE] ramp-down at tL=%ld\n", tL);
        }

        // PID: error = left ticks - right ticks
        // If error > 0 → left ahead → slow left OR speed up right
        // We correct right only (keeps left as stable reference)
        float err      = (float)(tL - tR);
        int   correction = (int)(Kp * err);
        int   pwmL     = baseL;
        int   pwmR     = constrain(baseR + correction, 0, 1023);

        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        // Log every 20 ms
        if (millis() - lastLog >= 20) {
            lastLog = millis();
            Serial.printf("[PID] tL=%4ld tR=%4ld err=%+.0f pwmL=%3d pwmR=%3d\n",
                          tL, tR, err, pwmL, pwmR);
        }

        // Done — left hit target (right should be close due to correction)
        if (tL >= TICKS_PER_CELL) {
            stopMotors();
            tL = leftEnc.getTicks();
            tR = rightEnc.getTicks();
            Serial.printf("[MOVE] done — L=%ld (%.1fmm)  R=%ld (%.1fmm)  drift=%ld ticks  t=%lums\n",
                          tL, tL * MM_PER_TICK,
                          tR, tR * MM_PER_TICK,
                          tL - tR,
                          millis() - startMs);
            if (abs(tL - tR) > 10)
                Serial.println("[MOVE] WARN: L/R drift > 10 ticks — raise Kp or check wheel");
            else
                Serial.println("[MOVE] PASS: straight line OK");
            return;
        }

        // Timeout
        if (millis() - startMs > TIMEOUT_MS) {
            stopMotors();
            Serial.printf("[MOVE] TIMEOUT — L=%ld R=%ld target=%ld\n",
                          leftEnc.getTicks(), rightEnc.getTicks(), TICKS_PER_CELL);
            Serial.println("[MOVE] FAIL — raise DRIVE_PWM or check motor power");
            return;
        }
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n[INIT] motor-pid-straight test");
    Serial.printf("[INIT] WHEEL_DIAMETER=%.0fmm  TICKS_PER_REV=%.0f\n",
                  (float)WHEEL_DIAMETER, (float)TICKS_PER_REV);
    Serial.printf("[INIT] MM_PER_TICK=%.4f  TICKS_PER_CELL=%ld  CELL_MM=%.0f\n",
                  MM_PER_TICK, TICKS_PER_CELL, CELL_MM);
    Serial.println("[INIT] !!! LIFT ROBOT OFF GROUND FOR VERIFY STEP !!!\n");

    pinMode(BUTTON_1, INPUT_PULLUP);

    // DRV8833 wake
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);
    Serial.println("[INIT] DRV8833 awake");

    leftMotor.begin();
    rightMotor.begin();

    // Attach encoder interrupts directly (same pattern as working motor-and-enc test)
    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, RISING);
    Serial.println("[INIT] encoders attached");

    delay(500);

    // STEP 1 — verify motors spin and encoders count
    // Robot should be lifted off ground for this
    bool ok = verifyMotorsMove();
    if (!ok) {
        Serial.println("\n[INIT] Fix wiring issues before proceeding.");
        Serial.println("[INIT] Halting. Press reset to retry.");
        while (true) delay(1000);
    }

    delay(1000);
    Serial.println("\n[INIT] STEP 1 passed.");
    Serial.println("[INIT] Place robot on ground, press BUTTON_1 to drive one cell.\n");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    static bool lastBtn = HIGH;
    bool btn = digitalRead(BUTTON_1);

    if (lastBtn == HIGH && btn == LOW) {
        delay(20);  // debounce
        Serial.println("[BTN] pressed → moveOneCell()");
        delay(500); // 0.5s to let user release and stand back
        moveOneCell();
        Serial.println("[BTN] ready — press again to move another cell\n");
    }
    lastBtn = btn;
}
