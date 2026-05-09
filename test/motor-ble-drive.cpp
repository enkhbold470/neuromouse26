// test/motor-ble-drive.cpp
//
// Simple open-loop encoder drive. No PID, no balance — both motors same PWM.
// Stops when average tick count hits target. Drift fixed by maze walls.
//
// BLE / Serial commands:
//   1-5  → move N cells (180mm each)
//   f    → free-run until 's'
//   s    → stop
//   r    → reset encoders
//   ?    → status

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// All tuning constants in PinConfig.h — no local overrides needed.

// ── Motors + Encoders ─────────────────────────────────────────────────────────
// pivot-mouse-v1: both inverted to run forward
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, true);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, true);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);

// Always count up — direction inferred from motor command, not quadrature.
// Noise filter: ignore pulses < 200µs apart (physically impossible at any N20 RPM).
void IRAM_ATTR MicromouseEncoder::handleInterrupt() {
    uint32_t now = micros();
    static uint32_t lastL = 0, lastR = 0;
    uint32_t* last = (pinA == ENC_L_A) ? &lastL : &lastR;
    if (now - *last < 200) return;
    *last = now;
    count++;
}

void IRAM_ATTR leftISR()  { leftEnc.handleInterrupt(); }
void IRAM_ATTR rightISR() { rightEnc.handleInterrupt(); }

// Scaled right encoder: compensates hardware tick-count difference.
static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

static void bleSend(const char* fmt, ...);
static void moveCells(int n);

void encodersEnable() {
    leftEnc.reset();
    rightEnc.reset();
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, RISING);
}

void encodersDisable() {
    detachInterrupt(digitalPinToInterrupt(ENC_L_A));
    detachInterrupt(digitalPinToInterrupt(ENC_R_A));
}

void stopMotors() {
    leftMotor.coast();
    rightMotor.coast();
}

void turnRight() {
    encodersEnable();
    bleSend("[TURN] right target=%ld ticks\n", TICKS_PER_90);
    leftMotor.drive(TURN_PWM);
    rightMotor.drive(-TURN_PWM);
    unsigned long startMs = millis();
    while (abs(leftEnc.getTicks()) < TICKS_PER_90) {
        if (millis() - startMs > 2000) break;
    }
    stopMotors();
    encodersDisable();
    bleSend("[TURN] done L=%ld\n", leftEnc.getTicks());
}

void turnLeft() {
    encodersEnable();
    bleSend("[TURN] left target=%ld ticks\n", TICKS_PER_90);
    leftMotor.drive(-TURN_PWM);
    rightMotor.drive(TURN_PWM);
    unsigned long startMs = millis();
    while (abs(rTicks()) < TICKS_PER_90) {
        if (millis() - startMs > 2000) break;
    }
    stopMotors();
    encodersDisable();
    bleSend("[TURN] done R=%ld\n", rTicks());
}

// Button 1 sequence: 5f → R → 1f → R → 5f → L → 1f → L → 5f
void runSequence() {
    bleSend("[SEQ] start  5-R-1-R-5-L-1-L-5\n");
    moveCells(5); delay(250);
    turnRight();  delay(250);
    moveCells(1); delay(250);
    turnRight();  delay(250);
    moveCells(5); delay(250);
    turnLeft();   delay(250);
    moveCells(1); delay(250);
    turnLeft();   delay(250);
    moveCells(5);
    bleSend("[SEQ] complete\n");
}

// ── BLE ───────────────────────────────────────────────────────────────────────
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* pTX     = nullptr;
bool                  bleConn = false;
char                  rxCmd   = 0;

static void bleSend(const char* fmt, ...) {
    char buf[128];
    va_list args; va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
    if (!bleConn || !pTX) return;
    size_t len = strlen(buf), off = 0;
    while (off < len) {
        size_t chunk = min((size_t)20, len - off);
        pTX->setValue((uint8_t*)(buf + off), chunk);
        pTX->notify();
        off += chunk;
        delay(10);
    }
}

class BLECb : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        bleConn = true; Serial.println("[BLE] connected");
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        bleConn = false;
        NimBLEDevice::getAdvertising()->start();
        Serial.println("[BLE] disconnected, re-advertising");
    }
};
class RXCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
        std::string v = c->getValue();
        if (!v.empty()) rxCmd = v[0];
    }
};

void bleSetup() {
    NimBLEDevice::init("pivot-mouse-v1");
    NimBLEDevice::setPower(9);  // 9 dBm — v2.x takes dBm directly
    auto* srv = NimBLEDevice::createServer();
    srv->setCallbacks(new BLECb());
    auto* svc = srv->createService(NUS_SERVICE_UUID);
    pTX = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    auto* pRX = svc->createCharacteristic(NUS_RX_UUID,
                    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pRX->setCallbacks(new RXCb());
    srv->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->setName("pivot-mouse-v1");
    pAdv->addServiceUUID(NUS_SERVICE_UUID);
    pAdv->enableScanResponse(true);
    pAdv->start();
    Serial.println("[BLE] advertising as 'pivot-mouse-v1'");
}

// ── Move N cells ──────────────────────────────────────────────────────────────
// Open-loop: both motors at DRIVE_PWM, stop when avg ticks >= target.
// No balance PID — encoder 2x mismatch causes spinning when PID is active.
static void moveCells(int n) {
    long target = TICKS_PER_CELL * (long)n - COAST_COMP_TICKS;
    encodersEnable();
    bleSend("[MOVE] %d cell(s) target=%ld ticks (%.0fmm)\n",
            n, target, target * MM_PER_TICK);

    unsigned long startMs = millis();
    unsigned long lastLog = 0;

    while (true) {
        if (Serial.available()) rxCmd = Serial.read();
        if (rxCmd == 's') {
            rxCmd = 0;
            stopMotors();
            encodersDisable();
            bleSend("[STOP]\n");
            return;
        }

        long tL   = leftEnc.getTicks();
        long tR   = rTicks();
        long avg  = (tL + tR) / 2;
        long remL = target - tL;
        long remR = target - tR;
        bool lDone = (remL <= 0);
        bool rDone = (remR <= 0);

        if (lDone && rDone) {
            stopMotors();
            encodersDisable();
            tL = leftEnc.getTicks(); tR = rTicks();
            long avgF = (tL + tR) / 2;
            bleSend("[DONE] avg=%ld(%.0fmm) L=%ld R=%ld t=%lums\n",
                    avgF, avgF * MM_PER_TICK, tL, tR, millis() - startMs);
            return;
        }

        // Per-motor decel ramp: full speed → DRIVE_PWM_MIN over last DECEL_TICKS, then coast.
        // Balance P correction only while both motors still running.
        int err  = (int)(tL - tR);
        int baseL = lDone ? 0 : (remL > DECEL_TICKS ? DRIVE_PWM
                        : (int)map(remL, 0, DECEL_TICKS, DRIVE_PWM_MIN, DRIVE_PWM));
        int baseR = rDone ? 0 : (remR > DECEL_TICKS ? DRIVE_PWM
                        : (int)map(remR, 0, DECEL_TICKS, DRIVE_PWM_MIN, DRIVE_PWM));
        int pwmL, pwmR;
        if (!lDone && !rDone && remL > DECEL_TICKS && remR > DECEL_TICKS) {
            pwmL = constrain(baseL - err * BALANCE_KP, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
            pwmR = constrain(baseR + err * BALANCE_KP, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        } else {
            pwmL = baseL;
            pwmR = baseR;
        }
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (millis() - lastLog >= 150) {
            lastLog = millis();
            bleSend("avg=%4ld/%-4ld L=%4ld R=%4ld err=%+d pwm=%d/%d\n",
                    avg, target, tL, tR, err, pwmL, pwmR);
        }

        if (millis() - startMs > (unsigned long)(TIMEOUT_MS * n)) {
            stopMotors();
            encodersDisable();
            bleSend("[TIMEOUT] avg=%ld target=%ld — raise DRIVE_PWM\n", avg, target);
            return;
        }
    }
}

// ── Free-run ──────────────────────────────────────────────────────────────────
void freeRun() {
    encodersEnable();
    bleSend("[FREE] DRIVE_PWM=%d BALANCE_KP=%d — send 's' to stop\n", DRIVE_PWM, BALANCE_KP);

    unsigned long lastLog = 0;

    while (true) {
        if (Serial.available()) rxCmd = Serial.read();
        if (rxCmd == 's') {
            rxCmd = 0;
            stopMotors();
            encodersDisable();
            long tL = leftEnc.getTicks(), tR = rTicks();
            bleSend("[FREE] stopped L=%ld(%.0fmm) R=%ld(%.0fmm)\n",
                    tL, tL * MM_PER_TICK, tR, tR * MM_PER_TICK);
            return;
        }
        long tL = leftEnc.getTicks(), tR = rTicks();
        int  err = (int)(tL - tR);
        leftMotor.drive( constrain(DRIVE_PWM - err * BALANCE_KP, 200, MOTOR_PWM_MAX));
        rightMotor.drive(constrain(DRIVE_PWM + err * BALANCE_KP, 200, MOTOR_PWM_MAX));

        if (millis() - lastLog >= 200) {
            lastLog = millis();
            bleSend("L=%ld R=%ld err=%+d\n", tL, tR, err);
        }
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n[INIT] motor-ble-drive — encoder-only");
    Serial.printf("[INIT] TICKS_PER_CELL=%ld  MM_PER_TICK=%.4f\n",
                  TICKS_PER_CELL, MM_PER_TICK);
    Serial.printf("[INIT] DRIVE_PWM=%d  TICKS_PER_CELL=%ld  MM_PER_TICK=%.4f\n",
                  DRIVE_PWM, TICKS_PER_CELL, MM_PER_TICK);

    // DRV_SLEEP_PIN jumpered to VCC on this board — skip pin setup
    pinMode(BUTTON_1, INPUT_PULLUP);

    leftMotor.begin();
    rightMotor.begin();

    // Set encoder pins but DO NOT attach ISRs yet — prevents noise counts at idle
    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

    bleSetup();

    Serial.println("[INIT] ready — cmds: 1-5=cells  f=free  b=sequence  s=stop  r=reset  ?=status");
    Serial.println("[INIT] diag: A=L-fwd  Z=L-back  B=R-fwd  X=R-back  T=enc-spin-test");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // Button 1 edge → run sequence
    {
        static bool lastBtn = HIGH;
        bool curBtn = digitalRead(BUTTON_1);
        if (lastBtn == HIGH && curBtn == LOW) {
            bleSend("[BTN] sequence triggered\n");
            runSequence();
        }
        lastBtn = curBtn;
    }

    if (Serial.available()) rxCmd = Serial.read();
    if (rxCmd == 0) return;
    char cmd = rxCmd;
    rxCmd = 0;

    switch (cmd) {
        case '1': moveCells(1); break;
        case '2': moveCells(2); break;
        case '3': moveCells(3); break;
        case '4': moveCells(4); break;
        case '5': moveCells(5); break;
        case 'f': freeRun(); break;
        case 'b': runSequence(); break;
        case 's': stopMotors(); encodersDisable(); bleSend("[STOP]\n"); break;

        // ── Single-motor diagnostics ──────────────────────────────────────────
        // A/Z: left fwd/back  B/X: right fwd/back  (raw 120 PWM, 600ms)
        case 'A': bleSend("[DIAG] L fwd\n");  leftMotor.drive( 500); delay(800); leftMotor.coast();  bleSend("[DIAG] done\n"); break;
        case 'Z': bleSend("[DIAG] L back\n"); leftMotor.drive(-500); delay(800); leftMotor.coast();  bleSend("[DIAG] done\n"); break;
        case 'B': bleSend("[DIAG] R fwd\n");  rightMotor.drive( 500); delay(800); rightMotor.coast(); bleSend("[DIAG] done\n"); break;
        case 'X': bleSend("[DIAG] R back\n"); rightMotor.drive(-500); delay(800); rightMotor.coast(); bleSend("[DIAG] done\n"); break;

        // T: spin both fwd 1s, report encoder ticks — tells direction + if counting
        case 'T': {
            encodersEnable();
            bleSend("[ENC-TEST] spinning fwd 1s...\n");
            leftMotor.drive(220); rightMotor.drive(220);
            delay(1000);
            stopMotors();
            long tL = leftEnc.getTicks(), tR = rTicks();
            encodersDisable();
            bleSend("[ENC-TEST] L=%ld  R=%ld\n", tL, tR);
            bleSend("  L %s  R %s\n",
                tL > 10 ? "FWD-OK" : tL < -10 ? "BACKWARD" : "DEAD(0)",
                tR > 10 ? "FWD-OK" : tR < -10 ? "BACKWARD" : "DEAD(0)");
            break;
        }
        case 'r':
            encodersDisable();
            bleSend("[RESET] encoders detached\n");
            break;
        case '?':
            bleSend("[STATUS] L=%ld(%.0fmm) R=%ld(%.0fmm) diff=%+ld\n",
                    leftEnc.getTicks(),  leftEnc.getTicks()  * MM_PER_TICK,
                    rTicks(), rTicks() * MM_PER_TICK,
                    leftEnc.getTicks() - rTicks());
            break;
        default:
            bleSend("[?] '%c' unknown\n", cmd);
    }
}
