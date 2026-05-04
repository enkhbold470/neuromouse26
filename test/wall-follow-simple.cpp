// test/wall-follow-simple.cpp
//
// NO encoders. NO PID. Fixed PWM only.
// IR values hardcoded from real calibration run.
// Behaviour:
//   - Drive forward at BASE_PWM
//   - L45 vs R45 → differential correction to stay centered
//   - LF or RF above threshold → STOP (front wall)
//   - Print estimated cell count (time-based)
//
// Hardcoded from user's calibration:
//   [IR] raw: LF=1122 L45=958 R45=890 RF=727
//   [IR] cal: LF(t=623) L45(t=783) R45(t=449) RF(t=387)
//
// BLE / Serial: stream of sensor + correction data.
// To stop manually: send 's'.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"

// ── Hardcoded IR calibration (from real hardware measurements) ────────────────
//
// Dead end (all walls, axle at cell center 9cm from each wall):
//   LF=1825  L45=1441  R45=1396  RF=1308
//
// Both side walls, no front wall (robot centered in corridor):
//   LF=113   L45=865   R45=477   RF=20
//
// Left wall only (right side open):
//   LF=175   L45=948   R45=54    RF=60
//
// Centering strategy: normalize both sensors to 1.0 at corridor center.
//   nL = raw_L45 / L45_CENTER  →  1.0 when 9cm from left wall
//   nR = raw_R45 / R45_CENTER  →  1.0 when 9cm from right wall
//   error = nR - nL            →  0.0 when perfectly centered
//
// Wall present thresholds (midpoint between open-air and wall-at-center):
#define L45_CENTER  865   // L45 reading when centered in corridor
#define R45_CENTER  477   // R45 reading when centered in corridor
#define L45_THRESH  433   // (0 + 865) / 2  — wall detected above this
#define R45_THRESH  238   // (0 + 477) / 2  — wall detected above this

// ── Motion tuning ─────────────────────────────────────────────────────────────
#define BASE_PWM      600   // base forward PWM (0-1023). Raise if too slow.
// Centering PID — error is normalized (typical range ±0.3 when slightly off center).
// At 2cm drift: error ≈ 0.15 → Kp=200 gives 30 PWM (gentle nudge).
// Raise Kp if drifts too much. Raise Kd first if robot wiggles.
#define CENTER_KP     200.0f
#define CENTER_KI     10.0f
#define CENTER_KD     40.0f
#define MAX_CORR      150
#define CELL_MS       800

// ── Hardware ──────────────────────────────────────────────────────────────────
MicromouseMotor leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "L");
MicromouseMotor rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "R");

// ── BLE ───────────────────────────────────────────────────────────────────────
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* pTX     = nullptr;
bool                  bleConn = false;
char                  rxCmd   = 0;

void bleSend(const char* fmt, ...) {
    char buf[128];
    va_list a; va_start(a, fmt);
    vsnprintf(buf, sizeof(buf), fmt, a);
    va_end(a);
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
    void onConnect(NimBLEServer*)    override { bleConn = true; }
    void onDisconnect(NimBLEServer*) override { bleConn = false; NimBLEDevice::startAdvertising(); }
};
class RXCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c) override {
        std::string v = c->getValue();
        if (!v.empty()) rxCmd = v[0];
    }
};

void bleSetup() {
    NimBLEDevice::init("Micromouse26-WF");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    auto* srv = NimBLEDevice::createServer();
    srv->setCallbacks(new BLECb());
    auto* svc = srv->createService(NUS_SERVICE_UUID);
    pTX = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    auto* pRX = svc->createCharacteristic(NUS_RX_UUID,
                    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pRX->setCallbacks(new RXCb());
    svc->start();
    NimBLEDevice::getAdvertising()->addServiceUUID(NUS_SERVICE_UUID);
    NimBLEDevice::startAdvertising();
}

// ── IR read helpers ───────────────────────────────────────────────────────────
static const uint8_t EMIT_PINS[4] = { EMIT_LF, EMIT_L45, EMIT_R45, EMIT_RF };
static const uint8_t RX_PINS[4]   = { RX_LF,   RX_L45,   RX_R45,   RX_RF  };

int irRead(int idx) {
    int amb = analogRead(RX_PINS[idx]);
    digitalWrite(EMIT_PINS[idx], HIGH);
    delayMicroseconds(50);
    int lit = analogRead(RX_PINS[idx]);
    digitalWrite(EMIT_PINS[idx], LOW);
    return max(0, lit - amb);
}

// Normalize: 1.0 = centered (9cm from wall). 0.0 = open. >1.0 = closer than center.
// error = nR - nL = 0 when perfectly centered between both walls.
float normL45(int raw) { return constrain((float)raw / L45_CENTER, 0.0f, 2.0f); }
float normR45(int raw) { return constrain((float)raw / R45_CENTER, 0.0f, 2.0f); }

void stopMotors() {
    leftMotor.brake();
    rightMotor.brake();
    delay(80);
    leftMotor.coast();
    rightMotor.coast();
}

// ── Centering PID state ───────────────────────────────────────────────────────
// error  = nR - nL
// error > 0 → closer to RIGHT wall → increase right motor, reduce left → turns LEFT
// error < 0 → closer to LEFT wall  → increase left motor, reduce right → turns RIGHT
struct CenterPID {
    float integral  = 0;
    float prevError = 0;
    unsigned long prevMs = 0;

    float compute(float error) {
        unsigned long now = millis();
        float dt = (prevMs == 0) ? 0.01f : constrain((now - prevMs) / 1000.0f, 0.001f, 0.1f);
        prevMs = now;

        integral += error * dt;
        integral  = constrain(integral, -2.0f, 2.0f);

        float derivative = (error - prevError) / dt;
        prevError = error;

        float out = CENTER_KP * error + CENTER_KI * integral + CENTER_KD * derivative;
        return constrain(out, -(float)MAX_CORR, (float)MAX_CORR);
    }

    void reset() { integral = 0; prevError = 0; prevMs = 0; }
} centerPid;

// ── State ─────────────────────────────────────────────────────────────────────
bool     running  = false;
int      cellCount = 0;
unsigned long cellTimer = 0;

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1500);

    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    leftMotor.begin();
    rightMotor.begin();

    for (int i = 0; i < 4; i++) {
        pinMode(EMIT_PINS[i], OUTPUT);
        digitalWrite(EMIT_PINS[i], LOW);
        pinMode(RX_PINS[i], INPUT);
    }

    bleSetup();

    Serial.println("[WALL-FOLLOW] ready");
    Serial.printf("[WALL-FOLLOW] BASE_PWM=%d  CENTER_KP=%.2f  CELL_MS=%d\n",
                  BASE_PWM, CENTER_KP, CELL_MS);
    Serial.println("[WALL-FOLLOW] send 'g' to start, 's' to stop");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    if (Serial.available()) rxCmd = Serial.read();

    if (rxCmd == 'g') {
        rxCmd     = 0;
        running   = true;
        cellCount = 0;
        cellTimer = millis();
        centerPid.reset();
        bleSend("[GO] running — send 's' to stop\n");
    }
    if (rxCmd == 's') {
        rxCmd   = 0;
        running = false;
        stopMotors();
        bleSend("[STOP] manual stop — cells≈%d\n", cellCount);
    }

    if (!running) { delay(20); return; }

    // ── Read side sensors only (front sensors unused) ──
    int l45Raw = irRead(1);
    int r45Raw = irRead(2);

    // ── Cell counting (time-based) ──
    if (millis() - cellTimer >= (unsigned long)CELL_MS) {
        cellTimer = millis();
        cellCount++;
        bleSend("[CELL] %d\n", cellCount);
    }

    // ── Centering PID ──
    // Normalize so both sensors on same 0.0–1.0 scale.
    float nL    = normL45(l45Raw);
    float nR    = normR45(r45Raw);
    bool  wallL = l45Raw > L45_THRESH;
    bool  wallR = r45Raw > R45_THRESH;

    // error > 0 → closer to RIGHT wall → correction positive
    //   → pwmR = BASE + corr (faster) , pwmL = BASE - corr (slower) → steers LEFT ✓
    // error < 0 → closer to LEFT wall  → correction negative
    //   → pwmL = BASE + |corr| (faster), pwmR = BASE - |corr| (slower) → steers RIGHT ✓
    float error = 0.0f;
    if (wallL && wallR) {
        error = nR - nL;         // both walls: nL==nR==1.0 when centered → error=0
    } else if (wallR) {
        error =  (nR - 1.0f);   // right wall: 1.0 = 9cm away (centered), >1.0 = too close
    } else if (wallL) {
        error = -(nL - 1.0f);   // left wall:  1.0 = 9cm away (centered), >1.0 = too close
    }

    float correction = (wallL || wallR) ? centerPid.compute(error) : 0.0f;

    // positive correction → right motor faster, left motor slower
    int pwmL = constrain(BASE_PWM - (int)correction, 0, 1023);
    int pwmR = constrain(BASE_PWM + (int)correction, 0, 1023);

    leftMotor.drive(pwmL);
    rightMotor.drive(pwmR);

    // ── Telemetry (200ms interval) ──
    static unsigned long lastLog = 0;
    if (millis() - lastLog >= 200) {
        lastLog = millis();
        bleSend("L45=%4d(%.2f) R45=%4d(%.2f) err=%+.2f corr=%+.0f pwm=%d/%d\n",
                l45Raw, nL, r45Raw, nR, error, correction, pwmL, pwmR);
    }
}
