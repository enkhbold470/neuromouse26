// test/motor-ble-drive.cpp
//
// Encoder-only straight drive test. No IMU.
// Each motor has its own speed PID targeting CRUISE_SPEED ticks/sec.
// Differential correction: if L faster than R, slow L / speed R and vice versa.
// Reports exact ticks + mm traveled over BLE so you can measure real distance.
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

// ── Tuning ────────────────────────────────────────────────────────────────────
static const float CRUISE_SPEED = 200.0f;  // ticks/sec — slow enough to stop accurately
static const float RAMP_TIME_MS = 150.0f;  // ms ramp 0 → cruise
static const float Kp_speed     = 1.5f;    // speed PID P gain (per motor)
static const float Ki_speed     = 0.5f;    // speed PID I gain (per motor)
static const float Kp_balance   = 3.0f;    // ticks/sec correction per tick of L-R error
static const int   MIN_PWM_OUT  = 150;     // PWM floor — lower = more precise stop

// ── Physics ───────────────────────────────────────────────────────────────────
#define CELL_MM        180.0f
#define MM_PER_TICK    ((float)(M_PI * WHEEL_DIAMETER) / TICKS_PER_REV)
#define TICKS_PER_CELL ((long)(CELL_MM / MM_PER_TICK))

// ── Motors + Encoders ─────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "L");
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "R");
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);

void IRAM_ATTR leftISR()  { leftEnc.handleInterrupt(); }
void IRAM_ATTR rightISR() { rightEnc.handleInterrupt(); }

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

// ── Speed PID ─────────────────────────────────────────────────────────────────
struct SpeedPID {
    float integral  = 0;
    long  prevTicks = 0;
    float prevSpeed = 0;
};
SpeedPID pidL, pidR;

// Returns PWM for motor.drive().
// applyFloor: true during ramp-up/cruise to overcome stiction.
//             false during ramp-down so PID can actually decelerate to zero.
int computeSpeedPID(SpeedPID& pid, long ticks, float target, float dt, bool applyFloor) {
    float rawSpeed   = (ticks - pid.prevTicks) / dt;
    pid.prevSpeed    = 0.7f * pid.prevSpeed + 0.3f * rawSpeed;
    pid.prevTicks    = ticks;

    float error      = target - pid.prevSpeed;
    pid.integral    += error * dt;
    pid.integral     = constrain(pid.integral, -800.0f, 800.0f);

    float output = Kp_speed * error + Ki_speed * pid.integral;

    // Floor only during acceleration phase — NOT during ramp-down
    if (applyFloor) {
        if (target > 0 && output < MIN_PWM_OUT) output = MIN_PWM_OUT;
        if (target < 0 && output > -MIN_PWM_OUT) output = -MIN_PWM_OUT;
    }

    return (int)constrain(output, -1023.0f, 1023.0f);
}

void resetPID(long tL, long tR) {
    pidL = SpeedPID(); pidL.prevTicks = tL;
    pidR = SpeedPID(); pidR.prevTicks = tR;
}

void stopMotors() {
    leftMotor.brake();
    rightMotor.brake();
    delay(80);
    leftMotor.coast();
    rightMotor.coast();
}

// ── BLE ───────────────────────────────────────────────────────────────────────
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* pTX     = nullptr;
bool                  bleConn = false;
char                  rxCmd   = 0;

void bleSend(const char* fmt, ...) {
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
    void onConnect(NimBLEServer*)    override { bleConn = true;  Serial.println("[BLE] connected"); }
    void onDisconnect(NimBLEServer*) override { bleConn = false; NimBLEDevice::startAdvertising(); }
};
class RXCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c) override {
        std::string v = c->getValue();
        if (!v.empty()) rxCmd = v[0];
    }
};

void bleSetup() {
    NimBLEDevice::init("Micromouse26-Motor");
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

// ── Move N cells ──────────────────────────────────────────────────────────────
//
// Velocity profile — TIME based (not encoder based):
//   0 → RAMP_TIME_MS         : ramp 0 → CRUISE_SPEED
//   until distance 80% done  : cruise
//   last 20% distance        : ramp down to 0
//
// Straight correction — encoder balance:
//   balance = Kp_balance * (tL - tR)
//   v_left  = v_fwd - balance
//   v_right = v_fwd + balance

void moveCells(int n) {
    long target = TICKS_PER_CELL * (long)n;

    encodersEnable();
    resetPID(0, 0);

    bleSend("[MOVE] %d cell(s) target=%ld ticks (%.0fmm)\n",
            n, target, target * MM_PER_TICK);

    unsigned long startMs = millis();
    unsigned long lastMs  = millis();
    unsigned long lastLog = 0;

    while (true) {
        if (Serial.available()) rxCmd = Serial.read();
        if (rxCmd == 's') {
            rxCmd = 0;
            stopMotors();
            encodersDisable();
            bleSend("[STOP] L=%ld(%.0fmm) R=%ld(%.0fmm)\n",
                    leftEnc.getTicks(),  leftEnc.getTicks()  * MM_PER_TICK,
                    rightEnc.getTicks(), rightEnc.getTicks() * MM_PER_TICK);
            return;
        }

        unsigned long now = millis();
        float dt = (now - lastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        lastMs = now;

        long tL  = leftEnc.getTicks();
        long tR  = rightEnc.getTicks();
        long avg = (tL + tR) / 2;

        // Time-based ramp up, encoder-based ramp down
        float elapsed = (float)(now - startMs);
        float v_fwd;
        bool  inDecel;
        if (elapsed < RAMP_TIME_MS) {
            v_fwd   = CRUISE_SPEED * (elapsed / RAMP_TIME_MS);
            inDecel = false;
        } else if (avg < (long)(target * 0.55f)) {
            v_fwd   = CRUISE_SPEED;
            inDecel = false;
        } else {
            // Ramp down over last 45% — floor disabled so PID can reach zero
            float rem = (float)(target - avg) / (target * 0.45f);
            v_fwd   = CRUISE_SPEED * constrain(rem, 0.0f, 1.0f);
            inDecel = true;
        }

        // Encoder balance: correct for L-R tick difference
        float balance = Kp_balance * (float)(tL - tR);
        float vL_cmd  = v_fwd - balance;
        float vR_cmd  = v_fwd + balance;

        int pwmL = computeSpeedPID(pidL, tL, vL_cmd, dt, !inDecel);
        int pwmR = computeSpeedPID(pidR, tR, vR_cmd, dt, !inDecel);

        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (now - lastLog >= 150) {
            lastLog = now;
            bleSend("avg=%4ld/%-4ld L=%4ld(%.0f) R=%4ld(%.0f) pwm=%d/%d\n",
                    avg, target,
                    tL, tL * MM_PER_TICK,
                    tR, tR * MM_PER_TICK,
                    pwmL, pwmR);
        }

        if (avg >= target - BRAKE_COMP_TICKS) {
            stopMotors();
            encodersDisable();
            tL = leftEnc.getTicks();
            tR = rightEnc.getTicks();
            long avgFinal = (tL + tR) / 2;
            float distMM  = avgFinal * MM_PER_TICK;
            float errMM   = (tL - tR) * MM_PER_TICK;
            bleSend("[DONE] avg=%ld(%.0fmm) L=%ld R=%ld diff=%.1fmm t=%lums\n",
                    avgFinal, distMM, tL, tR, errMM, millis() - startMs);
            if (fabsf(distMM - (target * MM_PER_TICK)) > 5.0f)
                bleSend("[WARN] dist err>5mm — tune BRAKE_COMP_TICKS (cur=%d)\n", BRAKE_COMP_TICKS);
            else
                bleSend("[PASS] distance OK\n");
            if (fabsf(errMM) > 5.0f)
                bleSend("[WARN] L-R diff>5mm — raise Kp_balance\n");
            return;
        }

        if (millis() - startMs > (unsigned long)(12000 * n)) {
            stopMotors();
            encodersDisable();
            bleSend("[TIMEOUT] avg=%ld target=%ld — raise MIN_PWM_OUT\n", avg, target);
            return;
        }
    }
}

// ── Free-run ──────────────────────────────────────────────────────────────────
void freeRun() {
    encodersEnable();
    resetPID(0, 0);
    bleSend("[FREE] send 's' to stop\n");

    unsigned long lastMs  = millis();
    unsigned long lastLog = 0;

    while (true) {
        if (Serial.available()) rxCmd = Serial.read();
        if (rxCmd == 's') {
            rxCmd = 0;
            stopMotors();
            encodersDisable();
            long tL = leftEnc.getTicks();
            long tR = rightEnc.getTicks();
            bleSend("[FREE] stopped L=%ld(%.0fmm) R=%ld(%.0fmm) diff=%.1fmm\n",
                    tL, tL * MM_PER_TICK,
                    tR, tR * MM_PER_TICK,
                    (tL - tR) * MM_PER_TICK);
            return;
        }

        unsigned long now = millis();
        float dt = (now - lastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        lastMs = now;

        long tL = leftEnc.getTicks();
        long tR = rightEnc.getTicks();

        float balance = Kp_balance * (float)(tL - tR);
        int pwmL = computeSpeedPID(pidL, tL, CRUISE_SPEED - balance, dt, true);
        int pwmR = computeSpeedPID(pidR, tR, CRUISE_SPEED + balance, dt, true);
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (now - lastLog >= 200) {
            lastLog = now;
            bleSend("L=%ld(%.0f) R=%ld(%.0f) diff=%+ld pwm=%d/%d\n",
                    tL, tL * MM_PER_TICK,
                    tR, tR * MM_PER_TICK,
                    tL - tR, pwmL, pwmR);
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
    Serial.printf("[INIT] CRUISE=%.0f t/s  MIN_PWM=%d  Kp_spd=%.1f  Kp_bal=%.1f\n",
                  CRUISE_SPEED, MIN_PWM_OUT, Kp_speed, Kp_balance);

    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    leftMotor.begin();
    rightMotor.begin();

    // Set encoder pins but DO NOT attach ISRs yet — prevents noise counts at idle
    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

    bleSetup();

    Serial.println("[INIT] ready — cmds: 1-5=cells  f=free  s=stop  r=reset  ?=status");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
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
        case 'f': freeRun();    break;
        case 's': stopMotors(); encodersDisable(); bleSend("[STOP]\n"); break;
        case 'r':
            encodersDisable();
            bleSend("[RESET] encoders detached\n");
            break;
        case '?':
            bleSend("[STATUS] L=%ld(%.0fmm) R=%ld(%.0fmm) diff=%+ld\n",
                    leftEnc.getTicks(),  leftEnc.getTicks()  * MM_PER_TICK,
                    rightEnc.getTicks(), rightEnc.getTicks() * MM_PER_TICK,
                    leftEnc.getTicks() - rightEnc.getTicks());
            break;
        default:
            bleSend("[?] '%c' unknown\n", cmd);
    }
}
