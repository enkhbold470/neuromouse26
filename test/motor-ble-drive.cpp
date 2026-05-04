// test/motor-ble-drive.cpp
//
// Fixes vs previous version:
//   1. Ramp is TIME-based, not encoder-based — no deadlock
//   2. Speed PID output has a PWM floor (MIN_PWM_OUT) — motor always gets
//      enough to overcome stiction regardless of integral windup state
//   3. Encoder ISRs disabled until motors actually start — kills false counts
//   4. drive() no longer prints every call — control loop can run at 100Hz
//
// BLE / Serial commands:
//   1-5  → move N cells (180mm each)
//   f    → free-run until 's'
//   s    → stop
//   r    → reset
//   ?    → status

#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── Tuning ────────────────────────────────────────────────────────────────────
static const float CRUISE_SPEED = 400.0f;  // ticks/sec
static const float RAMP_TIME_MS = 200.0f;  // ms to ramp from 0 → cruise
static const float Kp_speed     = 2.0f;
static const float Ki_speed     = 0.8f;
static const float Kp_heading   = 3.0f;    // ticks/sec per degree yaw error
static const int   MIN_PWM_OUT  = 200;     // minimum PWM written to motor — ensures motion
                                            // raise if motors still don't move (try 250, 300)

// ── Physics ───────────────────────────────────────────────────────────────────
#define CELL_MM        180.0f
#define MM_PER_TICK    ((float)(M_PI * WHEEL_DIAMETER) / TICKS_PER_REV)
#define TICKS_PER_CELL ((long)(CELL_MM / MM_PER_TICK))

// ── IMU ───────────────────────────────────────────────────────────────────────
#define MPU_ADDR         0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define GYRO_SCALE_DPS   131.0f
#define GYRO_NOISE_FLOOR 0.05f
#define CALIB_SAMPLES    200

float gyroBiasZ = 0.0f;
float yawDeg    = 0.0f;

struct IMURaw { int16_t ax, ay, az, temp, gx, gy, gz; };
bool imuReadAll(IMURaw& d) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
    if (Wire.available() < 14) return false;
    uint8_t b[14];
    for (int i = 0; i < 14; i++) b[i] = Wire.read();
    d.ax   = (int16_t)((b[0]  << 8) | b[1]);
    d.ay   = (int16_t)((b[2]  << 8) | b[3]);
    d.az   = (int16_t)((b[4]  << 8) | b[5]);
    d.temp = (int16_t)((b[6]  << 8) | b[7]);
    d.gx   = (int16_t)((b[8]  << 8) | b[9]);
    d.gy   = (int16_t)((b[10] << 8) | b[11]);
    d.gz   = (int16_t)((b[12] << 8) | b[13]);
    return true;
}

void imuUpdateYaw(float dt) {
    IMURaw d;
    if (!imuReadAll(d)) return;
    float rate = d.gz / GYRO_SCALE_DPS - gyroBiasZ;
    if (fabsf(rate) < GYRO_NOISE_FLOOR) rate = 0.0f;
    yawDeg += rate * dt;
}

void imuResetYaw() { yawDeg = 0.0f; }

void imuBegin() {
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1); Wire.write(0x00);
    if (Wire.endTransmission() != 0) { Serial.println("[IMU] wake FAIL"); return; }
    delay(100);
    Serial.printf("[IMU] calibrating %d samples — keep still...\n", CALIB_SAMPLES);
    float sum = 0; int good = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        IMURaw d;
        if (imuReadAll(d)) { sum += d.gz / GYRO_SCALE_DPS; good++; }
        delay(2);
    }
    gyroBiasZ = good ? sum / good : 0.0f;
    Serial.printf("[IMU] bias=%.4f dps (%d good)\n", gyroBiasZ, good);
    imuResetYaw();
}

// ── Motors + Encoders ─────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "L");
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "R");
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B, "L");
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B, "R");

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

// Returns PWM for motor.drive(). Always >= MIN_PWM_OUT when target > 0.
int computeSpeedPID(SpeedPID& pid, long ticks, float target, float dt) {
    float rawSpeed   = (ticks - pid.prevTicks) / dt;
    pid.prevSpeed    = 0.7f * pid.prevSpeed + 0.3f * rawSpeed;
    pid.prevTicks    = ticks;

    float error      = target - pid.prevSpeed;
    pid.integral    += error * dt;
    pid.integral     = constrain(pid.integral, -800.0f, 800.0f);

    float output = Kp_speed * error + Ki_speed * pid.integral;

    // Floor: if we want forward motion, never give less than MIN_PWM_OUT
    if (target > 0 && output < MIN_PWM_OUT) output = MIN_PWM_OUT;
    if (target < 0 && output > -MIN_PWM_OUT) output = -MIN_PWM_OUT;

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
// Heading correction — SYMMETRIC:
//   correction = Kp_heading * yawDeg  (ticks/sec)
//   v_left  = v_fwd - correction
//   v_right = v_fwd + correction

void moveCells(int n) {
    long target = TICKS_PER_CELL * (long)n;

    imuResetYaw();
    encodersEnable();   // attach ISRs fresh — clears noisy pre-counts
    resetPID(0, 0);

    bleSend("[MOVE] %d cell(s) target=%ld ticks (%.0fmm) MIN_PWM=%d\n",
            n, target, target * MM_PER_TICK, MIN_PWM_OUT);

    unsigned long startMs = millis();
    unsigned long lastMs  = millis();
    unsigned long lastLog = 0;

    while (true) {
        if (Serial.available()) rxCmd = Serial.read();
        if (rxCmd == 's') {
            rxCmd = 0;
            stopMotors();
            encodersDisable();
            bleSend("[MOVE] stopped L=%ld R=%ld\n",
                    leftEnc.getTicks(), rightEnc.getTicks());
            return;
        }

        unsigned long now = millis();
        float dt = (now - lastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        lastMs = now;

        imuUpdateYaw(dt);

        long tL  = leftEnc.getTicks();
        long tR  = rightEnc.getTicks();
        long avg = (tL + tR) / 2;

        // Time-based ramp up
        float elapsed = (float)(now - startMs);
        float v_forward;
        if (elapsed < RAMP_TIME_MS) {
            v_forward = CRUISE_SPEED * (elapsed / RAMP_TIME_MS);
        } else if (avg < (long)(target * 0.80f)) {
            v_forward = CRUISE_SPEED;
        } else {
            // Ramp down over last 20% of distance
            float remaining = (float)(target - avg) / (target * 0.20f);
            v_forward = CRUISE_SPEED * constrain(remaining, 0.0f, 1.0f);
        }

        // Symmetric heading correction
        float corr    = Kp_heading * yawDeg;
        float vL_cmd  = v_forward - corr;
        float vR_cmd  = v_forward + corr;

        int pwmL = computeSpeedPID(pidL, tL, vL_cmd, dt);
        int pwmR = computeSpeedPID(pidR, tR, vR_cmd, dt);

        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (now - lastLog >= 150) {
            lastLog = now;
            bleSend("avg=%4ld v=%.0f yaw=%+.2f L=%d R=%d\n",
                    avg, v_forward, yawDeg, pwmL, pwmR);
        }

        if (avg >= target) {
            stopMotors();
            encodersDisable();
            tL = leftEnc.getTicks();
            tR = rightEnc.getTicks();
            bleSend("[DONE] L=%ld(%.0fmm) R=%ld(%.0fmm) yaw=%+.2f t=%lums\n",
                    tL, tL*MM_PER_TICK, tR, tR*MM_PER_TICK,
                    yawDeg, millis() - startMs);
            if (fabsf(yawDeg) > 3.0f)
                bleSend("[WARN] yaw>3deg — raise Kp_heading\n");
            else
                bleSend("[PASS] heading OK\n");
            return;
        }

        if (millis() - startMs > (unsigned long)(8000 * n)) {
            stopMotors();
            encodersDisable();
            bleSend("[TIMEOUT] avg=%ld target=%ld — raise MIN_PWM_OUT or CRUISE_SPEED\n",
                    avg, target);
            return;
        }
    }
}

// ── Free-run ──────────────────────────────────────────────────────────────────
void freeRun() {
    imuResetYaw();
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
            bleSend("[FREE] stopped L=%ld R=%ld yaw=%.2f\n",
                    leftEnc.getTicks(), rightEnc.getTicks(), yawDeg);
            return;
        }

        unsigned long now = millis();
        float dt = (now - lastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        lastMs = now;

        imuUpdateYaw(dt);

        float corr = Kp_heading * yawDeg;
        int pwmL = computeSpeedPID(pidL, leftEnc.getTicks(),  CRUISE_SPEED - corr, dt);
        int pwmR = computeSpeedPID(pidR, rightEnc.getTicks(), CRUISE_SPEED + corr, dt);
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (now - lastLog >= 200) {
            lastLog = now;
            bleSend("L=%ld R=%ld yaw=%+.2f pwmL=%d pwmR=%d\n",
                    leftEnc.getTicks(), rightEnc.getTicks(), yawDeg, pwmL, pwmR);
        }
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n[INIT] motor-ble-drive v3");
    Serial.printf("[INIT] TICKS_PER_CELL=%ld  MM_PER_TICK=%.4f\n",
                  TICKS_PER_CELL, MM_PER_TICK);
    Serial.printf("[INIT] CRUISE=%.0f t/s  MIN_PWM=%d  Kp_spd=%.1f  Kp_hdg=%.1f\n",
                  CRUISE_SPEED, MIN_PWM_OUT, Kp_speed, Kp_heading);

    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    leftMotor.begin();
    rightMotor.begin();

    // Set encoder pins but DO NOT attach ISRs yet — prevents noise counts at idle
    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

    imuBegin();
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
            imuResetYaw();
            bleSend("[RESET] yaw=0 encoders detached\n");
            break;
        case '?':
            bleSend("[STATUS] L=%ld(%.0fmm) R=%ld yaw=%.2f\n",
                    leftEnc.getTicks(), leftEnc.getTicks()*MM_PER_TICK,
                    rightEnc.getTicks(), yawDeg);
            break;
        default:
            bleSend("[?] '%c' unknown\n", cmd);
    }
}
