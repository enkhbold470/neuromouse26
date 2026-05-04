// test/motor-ble-drive.cpp
//
// Straight-line drive using:
//   - Per-wheel speed PID (inner loop, 1 kHz via hw timer)
//   - Gyro yaw correction applied SYMMETRICALLY to both wheels (outer loop)
//   - Encoder distance for stop condition
//   - Trapezoidal velocity profile (ramp up → cruise → ramp down)
//
// Architecture matches micromouse-algorithm doc:
//   Inner 1kHz: encoder ticks → speed PID → PWM
//   Outer 100Hz: gyro heading → correction → adjust speed targets
//
// BLE / Serial commands:
//   1-5  → move N cells (180mm each)
//   f    → free-run until 's'
//   s    → stop
//   r    → reset
//   ?    → status
//
// ── Tuning (change these first) ──────────────────────────────────────────────
// CRUISE_SPEED_TICKS_S : target wheel speed in ticks/sec
//   500rpm motor → ~8.3 rev/s → 8.3 * 210 ticks ≈ 1750 ticks/s max
//   Start at 400 (~23% max speed) — reliable and easy to correct
// Kp_speed, Ki_speed   : inner speed loop. Start Kp=1.5, Ki=0.3
// Kp_heading           : yaw correction gain. Start at 3.0
//   At 1° error → 3 ticks/s correction on each motor (very mild)

#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── Tuning ────────────────────────────────────────────────────────────────────
static const float CRUISE_SPEED   = 400.0f;  // ticks/sec target speed
static const float RAMP_TICKS     = 30.0f;   // ticks to ramp up over (~15mm)
static const float Kp_speed       = 1.5f;    // inner speed PID
static const float Ki_speed       = 0.3f;
static const float Kp_heading     = 3.0f;    // outer heading correction (ticks/s per degree)

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

float         gyroBiasZ  = 0.0f;
volatile float yawDeg    = 0.0f;    // updated in 1kHz ISR
unsigned long lastIMU_us = 0;

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
    if (Wire.endTransmission() != 0) {
        Serial.println("[IMU] ERROR: wake failed");
        return;
    }
    delay(100);
    Serial.printf("[IMU] calibrating %d samples...\n", CALIB_SAMPLES);
    float sum = 0; int good = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        IMURaw d;
        if (imuReadAll(d)) { sum += d.gz / GYRO_SCALE_DPS; good++; }
        delay(2);
    }
    gyroBiasZ = good ? sum / good : 0;
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

// ── Inner speed PID state ─────────────────────────────────────────────────────
struct SpeedPID {
    float integral   = 0;
    long  prevTicks  = 0;
    float prevSpeed  = 0;
};
SpeedPID pidL, pidR;

// Compute PWM output for one wheel.
// target: desired speed in ticks/sec
// Returns signed PWM for motor.drive()
int computeSpeedPID(SpeedPID& pid, long currentTicks, float target, float dt) {
    // Speed estimate (ticks/sec), low-pass filtered
    float raw_speed = (currentTicks - pid.prevTicks) / dt;
    pid.prevSpeed   = 0.7f * pid.prevSpeed + 0.3f * raw_speed;  // LP filter
    pid.prevTicks   = currentTicks;

    float error     = target - pid.prevSpeed;
    pid.integral   += error * dt;
    pid.integral    = constrain(pid.integral, -500.0f, 500.0f);  // anti-windup

    float output = Kp_speed * error + Ki_speed * pid.integral;
    return (int)constrain(output, -1023.0f, 1023.0f);
}

void resetPID() {
    pidL = SpeedPID();
    pidR = SpeedPID();
    pidL.prevTicks = leftEnc.getTicks();
    pidR.prevTicks = rightEnc.getTicks();
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
    auto* pRX = svc->createCharacteristic(NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pRX->setCallbacks(new RXCb());
    svc->start();
    NimBLEDevice::getAdvertising()->addServiceUUID(NUS_SERVICE_UUID);
    NimBLEDevice::startAdvertising();
}

// ── Move N cells ──────────────────────────────────────────────────────────────
//
// Outer loop runs at ~100Hz (every 10ms).
// Inner speed PID runs inside this loop but uses its own dt measurement.
//
// Velocity profile:
//   0 → RAMP_TICKS       : linearly ramp from 0 → CRUISE_SPEED
//   RAMP_TICKS → target-RAMP_TICKS : cruise at CRUISE_SPEED
//   target-RAMP_TICKS → target      : linearly ramp from CRUISE_SPEED → 0
//
// Heading correction (symmetric):
//   yaw > 0 → drifting left → slow left, speed right
//   correction = Kp_heading * yaw  (in ticks/sec)
//   v_left  = v_forward - correction
//   v_right = v_forward + correction

void moveCells(int n) {
    long target   = TICKS_PER_CELL * (long)n;
    long rampDown = target - (long)RAMP_TICKS;

    leftEnc.reset();
    rightEnc.reset();
    imuResetYaw();
    resetPID();

    bleSend("[MOVE] %d cell(s) target=%ld ticks (%.0fmm)\n",
            n, target, target * MM_PER_TICK);
    bleSend("[MOVE] speed=%.0f t/s  Kp_h=%.1f  ramp=%d ticks\n",
            CRUISE_SPEED, Kp_heading, (int)RAMP_TICKS);

    unsigned long startMs  = millis();
    unsigned long lastMs   = millis();
    unsigned long lastLog  = 0;

    while (true) {
        // Check stop command
        if (Serial.available()) rxCmd = Serial.read();
        if (rxCmd == 's') {
            rxCmd = 0;
            stopMotors();
            bleSend("[MOVE] stopped L=%ld R=%ld yaw=%.2f\n",
                    leftEnc.getTicks(), rightEnc.getTicks(), yawDeg);
            return;
        }

        unsigned long now = millis();
        float dt = (now - lastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;  // guard against dt=0
        lastMs = now;

        // Update gyro
        imuUpdateYaw(dt);

        long tL  = leftEnc.getTicks();
        long tR  = rightEnc.getTicks();
        long avg = (tL + tR) / 2;

        // Trapezoidal speed profile
        float v_forward;
        if (avg < (long)RAMP_TICKS) {
            // Ramp up
            v_forward = CRUISE_SPEED * ((float)avg / RAMP_TICKS);
            v_forward = max(v_forward, CRUISE_SPEED * 0.15f);  // min 15% to overcome stiction
        } else if (avg < rampDown) {
            // Cruise
            v_forward = CRUISE_SPEED;
        } else {
            // Ramp down
            float remaining = (float)(target - avg);
            v_forward = CRUISE_SPEED * (remaining / RAMP_TICKS);
            v_forward = max(v_forward, CRUISE_SPEED * 0.15f);  // keep moving
        }

        // Heading correction — symmetric on both motors
        float correction = Kp_heading * yawDeg;  // ticks/sec
        float v_left_cmd  = v_forward - correction;
        float v_right_cmd = v_forward + correction;

        // Inner speed PID — converts speed targets to PWM
        int pwmL = computeSpeedPID(pidL, tL, v_left_cmd,  dt);
        int pwmR = computeSpeedPID(pidR, tR, v_right_cmd, dt);

        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        // Log every 150ms
        if (now - lastLog >= 150) {
            lastLog = now;
            bleSend("avg=%4ld v=%.0f yaw=%+.2f corr=%+.0f L=%d R=%d\n",
                    avg, v_forward, yawDeg, correction, pwmL, pwmR);
        }

        // Done
        if (avg >= target) {
            stopMotors();
            tL = leftEnc.getTicks();
            tR = rightEnc.getTicks();
            bleSend("[DONE] L=%ld(%.1fmm) R=%ld(%.1fmm) yaw=%+.2f t=%lums\n",
                    tL, tL * MM_PER_TICK, tR, tR * MM_PER_TICK,
                    yawDeg, millis() - startMs);
            if (fabsf(yawDeg) > 3.0f)
                bleSend("[WARN] yaw>3deg — raise Kp_heading (now %.1f)\n", Kp_heading);
            else
                bleSend("[PASS] heading OK\n");
            return;
        }

        // Timeout
        if (millis() - startMs > (unsigned long)(8000 * n)) {
            stopMotors();
            bleSend("[TIMEOUT] avg=%ld target=%ld — raise CRUISE_SPEED?\n", avg, target);
            return;
        }
    }
}

// ── Free-run ──────────────────────────────────────────────────────────────────
void freeRun() {
    leftEnc.reset();
    rightEnc.reset();
    imuResetYaw();
    resetPID();
    bleSend("[FREE] running — send 's' to stop\n");

    unsigned long lastMs  = millis();
    unsigned long lastLog = 0;

    while (true) {
        if (Serial.available()) rxCmd = Serial.read();
        if (rxCmd == 's') {
            rxCmd = 0;
            stopMotors();
            bleSend("[FREE] stopped L=%ld R=%ld yaw=%.2f\n",
                    leftEnc.getTicks(), rightEnc.getTicks(), yawDeg);
            return;
        }

        unsigned long now = millis();
        float dt = (now - lastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        lastMs = now;

        imuUpdateYaw(dt);

        float correction  = Kp_heading * yawDeg;
        int pwmL = computeSpeedPID(pidL, leftEnc.getTicks(),  CRUISE_SPEED - correction, dt);
        int pwmR = computeSpeedPID(pidR, rightEnc.getTicks(), CRUISE_SPEED + correction, dt);
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (now - lastLog >= 200) {
            lastLog = now;
            bleSend("L=%ld R=%ld yaw=%+.2f\n",
                    leftEnc.getTicks(), rightEnc.getTicks(), yawDeg);
        }
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n[INIT] motor-ble-drive v2 (speed PID + gyro heading)");
    Serial.printf("[INIT] TICKS_PER_CELL=%ld  MM_PER_TICK=%.4f\n",
                  TICKS_PER_CELL, MM_PER_TICK);
    Serial.printf("[INIT] CRUISE=%.0f t/s  Kp_spd=%.1f  Ki_spd=%.2f  Kp_hdg=%.1f\n",
                  CRUISE_SPEED, Kp_speed, Ki_speed, Kp_heading);
    Serial.println("[INIT] Keep robot STILL during IMU calibration...");

    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    leftMotor.begin();
    rightMotor.begin();

    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, RISING);

    imuBegin();
    bleSetup();

    Serial.println("[INIT] ready — BLE: 'Micromouse26-Motor'");
    Serial.println("[INIT] cmds: 1-5=N cells  f=freerun  s=stop  r=reset  ?=status");
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
        case 'f': freeRun(); break;
        case 's': stopMotors(); bleSend("[STOP]\n"); break;
        case 'r':
            leftEnc.reset(); rightEnc.reset(); imuResetYaw(); resetPID();
            bleSend("[RESET] done\n");
            break;
        case '?':
            bleSend("[STATUS] L=%ld(%.1fmm) R=%ld yaw=%.2f\n",
                    leftEnc.getTicks(), leftEnc.getTicks()*MM_PER_TICK,
                    rightEnc.getTicks(), yawDeg);
            break;
        default:
            bleSend("[?] unknown '%c'\n", cmd);
    }
}
