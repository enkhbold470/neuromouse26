// =============================================================================
// main.cpp — Micromouse26 firmware
//
// Architecture:
//   Motion timer : esp_timer @ 1kHz (ESP_TIMER_TASK)
//                  → fixed dt=1ms speed PID + encoder balance + g_yawCorr
//   Centering PID: main loop @ ~200Hz
//                  → IR L45/R45 + IMU heading → writes g_yawCorr for timer
//                  error = nR - nL:
//                    positive (closer to right) → g_yawCorr positive
//                    → vR increases, vL decreases → steers LEFT ✓
//                    negative (closer to left)  → g_yawCorr negative
//                    → vL increases, vR decreases → steers RIGHT ✓
//   Navigation   : FSM in loop() — maze, turns, BLE logging
//
// FSM:
//   STANDBY → (button) → ARMED → (3 waves) → EXPLORE → GOAL/STOP
//   STOP    → (button) → STANDBY
//
// Serial/BLE commands in STANDBY:
//   'c' = IR calibration   'd' = IR dump   'm' = motor test
// =============================================================================

#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include "esp_timer.h"
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseIR.h"
#include "MicromouseMaze.h"

// ── Tuning ────────────────────────────────────────────────────────────────────
#define CRUISE_SPEED    130.0f
#define RAMP_TICKS_MS   150
#define Kp_speed        1.5f
#define Ki_speed        0.5f
#define Kp_balance      3.0f   // encoder L-R balance gain
#define Kp_ir           25.0f  // IR centering P gain (ticks/sec per normalized error)
#define Ki_ir           1.0f   // IR centering I gain
#define Kd_ir           5.0f   // IR centering D gain (damps oscillation)
#define Kp_heading      2.0f   // IMU heading P gain
#define IR_LPF_BETA     0.15f
#define TURN_PWM        220
#define TURN_SLOW_DEG   75.0f
#define TURN_STOP_DEG   88.0f
#define TURN_TIMEOUT_MS 3000
// Positive MOTOR_R_TRIM slows right motor to offset mechanical dominance.
// Increase in steps of 5 until robot drives straight.
#define MOTOR_R_TRIM    0.0f

// ── Physics ───────────────────────────────────────────────────────────────────
#define CELL_MM         180.0f
#define MM_PER_TICK     ((float)(M_PI * WHEEL_DIAMETER) / TICKS_PER_REV)
#define TICKS_PER_CELL  ((long)(CELL_MM / MM_PER_TICK))

// ── Wave trigger ──────────────────────────────────────────────────────────────
#define FINGER_THRESH   0.5f
#define WAVES_NEEDED    3

// ── Hardware ──────────────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "L");
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "R");
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);
MicromouseIR      ir;
MicromouseMaze    maze;
IrCal             irCal[IR_COUNT];

// ── IMU ───────────────────────────────────────────────────────────────────────
#define MPU_ADDR         0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define GYRO_SCALE       131.0f
#define GYRO_NOISE_FLOOR 0.05f

float         gyroBiasZ  = 0.0f;
float         yawDeg     = 0.0f;
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

void imuUpdate() {
    unsigned long now = micros();
    float dt = (now - lastIMU_us) / 1e6f;
    if (dt <= 0.0f || dt > 0.1f) { lastIMU_us = now; return; }
    lastIMU_us = now;
    IMURaw d;
    if (!imuReadAll(d)) return;
    float rate = d.gz / GYRO_SCALE - gyroBiasZ;
    if (fabsf(rate) < GYRO_NOISE_FLOOR) rate = 0.0f;
    yawDeg += rate * dt;
}

void imuResetYaw() { yawDeg = 0.0f; lastIMU_us = micros(); }

void imuCalibrate() {
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT_1); Wire.write(0x00);
    Wire.endTransmission();
    delay(100);
    float sum = 0; int good = 0;
    for (int i = 0; i < 200; i++) {
        IMURaw d;
        if (imuReadAll(d)) { sum += d.gz / GYRO_SCALE; good++; }
        delay(2);
    }
    gyroBiasZ = good ? sum / good : 0.0f;
    Serial.printf("[IMU] bias=%.4f dps (%d/200 good)\n", gyroBiasZ, good);
    imuResetYaw();
}

// ── Encoder ISRs ──────────────────────────────────────────────────────────────
void IRAM_ATTR leftISR()  { leftEnc.handleInterrupt(); }
void IRAM_ATTR rightISR() { rightEnc.handleInterrupt(); }

void encodersEnable() {
    leftEnc.reset(); rightEnc.reset();
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightISR, RISING);
}
void encodersDisable() {
    detachInterrupt(digitalPinToInterrupt(ENC_L_A));
    detachInterrupt(digitalPinToInterrupt(ENC_R_A));
}

// =============================================================================
// MOTION TIMER — 1kHz fixed-rate PID in esp_timer task (Core 0).
// Never call Serial/BLE/delay/analogRead here. dt = exactly 0.001f.
// =============================================================================
struct SpeedPID {
    float integral  = 0;
    long  prevTicks = 0;
    float prevSpeed = 0;
};

static SpeedPID           pidL, pidR;
static esp_timer_handle_t motionTimer   = nullptr;
static volatile float     g_fwdSpeed    = 0.0f;
static volatile float     g_yawCorr     = 0.0f;  // written by main loop centering PID
static volatile long      g_tickTarget  = 0;
static volatile bool      g_motorOn     = false;
static volatile bool      g_motionDone  = false;
static volatile long      g_snapL       = 0;
static volatile long      g_snapR       = 0;
static volatile int       g_rampTick    = 0;

static int pidCompute(SpeedPID& pid, long ticks, float target) {
    const float dt = 0.001f;
    float raw      = (ticks - pid.prevTicks) / dt;
    pid.prevSpeed  = 0.7f * pid.prevSpeed + 0.3f * raw;
    pid.prevTicks  = ticks;
    float error    = target - pid.prevSpeed;
    pid.integral  += error * dt;
    pid.integral   = constrain(pid.integral, -800.0f, 800.0f);
    float out      = Kp_speed * error + Ki_speed * pid.integral;
    return (int)constrain(out, -1023.0f, 1023.0f);
}

static void motionTick(void*) {
    if (!g_motorOn) return;

    long tL  = leftEnc.getTicksRaw();
    long tR  = rightEnc.getTicksRaw();
    long avg = (tL + tR) / 2;
    g_snapL  = tL;
    g_snapR  = tR;

    g_rampTick++;
    float vfwd = g_fwdSpeed;
    if (g_rampTick < RAMP_TICKS_MS)
        vfwd *= (float)g_rampTick / (float)RAMP_TICKS_MS;

    if (g_tickTarget > 0) {
        long decelAt = (long)(g_tickTarget * 0.60f);
        if (avg > decelAt) {
            float rem = (float)(g_tickTarget - avg) / (float)(g_tickTarget - decelAt);
            vfwd *= constrain(rem, 0.0f, 1.0f);
        }
    }

    // g_yawCorr positive → vR increases, vL decreases → steers LEFT ✓
    float balance = Kp_balance * (float)(tL - tR) + g_yawCorr;
    int pwmL = pidCompute(pidL, tL, vfwd - balance);
    int pwmR = pidCompute(pidR, tR, vfwd + balance - MOTOR_R_TRIM);

    leftMotor.drive(pwmL);
    rightMotor.drive(pwmR);

    if (g_tickTarget > 0 && avg >= g_tickTarget - BRAKE_COMP_TICKS) {
        leftMotor.brake();
        rightMotor.brake();
        g_motorOn    = false;
        g_motionDone = true;
    }
}

void motionBegin() {
    esp_timer_create_args_t cfg = {};
    cfg.callback        = motionTick;
    cfg.dispatch_method = ESP_TIMER_TASK;
    cfg.name            = "motionPID";
    esp_timer_create(&cfg, &motionTimer);
}

void motionForward(long ticks, float speed) {
    encodersEnable();
    pidL = SpeedPID(); pidR = SpeedPID();
    g_fwdSpeed   = speed;
    g_tickTarget = ticks;
    g_yawCorr    = 0.0f;
    g_rampTick   = 0;
    g_motionDone = false;
    g_motorOn    = true;
    esp_timer_start_periodic(motionTimer, 1000);
}

void motionStop() {
    esp_timer_stop(motionTimer);
    g_motorOn = false;
    leftMotor.brake(); rightMotor.brake();
    delay(70);
    leftMotor.coast(); rightMotor.coast();
}

void motionCoast() { delay(70); leftMotor.coast(); rightMotor.coast(); }
bool motionDone()  { return g_motionDone; }

// =============================================================================
// CENTERING PID — runs in main loop ~200Hz, writes g_yawCorr for motion timer.
//
// error = nR - nL
//   R45 high (close to right wall) → error positive → g_yawCorr positive
//   → timer: vR += yawCorr, vL -= yawCorr → right faster, left slower → steers LEFT ✓
//   L45 high (close to left wall)  → error negative → g_yawCorr negative
//   → timer: vL += |yawCorr|, vR -= |yawCorr| → left faster, right slower → steers RIGHT ✓
// =============================================================================
static float irLfilt   = 0.0f;
static float irRfilt   = 0.0f;
static float irInteg   = 0.0f;
static float irPrevErr = 0.0f;

void centeringReset() {
    irLfilt = irRfilt = irInteg = irPrevErr = 0.0f;
    imuResetYaw();
}

void updateCentering() {
    imuUpdate();
    float headCorr = constrain(Kp_heading * yawDeg, -25.0f, 25.0f);

    float irCorr = 0.0f;
    if (irCal[IR_LEFT_45].calibrated || irCal[IR_RIGHT_45].calibrated) {
        ir.update();
        float nL = irCal[IR_LEFT_45].normalize(ir.raw[IR_LEFT_45]);
        float nR = irCal[IR_RIGHT_45].normalize(ir.raw[IR_RIGHT_45]);
        irLfilt += IR_LPF_BETA * (nL - irLfilt);
        irRfilt += IR_LPF_BETA * (nR - irRfilt);

        bool wallL = irLfilt > 0.15f;
        bool wallR = irRfilt > 0.15f;

        float error = 0.0f;
        if (wallL && wallR)   error =  irRfilt - irLfilt;
        else if (wallR)        error =  irRfilt - 0.4f;
        else if (wallL)        error = -(irLfilt - 0.4f);

        if (wallL || wallR) {
            const float dt = 0.005f;
            irInteg   += error * dt;
            irInteg    = constrain(irInteg, -2.0f, 2.0f);
            float deriv = (error - irPrevErr) / dt;
            irPrevErr  = error;
            irCorr = Kp_ir * error + Ki_ir * irInteg + Kd_ir * deriv;
            irCorr = constrain(irCorr, -40.0f, 40.0f);
        } else {
            irInteg = irPrevErr = 0.0f;
        }
    }

    g_yawCorr = headCorr + irCorr;
}

// =============================================================================
// LEDs
// =============================================================================
#define NUM_LEDS   8
#define LED_BRIGHT 5
CRGB leds[NUM_LEDS];

CRGB sensorColor(int raw, int threshold) {
    if (threshold <= 0) threshold = 500;
    uint8_t r = (uint8_t)constrain(map(raw, 0, threshold*2, 0,   255), 0, 255);
    uint8_t g = (uint8_t)constrain(map(raw, 0, threshold*2, 255, 0),   0, 255);
    return CRGB(r, g, 0);
}
CRGB battColor(float v) {
    float pct = constrain((v - 6.8f) / 1.6f, 0.0f, 1.0f);
    return CRGB((uint8_t)(255*(1-pct)), 0, (uint8_t)(255*pct));
}
void ledsUpdate(float vBat) {
    ir.update();
    leds[0] = sensorColor(ir.raw[IR_LEFT_FRONT],  irCal[IR_LEFT_FRONT].threshold);
    leds[1] = sensorColor(ir.raw[IR_LEFT_45],     irCal[IR_LEFT_45].threshold);
    leds[4] = sensorColor(ir.raw[IR_RIGHT_45],    irCal[IR_RIGHT_45].threshold);
    leds[5] = sensorColor(ir.raw[IR_RIGHT_FRONT], irCal[IR_RIGHT_FRONT].threshold);
    leds[6] = battColor(vBat);
    FastLED.show();
}

// =============================================================================
// BLE — main loop only, never from motion timer
// =============================================================================
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* pTX     = nullptr;
bool                  bleConn = false;
char                  rxCmd   = 0;

void bleSend(const char* fmt, ...) {
    char buf[160];
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
    NimBLEDevice::init("Micromouse26");
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

// =============================================================================
// Buzzer / Button
// =============================================================================
void beep(int ms) { ledcWrite(BUZZER_LEDC_CH, BUZZER_DUTY); delay(ms); ledcWrite(BUZZER_LEDC_CH, 0); }
void beepDone()   { beep(60); delay(80); beep(60); }
void beepGoal()   { for (int i=0; i<3; i++) { beep(120); delay(100); } }
void beepError()  { beep(500); }

bool buttonPressed() {
    static bool last = HIGH;
    bool now = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && now == LOW);
    last = now;
    return edge;
}
void waitButton() { while (!buttonPressed()) delay(10); delay(30); }

// =============================================================================
// Robot position / maze
// =============================================================================
uint8_t robotRow = 0, robotCol = 0;
AbsDir  heading  = DIR_NORTH;

void updatePosition(AbsDir d) {
    robotRow = (uint8_t)(robotRow + DIR_DR[d]);
    robotCol = (uint8_t)(robotCol + DIR_DC[d]);
}
void resetRunState() {
    robotRow = 0; robotCol = 0; heading = DIR_NORTH;
    maze.reset();
    maze.setGoalSingle(5, 2);
    maze.floodFill();
    bleSend("[RUN] reset pos=(0,0) N\n");
}

// =============================================================================
// moveForwardOneCell — starts timer, waits with centering active
// =============================================================================
void moveForwardOneCell() {
    centeringReset();
    motionForward(TICKS_PER_CELL, CRUISE_SPEED);

    unsigned long startMs = millis();
    unsigned long lastLog = 0;

    while (!motionDone()) {
        updateCentering();
        if (millis() - lastLog >= 400) {
            lastLog = millis();
            bleSend("[FWD] L=%ld R=%ld yaw=%.1f yawCorr=%.1f\n",
                    g_snapL, g_snapR, yawDeg, g_yawCorr);
        }
        if (millis() - startMs > 10000UL) {
            motionStop();
            beepError();
            bleSend("[FWD] timeout\n");
            return;
        }
    }
    motionCoast();
    encodersDisable();
}

// =============================================================================
// Turns — blocking IMU loop (short, ~300-500ms)
// =============================================================================
void turnRight90() {
    imuResetYaw();
    leftMotor.drive(TURN_PWM); rightMotor.drive(-TURN_PWM);
    unsigned long startMs = millis(), lastDbg = 0;
    bleSend("[TURN R] start\n");
    while (true) {
        imuUpdate();
        float absA = fabsf(yawDeg);
        if (millis() - lastDbg >= 250) { lastDbg = millis(); bleSend("[TURN R] yaw=%.1f\n", yawDeg); }
        if (absA >= TURN_SLOW_DEG) { leftMotor.drive(TURN_PWM/2); rightMotor.drive(-TURN_PWM/2); }
        if (absA >= TURN_STOP_DEG) {
            leftMotor.brake(); rightMotor.brake(); delay(60);
            leftMotor.coast(); rightMotor.coast();
            heading = (AbsDir)((heading + 1) % 4);
            bleSend("[TURN R] done yaw=%.1f\n", yawDeg); return;
        }
        if (millis() - startMs > TURN_TIMEOUT_MS) {
            leftMotor.brake(); rightMotor.brake(); delay(60);
            leftMotor.coast(); rightMotor.coast();
            heading = (AbsDir)((heading + 1) % 4);
            bleSend("[TURN R] TIMEOUT yaw=%.1f\n", yawDeg); beepError(); return;
        }
    }
}

void turnLeft90() {
    imuResetYaw();
    leftMotor.drive(-TURN_PWM); rightMotor.drive(TURN_PWM);
    unsigned long startMs = millis(), lastDbg = 0;
    bleSend("[TURN L] start\n");
    while (true) {
        imuUpdate();
        float absA = fabsf(yawDeg);
        if (millis() - lastDbg >= 250) { lastDbg = millis(); bleSend("[TURN L] yaw=%.1f\n", yawDeg); }
        if (absA >= TURN_SLOW_DEG) { leftMotor.drive(-TURN_PWM/2); rightMotor.drive(TURN_PWM/2); }
        if (absA >= TURN_STOP_DEG) {
            leftMotor.brake(); rightMotor.brake(); delay(60);
            leftMotor.coast(); rightMotor.coast();
            heading = (AbsDir)((heading + 3) % 4);
            bleSend("[TURN L] done yaw=%.1f\n", yawDeg); return;
        }
        if (millis() - startMs > TURN_TIMEOUT_MS) {
            leftMotor.brake(); rightMotor.brake(); delay(60);
            leftMotor.coast(); rightMotor.coast();
            heading = (AbsDir)((heading + 3) % 4);
            bleSend("[TURN L] TIMEOUT yaw=%.1f\n", yawDeg); beepError(); return;
        }
    }
}

void turnAround() { turnRight90(); delay(100); turnRight90(); }

// =============================================================================
// Sensing + calibration
// =============================================================================
void senseWalls() {
    ir.update();
    maze.setWall(robotRow, robotCol, heading,                      ir.wallFront(irCal));
    maze.setWall(robotRow, robotCol, (AbsDir)((heading+3)%4),      ir.wallLeft(irCal));
    maze.setWall(robotRow, robotCol, (AbsDir)((heading+1)%4),      ir.wallRight(irCal));
    maze.setWall(robotRow, robotCol, (AbsDir)((heading+2)%4),      false);
}

void runIrCalibration() {
    leds[7] = CRGB::Yellow; FastLED.show();
    bleSend("[CAL] Step 1: open air — press button\n");
    waitButton(); beep(60);
    for (int s = 0; s < IR_COUNT; s++) irCal[s].noWall = ir.sampleAvg(s, 64);
    bleSend("[CAL] noWall: LF=%d L45=%d R45=%d RF=%d\n",
            irCal[0].noWall, irCal[1].noWall, irCal[2].noWall, irCal[3].noWall);
    bleSend("[CAL] Step 2: dead-end (3 walls) — press button\n");
    waitButton(); beep(60);
    for (int s = 0; s < IR_COUNT; s++) {
        irCal[s].wall      = ir.sampleAvg(s, 64);
        irCal[s].threshold = (irCal[s].noWall + irCal[s].wall) / 2;
    }
    bleSend("[CAL] wall:   LF=%d L45=%d R45=%d RF=%d\n",
            irCal[0].wall, irCal[1].wall, irCal[2].wall, irCal[3].wall);
    bleSend("[CAL] thresh: LF=%d L45=%d R45=%d RF=%d\n",
            irCal[0].threshold, irCal[1].threshold,
            irCal[2].threshold, irCal[3].threshold);
    for (int s = 0; s < IR_COUNT; s++) irCal[s].calibrated = true;
    beepDone();
    bleSend("[CAL] done\n");
}

// =============================================================================
// Motor test helpers
// =============================================================================
void moveCells(int n) {
    long target = TICKS_PER_CELL * (long)n;
    bleSend("[MOVE] %d cell(s) target=%ld\n", n, target);
    motionForward(target, CRUISE_SPEED);
    unsigned long startMs = millis(), lastLog = 0;
    while (!motionDone()) {
        if (millis() - lastLog >= 150) {
            lastLog = millis();
            bleSend("L=%ld R=%ld avg=%ld/%ld\n",
                    g_snapL, g_snapR, (g_snapL+g_snapR)/2, target);
        }
        if (millis() - startMs > (unsigned long)(12000*n)) {
            motionStop(); bleSend("[TIMEOUT]\n"); return;
        }
    }
    motionCoast(); encodersDisable();
    long avg = (g_snapL + g_snapR) / 2;
    bleSend("[DONE] avg=%ld(%.0fmm) diff=%.1fmm\n",
            avg, avg * MM_PER_TICK, (g_snapL - g_snapR) * MM_PER_TICK);
    if (fabsf((g_snapL - g_snapR) * MM_PER_TICK) > 5.0f)
        bleSend("[WARN] L-R diff>5mm — tune MOTOR_R_TRIM (cur=%.0f)\n", (float)MOTOR_R_TRIM);
}

void freeRun() {
    bleSend("[FREE] send 's' to stop\n");
    motionForward(0, CRUISE_SPEED);
    unsigned long lastLog = 0;
    while (true) {
        if (Serial.available()) rxCmd = Serial.read();
        if (rxCmd == 's') { rxCmd = 0; motionStop(); encodersDisable(); break; }
        if (millis() - lastLog >= 200) {
            lastLog = millis();
            bleSend("L=%ld R=%ld diff=%+ld\n", g_snapL, g_snapR, g_snapL - g_snapR);
        }
    }
    bleSend("[FREE] stopped\n");
}

// =============================================================================
// FSM
// =============================================================================
enum State {
    STATE_STANDBY, STATE_CALIBRATE, STATE_ARMED,
    STATE_EXPLORE, STATE_GOAL_REACHED, STATE_STOP,
    STATE_MOTOR_TEST
};
State state = STATE_STANDBY;

// =============================================================================
// setup
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("[INIT] Micromouse26 boot");

    ledcSetup(BUZZER_LEDC_CH, BUZZER_FREQ, BUZZER_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH);
    ledcWrite(BUZZER_LEDC_CH, 0);
    beep(80);

    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    leftMotor.begin();
    rightMotor.begin();
    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

    ir.begin();

    FastLED.addLeds<WS2812B, WS2812_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHT);
    fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show();

    Serial.println("[INIT] IMU calibrating — keep still...");
    imuCalibrate();

    motionBegin();
    bleSetup();

    maze.reset();
    maze.setGoalSingle(5, 2);
    maze.floodFill();

    Serial.printf("[INIT] TICKS_PER_CELL=%ld  MM/tick=%.4f  CRUISE=%.0f\n",
                  TICKS_PER_CELL, MM_PER_TICK, (float)CRUISE_SPEED);
    Serial.println("[INIT] STANDBY — 'c'=cal 'd'=IR dump 'm'=motor test  button=ARM");

    beepDone();
    state = STATE_STANDBY;
}

// =============================================================================
// loop
// =============================================================================
void loop() {
    int   batRaw = analogRead(BAT_V_SENSE);
    float vBat   = (batRaw / 4095.0f) * 3.3f * BAT_VDIV_MULT;

    switch (state) {

    case STATE_STANDBY:
        leds[7] = CRGB(0, 200, 200);
        ledsUpdate(vBat);
        if (Serial.available()) rxCmd = Serial.read();
        if (rxCmd == 'c') { rxCmd = 0; state = STATE_CALIBRATE; break; }
        if (rxCmd == 'd') {
            rxCmd = 0;
            ir.update();
            bleSend("[IR] raw: LF=%d L45=%d R45=%d RF=%d\n",
                    ir.raw[0], ir.raw[1], ir.raw[2], ir.raw[3]);
            bleSend("[IR] cal: LF(t=%d,ok=%d) L45(t=%d,ok=%d) R45(t=%d,ok=%d) RF(t=%d,ok=%d)\n",
                    irCal[0].threshold, irCal[0].calibrated, irCal[1].threshold, irCal[1].calibrated,
                    irCal[2].threshold, irCal[2].calibrated, irCal[3].threshold, irCal[3].calibrated);
            bleSend("[IR] wallFront=%d wallLeft=%d wallRight=%d\n",
                    ir.wallFront(irCal), ir.wallLeft(irCal), ir.wallRight(irCal));
            break;
        }
        if (rxCmd == 'm') {
            rxCmd = 0;
            bleSend("[MOTOR_TEST] 1-5=cells f=free s=stop ?=status q=exit\n");
            state = STATE_MOTOR_TEST; break;
        }
        rxCmd = 0;
        if (buttonPressed()) {
            resetRunState(); beep(80);
            bleSend("[ARMED] wave 3x past LF sensor\n");
            state = STATE_ARMED;
        }
        delay(50);
        break;

    case STATE_CALIBRATE:
        runIrCalibration();
        state = STATE_STANDBY;
        break;

    case STATE_ARMED: {
        leds[7] = CRGB::White;
        ledsUpdate(vBat);
        static int  waveCount   = 0;
        static bool handPresent = false;
        ir.update();
        float nLF = irCal[IR_LEFT_FRONT].normalize(ir.raw[IR_LEFT_FRONT]);
        if (!handPresent && nLF > FINGER_THRESH)       handPresent = true;
        else if (handPresent && nLF <= FINGER_THRESH) {
            handPresent = false; waveCount++;
            bleSend("[ARMED] wave %d/%d\n", waveCount, WAVES_NEEDED); beep(30);
        }
        if (waveCount >= WAVES_NEEDED) {
            waveCount = 0; handPresent = false;
            bleSend("[ARMED] starting in 1s\n");
            delay(1000); beep(80); state = STATE_EXPLORE; break;
        }
        if (buttonPressed()) {
            waveCount = 0; handPresent = false;
            bleSend("[ARMED] cancelled\n"); state = STATE_STANDBY;
        }
        delay(20);
        break;
    }

    case STATE_EXPLORE: {
        leds[7] = CRGB::Green;
        maze.visited[robotRow][robotCol] = true;
        if (maze.isGoal(robotRow, robotCol)) { state = STATE_GOAL_REACHED; break; }

        senseWalls();
        maze.floodFill();

        uint8_t bestDist;
        AbsDir nextDir = maze.bestDirectionBiased(robotRow, robotCol, heading, bestDist);
        if (bestDist == FLOOD_INFINITY) {
            bleSend("[EXPLORE] TRAPPED at (%d,%d)\n", robotRow, robotCol);
            state = STATE_STOP; break;
        }

        bleSend("[EXPLORE] (%d,%d) h=%d → dir=%d dist=%d\n",
                robotRow, robotCol, heading, nextDir, bestDist);

        int turnSteps = ((int)nextDir - (int)heading + 4) % 4;
        switch (turnSteps) {
            case 1: turnRight90(); break;
            case 2: turnAround();  break;
            case 3: turnLeft90();  break;
            default: break;
        }
        moveForwardOneCell();
        delay(40);
        updatePosition(nextDir);
        break;
    }

    case STATE_GOAL_REACHED:
        fill_solid(leds, NUM_LEDS, CRGB::White); FastLED.show();
        beepGoal();
        bleSend("[GOAL] reached (%d,%d)!\n", robotRow, robotCol);
        state = STATE_STOP;
        break;

    case STATE_STOP:
        motionStop();
        leds[7] = CRGB::Red;
        ledsUpdate(vBat);
        if (buttonPressed()) {
            fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show();
            bleSend("[STOP] → STANDBY\n");
            state = STATE_STANDBY;
        }
        delay(100);
        break;

    case STATE_MOTOR_TEST:
        leds[7] = CRGB(0, 0, 200); FastLED.show();
        if (Serial.available()) rxCmd = Serial.read();
        if (rxCmd == 0) { delay(20); break; }
        {
            char cmd = rxCmd; rxCmd = 0;
            switch (cmd) {
                case '1': moveCells(1); break;
                case '2': moveCells(2); break;
                case '3': moveCells(3); break;
                case '4': moveCells(4); break;
                case '5': moveCells(5); break;
                case 'f': freeRun(); break;
                case 's': motionStop(); encodersDisable(); bleSend("[STOP]\n"); break;
                case '?':
                    bleSend("[STATUS] L=%ld(%.0fmm) R=%ld(%.0fmm) diff=%+ld\n",
                            g_snapL, g_snapL*MM_PER_TICK,
                            g_snapR, g_snapR*MM_PER_TICK,
                            g_snapL - g_snapR);
                    break;
                case 'q':
                    motionStop(); encodersDisable();
                    fill_solid(leds, NUM_LEDS, CRGB::Black); FastLED.show();
                    bleSend("[MOTOR_TEST] exit\n"); state = STATE_STANDBY; break;
                default: bleSend("[?] '%c' unknown\n", cmd);
            }
        }
        break;
    }
}
