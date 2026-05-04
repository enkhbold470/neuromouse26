// =============================================================================
// main.cpp — Micromouse26 full firmware
//
// FSM:
//   IDLE → (button) → CALIBRATE → (cal done) → READY
//   READY → (finger on LF sensor) → 1s delay → EXPLORE
//   EXPLORE → (goal reached) → GOAL_REACHED → STOP
//   STOP → (button) → IDLE
//
// Maze: 3×6 draft, goal = (row=5, col=2)
//   Uses 16×16 MicromouseMaze with setGoalSingle(5,2)
//
// Straight-line steering: 45° IR sensors (L45/R45) for lateral centering.
//   Gyro heading hold as fallback when no side walls.
//
// No Serial prints during motion. BLE only used outside move loops.
// LEDs provide real-time status during run.
//
// LED map:
//   0 = LF sensor    (green=open → red=wall)
//   1 = L45 sensor   (green=open → red=wall)
//   2 = left motor   (brightness = PWM%)
//   3 = right motor  (brightness = PWM%)
//   4 = R45 sensor   (green=open → red=wall)
//   5 = RF sensor    (green=open → red=wall)
//   6 = battery      (blue=full → red=low)
//   7 = state        (IDLE=off CALIBRATE=yellow READY=cyan EXPLORE=green GOAL=white)
// =============================================================================

#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseIR.h"
#include "MicromouseMaze.h"

// ── Tuning ────────────────────────────────────────────────────────────────────
#define CRUISE_PWM      350    // base drive PWM (0-1023). Slow and safe first.
#define TURN_PWM        300    // spin PWM for 90° turns
#define RAMP_MS         180    // ms to ramp from 0 → CRUISE_PWM
#define Kp_speed        1.8f   // inner speed PID proportional
#define Ki_speed        0.4f   // inner speed PID integral
#define Kp_lat          25.0f  // 45° IR lateral correction gain (PWM per normalized unit)
#define Kp_yaw          4.0f   // gyro heading correction gain (PWM per degree)
#define TURN_SLOW_DEG   75.0f  // start slowing at this angle during turns
#define TURN_STOP_DEG   88.0f  // stop turn at this angle
#define MOVE_TIMEOUT_MS 4000   // forward move safety cutoff
#define TURN_TIMEOUT_MS 3000   // turn safety cutoff
#define FINGER_THRESH   0.5f   // normalized LF reading to trigger start (finger blocks sensor)
#define FINGER_CONFIRM  3      // consecutive readings above threshold to confirm

// ── Physics ───────────────────────────────────────────────────────────────────
#define CELL_MM        180.0f
#define MM_PER_TICK    ((float)(M_PI * WHEEL_DIAMETER) / TICKS_PER_REV)
#define TICKS_PER_CELL ((long)(CELL_MM / MM_PER_TICK))

// ── Hardware objects ──────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "L");
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "R");
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);
MicromouseIR      ir;
MicromouseMaze    maze;

void IRAM_ATTR leftISR()  { leftEnc.handleInterrupt(); }
void IRAM_ATTR rightISR() { rightEnc.handleInterrupt(); }

// ── IR calibration ────────────────────────────────────────────────────────────
IrCal irCal[IR_COUNT];   // LF, L45, R45, RF

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

// No Serial — safe to call inside motion loops
void imuUpdate() {
    unsigned long now = micros();
    float dt = (now - lastIMU_us) / 1e6f;
    lastIMU_us = now;
    IMURaw d;
    if (!imuReadAll(d)) return;
    float rate = d.gz / GYRO_SCALE - gyroBiasZ;
    if (fabsf(rate) < GYRO_NOISE_FLOOR) rate = 0.0f;
    yawDeg += rate * dt;
}

void imuResetYaw() {
    yawDeg     = 0.0f;
    lastIMU_us = micros();
}

void imuCalibrate() {
    // Called once at boot — Serial only, BLE not up yet
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

// ── Speed PID ─────────────────────────────────────────────────────────────────
struct SpeedPID {
    float integral = 0, prevSpeed = 0;
    long  prevTicks = 0;
};
SpeedPID pidL, pidR;

void resetPIDs() {
    pidL = SpeedPID(); pidL.prevTicks = leftEnc.getTicks();
    pidR = SpeedPID(); pidR.prevTicks = rightEnc.getTicks();
}

// Returns PWM magnitude — no Serial inside
int speedPID(SpeedPID& p, long ticks, float target, float dt) {
    float speed     = (ticks - p.prevTicks) / dt;
    p.prevSpeed     = 0.7f * p.prevSpeed + 0.3f * speed;
    p.prevTicks     = ticks;
    float err       = target - p.prevSpeed;
    p.integral     += err * dt;
    p.integral      = constrain(p.integral, -600.0f, 600.0f);
    float out       = Kp_speed * err + Ki_speed * p.integral;
    // floor: always enough to overcome stiction when target > 0
    if (target > 0 && out < (float)CRUISE_PWM * 0.3f)
        out = (float)CRUISE_PWM * 0.3f;
    return (int)constrain(out, 0.0f, 1023.0f);
}

// ── LEDs ──────────────────────────────────────────────────────────────────────
#define NUM_LEDS   8
#define LED_BRIGHT 5
CRGB leds[NUM_LEDS];

CRGB sensorColor(int raw, int threshold) {
    if (threshold <= 0) threshold = 500;
    uint8_t r = (uint8_t)constrain(map(raw, 0, threshold * 2, 0,   255), 0, 255);
    uint8_t g = (uint8_t)constrain(map(raw, 0, threshold * 2, 255, 0),   0, 255);
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

// ── BLE ───────────────────────────────────────────────────────────────────────
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* pTX     = nullptr;
bool                  bleConn = false;

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
    void onConnect(NimBLEServer*)    override { bleConn = true;  }
    void onDisconnect(NimBLEServer*) override { bleConn = false; NimBLEDevice::startAdvertising(); }
};

void bleSetup() {
    NimBLEDevice::init("Micromouse26");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    auto* srv = NimBLEDevice::createServer();
    srv->setCallbacks(new BLECb());
    auto* svc = srv->createService(NUS_SERVICE_UUID);
    pTX = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    svc->createCharacteristic(NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    svc->start();
    NimBLEDevice::getAdvertising()->addServiceUUID(NUS_SERVICE_UUID);
    NimBLEDevice::startAdvertising();
}

// ── Buzzer ────────────────────────────────────────────────────────────────────
void beep(int ms) {
    ledcWrite(BUZZER_LEDC_CH, BUZZER_DUTY);
    delay(ms);
    ledcWrite(BUZZER_LEDC_CH, 0);
}
void beepDone()  { beep(60); delay(80); beep(60); }
void beepGoal()  { for (int i=0; i<3; i++) { beep(120); delay(100); } }
void beepError() { beep(500); }

// ── Button ────────────────────────────────────────────────────────────────────
bool buttonPressed() {
    static bool last = HIGH;
    bool now = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && now == LOW);
    last = now;
    return edge;
}

void waitButton() {
    while (!buttonPressed()) delay(10);
    delay(30); // debounce
}

// ── Robot position ────────────────────────────────────────────────────────────
uint8_t robotRow = 0;
uint8_t robotCol = 0;
AbsDir  heading  = DIR_NORTH;

void updatePosition(AbsDir moveDir) {
    robotRow = (uint8_t)(robotRow + DIR_DR[moveDir]);
    robotCol = (uint8_t)(robotCol + DIR_DC[moveDir]);
}

// ── Stop motors ───────────────────────────────────────────────────────────────
void stopMotors() {
    leftMotor.brake();
    rightMotor.brake();
    delay(80);
    leftMotor.coast();
    rightMotor.coast();
}

// ── moveForwardOneCell() ──────────────────────────────────────────────────────
// Steering priority:
//   Both 45° sensors see walls → IR lateral centering
//   One or no walls            → gyro heading hold
// No Serial inside loop.
void moveForwardOneCell() {
    leftEnc.reset();
    rightEnc.reset();
    imuResetYaw();
    resetPIDs();

    // Speed in ticks/sec: CRUISE_PWM maps to roughly CRUISE_PWM ticks/s at this motor
    // PID handles the actual conversion — just use PWM as a proxy target
    const float CRUISE_SPEED = (float)CRUISE_PWM;
    unsigned long startMs = millis();
    unsigned long lastMs  = millis();

    // LED feedback: 2=left motor, 3=right motor
    leds[2] = CRGB(0, 40, 0);
    leds[3] = CRGB(0, 40, 0);
    FastLED.show();

    while (true) {
        unsigned long now = millis();
        float dt = (now - lastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        lastMs = now;

        imuUpdate();
        ir.update();

        long tL  = leftEnc.getTicks();
        long tR  = rightEnc.getTicks();
        long avg = (tL + tR) / 2;

        // Time-based trapezoidal ramp
        float elapsed = (float)(now - startMs);
        float v_fwd;
        if (elapsed < (float)RAMP_MS) {
            v_fwd = CRUISE_SPEED * (elapsed / (float)RAMP_MS);
        } else if (avg < (long)(TICKS_PER_CELL * 0.80f)) {
            v_fwd = CRUISE_SPEED;
        } else {
            float rem = (float)(TICKS_PER_CELL - avg) / (float)(TICKS_PER_CELL * 0.20f);
            v_fwd = CRUISE_SPEED * constrain(rem, 0.15f, 1.0f);
        }

        // Steering
        float nL = irCal[IR_LEFT_45].normalize(ir.raw[IR_LEFT_45]);
        float nR = irCal[IR_RIGHT_45].normalize(ir.raw[IR_RIGHT_45]);
        bool  wL = nL > 0.3f;
        bool  wR = nR > 0.3f;

        float correction;
        if (wL && wR) {
            // Both walls: center between them
            correction = Kp_lat * (nL - nR);
        } else if (wL) {
            // Only left wall: maintain half-normalized distance
            correction = Kp_lat * (nL - 0.5f);
        } else if (wR) {
            // Only right wall
            correction = Kp_lat * (0.5f - nR);
        } else {
            // No walls: gyro heading hold
            correction = Kp_yaw * yawDeg;
        }

        float vL_cmd = v_fwd - correction;
        float vR_cmd = v_fwd + correction;

        int pwmL = speedPID(pidL, tL, vL_cmd, dt);
        int pwmR = speedPID(pidR, tR, vR_cmd, dt);

        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        // Motor activity LEDs (no Serial)
        leds[2] = CRGB(0, (uint8_t)(pwmL / 4), 0);
        leds[3] = CRGB(0, (uint8_t)(pwmR / 4), 0);

        if (avg >= TICKS_PER_CELL) {
            stopMotors();
            leds[2] = CRGB::Black;
            leds[3] = CRGB::Black;
            FastLED.show();
            return;
        }

        if (millis() - startMs > MOVE_TIMEOUT_MS) {
            stopMotors();
            leds[2] = CRGB::Red;
            leds[3] = CRGB::Red;
            FastLED.show();
            beepError();
            return;
        }
    }
}

// ── turnRight90() ─────────────────────────────────────────────────────────────
void turnRight90() {
    imuResetYaw();
    leftMotor.drive(TURN_PWM);
    rightMotor.drive(-TURN_PWM);
    unsigned long startMs = millis();

    while (true) {
        imuUpdate();
        float a = yawDeg;   // negative = right turn (robot turns clockwise)
        float absA = fabsf(a);

        if (absA >= TURN_SLOW_DEG) {
            // Slow zone
            leftMotor.drive(TURN_PWM / 2);
            rightMotor.drive(-TURN_PWM / 2);
        }
        if (absA >= TURN_STOP_DEG) {
            stopMotors();
            heading = (AbsDir)((heading + 1) % 4);
            return;
        }
        if (millis() - startMs > TURN_TIMEOUT_MS) {
            stopMotors();
            heading = (AbsDir)((heading + 1) % 4);  // assume completed
            beepError();
            return;
        }
    }
}

// ── turnLeft90() ─────────────────────────────────────────────────────────────
void turnLeft90() {
    imuResetYaw();
    leftMotor.drive(-TURN_PWM);
    rightMotor.drive(TURN_PWM);
    unsigned long startMs = millis();

    while (true) {
        imuUpdate();
        float absA = fabsf(yawDeg);

        if (absA >= TURN_SLOW_DEG) {
            leftMotor.drive(-TURN_PWM / 2);
            rightMotor.drive(TURN_PWM / 2);
        }
        if (absA >= TURN_STOP_DEG) {
            stopMotors();
            heading = (AbsDir)((heading + 3) % 4);
            return;
        }
        if (millis() - startMs > TURN_TIMEOUT_MS) {
            stopMotors();
            heading = (AbsDir)((heading + 3) % 4);
            beepError();
            return;
        }
    }
}

void turnAround() {
    turnRight90();
    delay(100);
    turnRight90();
}

// ── senseWalls() ─────────────────────────────────────────────────────────────
// Maps sensor readings to absolute wall directions based on current heading.
// 45° sensors approximate the left/right walls.
void senseWalls() {
    ir.update();

    AbsDir frontDir = heading;
    AbsDir leftDir  = (AbsDir)((heading + 3) % 4);
    AbsDir rightDir = (AbsDir)((heading + 1) % 4);

    // Front wall
    maze.setWall(robotRow, robotCol, frontDir,
                 ir.wallFront(irCal));

    // Left/right walls via 45° sensors
    maze.setWall(robotRow, robotCol, leftDir,
                 ir.wallLeft(irCal));
    maze.setWall(robotRow, robotCol, rightDir,
                 ir.wallRight(irCal));

    // Behind us we came from — no wall (we just entered from there)
    AbsDir backDir = (AbsDir)((heading + 2) % 4);
    maze.setWall(robotRow, robotCol, backDir, false);
}

// ── IR calibration ────────────────────────────────────────────────────────────
void runIrCalibration() {
    // STEP 1: open air — hold robot away from all walls
    leds[7] = CRGB::Yellow;
    FastLED.show();
    bleSend("[CAL] Step 1: hold robot in OPEN AIR (no walls nearby)\n");
    bleSend("[CAL] Press button when ready...\n");
    waitButton();
    beep(60);

    for (int s = 0; s < IR_COUNT; s++) {
        irCal[s].noWall = ir.sampleAvg(s, 64);
    }
    bleSend("[CAL] noWall: LF=%d L45=%d R45=%d RF=%d\n",
            irCal[0].noWall, irCal[1].noWall, irCal[2].noWall, irCal[3].noWall);

    // STEP 2: dead-end — robot centered, front+both sides walled
    bleSend("[CAL] Step 2: place robot CENTERED in dead-end (3 walls)\n");
    bleSend("[CAL] Press button when ready...\n");
    waitButton();
    beep(60);

    for (int s = 0; s < IR_COUNT; s++) {
        irCal[s].wall = ir.sampleAvg(s, 64);
        irCal[s].threshold = (irCal[s].noWall + irCal[s].wall) / 2;
    }
    bleSend("[CAL] wall:   LF=%d L45=%d R45=%d RF=%d\n",
            irCal[0].wall, irCal[1].wall, irCal[2].wall, irCal[3].wall);
    bleSend("[CAL] thresh: LF=%d L45=%d R45=%d RF=%d\n",
            irCal[0].threshold, irCal[1].threshold,
            irCal[2].threshold, irCal[3].threshold);

    beepDone();
    bleSend("[CAL] done — entering READY\n");
}

// ── FSM ───────────────────────────────────────────────────────────────────────
enum State {
    STATE_IDLE,
    STATE_CALIBRATE,
    STATE_READY,
    STATE_EXPLORE,
    STATE_GOAL_REACHED,
    STATE_STOP
};
State state = STATE_IDLE;

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("[INIT] Micromouse26 boot");

    // Buzzer first — confirms MCU alive
    ledcSetup(BUZZER_LEDC_CH, BUZZER_FREQ, BUZZER_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH);
    ledcWrite(BUZZER_LEDC_CH, 0);
    beep(80);

    // Button
    pinMode(BUTTON_1, INPUT_PULLUP);

    // DRV8833 wake
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    // Motors + encoders
    leftMotor.begin();
    rightMotor.begin();
    leftEnc.begin(leftISR);
    rightEnc.begin(rightISR);

    // IR
    ir.begin();

    // LEDs
    FastLED.addLeds<WS2812B, WS2812_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHT);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    // IMU — calibrate before BLE so Serial is the output
    Serial.println("[INIT] IMU calibrating — keep still...");
    imuCalibrate();

    // BLE
    bleSetup();

    // Maze — 3x6 draft, goal top-right
    maze.reset();
    maze.setGoalSingle(5, 2);
    // Start cell (0,0) facing north: south+west walls already set by reset()
    // Add east wall of start cell (competition rule: start bounded 3 sides)
    maze.setWall(0, 0, DIR_EAST, true);

    Serial.printf("[INIT] TICKS_PER_CELL=%ld  MM_PER_TICK=%.4f\n",
                  TICKS_PER_CELL, MM_PER_TICK);

    beepDone();
    Serial.println("[INIT] ready — press button to calibrate IR");
    state = STATE_IDLE;
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // Battery reading (used for LED 6)
    int   batRaw = analogRead(BAT_V_SENSE);
    float vBat   = (batRaw / 4095.0f) * 3.3f * BAT_VDIV_MULT;

    switch (state) {

    // ── IDLE ──────────────────────────────────────────────────────────────────
    case STATE_IDLE:
        leds[7] = CRGB::Black;
        ledsUpdate(vBat);
        if (buttonPressed()) {
            state = STATE_CALIBRATE;
        }
        break;

    // ── CALIBRATE ─────────────────────────────────────────────────────────────
    case STATE_CALIBRATE:
        leds[7] = CRGB::Yellow;
        FastLED.show();
        runIrCalibration();
        // Reset robot position and maze for new run
        robotRow = 0;
        robotCol = 0;
        heading  = DIR_NORTH;
        maze.reset();
        maze.setGoalSingle(5, 2);
        maze.setWall(0, 0, DIR_EAST, true);
        maze.floodFill();
        state = STATE_READY;
        break;

    // ── READY ─────────────────────────────────────────────────────────────────
    // Wait for finger to block LF sensor → start run
    case STATE_READY: {
        leds[7] = CRGB(0, 200, 200);  // cyan
        ledsUpdate(vBat);
        bleSend("[READY] wave hand in front of LF sensor to start\n");

        // Poll for finger trigger
        static int fingerCount = 0;
        ir.update();
        float nLF = irCal[IR_LEFT_FRONT].normalize(ir.raw[IR_LEFT_FRONT]);
        if (nLF > FINGER_THRESH) {
            fingerCount++;
        } else {
            fingerCount = 0;
        }
        if (fingerCount >= FINGER_CONFIRM) {
            fingerCount = 0;
            bleSend("[READY] start trigger detected — starting in 1s\n");
            delay(1000);  // let hand move away
            beep(80);
            state = STATE_EXPLORE;
        }
        delay(50);
        break;
    }

    // ── EXPLORE ───────────────────────────────────────────────────────────────
    case STATE_EXPLORE: {
        leds[7] = CRGB(0, 60, 0);  // dim green
        FastLED.show();

        // 1. Mark current cell visited
        maze.visited[robotRow][robotCol] = true;

        // 2. Check goal
        if (maze.isGoal(robotRow, robotCol)) {
            state = STATE_GOAL_REACHED;
            break;
        }

        // 3. Sense walls and update map
        senseWalls();

        // 4. Re-flood
        maze.floodFill();

        // 5. Choose best direction
        uint8_t bestDist;
        AbsDir  nextDir = maze.bestDirection(robotRow, robotCol, bestDist);

        if (bestDist == FLOOD_INFINITY) {
            // Trapped — no reachable path
            bleSend("[EXPLORE] TRAPPED at (%d,%d) — no path\n", robotRow, robotCol);
            state = STATE_STOP;
            break;
        }

        // 6. Turn to face nextDir
        int turnSteps = ((int)nextDir - (int)heading + 4) % 4;
        switch (turnSteps) {
            case 1: turnRight90();  break;
            case 2: turnAround();   break;
            case 3: turnLeft90();   break;
            default: break;  // 0 = already facing right
        }

        // 7. Move forward one cell
        moveForwardOneCell();
        delay(50);  // brief settle between cells

        // 8. Update position
        updatePosition(nextDir);
        break;
    }

    // ── GOAL_REACHED ──────────────────────────────────────────────────────────
    case STATE_GOAL_REACHED:
        fill_solid(leds, NUM_LEDS, CRGB::White);
        FastLED.show();
        beepGoal();
        bleSend("[GOAL] reached (%d,%d)!\n", robotRow, robotCol);
        // Dump flood map over BLE
        for (int r = 5; r >= 0; r--) {
            bleSend("%d: ", r);
            for (int c = 0; c < 3; c++) {
                if (maze.flood[r][c] == FLOOD_INFINITY)
                    bleSend(" ?? ");
                else
                    bleSend("%3d ", maze.flood[r][c]);
            }
            bleSend("\n");
        }
        state = STATE_STOP;
        break;

    // ── STOP ──────────────────────────────────────────────────────────────────
    case STATE_STOP:
        stopMotors();
        leds[7] = CRGB::Red;
        ledsUpdate(vBat);
        if (buttonPressed()) {
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
            state = STATE_IDLE;
        }
        delay(100);
        break;
    }
}
