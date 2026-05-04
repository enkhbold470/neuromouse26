// =============================================================================
// main.cpp — Micromouse26 full firmware
//
// FSM:
//   IDLE      → (button)         → CALIBRATE
//   CALIBRATE → (cal done)       → STANDBY
//   STANDBY   → (button)         → ARMED        ← place robot at (0,0) first
//   ARMED     → (3 waves on LF)  → 1s delay → EXPLORE
//   ARMED     → (button)         → STANDBY       ← cancel / re-place
//   EXPLORE   → (goal reached)   → GOAL_REACHED → STOP
//   EXPLORE   → (trapped/timeout)→ STOP
//   STOP      → (button)         → STANDBY       ← irCal preserved, maze reset
//
// Motor control: encoder-only speed PID + encoder balance for straight driving.
// No IMU used for drive — gyro available for future turn accuracy.
//
// IR calibration survives across runs. Only re-run explicitly from IDLE.
//
// LED map:
//   0 = LF sensor    (green=open → red=wall)
//   1 = L45 sensor   (green=open → red=wall)
//   2 = left motor   (brightness = PWM%)
//   3 = right motor  (brightness = PWM%)
//   4 = R45 sensor   (green=open → red=wall)
//   5 = RF sensor    (green=open → red=wall)
//   6 = battery      (blue=full → red=low)
//   7 = state        (IDLE=off CALIBRATE=yellow STANDBY=cyan ARMED=white EXPLORE=green GOAL=white STOP=red)
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

// ── Motor tuning ──────────────────────────────────────────────────────────────
#define CRUISE_SPEED   200.0f  // ticks/sec
#define RAMP_TIME_MS   150.0f  // ms ramp 0 → cruise
#define Kp_speed       1.5f
#define Ki_speed       0.5f
#define Kp_balance     3.0f    // ticks/sec per tick of L-R encoder error
#define MIN_PWM_OUT    150     // PWM floor during accel (disabled during decel)
#define TURN_PWM       300     // spin PWM for 90° turns

// ── Turn thresholds ───────────────────────────────────────────────────────────
#define TURN_SLOW_DEG  75.0f
#define TURN_STOP_DEG  88.0f
#define TURN_TIMEOUT_MS 3000

// ── Physics ───────────────────────────────────────────────────────────────────
#define CELL_MM        180.0f
#define MM_PER_TICK    ((float)(M_PI * WHEEL_DIAMETER) / TICKS_PER_REV)
#define TICKS_PER_CELL ((long)(CELL_MM / MM_PER_TICK))

// ── Wave trigger ──────────────────────────────────────────────────────────────
#define FINGER_THRESH  0.5f
#define WAVES_NEEDED   3

// ── Hardware objects ──────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "L");
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "R");
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);
MicromouseIR      ir;
MicromouseMaze    maze;

// ── IR calibration ────────────────────────────────────────────────────────────
IrCal irCal[IR_COUNT];  // persists across runs

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

// ── Encoder ISR management ────────────────────────────────────────────────────
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

void resetPIDs() {
    pidL = SpeedPID();
    pidR = SpeedPID();
}

// applyFloor: true during accel/cruise, false during decel
int computeSpeedPID(SpeedPID& pid, long ticks, float target, float dt, bool applyFloor) {
    float rawSpeed  = (ticks - pid.prevTicks) / dt;
    pid.prevSpeed   = 0.7f * pid.prevSpeed + 0.3f * rawSpeed;
    pid.prevTicks   = ticks;
    float error     = target - pid.prevSpeed;
    pid.integral   += error * dt;
    pid.integral    = constrain(pid.integral, -800.0f, 800.0f);
    float output    = Kp_speed * error + Ki_speed * pid.integral;
    if (applyFloor) {
        if (target > 0 && output < MIN_PWM_OUT) output = MIN_PWM_OUT;
        if (target < 0 && output > -MIN_PWM_OUT) output = -MIN_PWM_OUT;
    }
    return (int)constrain(output, -1023.0f, 1023.0f);
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
    bool now  = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && now == LOW);
    last = now;
    return edge;
}

void waitButton() {
    while (!buttonPressed()) delay(10);
    delay(30);
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

// ── resetRunState() ───────────────────────────────────────────────────────────
void resetRunState() {
    robotRow = 0;
    robotCol = 0;
    heading  = DIR_NORTH;
    maze.reset();
    maze.setGoalSingle(5, 2);
    maze.floodFill();
    bleSend("[RUN] maze reset, pos=(0,0) N\n");
}

// ── moveForwardOneCell() ──────────────────────────────────────────────────────
void moveForwardOneCell() {
    long target = TICKS_PER_CELL;

    encodersEnable();
    resetPIDs();

    leds[2] = CRGB(0, 40, 0);
    leds[3] = CRGB(0, 40, 0);
    FastLED.show();

    unsigned long startMs = millis();
    unsigned long lastMs  = millis();

    while (true) {
        unsigned long now = millis();
        float dt = (now - lastMs) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        lastMs = now;

        long tL  = leftEnc.getTicks();
        long tR  = rightEnc.getTicks();
        long avg = (tL + tR) / 2;

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
            float rem = (float)(target - avg) / (target * 0.45f);
            v_fwd   = CRUISE_SPEED * constrain(rem, 0.0f, 1.0f);
            inDecel = true;
        }

        float balance = Kp_balance * (float)(tL - tR);
        float vL_cmd  = v_fwd - balance;
        float vR_cmd  = v_fwd + balance;

        int pwmL = computeSpeedPID(pidL, tL, vL_cmd, dt, !inDecel);
        int pwmR = computeSpeedPID(pidR, tR, vR_cmd, dt, !inDecel);

        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        leds[2] = CRGB(0, (uint8_t)(pwmL / 4), 0);
        leds[3] = CRGB(0, (uint8_t)(pwmR / 4), 0);

        if (avg >= target - BRAKE_COMP_TICKS) {
            stopMotors();
            encodersDisable();
            leds[2] = CRGB::Black;
            leds[3] = CRGB::Black;
            FastLED.show();
            return;
        }

        if (millis() - startMs > 12000UL) {
            stopMotors();
            encodersDisable();
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
        float absA = fabsf(yawDeg);
        if (absA >= TURN_SLOW_DEG) {
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
            heading = (AbsDir)((heading + 1) % 4);
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
void senseWalls() {
    ir.update();
    AbsDir frontDir = heading;
    AbsDir leftDir  = (AbsDir)((heading + 3) % 4);
    AbsDir rightDir = (AbsDir)((heading + 1) % 4);
    AbsDir backDir  = (AbsDir)((heading + 2) % 4);
    maze.setWall(robotRow, robotCol, frontDir, ir.wallFront(irCal));
    maze.setWall(robotRow, robotCol, leftDir,  ir.wallLeft(irCal));
    maze.setWall(robotRow, robotCol, rightDir, ir.wallRight(irCal));
    maze.setWall(robotRow, robotCol, backDir,  false);
}

// ── IR calibration ────────────────────────────────────────────────────────────
void runIrCalibration() {
    leds[7] = CRGB::Yellow;
    FastLED.show();

    bleSend("[CAL] Step 1: hold robot in OPEN AIR\n");
    bleSend("[CAL] Press button when ready...\n");
    waitButton();
    beep(60);
    for (int s = 0; s < IR_COUNT; s++)
        irCal[s].noWall = ir.sampleAvg(s, 64);
    bleSend("[CAL] noWall: LF=%d L45=%d R45=%d RF=%d\n",
            irCal[0].noWall, irCal[1].noWall, irCal[2].noWall, irCal[3].noWall);

    bleSend("[CAL] Step 2: place robot in dead-end (3 walls)\n");
    bleSend("[CAL] Press button when ready...\n");
    waitButton();
    beep(60);
    for (int s = 0; s < IR_COUNT; s++) {
        irCal[s].wall      = ir.sampleAvg(s, 64);
        irCal[s].threshold = (irCal[s].noWall + irCal[s].wall) / 2;
    }
    bleSend("[CAL] wall:   LF=%d L45=%d R45=%d RF=%d\n",
            irCal[0].wall, irCal[1].wall, irCal[2].wall, irCal[3].wall);
    bleSend("[CAL] thresh: LF=%d L45=%d R45=%d RF=%d\n",
            irCal[0].threshold, irCal[1].threshold,
            irCal[2].threshold, irCal[3].threshold);

    beepDone();
    bleSend("[CAL] done\n");
}

// ── FSM ───────────────────────────────────────────────────────────────────────
enum State {
    STATE_IDLE,
    STATE_CALIBRATE,
    STATE_STANDBY,
    STATE_ARMED,
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

    ledcSetup(BUZZER_LEDC_CH, BUZZER_FREQ, BUZZER_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH);
    ledcWrite(BUZZER_LEDC_CH, 0);
    beep(80);

    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    leftMotor.begin();
    rightMotor.begin();

    // Encoder pins ready but ISRs not attached until move starts
    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);

    ir.begin();

    FastLED.addLeds<WS2812B, WS2812_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHT);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    Serial.println("[INIT] IMU calibrating — keep still...");
    imuCalibrate();

    bleSetup();

    maze.reset();
    maze.setGoalSingle(5, 2);

    Serial.printf("[INIT] TICKS_PER_CELL=%ld  MM_PER_TICK=%.4f\n",
                  TICKS_PER_CELL, MM_PER_TICK);
    Serial.printf("[INIT] CRUISE=%.0f t/s  MIN_PWM=%d  BRAKE_COMP=%d\n",
                  (float)CRUISE_SPEED, MIN_PWM_OUT, BRAKE_COMP_TICKS);

    beepDone();
    Serial.println("[INIT] ready — press button to calibrate IR");
    state = STATE_IDLE;
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    int   batRaw = analogRead(BAT_V_SENSE);
    float vBat   = (batRaw / 4095.0f) * 3.3f * BAT_VDIV_MULT;

    switch (state) {

    case STATE_IDLE:
        leds[7] = CRGB::Black;
        ledsUpdate(vBat);
        if (buttonPressed()) state = STATE_CALIBRATE;
        break;

    case STATE_CALIBRATE:
        runIrCalibration();
        state = STATE_STANDBY;
        break;

    case STATE_STANDBY:
        leds[7] = CRGB(0, 200, 200);  // cyan
        ledsUpdate(vBat);
        bleSend("[STANDBY] place robot at (0,0), press button to arm\n");
        if (buttonPressed()) {
            resetRunState();
            beep(80);
            bleSend("[ARMED] wave 3x past LF sensor to start\n");
            state = STATE_ARMED;
        }
        delay(50);
        break;

    case STATE_ARMED: {
        leds[7] = CRGB::White;
        ledsUpdate(vBat);

        static int  waveCount   = 0;
        static bool handPresent = false;

        ir.update();
        float nLF = irCal[IR_LEFT_FRONT].normalize(ir.raw[IR_LEFT_FRONT]);

        if (!handPresent && nLF > FINGER_THRESH) {
            handPresent = true;
        } else if (handPresent && nLF <= FINGER_THRESH) {
            handPresent = false;
            waveCount++;
            bleSend("[ARMED] wave %d/%d\n", waveCount, WAVES_NEEDED);
            beep(30);
        }

        if (waveCount >= WAVES_NEEDED) {
            waveCount   = 0;
            handPresent = false;
            bleSend("[ARMED] starting in 1s\n");
            delay(1000);
            beep(80);
            state = STATE_EXPLORE;
            break;
        }

        if (buttonPressed()) {
            waveCount   = 0;
            handPresent = false;
            bleSend("[ARMED] cancelled\n");
            state = STATE_STANDBY;
        }
        delay(20);
        break;
    }

    case STATE_EXPLORE: {
        leds[7] = CRGB(0, 60, 0);
        FastLED.show();

        maze.visited[robotRow][robotCol] = true;

        if (maze.isGoal(robotRow, robotCol)) {
            state = STATE_GOAL_REACHED;
            break;
        }

        senseWalls();
        maze.floodFill();

        uint8_t bestDist;
        AbsDir  nextDir = maze.bestDirection(robotRow, robotCol, bestDist);

        if (bestDist == FLOOD_INFINITY) {
            bleSend("[EXPLORE] TRAPPED at (%d,%d)\n", robotRow, robotCol);
            state = STATE_STOP;
            break;
        }

        int turnSteps = ((int)nextDir - (int)heading + 4) % 4;
        switch (turnSteps) {
            case 1: turnRight90(); break;
            case 2: turnAround();  break;
            case 3: turnLeft90();  break;
            default: break;
        }

        moveForwardOneCell();
        delay(50);
        updatePosition(nextDir);
        break;
    }

    case STATE_GOAL_REACHED:
        fill_solid(leds, NUM_LEDS, CRGB::White);
        FastLED.show();
        beepGoal();
        bleSend("[GOAL] reached (%d,%d)!\n", robotRow, robotCol);
        for (int r = 5; r >= 0; r--) {
            bleSend("%d: ", r);
            for (int c = 0; c < 3; c++) {
                if (maze.flood[r][c] == FLOOD_INFINITY) bleSend(" ?? ");
                else bleSend("%3d ", maze.flood[r][c]);
            }
            bleSend("\n");
        }
        state = STATE_STOP;
        break;

    case STATE_STOP:
        stopMotors();
        leds[7] = CRGB::Red;
        ledsUpdate(vBat);
        if (buttonPressed()) {
            fill_solid(leds, NUM_LEDS, CRGB::Black);
            FastLED.show();
            bleSend("[STOP] back to STANDBY (irCal kept)\n");
            state = STATE_STANDBY;
        }
        delay(100);
        break;
    }
}
