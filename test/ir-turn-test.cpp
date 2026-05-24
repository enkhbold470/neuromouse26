// ir-turn-test.cpp — Clean stop-and-sense navigation pipeline
//
// Sensor roles (standard micromouse approach):
//   Encoders → cell-boundary stop (TICKS_PER_CELL)   — NOT IR
//   IR       → wall topology AFTER stopping (8 averaged samples)
//   Gyro     → turn execution only (trapezoid+KFF+PD, same as main.cpp)
//
// Decision (right-hand wall follower):
//   right open  → turn right
//   front open  → straight
//   left open   → turn left
//   all blocked → U-turn
//
// NOTE: place robot at cell CENTER to start (not against south wall).
// First-cell offset not applied here — this is a turn-validation test.
//
// Flash: pio run -e ir-turn-test -t upload

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseMaze.h"

// ── Hardware ──────────────────────────────────────────────────────────────────
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
static MicromouseMotor motorL(MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
static MicromouseMotor motorR(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
static MicromouseEncoder encL(ENC_L_A, ENC_L_B);
static MicromouseEncoder encR(ENC_R_A, ENC_R_B);

static inline long rTicks() { return (long)(encR.getTicks() * RIGHT_ENC_SCALE); }
static void stopMotors() { motorL.brake(); motorR.brake(); }

// ── IR ────────────────────────────────────────────────────────────────────────
struct IRPair { uint8_t emit, rx; };
static const IRPair PAIRS[4] = {
    { EMIT_LF,  RX_LF  },
    { EMIT_L45, RX_L45 },
    { EMIT_R45, RX_R45 },
    { EMIT_RF,  RX_RF  },
};
static int irVal[4];

static int readIR(const IRPair& p) {
    digitalWrite(p.emit, LOW);  delayMicroseconds(80);
    int amb = analogRead(p.rx);
    digitalWrite(p.emit, HIGH); delayMicroseconds(80);
    int lit = analogRead(p.rx);
    digitalWrite(p.emit, LOW);
    int d = lit - amb;
    return d < 0 ? 0 : d;
}
static void sampleIR() { for (int i = 0; i < 4; i++) irVal[i] = readIR(PAIRS[i]); }

// ── Gyro (MPU-6500) ───────────────────────────────────────────────────────────
#define MPU_ADDR         0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_GYRO_CFG     0x1B
#define REG_ACCEL_XOUT_H 0x3B
#define GYRO_FS_SEL      0x10
#define GYRO_SCALE       32.8f

struct ImuRaw { int16_t ax, ay, az, temp, gx, gy, gz; };

static float         gyroBiasZ = 0.0f;
static float         yaw       = 0.0f;
static unsigned long lastImuUs = 0;

static bool mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}
static bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }
    return true;
}
static int16_t imu16(uint8_t hi, uint8_t lo) { return (int16_t)((hi << 8) | lo); }

static bool imuReadAll(ImuRaw& d) {
    uint8_t b[14];
    if (!mpuRead(REG_ACCEL_XOUT_H, b, 14)) return false;
    d.ax=imu16(b[0],b[1]); d.ay=imu16(b[2],b[3]); d.az=imu16(b[4],b[5]);
    d.temp=imu16(b[6],b[7]);
    d.gx=imu16(b[8],b[9]); d.gy=imu16(b[10],b[11]); d.gz=imu16(b[12],b[13]);
    return true;
}
static void updateYaw() {
    ImuRaw d;
    if (!imuReadAll(d)) return;
    unsigned long now = micros();
    float dt = (lastImuUs == 0) ? 0.001f : (now - lastImuUs) / 1e6f;
    if (dt > 0.05f) dt = 0.05f;
    lastImuUs = now;
    float gz = d.gz / GYRO_SCALE - gyroBiasZ;
    if (fabsf(gz) < 0.05f) gz = 0;
    yaw += gz * dt;
}

static float readVbat() {
    return (analogRead(BAT_V_SENSE) / 4095.0f) * 3.3f * BAT_VDIV_MULT;
}

// ── Turn profile (trapezoidal ω) ──────────────────────────────────────────────
struct TurnProfile {
    float accel, peak, target;
    float t_acc, t_cru, t_tot;
    float d_acc, d_cru;
    int   sign;

    void init(float targetDeg) {
        sign   = (targetDeg >= 0) ? 1 : -1;
        target = fabsf(targetDeg);
        accel  = TURN_ACCEL_DPS2;
        peak   = TURN_PEAK_OMEGA_DPS;
        float d_full_acc = (peak * peak) / (2.0f * accel);
        if (2.0f * d_full_acc >= target) {
            peak  = sqrtf(accel * target);
            d_acc = target * 0.5f;
            d_cru = 0;
        } else {
            d_acc = d_full_acc;
            d_cru = target - 2.0f * d_full_acc;
        }
        t_acc = peak / accel;
        t_cru = (peak > 0) ? d_cru / peak : 0;
        t_tot = 2.0f * t_acc + t_cru;
    }

    void at(float t, float& angle_des, float& omega_des) const {
        float a, w;
        if (t < t_acc) {
            w = accel * t;
            a = 0.5f * accel * t * t;
        } else if (t < t_acc + t_cru) {
            w = peak;
            a = d_acc + peak * (t - t_acc);
        } else if (t < t_tot) {
            float tp = t - t_acc - t_cru;
            w = peak - accel * tp;
            a = d_acc + d_cru + peak * tp - 0.5f * accel * tp * tp;
        } else {
            w = 0; a = target;
        }
        angle_des = sign * a;
        omega_des = sign * w;
    }
};

// ── doTurn — exact copy of main.cpp trapezoid+KFF+PD ─────────────────────────
static bool doTurn(float targetDeg) {
    yaw = 0;
    lastImuUs = micros();
    TurnProfile prof; prof.init(targetDeg);

    float vbat   = readVbat();
    float vScale = (vbat > 5.5f) ? (NOMINAL_VBAT / vbat) : 1.0f;

    unsigned long t0          = millis();
    unsigned long lastUs      = micros();
    unsigned long inBandStart = 0;
    float prevErr = 0, peakErr = 0;
    bool  converged = false;

    while (true) {
        updateYaw();
        unsigned long nowMs = millis();
        unsigned long nowUs = micros();
        float t  = (nowMs - t0) / 1000.0f;
        float dt = (nowUs - lastUs) / 1.0e6f;
        if (dt < 1.0e-4f) continue;
        lastUs = nowUs;

        float angle_des, omega_des;
        prof.at(t, angle_des, omega_des);

        float err  = angle_des - yaw;
        float derr = (err - prevErr) / dt;
        prevErr    = err;

        float pwm = TURN_KFF_PWM_PER_DPS * omega_des * vScale
                  + TURN_KP_PWM_PER_DEG  * err
                  + TURN_KD_PWM_PER_DPS  * derr;

        bool holding = (t >= prof.t_tot);
        if (holding && fabsf(pwm) > 1.0f && fabsf(pwm) < TURN_MIN_HOLD_PWM)
            pwm = (pwm > 0) ? TURN_MIN_HOLD_PWM : -TURN_MIN_HOLD_PWM;
        if (pwm >  TURN_PWM) pwm =  TURN_PWM;
        if (pwm < -TURN_PWM) pwm = -TURN_PWM;

        motorL.drive(-(int)pwm);
        motorR.drive( (int)pwm);

        if (fabsf(err) > peakErr) peakErr = fabsf(err);

        if (holding && fabsf(targetDeg - yaw) <= TURN_DEADBAND_DEG) {
            if (inBandStart == 0) inBandStart = nowMs;
            if (nowMs - inBandStart >= TURN_HOLD_MS) { converged = true; break; }
        } else {
            inBandStart = 0;
        }
        if (nowMs - t0 > TURN_TIMEOUT_MS) break;
    }
    stopMotors();
    unsigned long s = millis();
    while (millis() - s < TURN_SETTLE_MS) updateYaw();

    bool ok = converged && fabsf(targetDeg - yaw) <= 2.0f * TURN_DEADBAND_DEG;
    Serial.printf("[TURN] tgt=%+.1f  fin=%+.2f  err=%+.2f  peak=%.1f  %s\n",
                  targetDeg, yaw, targetDeg - yaw, peakErr, ok ? "OK" : "FAIL");
    return ok;
}

// ── quickGyroRecal — re-zero bias before each turn ────────────────────────────
static void quickGyroRecal(unsigned ms = 80) {
    stopMotors();
    delay(120);  // chassis must fully stop before bias sample — old senseWalls() 180ms provided this
    unsigned long t0 = millis();
    float sum = 0; int n = 0;
    while (millis() - t0 < ms) {
        ImuRaw d;
        if (imuReadAll(d)) { sum += d.gz / GYRO_SCALE; n++; }
        delayMicroseconds(500);
    }
    if (n > 0) gyroBiasZ = sum / n;
    yaw = 0;
    lastImuUs = micros();
}

// ── Turn helpers ──────────────────────────────────────────────────────────────
static bool turnLeft()  { quickGyroRecal(); return doTurn(+90.0f); }
static bool turnRight() { quickGyroRecal(); return doTurn(-90.0f); }
static bool turnAround() {
    quickGyroRecal();
    bool ok1 = doTurn(-90.0f);
    float residual = -90.0f - yaw;  // first-turn error carried into second
    delay(80);
    quickGyroRecal();
    bool ok2 = doTurn(-90.0f + residual);
    return ok1 && ok2;
}

// ── squareToFrontWall — pivot to perpendicular using LF/RF differential ───────
// Called after any turn if front wall is in range. Zeros residual heading error
// that doTurn() leaves behind. Resets yaw=0 after converging.
static void squareToFrontWall(unsigned long timeoutMs = 1200) {
    constexpr float SQUARE_KP       = 0.30f;
    constexpr int   SQUARE_MAX_PWM  = 120;
    constexpr int   SQUARE_MIN_PWM  = 70;
    constexpr int   SQUARE_TOL      = 40;
    constexpr unsigned long HOLD_MS = 150;

    unsigned long t0 = millis(), inBand = 0;
    while (millis() - t0 < timeoutMs) {
        sampleIR();
        if (irVal[0] < WALL_FRONT_THRESH || irVal[3] < WALL_FRONT_THRESH) break;
        int diff = irVal[0] - irVal[3];  // +: LF closer → robot rotated left
        if (abs(diff) <= SQUARE_TOL) {
            if (inBand == 0) inBand = millis();
            if (millis() - inBand >= HOLD_MS) { stopMotors(); break; }
            stopMotors(); delay(10); continue;
        } else { inBand = 0; }
        int pwm = (int)constrain(SQUARE_KP * (float)diff,
                                 (float)-SQUARE_MAX_PWM, (float)SQUARE_MAX_PWM);
        if (pwm > 0 && pwm <  SQUARE_MIN_PWM) pwm =  SQUARE_MIN_PWM;
        if (pwm < 0 && pwm > -SQUARE_MIN_PWM) pwm = -SQUARE_MIN_PWM;
        constexpr float kV_avg = 0.5f * (KV_L + KV_R);
        motorL.drive( (int)( pwm * (kV_avg / KV_L)));
        motorR.drive((int)(-pwm * (kV_avg / KV_R)));
        delay(8);
    }
    stopMotors();
    yaw = 0; lastImuUs = micros();
    Serial.println("[SQR] done");
}

// ── Maze state ────────────────────────────────────────────────────────────────
static MicromouseMaze maze;
static int    robotRow     = 0;
static int    robotCol     = 0;
static AbsDir robotHeading = DIR_NORTH;

static void setupMaze() {
    maze.reset();  // full 16×16 + centre 2×2 goal
    maze.floodFill();
}

// ── Wall struct (used by driveCell return + oledUpdate) ──────────────────────
struct Walls { bool front, left, right, completed; };

// ── driveCell — drive one cell, sense walls during last 40%, return result ────
// No separate stop-and-sense step. Votes on walls from samples taken when
// sensor geometry is correct (avg > 60% of cell). Straight = restart immediately.
static constexpr int IR_CRASH_THRESH = 3200;

static Walls driveCell() {
    encL.reset(); encR.reset();
    yaw = 0; lastImuUs = micros();

    float intSpeed   = 0;
    float velL_ema   = 0, velR_ema = 0;
    long  prevL      = 0, prevR = 0;
    float vbat       = readVbat();
    float vScale     = (vbat > 5.5f) ? (NOMINAL_VBAT / vbat) : 1.0f;

    constexpr float YAW_KP  = 4.0f;
    constexpr int   YAW_MAX = 150;

    unsigned long nextUs = micros();
    unsigned long t0     = millis();
    long curL = 0, curR = 0;
    bool cellCompleted = false;

    while (true) {
        while ((long)(micros() - nextUs) < 0) {}
        nextUs += VPID_LOOP_US;

        updateYaw();
        sampleIR();

        curL = encL.getTicks();
        curR = rTicks();
        long avg = (curL + curR) / 2;

        if (avg >= TICKS_PER_CELL) { cellCompleted = true; break; }
        if (millis() - t0 > TIMEOUT_MS) { Serial.println("[DRIVE] timeout"); break; }
        if (irVal[0] > IR_CRASH_THRESH || irVal[3] > IR_CRASH_THRESH) {
            Serial.println("[DRIVE] crash brake");
            break;
        }

        float dt    = VPID_LOOP_US / 1000000.0f;
        float instL = ((curL - prevL) * MM_PER_TICK) / dt;
        float instR = ((curR - prevR) * MM_PER_TICK) / dt;
        prevL = curL; prevR = curR;
        velL_ema += VPID_EMA_ALPHA * (instL - velL_ema);
        velR_ema += VPID_EMA_ALPHA * (instR - velR_ema);

        float errSpeed = (float)CELL_TARGET_MMS - 0.5f * (velL_ema + velR_ema);
        intSpeed = constrain(intSpeed + errSpeed * dt, -VPID_INTEG_LIM, VPID_INTEG_LIM);
        int pidSpeed = (int)(VPID_LOOP_KP * errSpeed + VPID_LOOP_KI * intSpeed);

        bool hasL = irVal[1] > WALL_SIDE_THRESH;
        bool hasR = irVal[2] > WALL_SIDE_THRESH;

        // Straight PI: equalize encoder ticks so robot drives straight.
        // DISABLED in single-wall mode — IR correction intentionally needs unequal wheel
        // speeds to curve. pidStr gain=6 cap=500 would cancel all IR lateral correction.
        long straightErr = curL - curR;
        int pidStr = 0;
        if (hasL == hasR)  // both walls or no walls: go straight
            pidStr = (int)constrain(VPID_STRAIGHT_KP * (float)straightErr,
                                    (float)-VPID_STRAIGHT_MAX, (float)VPID_STRAIGHT_MAX);

        // Lateral: IR position control (never mixed with gyro — they oscillate).
        // Target: L≈1491, R≈1590 at center. posErr>0 → too close left → steer right.
        // Single-wall: pidStr off so lateral correction reaches motors uncontested.
        constexpr float IR_KP_DUAL   = 0.04f;
        constexpr float IR_KP_SINGLE = 0.05f;
        constexpr int   IR_DEADBAND  = 80;
        constexpr int   IR_MAX       = 120;
        int lateral = 0;

        if (hasL && hasR) {
            int posErr = (irVal[1] - IR_CAL_L45) - (irVal[2] - IR_CAL_R45);
            if (abs(posErr) > IR_DEADBAND)
                lateral = (int)constrain(IR_KP_DUAL * (float)posErr, (float)-IR_MAX, (float)IR_MAX);
        } else if (hasL) {
            int posErr = irVal[1] - IR_CAL_L45;
            if (abs(posErr) > IR_DEADBAND)
                lateral = (int)constrain(IR_KP_SINGLE * (float)posErr, (float)-IR_MAX, (float)IR_MAX);
        } else if (hasR) {
            int posErr = -(irVal[2] - IR_CAL_R45);
            if (abs(posErr) > IR_DEADBAND)
                lateral = (int)constrain(IR_KP_SINGLE * (float)posErr, (float)-IR_MAX, (float)IR_MAX);
        } else {
            // no walls: gyro holds heading, straight PI already active above
            if (fabsf(yaw) > 1.0f)
                lateral = (int)constrain(YAW_KP * yaw, (float)-YAW_MAX, (float)YAW_MAX);
        }

        // Per-wheel FF so both wheels reach the same target velocity.
        // kV_avg gives equal PWM → L (higher kV) runs faster → drifts right.
        int ffL  = (int)((KV_L > 0 ? (float)CELL_TARGET_MMS / KV_L : 0) * vScale);
        int ffR  = (int)((KV_R > 0 ? (float)CELL_TARGET_MMS / KV_R : 0) * vScale);
        int pwmL = constrain(ffL + pidSpeed - pidStr + L_PWM_BIAS + lateral, 0, MOTOR_PWM_MAX);
        int pwmR = constrain(ffR + pidSpeed + pidStr + R_PWM_BIAS - lateral, 0, MOTOR_PWM_MAX);

        motorL.drive(pwmL);
        motorR.drive(pwmR);
    }
    stopMotors();
    delay(30);  // brief settle for clean static read
    int sLF=0, sL=0, sR=0, sRF=0;
    for (int i = 0; i < 3; i++) {
        sLF += readIR(PAIRS[0]); sL += readIR(PAIRS[1]);
        sR  += readIR(PAIRS[2]); sRF+= readIR(PAIRS[3]);
    }
    Walls w = {
        (sLF/3 > WALL_FRONT_THRESH || sRF/3 > WALL_FRONT_THRESH),
        (sL/3  > WALL_SIDE_THRESH),
        (sR/3  > WALL_SIDE_THRESH),
        cellCompleted
    };
    Serial.printf("[DRIVE] L=%ld R=%ld avg=%ld yaw=%.1f | F=%d L=%d R=%d done=%d\n",
                  curL, curR, (curL + curR) / 2, yaw, w.front, w.left, w.right, w.completed);
    return w;
}

// ── OLED ──────────────────────────────────────────────────────────────────────
static int         gCell   = 0;
static const char* gStatus = "READY";

static void oledUpdate(const Walls& w) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_10x20_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "%s", gStatus);
    oled.drawStr(0, 20, buf);
    snprintf(buf, sizeof(buf), "(%d,%d) #%d", robotRow, robotCol, gCell);
    oled.drawStr(0, 44, buf);
    oled.sendBuffer();
}

static void oledIdle() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_10x20_tf);
    oled.drawStr(0, 20, "READY");
    oled.drawStr(0, 44, "press btn");
    oled.sendBuffer();
}

// ── Button edge detect ────────────────────────────────────────────────────────
static bool buttonEdge() {
    static bool last = HIGH;
    bool cur  = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && cur == LOW);
    last = cur;
    return edge;
}

// ── State ─────────────────────────────────────────────────────────────────────
static bool running = false;

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_1, INPUT_PULLUP);
    analogReadResolution(12);

    motorL.begin();
    motorR.begin();
    encL.begin();
    encR.begin();

    for (auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT);
        digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    mpuWrite(REG_PWR_MGMT_1, 0x00); delay(50);
    mpuWrite(REG_GYRO_CFG, GYRO_FS_SEL);
    lastImuUs = micros();

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 12, "CAL GYRO - still");
    oled.sendBuffer();
    quickGyroRecal(600);  // longer initial bias sample
    Serial.printf("[CAL] gyro bias=%.3f\n", gyroBiasZ);

    setupMaze();
    robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;

    oledIdle();
    Serial.println("[READY] press button to start/stop");
}

void loop() {
    if (buttonEdge()) {
        running = !running;
        if (!running) {
            stopMotors();
            gStatus = "STOP"; gCell = 0;
            Walls empty = {false, false, false};
            oledUpdate(empty);
            Serial.println("[STOP]");
            return;
        }
        Serial.println("[START]");
        gStatus = "GO"; gCell = 0;
        setupMaze();
        robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
        for (int n = 3; n >= 1; n--) {
            oled.clearBuffer();
            oled.setFont(u8g2_font_10x20_tf);
            char c[4]; snprintf(c, sizeof(c), "%d", n);
            oled.drawStr(56, 36, c);
            oled.sendBuffer();
            delay(700);
        }
    }

    if (!running) {
        static unsigned long lastIdle = 0;
        if (millis() - lastIdle > 500) { oledIdle(); lastIdle = millis(); }
        delay(10);
        return;
    }

    // ── One cell: drive+sense → update maze → flood fill → decide ────────────
    gCell++;
    gStatus = "DRIVE";
    Walls w = driveCell();

    if (!running) return;

    // Advance position only on full cell completion.
    // Crash brake / timeout = robot stopped mid-cell, position unchanged.
    if (w.completed) {
        robotRow += DIR_DR[robotHeading];
        robotCol += DIR_DC[robotHeading];
    }

    // Update maze walls — only ADD walls (set true), never erase.
    // Erasing would override setupMaze() boundary walls on a false sensor read.
    AbsDir frontDir = robotHeading;
    AbsDir rightDir = (AbsDir)((robotHeading + 1) % 4);
    AbsDir leftDir  = (AbsDir)((robotHeading + 3) % 4);
    if (w.front) maze.setWall(robotRow, robotCol, frontDir, true);
    if (w.right) maze.setWall(robotRow, robotCol, rightDir, true);
    if (w.left)  maze.setWall(robotRow, robotCol, leftDir,  true);
    maze.visited[robotRow][robotCol] = true;

    Serial.printf("[POS] (%d,%d) h=%d  F=%d L=%d R=%d  flood=%d\n",
                  robotRow, robotCol, robotHeading,
                  w.front, w.left, w.right, maze.flood[robotRow][robotCol]);
    oledUpdate(w);

    // Goal check
    if (maze.isGoal(robotRow, robotCol)) {
        Serial.println("[GOAL] reached!");
        running = false; stopMotors();
        gStatus = "GOAL"; oledUpdate(w);
        return;
    }

    // Flood fill with updated walls, pick best next direction
    maze.floodFill();
    uint8_t bestDist;
    AbsDir nextDir = maze.bestDirectionBiased(robotRow, robotCol, robotHeading, bestDist);

    if (bestDist == FLOOD_INFINITY) {
        Serial.println("[ERR] boxed in — no path to goal");
        running = false; stopMotors();
        gStatus = "ERR"; oledUpdate(w);
        return;
    }

    Serial.printf("[FLOOD] next=%d dist=%d\n", nextDir, bestDist);

    // Turn to face next direction, then drive
    int turnDiff = ((int)nextDir - (int)robotHeading + 4) % 4;
    bool turnOk = true;
    if (turnDiff == 0) {
        gStatus = "FWRD";
    } else if (turnDiff == 1) {
        gStatus = "RIGHT";
        Serial.println("[ACT] turn right");
        turnOk = turnRight();
    } else if (turnDiff == 3) {
        gStatus = "LEFT";
        Serial.println("[ACT] turn left");
        turnOk = turnLeft();
    } else {
        gStatus = "180";
        Serial.println("[ACT] U-turn");
        turnOk = turnAround();
    }
    robotHeading = nextDir;

    // Post-turn squaring: if front wall visible after turn, align perpendicular
    if (turnOk && turnDiff != 0) {
        sampleIR();
        if (irVal[0] > WALL_FRONT_THRESH && irVal[3] > WALL_FRONT_THRESH)
            squareToFrontWall();
    }

    if (!turnOk) {
        Serial.println("[ERR] turn failed — stopping");
        running = false;
        stopMotors();
        gStatus = "ERR";
        Walls err = {true, true, true};
        oledUpdate(err);
        return;
    }

    oledUpdate(w);
}
