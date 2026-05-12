// src/main.cpp — Micromouse26 flood-fill solver
//
// Motion: constant DRIVE_PWM cruise, IR wall-follow PID centering,
// brake at (TICKS_PER_CELL × N − COAST_COMP_TICKS), gyro-based 90/180 turns.
//
// Turn policy: side-sensor wall→open mid-cell does NOT trigger early brake.
// Robot completes the cell first, then at the cell boundary senseWalls()
// re-samples (LF/RF + L/R). flood-fill on updated wall map picks bestDir.
// Pivot uses MPU-6500 gz integration (see test/mpu6500.cpp for tuning).
//
// Menu:
//   Cal IR       — capture LF/L/R/RF in dead-end (centered)
//   Cal Gyro     — capture gz bias (300 samples, keep STILL)
//   Test Motor   — short fwd/rev each motor
//   Test Encoder — live L/R tick counts
//   Test IR      — live 4-bar graph
//   START        — flood-fill solver

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseMaze.h"

// ── OLED ─────────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ── Maze geometry ────────────────────────────────────────────────────────────
constexpr uint8_t MAZE_ROWS = 6; // south to north 
constexpr uint8_t MAZE_COLS = 3; // west to east
constexpr uint8_t GOAL_ROW  = 2; 
constexpr uint8_t GOAL_COL  = 1;

// ── IR thresholds (calibrated: no-wall ~0, wall ~400–550) ────────────────────
// Sensors: L/R perpendicular to side walls, LF/RF angled forward to catch
// front wall. wallFront() = LF or RF over threshold.
constexpr int WALL_SIDE_PRESENT = 2000;
constexpr int WALL_FRONT_STOP   = 1500;

// ── Hardware ─────────────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);
MicromouseMaze    maze;

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ── IR ───────────────────────────────────────────────────────────────────────
// PAIRS index: 0=LF, 1=L, 2=R, 3=RF
struct IRPair { uint8_t emit, rx; };
static IRPair PAIRS[4] = {
    { EMIT_LF, RX_LF },
    { EMIT_L,  RX_L  },
    { EMIT_R,  RX_R  },
    { EMIT_RF, RX_RF },
};
static int irVal[4] = {0, 0, 0, 0};

static int readIR(const IRPair& p) {
    digitalWrite(p.emit, LOW);
    delayMicroseconds(80);
    int amb = analogRead(p.rx);
    digitalWrite(p.emit, HIGH);
    delayMicroseconds(80);
    int lit = analogRead(p.rx);
    digitalWrite(p.emit, LOW);
    int d = amb - lit;
    return d < 0 ? 0 : d;
}
static void sampleIR() { for (int i = 0; i < 4; i++) irVal[i] = readIR(PAIRS[i]); }

// ── Calibration (capture in dead-end: all 4 walls present, centered) ─────────
// Defaults from repeated empirical calibration of this hardware.
static int calLF = 3152;
static int calL  = 1718;
static int calR  = 2209;
static int calRF = 2339;

// Fixed thresholds from empirical readings:
//   side wall L/R: present > 1000, absent < 1000
//   front wall LF/RF: present > 1500 (typically 3000+ close range)
constexpr int WALL_SIDE_THRESH  = 1000;
constexpr int WALL_FRONT_THRESH = 1500;
static inline bool wallFront() {
    return irVal[0] > WALL_FRONT_THRESH || irVal[3] > WALL_FRONT_THRESH;
}
static inline bool wallLeft()  { return irVal[1] > WALL_SIDE_THRESH; }
static inline bool wallRight() { return irVal[2] > WALL_SIDE_THRESH; }

// ── Wall-follow PID ──────────────────────────────────────────────────────────
constexpr float CENTER_KP   = 0.12f;
constexpr float CENTER_KI   = 0.0f;
constexpr float CENTER_KD   = 0.03f;
constexpr int   MAX_CORR    = 250;

struct PID {
    float integral = 0, prevError = 0;
    unsigned long prevUs = 0;
    float compute(float err) {
        unsigned long now = micros();
        float dt = (prevUs == 0) ? 0.001f
                                 : constrain((now - prevUs) / 1e6f, 0.0001f, 0.05f);
        prevUs = now;
        integral += err * dt;
        integral  = constrain(integral, -2000.0f, 2000.0f);
        float deriv = (err - prevError) / dt;
        prevError = err;
        float out = CENTER_KP * err + CENTER_KI * integral + CENTER_KD * deriv;
        return constrain(out, -(float)MAX_CORR, (float)MAX_CORR);
    }
    void reset() { integral = 0; prevError = 0; prevUs = 0; }
} pid;

// ── Robot pose (declared early — driveChain mutates) ─────────────────────────
uint8_t robotRow = 0;
uint8_t robotCol = 0;
AbsDir  robotHeading = DIR_NORTH;

// ── Crash report (set by driveChain on abort, shown in CRASH state) ──────────
static bool    crashFlag = false;
static int     crashIR[4] = {0,0,0,0};
static uint8_t crashRow = 0, crashCol = 0;
static AbsDir  crashHeading = DIR_NORTH;
static const char* crashReason = "";

// ── MPU-6500 gyro (yaw integration for pivot turns) ──────────────────────────
#define MPU_ADDR          0x68
#define REG_WHO_AM_I      0x75
#define REG_PWR_MGMT_1    0x6B
#define REG_GYRO_CFG      0x1B
#define REG_ACCEL_CFG     0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define GYRO_SCALE        131.0f

struct ImuRaw { int16_t ax, ay, az, temp, gx, gy, gz; };

static float gyroBiasZ = 0.0f;
static float yaw = 0.0f;
static unsigned long lastImuUs = 0;

constexpr float TURN_OVERSHOOT_90  = 10.0f;
constexpr float TURN_OVERSHOOT_180 = 20.0f;

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
static int16_t imu_to16(uint8_t hi, uint8_t lo) { return (int16_t)((hi << 8) | lo); }

static bool imuReadAll(ImuRaw& d) {
    uint8_t b[14];
    if (!mpuRead(REG_ACCEL_XOUT_H, b, 14)) return false;
    d.ax=imu_to16(b[0],b[1]); d.ay=imu_to16(b[2],b[3]); d.az=imu_to16(b[4],b[5]);
    d.temp=imu_to16(b[6],b[7]);
    d.gx=imu_to16(b[8],b[9]); d.gy=imu_to16(b[10],b[11]); d.gz=imu_to16(b[12],b[13]);
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

static inline float turnOvershootDeg(float target) {
    float a = fabsf(target);
    float slope = (TURN_OVERSHOOT_180 - TURN_OVERSHOOT_90) / 90.0f;
    return TURN_OVERSHOOT_90 + slope * (a - 90.0f);
}

// ── Motion ───────────────────────────────────────────────────────────────────
void stopMotors()    { leftMotor.brake(); rightMotor.brake(); }
void encodersReset() { leftEnc.reset(); rightEnc.reset(); }

// Gyro-based pivot. Right=neg yaw, Left=pos yaw. Brakes before target by
// turnOvershootDeg(target), coast lands on target. Settles 200ms post-brake.
void doTurn(float targetDeg) {
    yaw = 0;
    lastImuUs = micros();
    float overshoot = turnOvershootDeg(targetDeg);
    float stopAt = (targetDeg > 0) ? (targetDeg - overshoot)
                                   : (targetDeg + overshoot);
    int dir = (targetDeg > 0) ? 1 : -1;
    leftMotor.drive(-dir * TURN_PWM);
    rightMotor.drive( dir * TURN_PWM);
    unsigned long t0 = millis();
    while (true) {
        updateYaw();
        if (targetDeg > 0 ? yaw >= stopAt : yaw <= stopAt) break;
        if (millis() - t0 > 3000) break;
    }
    stopMotors();
    unsigned long settleStart = millis();
    while (millis() - settleStart < 200) { updateYaw(); }
}

// Forward declarations for chained-run logic.
void senseWalls();
void rotateToHeading(AbsDir target);

// Drive forward, chaining as many same-heading cells as possible without
// stopping. On each cell-boundary crossing: sense, flood, decide. If next
// best dir == heading, extend target by another cell. Else brake and return.
// On return, robotRow/robotCol have been advanced to the cell the robot
// physically occupies (last cell entered). Caller pivots if needed.
void driveChain() {
    encodersReset();
    pid.reset();
    long cellBoundary = TICKS_PER_CELL;
    bool front = false;
    // Side-wall state at last decision point. Wall→open transition signals
    // opening on that side → brake + advance + replan.
    bool prevWL = wallLeft();
    bool prevWR = wallRight();
    unsigned long startMs = millis();

    while (true) {
        long tL  = leftEnc.getTicks();
        long tR  = rTicks();
        long avg = (tL + tR) / 2;

        sampleIR();
        front = wallFront();
        bool wL = wallLeft();
        bool wR = wallRight();

        // Mid-cell: never brake. Complete cell first. Side-sensor wall→open
        // is only acted on at cell boundary via senseWalls + flood.

        // Cell-boundary crossing: advance position + replan.
        if (avg >= cellBoundary) {
            robotRow += DIR_DR[robotHeading];
            robotCol += DIR_DC[robotHeading];
            maze.visited[robotRow][robotCol] = true;

            // Brake first so subsequent IR sample isn't motion-noisy. If
            // decision is "continue straight", we'll release brake. Cost of
            // brake-then-release ≈ 100ms of stopped time per cell — only paid
            // when actually crossing into a new cell.
            stopMotors();
            delay(100);
            sampleIR();
            senseWalls();
            maze.floodFill();

            if (maze.isGoal(robotRow, robotCol)) {
                return;
            }
            uint8_t bd;
            AbsDir best = maze.bestDirectionBiased(robotRow, robotCol, robotHeading, bd);
            if (bd == FLOOD_INFINITY || best != robotHeading) {
                return;  // RUN handles turn/no-path
            }
            // Continue straight: extend target, refresh PID, restart drive.
            cellBoundary += TICKS_PER_CELL;
            prevWL = wallLeft();
            prevWR = wallRight();
            pid.reset();
        }

        // No mid-cell front brake — LF/RF cone sees next cell's wall early
        // and triggers spurious stop. Only imminent-crash threshold (both
        // sensors very close = ≥2500) brakes mid-cell.
        if (irVal[0] > 3500 && irVal[3] > 3500) {
            stopMotors();
            crashFlag    = true;
            crashRow     = robotRow;
            crashCol     = robotCol;
            crashHeading = robotHeading;
            for (int i = 0; i < 4; i++) crashIR[i] = irVal[i];
            crashReason  = "front imminent";
            return;
        }
        int errR = wR ? (irVal[2] - calR) : 0;
        int errL = wL ? (irVal[1] - calL) : 0;
        int err  = errR - errL;
        float corr = (wL || wR) ? pid.compute((float)err) : 0.0f;

        int pwmL, pwmR;
        if (wL || wR) {
            pwmL = constrain(DRIVE_PWM - (int)corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
            pwmR = constrain(DRIVE_PWM + (int)corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        } else {
            int encErr = (int)(tL - tR);
            pwmL = constrain(DRIVE_PWM - (int)(encErr * BALANCE_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
            pwmR = constrain(DRIVE_PWM + (int)(encErr * BALANCE_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        }
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (millis() - startMs > (unsigned long)(TIMEOUT_MS * 10)) { stopMotors(); return; }
    }
}

void turnRight()  { doTurn(-90); }    // R = yaw negative
void turnLeft()   { doTurn(+90); }
void turnAround() { doTurn(-180); }

// ── State + menu ─────────────────────────────────────────────────────────────
enum State { IDLE, CAL, CAL_GYRO, TEST_MOTOR, TEST_ENC, TEST_IR, RUN, GOAL, CRASH };
State robotState = IDLE;

enum MenuItem { M_CAL = 0, M_CAL_GYRO, M_TEST_MOTOR, M_TEST_ENC, M_TEST_IR, M_START, M_COUNT };
static const char* MENU_LABELS[M_COUNT] = {
    "Cal IR", "Cal Gyro", "Test Motor", "Test Encoder", "Test IR", "START"
};
static int  menuSel    = M_START;
static long menuEncRef = 0;
constexpr long ENC_PER_MENU_STEP = 80;

// Mechanical keyswitch debounce — BUTTON_HOLD_MS in PinConfig.h.
bool buttonEdge() {
    static unsigned long pressStart = 0;
    static bool armed = true;
    bool low = (digitalRead(BUTTON_1) == LOW);
    unsigned long now = millis();
    if (!low) { pressStart = 0; armed = true; return false; }
    if (pressStart == 0) pressStart = now;
    if (armed && (now - pressStart >= BUTTON_HOLD_MS)) {
        armed = false;
        return true;
    }
    return false;
}

// ── OLED screens ─────────────────────────────────────────────────────────────
void oledMenu() {
    const int VIS = 5;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > M_COUNT - VIS) top = M_COUNT - VIS;
    if (top < 0) top = 0;

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "MM26");
    char hdr[20]; snprintf(hdr, sizeof(hdr), "%d/%d", menuSel + 1, M_COUNT);
    oled.drawStr(96, 8, hdr);
    oled.drawHLine(0, 10, 128);

    const int LH = 10;
    for (int i = 0; i < VIS; i++) {
        int idx = top + i;
        if (idx >= M_COUNT) break;
        int y = 12 + i * LH;
        if (idx == menuSel) {
            oled.drawBox(0, y, 128, LH);
            oled.setDrawColor(0);
            oled.drawStr(3, y + 8, MENU_LABELS[idx]);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(3, y + 8, MENU_LABELS[idx]);
        }
    }
    oled.drawHLine(0, 64 - 10, 128);
    oled.setFont(u8g2_font_5x7_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "gz%+.2f LF%d L%d R%d RF%d", gyroBiasZ, calLF, calL, calR, calRF);
    oled.drawStr(0, 63, buf);
    oled.sendBuffer();
}

void oledGyroCal(int prog, int total) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "CAL GYRO");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    oled.drawStr(0, 32, "STILL");
    char buf[24]; snprintf(buf, sizeof(buf), "%d/%d", prog, total);
    oled.drawStr(0, 50, buf);
    oled.sendBuffer();
}

void calibrateGyro() {
    constexpr int N = 300;
    float sum = 0;
    int good = 0;
    for (int i = 0; i < N; i++) {
        ImuRaw d;
        if (imuReadAll(d)) { sum += d.gz / GYRO_SCALE; good++; }
        if ((i & 0x3F) == 0) oledGyroCal(i, N);
        delay(2);
    }
    gyroBiasZ = (good > 0) ? sum / good : 0;
    yaw = 0;
    lastImuUs = micros();
}

void oledCal() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "CAL: dead-end");
    oled.drawHLine(0, 10, 128);
    char buf[24];
    snprintf(buf, sizeof(buf), "LF %4d  c%d", irVal[0], calLF);
    oled.drawStr(0, 22, buf);
    snprintf(buf, sizeof(buf), "L  %4d  c%d", irVal[1], calL);
    oled.drawStr(0, 34, buf);
    snprintf(buf, sizeof(buf), "R  %4d  c%d", irVal[2], calR);
    oled.drawStr(0, 46, buf);
    snprintf(buf, sizeof(buf), "RF %4d  c%d", irVal[3], calRF);
    oled.drawStr(0, 58, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 64, "btn=save");
    oled.sendBuffer();
}

void oledEncoderTest() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "Encoder Test");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "L %ld", (long)leftEnc.getTicks());
    oled.drawStr(0, 32, buf);
    snprintf(buf, sizeof(buf), "R %ld", (long)rightEnc.getTicks());
    oled.drawStr(0, 50, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 62, "btn = back");
    oled.sendBuffer();
}

void oledMotorMsg(const char* line1, const char* line2) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "Motor Test");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    oled.drawStr(0, 32, line1);
    if (line2) oled.drawStr(0, 50, line2);
    oled.sendBuffer();
}

void oledBars() {
    static const uint8_t order[4] = { 1, 0, 3, 2 };
    static const char*   lbl  [4] = { "L", "LF", "RF", "R" };
    const int H = 52, Y0 = 62, W = 26, GAP = 6, X0 = 4;

    oled.clearBuffer();
    oled.setFont(u8g2_font_5x7_tf);
    for (int i = 0; i < 4; i++) {
        int v = irVal[order[i]];
        if (v < 0) v = 0; if (v > 4095) v = 4095;
        int h = (v * H) / 4095;
        int x = X0 + i * (W + GAP);
        oled.drawFrame(x, Y0 - H, W, H);
        if (h > 0) oled.drawBox(x, Y0 - h, W, h);
        oled.drawStr(x + (W - (int)oled.getStrWidth(lbl[i])) / 2, Y0 - H - 2, lbl[i]);
    }
    oled.sendBuffer();
}

void oledRunStatus(const char* msg) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "RUN");
    oled.drawHLine(0, 12, 128);
    char buf[24];
    snprintf(buf, sizeof(buf), "r%u c%u", robotRow, robotCol);
    oled.drawStr(0, 28, buf);
    snprintf(buf, sizeof(buf), "h%d d%u", (int)robotHeading, maze.flood[robotRow][robotCol]);
    oled.drawStr(0, 44, buf);
    if (msg) oled.drawStr(0, 62, msg);
    oled.sendBuffer();
}

// Crash screen: 128x64 layout
//   row1: "CRASH" + reason
//   row2: cell r=X c=Y h=N
//   row3: LF#### L####
//   row4: RF#### R####
//   row5: open L:0 R:1 F:1
//   row6: btn=back
void oledCrash() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "CRASH %s", crashReason);
    oled.drawStr(0, 8, buf);
    oled.drawHLine(0, 10, 128);

    snprintf(buf, sizeof(buf), "r=%u c=%u h=%d", crashRow, crashCol, (int)crashHeading);
    oled.drawStr(0, 20, buf);
    snprintf(buf, sizeof(buf), "LF%4d L%4d", crashIR[0], crashIR[1]);
    oled.drawStr(0, 30, buf);
    snprintf(buf, sizeof(buf), "RF%4d R%4d", crashIR[3], crashIR[2]);
    oled.drawStr(0, 40, buf);

    bool oL = crashIR[1] < WALL_SIDE_THRESH;
    bool oR = crashIR[2] < WALL_SIDE_THRESH;
    bool wF = crashIR[0] > WALL_FRONT_THRESH || crashIR[3] > WALL_FRONT_THRESH;
    snprintf(buf, sizeof(buf), "openL%d openR%d F%d", oL, oR, wF);
    oled.drawStr(0, 50, buf);

    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "btn=back");
    oled.sendBuffer();
}

void oledCountdown(int n) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "START");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_logisoso42_tn);
    char buf[4]; snprintf(buf, sizeof(buf), "%d", n);
    int w = oled.getStrWidth(buf);
    oled.drawStr((128 - w) / 2, 60, buf);
    oled.sendBuffer();
}

// ── Maze setup ───────────────────────────────────────────────────────────────
void senseWalls() {
    if (wallFront()) maze.setWall(robotRow, robotCol, robotHeading, true);
    AbsDir leftDir  = (AbsDir)(((int)robotHeading + 3) % 4);
    AbsDir rightDir = (AbsDir)(((int)robotHeading + 1) % 4);
    if (wallLeft())  maze.setWall(robotRow, robotCol, leftDir,  true);
    if (wallRight()) maze.setWall(robotRow, robotCol, rightDir, true);
}

void rotateToHeading(AbsDir target) {
    int diff = ((int)target - (int)robotHeading + 4) % 4;
    if      (diff == 1) { turnRight();  robotHeading = (AbsDir)(((int)robotHeading + 1) % 4); }
    else if (diff == 3) { turnLeft();   robotHeading = (AbsDir)(((int)robotHeading + 3) % 4); }
    else if (diff == 2) { turnAround(); robotHeading = (AbsDir)(((int)robotHeading + 2) % 4); }
}

void setupMaze() {
    maze.reset();
    for (int c = 0; c < MAZE_SIZE; c++) maze.setWall(MAZE_ROWS - 1, c, DIR_NORTH, true);
    for (int r = 0; r < MAZE_SIZE; r++) maze.setWall(r, MAZE_COLS - 1, DIR_EAST,  true);
    maze.setGoalSingle(GOAL_ROW, GOAL_COL);
    maze.floodFill();
}

// ── Test routines ────────────────────────────────────────────────────────────
void runMotorTest() {
    const int PWM = DRIVE_PWM;
    const int DUR = 350;
    oledMotorMsg("L fwd", nullptr);  leftMotor.drive( PWM);  delay(DUR); leftMotor.coast();  delay(200);
    oledMotorMsg("L rev", nullptr);  leftMotor.drive(-PWM);  delay(DUR); leftMotor.coast();  delay(300);
    oledMotorMsg("R fwd", nullptr);  rightMotor.drive( PWM); delay(DUR); rightMotor.coast(); delay(200);
    oledMotorMsg("R rev", nullptr);  rightMotor.drive(-PWM); delay(DUR); rightMotor.coast(); delay(200);
    stopMotors();
    oledMotorMsg("done", "btn = back");
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_1, INPUT_PULLUP);

    leftMotor.begin();
    rightMotor.begin();
    leftEnc.begin();
    rightEnc.begin();

    for (auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT);
        digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }
    analogReadResolution(12);

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    // MPU-6500 on same I2C bus.
    uint8_t who = 0;
    mpuRead(REG_WHO_AM_I, &who, 1);
    mpuWrite(REG_PWR_MGMT_1, 0x00); delay(50);
    mpuWrite(REG_GYRO_CFG,   0x00);
    mpuWrite(REG_ACCEL_CFG,  0x00);
    lastImuUs = micros();
    Serial.printf("[INIT] MPU WHO=0x%02X\n", who);

    // Auto-calibrate gyro at boot (robot must be still).
    calibrateGyro();

    setupMaze();
    menuEncRef = rightEnc.getTicks();
    oledMenu();
    Serial.println("[INIT] ready");
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    switch (robotState) {

        case IDLE: {
            long delta = rightEnc.getTicks() - menuEncRef;
            if (delta >= ENC_PER_MENU_STEP) {
                menuSel = (menuSel + 1) % M_COUNT;
                menuEncRef += ENC_PER_MENU_STEP;
                oledMenu();
            } else if (delta <= -ENC_PER_MENU_STEP) {
                menuSel = (menuSel - 1 + M_COUNT) % M_COUNT;
                menuEncRef -= ENC_PER_MENU_STEP;
                oledMenu();
            }
            if (buttonEdge()) {
                switch (menuSel) {
                    case M_CAL:        sampleIR(); oledCal(); robotState = CAL; break;
                    case M_CAL_GYRO:   calibrateGyro(); oledMenu(); break;
                    case M_TEST_MOTOR: robotState = TEST_MOTOR; break;
                    case M_TEST_ENC:   leftEnc.reset(); rightEnc.reset();
                                       oledEncoderTest();
                                       robotState = TEST_ENC; break;
                    case M_TEST_IR:    sampleIR(); oledBars();
                                       robotState = TEST_IR; break;
                    case M_START:      for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(1000); }
                                       robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
                                       setupMaze();
                                       oledRunStatus("go");
                                       robotState = RUN; break;
                }
            }
            break;
        }

        case CAL: {
            static uint32_t last = 0;
            if (millis() - last > 100) { sampleIR(); oledCal(); last = millis(); }
            if (buttonEdge()) {
                calLF = irVal[0];
                calL  = irVal[1];
                calR  = irVal[2];
                calRF = irVal[3];
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE;
            }
            break;
        }

        case TEST_MOTOR: {
            runMotorTest();
            while (!buttonEdge()) { delay(20); }
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            robotState = IDLE;
            break;
        }

        case TEST_ENC: {
            static uint32_t last = 0;
            if (millis() - last > 150) { oledEncoderTest(); last = millis(); }
            if (buttonEdge()) {
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE;
            }
            break;
        }

        case TEST_IR: {
            static uint32_t last = 0;
            if (millis() - last > 100) { sampleIR(); oledBars(); last = millis(); }
            if (buttonEdge()) {
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE;
            }
            break;
        }

        case RUN: {
            if (maze.isGoal(robotRow, robotCol)) {
                robotState = GOAL;
                break;
            }
            maze.visited[robotRow][robotCol] = true;

            sampleIR();
            senseWalls();
            maze.floodFill();

            uint8_t bestDist;
            AbsDir best = maze.bestDirectionBiased(robotRow, robotCol, robotHeading, bestDist);
            if (bestDist == FLOOD_INFINITY) {
                stopMotors();
                crashFlag = true;
                crashRow = robotRow; crashCol = robotCol;
                crashHeading = robotHeading;
                for (int i = 0; i < 4; i++) crashIR[i] = irVal[i];
                crashReason = "no path";
                robotState = CRASH;
                break;
            }

            rotateToHeading(best);
            driveChain();   // chains straight cells, updates robotRow/Col
            if (crashFlag) { robotState = CRASH; }
            break;
        }

        case GOAL: {
            stopMotors();
            oledRunStatus("GOAL");
            if (buttonEdge()) {
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE;
            }
            break;
        }

        case CRASH: {
            static bool drawn = false;
            if (!drawn) { oledCrash(); drawn = true; }
            if (buttonEdge()) {
                crashFlag = false;
                drawn = false;
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE;
            }
            break;
        }
    }
}
