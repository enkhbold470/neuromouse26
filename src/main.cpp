// src/main.cpp — Micromouse26 flood-fill solver
//
// Motion stack matches test/wall-follow-encoder-count-cell.cpp:
//   constant cruise DRIVE_PWM → tick target (TICKS_PER_CELL − COAST_COMP_TICKS)
//   wall-follow PID centering when side walls present, encoder balance otherwise
//   brake stop, coast residual ≈ COAST_COMP_MM
//
// Menu (right wheel scroll, BUTTON_1 select):
//   Calibrate    — capture L/R IR centers (centered in cell)
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
constexpr uint8_t MAZE_ROWS = 6;
constexpr uint8_t MAZE_COLS = 3;
constexpr uint8_t GOAL_ROW  = 5;
constexpr uint8_t GOAL_COL  = 1;

// ── IR thresholds (post-calibration: no-wall ~0, wall ~400–550) ──────────────
constexpr int WALL_SIDE_PRESENT = 400;
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

static inline bool wallFront() { return irVal[0] > WALL_FRONT_STOP || irVal[3] > WALL_FRONT_STOP; }
static inline bool wallLeft()  { return irVal[1] > WALL_SIDE_PRESENT; }
static inline bool wallRight() { return irVal[2] > WALL_SIDE_PRESENT; }

// ── Calibration (L/R wall reference) ─────────────────────────────────────────
static int calL = L_CENTER;
static int calR = R_CENTER;

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

// ── Motion ───────────────────────────────────────────────────────────────────
void stopMotors()    { leftMotor.brake(); rightMotor.brake(); }
void encodersReset() { leftEnc.reset(); rightEnc.reset(); }

// Drive one cell: constant DRIVE_PWM cruise, wall-follow PID centering or
// encoder balance, brake at (TICKS_PER_CELL − COAST_COMP_TICKS).
void moveCell() {
    long target = (long)TICKS_PER_CELL - COAST_COMP_TICKS;
    encodersReset();
    pid.reset();
    unsigned long startMs = millis();

    while (true) {
        long tL  = leftEnc.getTicks();
        long tR  = rTicks();
        long avg = (tL + tR) / 2;

        if (avg >= target) { stopMotors(); return; }

        sampleIR();
        if (wallFront()) { stopMotors(); return; }

        bool wL = wallLeft();
        bool wR = wallRight();

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

        if (millis() - startMs > (unsigned long)TIMEOUT_MS) { stopMotors(); return; }
    }
}

void turnRight() {
    encodersReset();
    leftMotor.drive(TURN_PWM);
    rightMotor.drive(-TURN_PWM);
    unsigned long t = millis();
    while (leftEnc.getTicks() < TURN_TICKS_90_R) {
        if (millis() - t > 2000) break;
    }
    stopMotors();
}

void turnLeft() {
    encodersReset();
    leftMotor.drive(-TURN_PWM);
    rightMotor.drive(TURN_PWM);
    unsigned long t = millis();
    while (rTicks() < TURN_TICKS_90_L) {
        if (millis() - t > 2000) break;
    }
    stopMotors();
}

void turnAround() { turnRight(); delay(150); turnRight(); }

// ── State + menu ─────────────────────────────────────────────────────────────
enum State { IDLE, CAL, TEST_MOTOR, TEST_ENC, TEST_IR, RUN, GOAL };
State robotState = IDLE;

enum MenuItem { M_CAL = 0, M_TEST_MOTOR, M_TEST_ENC, M_TEST_IR, M_START, M_COUNT };
static const char* MENU_LABELS[M_COUNT] = {
    "Calibrate", "Test Motor", "Test Encoder", "Test IR", "START"
};
static int  menuSel    = M_START;
static long menuEncRef = 0;
constexpr long ENC_PER_MENU_STEP = 80;

uint8_t robotRow = 0;
uint8_t robotCol = 0;
AbsDir  robotHeading = DIR_NORTH;

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
    snprintf(buf, sizeof(buf), "cal L%d R%d", calL, calR);
    oled.drawStr(0, 63, buf);
    oled.sendBuffer();
}

void oledCal() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "CALIBRATE");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 24, "Center robot in cell");
    char buf[24];
    snprintf(buf, sizeof(buf), "L now %4d (cal %d)", irVal[1], calL);
    oled.drawStr(0, 36, buf);
    snprintf(buf, sizeof(buf), "R now %4d (cal %d)", irVal[2], calR);
    oled.drawStr(0, 48, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "btn = save & exit");
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
                calL = irVal[1];
                calR = irVal[2];
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
                Serial.println("[ERR] no path");
                stopMotors();
                oledRunStatus("no path");
                delay(800);
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE;
                break;
            }

            oledRunStatus("step");
            rotateToHeading(best);
            moveCell();
            robotRow += DIR_DR[robotHeading];
            robotCol += DIR_DC[robotHeading];
            delay(CELL_PAUSE_MS);
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
    }
}
