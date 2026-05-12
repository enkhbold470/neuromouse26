// src/main.cpp — Micromouse26 flood-fill solver, reliability-first
//
// No FreeRTOS task, no buzzer/tone, no periodic redraw.
// IR sampled inline on demand. OLED redraws only on change.
//
// Menu (right wheel scroll, BUTTON_1 select):
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

// ── IR thresholds (peaks ~3900 at wall, ~0 open) ─────────────────────────────
constexpr int WALL_FRONT = 1500;
constexpr int WALL_SIDE  = 1500;

// ── Hardware ─────────────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
// Encoders not inverted — wiring already produces positive ticks for forward.
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);
MicromouseMaze    maze;

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ── IR (inline, no background task) ──────────────────────────────────────────
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

static void sampleIR() {
    for (int i = 0; i < 4; i++) irVal[i] = readIR(PAIRS[i]);
}

static inline bool wallFront() { return irVal[0] > WALL_FRONT || irVal[3] > WALL_FRONT; }
static inline bool wallLeft()  { return irVal[1] > WALL_SIDE; }
static inline bool wallRight() { return irVal[2] > WALL_SIDE; }

// ── Motion ───────────────────────────────────────────────────────────────────
void stopMotors()    { leftMotor.brake(); rightMotor.brake(); }
void encodersReset() { leftEnc.reset(); rightEnc.reset(); }

void moveCell() {
    long target = TICKS_PER_CELL - COAST_COMP_TICKS;
    encodersReset();
    unsigned long startMs = millis();
    while (true) {
        long tL = leftEnc.getTicks();
        long tR = rTicks();
        long remL = target - tL;
        long remR = target - tR;
        bool lDone = (remL <= 0);
        bool rDone = (remR <= 0);
        if (lDone && rDone) { stopMotors(); return; }

        int err   = (int)(tL - tR);
        int baseL = lDone ? 0 : (remL > DECEL_TICKS ? DRIVE_PWM
                        : (int)map(remL, 0, DECEL_TICKS, DRIVE_PWM_MIN, DRIVE_PWM));
        int baseR = rDone ? 0 : (remR > DECEL_TICKS ? DRIVE_PWM
                        : (int)map(remR, 0, DECEL_TICKS, DRIVE_PWM_MIN, DRIVE_PWM));
        int pwmL, pwmR;
        if (!lDone && !rDone && remL > DECEL_TICKS && remR > DECEL_TICKS) {
            pwmL = constrain(baseL - (int)(err * BALANCE_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
            pwmR = constrain(baseR + (int)(err * BALANCE_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        } else {
            pwmL = baseL; pwmR = baseR;
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
enum State { IDLE, TEST_MOTOR, TEST_ENC, TEST_IR, RUN, GOAL };
State robotState = IDLE;

enum MenuItem { M_TEST_MOTOR = 0, M_TEST_ENC, M_TEST_IR, M_START, M_COUNT };
static const char* MENU_LABELS[M_COUNT] = {
    "Test Motor", "Test Encoder", "Test IR", "START"
};
static int  menuSel    = M_START;
static long menuEncRef = 0;
constexpr long ENC_PER_MENU_STEP = 80;

uint8_t robotRow = 0;
uint8_t robotCol = 0;
AbsDir  robotHeading = DIR_NORTH;

// Mechanical keyswitch, no debounce cap. BUTTON_HOLD_MS in PinConfig.h.
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
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "MM26 Menu");
    oled.drawHLine(0, 12, 128);
    const int Y0 = 24;
    const int LH = 12;
    for (int i = 0; i < M_COUNT; i++) {
        int y = Y0 + i * LH;
        if (i == menuSel) {
            oled.drawBox(0, y - 10, 128, LH);
            oled.setDrawColor(0);
            oled.drawStr(4, y, MENU_LABELS[i]);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(4, y, MENU_LABELS[i]);
        }
    }
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
    // PAIRS order 0=LF 1=L 2=R 3=RF → display L LF RF R: idx 1,0,3,2
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

    // motor.begin() handles IN-pin lockdown + LEDC duty=0 before attach.
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

        // ───────────────── IDLE ─────────────────
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
                    case M_TEST_MOTOR: robotState = TEST_MOTOR; break;
                    case M_TEST_ENC:   leftEnc.reset(); rightEnc.reset();
                                       oledEncoderTest();
                                       robotState = TEST_ENC; break;
                    case M_TEST_IR:    sampleIR(); oledBars();
                                       robotState = TEST_IR; break;
                    case M_START:      delay(400);
                                       robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
                                       setupMaze();
                                       oledRunStatus("go");
                                       robotState = RUN; break;
                }
            }
            break;
        }

        // ───────────────── TEST_MOTOR ─────────────────
        case TEST_MOTOR: {
            runMotorTest();
            while (!buttonEdge()) { delay(20); }
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            robotState = IDLE;
            break;
        }

        // ───────────────── TEST_ENC ─────────────────
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

        // ───────────────── TEST_IR ─────────────────
        case TEST_IR: {
            static uint32_t last = 0;
            if (millis() - last > 100) {
                sampleIR();
                oledBars();
                last = millis();
            }
            if (buttonEdge()) {
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE;
            }
            break;
        }

        // ───────────────── RUN ─────────────────
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

        // ───────────────── GOAL ─────────────────
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
