// src/main.cpp — Micromouse26, PCNT-encoder + MPU-6500 + IR-centered + flood-fill.
//
// Drivetrain stack mirrors test/wall-follow-encoder-pcnt.cpp exactly:
//   - 4x quadrature encoders via ESP32-S3 PCNT peripheral
//   - MPU-6500 gyro yaw integration with DLPF + auto bias capture
//   - Position-PID with stall escape, settle band, stiction floor, stopBias
//   - IMU-based pivot (PH_PIVOT) and spot rotation (PH_SPOT) for turns
//   - IR confidence-weighted centering during forward phase
//   - Phase-script executor (each move = list of FWD/PIVOT/SPOT steps)
//
// Maze layer on top:
//   - MicromouseMaze flood-fill BFS (16×16 alloc, 6×3 active sub-region)
//   - Cell-by-cell explore: sense walls → flood → pick best dir → script
//     (SPOT 90/180 + FWD 1 cell) → execute → repeat.
//   - On reaching goal (5,2): save walls to NVS, stop.
//   - Fast Run: load NVS walls, no sensing, drive straight to goal.

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoderPCNT.h"
#include "MicromouseMaze.h"

// ── MPU-6500 ─────────────────────────────────────────────────────────────────
#define MPU_ADDR        0x68
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define GYRO_FS_SEL     0x10
#define GYRO_SCALE      32.8f
#define DLPF_CFG_VAL    0x03

static float    gyroBiasZ = 0.0f;
static float    yawDeg    = 0.0f;
static float    gzFilt    = 0.0f;
static uint32_t lastYawUs = 0;
static bool     imuReady  = false;

static bool mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}
static bool mpuReadN(uint8_t reg, uint8_t* buf, uint8_t len) {
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
static bool readGzRaw(int16_t& out) {
    uint8_t b[2];
    if (!mpuReadN(0x47, b, 2)) return false;
    out = (int16_t)((b[0] << 8) | b[1]);
    return true;
}
static bool mpuInit() {
    uint8_t who = 0;
    if (!mpuReadN(REG_WHO_AM_I, &who, 1)) return false;
    mpuWrite(REG_PWR_MGMT_1, 0x00); delay(50);
    mpuWrite(REG_CONFIG,     DLPF_CFG_VAL);
    mpuWrite(REG_GYRO_CFG,   GYRO_FS_SEL);
    mpuWrite(REG_ACCEL_CFG,  0x00);
    delay(20);
    return true;
}
static void calibrateGyroBias(int N = 300, int sampleDelayMs = 2) {
    float sum = 0; int good = 0;
    for (int i = 0; i < N; i++) {
        int16_t raw;
        if (readGzRaw(raw)) { sum += raw / GYRO_SCALE; good++; }
        delay(sampleDelayMs);
    }
    if (good > 0) gyroBiasZ = sum / good;
    gzFilt    = 0.0f;
    lastYawUs = micros();
}
static void updateYaw() {
    if (!imuReady) return;
    int16_t raw;
    if (!readGzRaw(raw)) return;
    uint32_t now = micros();
    float dt = (lastYawUs == 0) ? 0.001f : (now - lastYawUs) / 1.0e6f;
    if (dt > 0.05f) dt = 0.05f;
    lastYawUs = now;
    float gz = raw / GYRO_SCALE - gyroBiasZ;
    if (fabsf(gz) < 0.05f) gz = 0.0f;
    gzFilt = 0.7f * gzFilt + 0.3f * gz;
    yawDeg += gz * dt;
}

// ── Hardware ─────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

MicromouseMotor       leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor       rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoderPCNT leftEnc   (PCNT_UNIT_0, ENC_L_A, ENC_L_B, /*inverted=*/true);
MicromouseEncoderPCNT rightEnc  (PCNT_UNIT_1, ENC_R_A, ENC_R_B, /*inverted=*/true);

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ── IR ───────────────────────────────────────────────────────────────────────
struct IRPair { uint8_t emit, rx; };
static IRPair PAIRS[4] = {
    { EMIT_LF, RX_LF },
    { EMIT_L,  RX_L  },
    { EMIT_R,  RX_R  },
    { EMIT_RF, RX_RF },
};
static int irVal[4] = {0,0,0,0};

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

static int calL = IR_CAL_L;
static int calR = IR_CAL_R;

// ── Battery ──────────────────────────────────────────────────────────────────
static float readVbat() {
    int raw = analogRead(BAT_V_SENSE);
    return (raw / 4095.0f) * 3.3f * BAT_VDIV_MULT;
}
static int batPct() {
    float v = readVbat();
    if (v < 6.4f) return 0;
    if (v > 8.4f) return 100;
    return (int)((v - 6.4f) * 50.0f + 0.5f);
}

// ── Live tuning ──────────────────────────────────────────────────────────────
struct Tuning {
    float    kp           = 0.20f;
    float    kd           = 0.05f;
    int      maxPwm       = 200;
    int      stictionPwm  = 110;
    int      frictionZone = 10;
    int      holdBand     = 20;
    uint32_t settleMs     = 80;
    int      stopBias     = 0;

    float    stallVel     = 30.0f;
    uint32_t stallMs      = 200;
    int      stallErrMax  = 40;

    float    balanceKp    = 0.03f;
    float    yawHoldKp    = 5.0f;
    float    centerKp     = 0.04f;
    float    centerKi     = 0.0f;
    float    centerKd     = 0.0f;
    int      centerMax    = 15;

    long     ticksPerCell    = 1405;
    long     turnPivotTicks  = 900;
    long     spot180Ticks    = 906;

    bool     useImu          = true;
    float    pivot90Deg      = 90.0f;
    float    spot180Deg      = 180.0f;
    float    yawKp           = 6.0f;
    float    yawKd           = 0.3f;
    int      yawMaxPwm       = 230;
    int      yawStictionPwm  = 130;
    float    yawFrictionZone = 3.0f;
    float    yawHoldBand     = 1.5f;
    uint32_t yawSettleMs     = 80;
    float    yawStallVel     = 5.0f;
    uint32_t yawStallMs      = 250;
    float    yawStallErrMax  = 4.0f;

    bool     telemetry       = true;
    bool     useIr           = true;
};
static Tuning T;

// ── PID (IR centering) ──────────────────────────────────────────────────────
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
        float out = T.centerKp * err + T.centerKi * integral + T.centerKd * deriv;
        return constrain(out, -(float)T.centerMax, (float)T.centerMax);
    }
    void reset() { integral = 0; prevError = 0; prevUs = 0; }
} pid;

void stopMotors() { leftMotor.brake(); rightMotor.brake(); }

bool buttonEdge() {
    static unsigned long pressStart = 0;
    static bool armed = true;
    bool low = (digitalRead(BUTTON_1) == LOW);
    unsigned long now = millis();
    if (!low) { pressStart = 0; armed = true; return false; }
    if (pressStart == 0) pressStart = now;
    if (armed && (now - pressStart >= BUTTON_HOLD_MS)) { armed = false; return true; }
    return false;
}

// ── Maze + robot state ──────────────────────────────────────────────────────
constexpr uint8_t MAZE_ROWS = 6;
constexpr uint8_t MAZE_COLS = 3;
constexpr uint8_t START_ROW = 0;
constexpr uint8_t START_COL = 0;
constexpr uint8_t GOAL_ROW  = 5;
constexpr uint8_t GOAL_COL  = 2;

static MicromouseMaze maze;
static uint8_t robotRow    = START_ROW;
static uint8_t robotCol    = START_COL;
static uint8_t robotHeading = DIR_NORTH;

static uint8_t plannedRow     = START_ROW;
static uint8_t plannedCol     = START_COL;
static uint8_t plannedHeading = DIR_NORTH;

static bool    exploreMode = false;
static bool    fastRunMode = false;

static void setupMaze() {
    maze.reset();
    for (int c = 0; c < MAZE_COLS; c++) {
        maze.setWall(MAZE_ROWS - 1, c, DIR_NORTH, true);
    }
    for (int r = 0; r < MAZE_ROWS; r++) {
        maze.setWall(r, MAZE_COLS - 1, DIR_EAST, true);
    }
    maze.setGoalSingle(GOAL_ROW, GOAL_COL);
}

// ── NVS wall persistence ────────────────────────────────────────────────────
static Preferences prefs;
static const char* NVS_NS    = "mm26";
static const char* NVS_WALLS = "walls";

static bool nvsSaveWalls() {
    if (!prefs.begin(NVS_NS, false)) return false;
    prefs.putBytes(NVS_WALLS, maze.walls, sizeof(maze.walls));
    prefs.end();
    return true;
}
static bool nvsLoadWalls() {
    if (!prefs.begin(NVS_NS, true)) return false;
    bool ok = prefs.isKey(NVS_WALLS);
    if (ok) prefs.getBytes(NVS_WALLS, maze.walls, sizeof(maze.walls));
    prefs.end();
    return ok;
}
static void nvsClearWalls() {
    if (!prefs.begin(NVS_NS, false)) return;
    prefs.remove(NVS_WALLS);
    prefs.end();
}

// ── Phase script ────────────────────────────────────────────────────────────
enum TurnDir   { TURN_NONE, TURN_RIGHT, TURN_LEFT };
enum RunPhase  { PH_FORWARD, PH_PIVOT, PH_SPOT };

struct PhaseStep {
    RunPhase phase;
    long     target;
    TurnDir  dir;
};

constexpr int MAX_SCRIPT = 6;
static PhaseStep script[MAX_SCRIPT];
static int       scriptLen = 0;
static int       scriptIdx = 0;

static TurnDir   runTurnDir = TURN_NONE;
static RunPhase  runPhase   = PH_FORWARD;
static long      runTarget  = 0;

static void scriptReset() { scriptLen = 0; scriptIdx = 0; }
static void scriptPush(RunPhase ph, long target, TurnDir d = TURN_NONE) {
    if (scriptLen < MAX_SCRIPT) script[scriptLen++] = { ph, target, d };
}
static void scriptPushFwd(long ticks) {
    scriptPush(PH_FORWARD, ticks + (long)T.stopBias, TURN_NONE);
}
static void scriptPushSpot(TurnDir d, float deg) {
    long target = T.useImu ? (long)(deg + 0.5f) : T.spot180Ticks;
    scriptPush(PH_SPOT, target, d);
}

// ── State ───────────────────────────────────────────────────────────────────
enum State { IDLE, ENC_TEST, GYRO_CAL, EXPLORE_THINK, RUN, GOAL, CRASH };
static State state = IDLE;

// ── OLED screens ────────────────────────────────────────────────────────────
static void drawBatteryTopRight() {
    int pct = batPct();
    char b[8]; snprintf(b, sizeof(b), "%d%%", pct);
    oled.setFont(u8g2_font_5x7_tf);
    int w = oled.getStrWidth(b);
    int xText = 128 - 16 - w - 1;
    int yText = 7;
    oled.drawStr(xText, yText, b);
    int xIcon = 128 - 14, yIcon = 1;
    oled.drawFrame(xIcon, yIcon, 12, 6);
    oled.drawBox  (xIcon + 12, yIcon + 2, 2, 2);
    int fillW = (pct * 10 + 50) / 100;
    if (fillW > 0) oled.drawBox(xIcon + 1, yIcon + 1, fillW, 4);
}

enum MenuItem {
    M_EXPLORE = 0,
    M_FAST,
    M_CAL_GYRO,
    M_ENC,
    M_TURN_R,
    M_TURN_L,
    M_NVS_CLR,
    M_COUNT
};
static const char* MENU_LABELS[M_COUNT] = {
    "Explore",
    "Fast Run",
    "Cal Gyro",
    "Encoder Test",
    "Turn R 90",
    "Turn L 90",
    "Clear NVS"
};
static int  menuSel    = M_EXPLORE;
static long menuEncRef = 0;
constexpr long ENC_PER_STEP = 80;

void oledMenu() {
    const int VIS = 5;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > M_COUNT - VIS) top = M_COUNT - VIS;
    if (top < 0) top = 0;

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "mm26 flood");
    drawBatteryTopRight();
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
    oled.sendBuffer();
}

void oledRun(long avg, long tgt, long tL, long tR) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    const char* phN = (runPhase == PH_FORWARD) ? "FWD"
                    : (runPhase == PH_PIVOT)   ? "PIV" : "SPOT";
    const char* dN  = (runTurnDir == TURN_RIGHT) ? "R"
                    : (runTurnDir == TURN_LEFT)  ? "L" : "-";
    char hdr[24];
    snprintf(hdr, sizeof(hdr), "%s/%s %d/%d", phN, dN, scriptIdx + 1, scriptLen);
    oled.drawStr(0, 8, hdr);
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_5x7_tf);
    char buf[28];
    snprintf(buf, sizeof(buf), "cell (%d,%d) hd=%c",
             robotRow, robotCol, "NESW"[robotHeading]);
    oled.drawStr(0, 22, buf);
    snprintf(buf, sizeof(buf), "tL%+5ld tR%+5ld", tL, tR);
    oled.drawStr(0, 32, buf);
    snprintf(buf, sizeof(buf), "tgt %ld avg %ld", tgt, avg);
    oled.drawStr(0, 42, buf);
    snprintf(buf, sizeof(buf), "yaw %+.1f", yawDeg);
    oled.drawStr(0, 52, buf);
    int pw = (tgt > 0) ? (int)((labs(avg) * 124L) / tgt) : 0;
    if (pw < 0) pw = 0; if (pw > 124) pw = 124;
    oled.drawFrame(0, 54, 126, 8);
    oled.drawBox(1, 55, pw, 6);
    oled.sendBuffer();
}

void oledTerminal(const char* title, const char* msg) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, title);
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    if (msg) oled.drawStr(0, 32, msg);
    oled.setFont(u8g2_font_5x7_tf);
    char buf[28];
    snprintf(buf, sizeof(buf), "cell (%d,%d)", robotRow, robotCol);
    oled.drawStr(0, 50, buf);
    oled.drawStr(0, 63, "btn=back");
    oled.sendBuffer();
}

void oledEncTest() {
    long tL = leftEnc.getTicks();
    long tR = rightEnc.getTicks();
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "Enc Test");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "L %ld", tL); oled.drawStr(0, 28, buf);
    snprintf(buf, sizeof(buf), "R %ld", tR); oled.drawStr(0, 44, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "btn=reset+back");
    oled.sendBuffer();
}

void oledGyroCal(int irLF, int irRF, bool still, int holdMs, const char* msg) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "Cal Gyro");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_5x7_tf);
    char buf[32];
    snprintf(buf, sizeof(buf), "LF %4d / %4d", irLF, IR_CAL_LF);
    oled.drawStr(0, 22, buf);
    snprintf(buf, sizeof(buf), "RF %4d / %4d", irRF, IR_CAL_RF);
    oled.drawStr(0, 32, buf);
    snprintf(buf, sizeof(buf), "still=%d  hold=%d/100ms", still ? 1 : 0, holdMs);
    oled.drawStr(0, 44, buf);
    snprintf(buf, sizeof(buf), "yaw=%+.2f bias=%.3f", yawDeg, gyroBiasZ);
    oled.drawStr(0, 54, buf);
    oled.drawStr(0, 63, msg ? msg : "btn=exit");
    oled.sendBuffer();
}

// ── Sensing & planning ──────────────────────────────────────────────────────
static void senseAndStoreWalls() {
    sampleIR();
    bool wF = irVal[0] > WALL_FRONT_THRESH || irVal[3] > WALL_FRONT_THRESH;
    bool wL = irVal[1] > WALL_SIDE_THRESH;
    bool wR = irVal[2] > WALL_SIDE_THRESH;
    AbsDir hd = (AbsDir)robotHeading;
    AbsDir lt = (AbsDir)((robotHeading + 3) % 4);
    AbsDir rt = (AbsDir)((robotHeading + 1) % 4);
    maze.setWall(robotRow, robotCol, hd, wF);
    maze.setWall(robotRow, robotCol, lt, wL);
    maze.setWall(robotRow, robotCol, rt, wR);
    Serial.printf("[SENSE] (%d,%d) hd=%c F=%d L=%d R=%d irLF=%d irRF=%d irL=%d irR=%d\n",
                  robotRow, robotCol, "NESW"[robotHeading], wF, wL, wR,
                  irVal[0], irVal[3], irVal[1], irVal[2]);
}

static void buildMoveScript(AbsDir bestDir) {
    int diff = ((int)bestDir - (int)robotHeading + 4) % 4;
    scriptReset();
    if      (diff == 1) scriptPushSpot(TURN_RIGHT, 90.0f);
    else if (diff == 2) scriptPushSpot(TURN_RIGHT, 180.0f);
    else if (diff == 3) scriptPushSpot(TURN_LEFT, 90.0f);
    scriptPushFwd(T.ticksPerCell);

    plannedHeading = bestDir;
    plannedRow     = robotRow + DIR_DR[bestDir];
    plannedCol     = robotCol + DIR_DC[bestDir];
}

static void scriptKick() {
    if (scriptLen == 0) return;
    scriptIdx  = 0;
    runPhase   = script[0].phase;
    runTarget  = script[0].target;
    runTurnDir = script[0].dir;
    leftEnc.reset(); rightEnc.reset();
    pid.reset();
    if (T.useImu && imuReady) {
        yawDeg = 0.0f; gzFilt = 0.0f; lastYawUs = micros();
    }
}

// ── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_1, INPUT_PULLUP);

    leftMotor.begin(); rightMotor.begin();
    leftEnc.begin();   rightEnc.begin();

    for (auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT);
        digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }
    analogReadResolution(12);

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    imuReady = mpuInit();
    if (imuReady) {
        Serial.println("[IMU] mpu6500 ok, calibrating bias... keep still");
        calibrateGyroBias(300, 2);
        yawDeg = 0.0f;
        Serial.printf("[IMU] bias=%.4f deg/s\n", gyroBiasZ);
    } else {
        Serial.println("[IMU] mpu6500 NOT detected");
    }

    setupMaze();

    menuEncRef = rightEnc.getTicks();
    oledMenu();

    Serial.println();
    Serial.println("mm26 flood ready");
}

// ── Loop ────────────────────────────────────────────────────────────────────
void loop() {
    updateYaw();

    switch (state) {

    case IDLE: {
        long delta = rightEnc.getTicks() - menuEncRef;
        if (delta >= ENC_PER_STEP) {
            menuSel = (menuSel + 1) % M_COUNT;
            menuEncRef += ENC_PER_STEP;
            oledMenu();
        } else if (delta <= -ENC_PER_STEP) {
            menuSel = (menuSel - 1 + M_COUNT) % M_COUNT;
            menuEncRef -= ENC_PER_STEP;
            oledMenu();
        }
        if (buttonEdge()) {
            switch (menuSel) {
            case M_EXPLORE:
                setupMaze();
                robotRow = START_ROW; robotCol = START_COL; robotHeading = DIR_NORTH;
                exploreMode = true; fastRunMode = false;
                Serial.println("--- EXPLORE START ---");
                state = EXPLORE_THINK;
                break;
            case M_FAST:
                setupMaze();
                if (!nvsLoadWalls()) {
                    Serial.println("[FAST] no saved walls");
                    oledTerminal("FAST", "no walls");
                    delay(800); oledMenu();
                    break;
                }
                robotRow = START_ROW; robotCol = START_COL; robotHeading = DIR_NORTH;
                exploreMode = false; fastRunMode = true;
                Serial.println("--- FAST RUN START ---");
                state = EXPLORE_THINK;
                break;
            case M_CAL_GYRO:
                state = GYRO_CAL;
                oledGyroCal(0, 0, false, 0, "btn=exit");
                break;
            case M_ENC:
                leftEnc.reset(); rightEnc.reset();
                state = ENC_TEST;
                oledEncTest();
                break;
            case M_TURN_R:
            case M_TURN_L:
                scriptReset();
                scriptPushSpot(menuSel == M_TURN_R ? TURN_RIGHT : TURN_LEFT, 90.0f);
                plannedRow = robotRow; plannedCol = robotCol;
                plannedHeading = (menuSel == M_TURN_R)
                                 ? (AbsDir)((robotHeading + 1) % 4)
                                 : (AbsDir)((robotHeading + 3) % 4);
                exploreMode = false; fastRunMode = false;
                Serial.printf("--- TURN %s ---\n", menuSel == M_TURN_R ? "R" : "L");
                scriptKick();
                state = RUN;
                break;
            case M_NVS_CLR:
                nvsClearWalls();
                Serial.println("[NVS] walls cleared");
                oledTerminal("NVS", "cleared");
                delay(600); oledMenu();
                break;
            }
        }
        break;
    }

    case ENC_TEST: {
        static uint32_t last = 0;
        if (millis() - last > 100) { oledEncTest(); last = millis(); }
        if (buttonEdge()) {
            leftEnc.reset(); rightEnc.reset();
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
        }
        break;
    }

    case GYRO_CAL: {
        static uint32_t lastUiMs   = 0;
        static uint32_t stillStart = 0;
        static long     prevTL = 0, prevTR = 0;
        constexpr uint32_t STILL_HOLD_MS = 100;
        constexpr float    STILL_GZ      = 1.0f;
        constexpr long     STILL_TICKS   = 1;
        if (buttonEdge()) {
            stopMotors();
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
            break;
        }
        if (millis() - lastUiMs > 50) {
            lastUiMs = millis();
            sampleIR();
            int irLF = irVal[0], irRF = irVal[3];
            long tL = leftEnc.getTicks(), tR = rTicks();
            bool encStill = (labs(tL - prevTL) <= STILL_TICKS)
                         && (labs(tR - prevTR) <= STILL_TICKS);
            prevTL = tL; prevTR = tR;
            bool gzStill = fabsf(gzFilt) < STILL_GZ;
            bool still   = encStill && gzStill && imuReady;
            if (still) { if (stillStart == 0) stillStart = millis(); }
            else       { stillStart = 0; }
            uint32_t held = stillStart ? (millis() - stillStart) : 0;
            if (held > 500) held = 500;
            if (stillStart && (millis() - stillStart) >= STILL_HOLD_MS) {
                oledGyroCal(irLF, irRF, true, (int)held, "calibrating...");
                delay(20);
                calibrateGyroBias(300, 2);
                yawDeg = 0.0f;
                Serial.printf("[GCAL] bias=%.4f deg/s\n", gyroBiasZ);
                oledGyroCal(irLF, irRF, true, (int)held, "DONE — btn=exit");
                while (digitalRead(BUTTON_1) == LOW) { delay(5); }
                delay(150);
                stillStart = 0; prevTL = 0; prevTR = 0;
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                state = IDLE;
                break;
            }
            oledGyroCal(irLF, irRF, still, (int)held,
                        imuReady ? "btn=exit" : "no IMU");
        }
        break;
    }

    case EXPLORE_THINK: {
        if (exploreMode) {
            senseAndStoreWalls();
        }
        maze.visited[robotRow][robotCol] = true;
        if (maze.isGoal(robotRow, robotCol)) {
            stopMotors();
            if (exploreMode) {
                if (nvsSaveWalls()) Serial.println("[NVS] walls saved");
            }
            Serial.printf("--- GOAL reached (%d,%d) ---\n", robotRow, robotCol);
            oledTerminal("GOAL!", "reached");
            state = GOAL;
            break;
        }
        maze.floodFill();
        uint8_t bestDist;
        AbsDir bestDir = maze.bestDirectionBiased(robotRow, robotCol,
                                                  (AbsDir)robotHeading, bestDist);
        if (bestDist == FLOOD_INFINITY) {
            stopMotors();
            Serial.println("[CRASH] no reachable goal — boxed in");
            oledTerminal("CRASH", "boxed");
            state = CRASH;
            break;
        }
        Serial.printf("[PLAN] (%d,%d,%c) dist=%d → %c\n",
                      robotRow, robotCol, "NESW"[robotHeading], bestDist, "NESW"[bestDir]);
        buildMoveScript(bestDir);
        scriptKick();
        state = RUN;
        break;
    }

    case RUN: {
        if (buttonEdge()) {
            stopMotors();
            Serial.println("--- RUN aborted ---");
            exploreMode = false; fastRunMode = false;
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
            break;
        }

        long tL = leftEnc.getTicks();
        long tR = rTicks();
        bool imuMode = T.useImu && imuReady && (runPhase != PH_FORWARD);

        float avg;
        if (runPhase == PH_FORWARD) {
            avg = (float)((tL + tR) / 2);
        } else if (imuMode) {
            avg = (runTurnDir == TURN_RIGHT) ? -yawDeg : +yawDeg;
        } else if (runPhase == PH_PIVOT) {
            avg = (float)((runTurnDir == TURN_RIGHT) ? tL : tR);
        } else {
            avg = (float)((runTurnDir == TURN_RIGHT) ? (tL - tR) / 2 : (tR - tL) / 2);
        }

        float    effKp        = imuMode ? T.yawKp           : T.kp;
        float    effKd        = imuMode ? T.yawKd           : T.kd;
        int      effMaxPwm    = imuMode ? T.yawMaxPwm       : T.maxPwm;
        int      effStkPwm    = imuMode ? T.yawStictionPwm  : T.stictionPwm;
        float    effFz        = imuMode ? T.yawFrictionZone : (float)T.frictionZone;
        float    effHb        = imuMode ? T.yawHoldBand     : (float)T.holdBand;
        uint32_t effSettleMs  = imuMode ? T.yawSettleMs     : T.settleMs;
        float    effStallVel  = imuMode ? T.yawStallVel     : T.stallVel;
        uint32_t effStallMs   = imuMode ? T.yawStallMs      : T.stallMs;
        float    effStallEmax = imuMode ? T.yawStallErrMax  : (float)T.stallErrMax;

        static float    posAvgPrev = 0.0f;
        static uint32_t posPrevUs  = 0;
        static float    velFilt    = 0.0f;
        static uint32_t settleStart = 0;
        static uint32_t stallStart  = 0;

        float posErr = (float)runTarget - avg;

        auto resetPidState = [&]() {
            posAvgPrev = 0.0f; posPrevUs = 0; velFilt = 0.0f;
            settleStart = 0; stallStart = 0;
        };

        auto endPhase = [&](const char* reason) {
            stopMotors();
            const char* phN = (runPhase == PH_FORWARD) ? "FWD"
                            : (runPhase == PH_PIVOT)   ? "PIV" : "SPOT";
            Serial.printf("--- STEP END idx=%d/%d ph=%s reason=%s err=%+.2f tL=%ld tR=%ld yaw=%+.2f ---\n",
                          scriptIdx + 1, scriptLen, phN, reason, posErr, tL, tR, yawDeg);

            if (scriptIdx + 1 >= scriptLen) {
                robotRow = plannedRow; robotCol = plannedCol; robotHeading = plannedHeading;
                resetPidState();
                runTurnDir = TURN_NONE;
                Serial.printf("--- MOVE DONE pos=(%d,%d,%c) ---\n",
                              robotRow, robotCol, "NESW"[robotHeading]);
                if (exploreMode || fastRunMode) {
                    state = EXPLORE_THINK;
                } else {
                    menuEncRef = rightEnc.getTicks();
                    oledMenu();
                    state = IDLE;
                }
                return;
            }
            scriptIdx++;
            PhaseStep& next = script[scriptIdx];
            runPhase   = next.phase;
            runTarget  = next.target;
            runTurnDir = next.dir;
            leftEnc.reset(); rightEnc.reset();
            resetPidState();
            pid.reset();
            if (T.useImu && imuReady) {
                yawDeg = 0.0f; gzFilt = 0.0f; lastYawUs = micros();
            }
        };

        if (fabsf(posErr) < effHb) {
            stopMotors();
            if (settleStart == 0) settleStart = millis();
            if (millis() - settleStart > effSettleMs) {
                endPhase("SETTLED");
            }
            break;
        }
        settleStart = 0;

        uint32_t nowUs = micros();
        float dt = (posPrevUs == 0) ? 0.005f
                                    : constrain((nowUs - posPrevUs) / 1e6f, 1e-4f, 0.05f);
        posPrevUs = nowUs;

        if (imuMode) {
            velFilt = (runTurnDir == TURN_RIGHT) ? -gzFilt : +gzFilt;
            posAvgPrev = avg;
        } else {
            float velRaw = (avg - posAvgPrev) / dt;
            posAvgPrev   = avg;
            velFilt      = 0.7f * velFilt + 0.3f * velRaw;
        }

        if (fabsf(velFilt) < effStallVel && fabsf(posErr) < effStallEmax) {
            if (stallStart == 0) stallStart = millis();
            if (millis() - stallStart > effStallMs) {
                endPhase("STALL");
                break;
            }
        } else {
            stallStart = 0;
        }

        float u   = effKp * posErr - effKd * velFilt;
        int   mag = (int)fabsf(u);
        if (mag > effMaxPwm) mag = effMaxPwm;
        if (fabsf(posErr) > effFz && mag < effStkPwm) mag = effStkPwm;
        int throttle = (u >= 0) ? mag : -mag;

        // IR centering (forward phase only).
        float corr = 0.0f;
        if (runPhase == PH_FORWARD && T.useIr) {
            sampleIR();
            constexpr float IR_CONF_LO    = 200.0f;
            constexpr float IR_CONF_HI    = 800.0f;
            constexpr float IR_EDGE_DELTA = 300.0f;

            static float irLSm = 0.0f, irRSm = 0.0f;
            static float irLPrev = 0.0f, irRPrev = 0.0f;
            static bool  firstSample = true;
            if (firstSample) {
                irLSm = irVal[1]; irRSm = irVal[2];
                irLPrev = irLSm;  irRPrev = irRSm;
                firstSample = false;
            }
            irLSm = 0.7f * irLSm + 0.3f * irVal[1];
            irRSm = 0.7f * irRSm + 0.3f * irVal[2];
            float dL = irLSm - irLPrev, dR = irRSm - irRPrev;
            irLPrev = irLSm; irRPrev = irRSm;
            if (dL < -IR_EDGE_DELTA) Serial.println("[EVENT] L wall opened");
            if (dR < -IR_EDGE_DELTA) Serial.println("[EVENT] R wall opened");
            if (dL >  IR_EDGE_DELTA) Serial.println("[EVENT] L wall appeared");
            if (dR >  IR_EDGE_DELTA) Serial.println("[EVENT] R wall appeared");

            auto wallConf = [](float v) -> float {
                if (v < IR_CONF_LO) return 0.0f;
                if (v > IR_CONF_HI) return 1.0f;
                return (v - IR_CONF_LO) / (IR_CONF_HI - IR_CONF_LO);
            };
            float cL = wallConf(irLSm);
            float cR = wallConf(irRSm);
            float fErrL = cL * (irLSm - (float)calL);
            float fErrR = cR * (irRSm - (float)calR);
            float fErr  = fErrR - fErrL;
            corr = (cL > 0.05f || cR > 0.05f) ? pid.compute(fErr) : 0.0f;
        }

        int pwmL, pwmR;
        if (runPhase == PH_FORWARD) {
            int yawBias    = (T.useImu && imuReady) ? (int)(-T.yawHoldKp * yawDeg) : 0;
            int encBalance = (int)((tL - tR) * T.balanceKp);
            int bias = (int)corr + encBalance + yawBias;
            pwmL = constrain(throttle - bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            pwmR = constrain(throttle + bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            leftMotor.drive(pwmL); rightMotor.drive(pwmR);
        } else if (runPhase == PH_PIVOT) {
            if (runTurnDir == TURN_RIGHT) {
                pwmL = throttle; pwmR = 0;
                leftMotor.drive(pwmL); rightMotor.brake();
            } else {
                pwmL = 0; pwmR = throttle;
                leftMotor.brake(); rightMotor.drive(pwmR);
            }
        } else {
            int sign = (runTurnDir == TURN_RIGHT) ? +1 : -1;
            pwmL = constrain( sign * throttle, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            pwmR = constrain(-sign * throttle, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            leftMotor.drive(pwmL); rightMotor.drive(pwmR);
        }

        static uint32_t lastOled = 0;
        if (millis() - lastOled > 150) {
            oledRun((long)avg, runTarget, tL, tR);
            lastOled = millis();
        }
        if (T.telemetry) {
            static uint32_t lastTel = 0;
            if (millis() - lastTel > 80) {
                const char* phn = (runPhase == PH_FORWARD) ? "FWD"
                                : (runPhase == PH_PIVOT)   ? "PIV" : "SPOT";
                Serial.printf("t=%lu ph=%s%s tgt=%ld avg=%+.1f err=%+.1f v=%+.1f thr=%+d pwmL=%+d pwmR=%+d yaw=%+.2f tL=%ld tR=%ld\n",
                              (unsigned long)millis(), phn, imuMode ? "/IMU" : "",
                              runTarget, avg, posErr, velFilt, throttle, pwmL, pwmR, yawDeg, tL, tR);
                lastTel = millis();
            }
        }
        break;
    }

    case GOAL:
    case CRASH: {
        if (buttonEdge()) {
            exploreMode = false; fastRunMode = false;
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
        }
        break;
    }

    }
}
