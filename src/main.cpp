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
constexpr uint8_t GOAL_ROW  = 5; 
constexpr uint8_t GOAL_COL  = 2;

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
static int calLF = 3300;
static int calL  = 1800;
static int calR  = 1800;
static int calRF = 2500;

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
static bool   crashDrawn = false;

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

// Front-wall safety derived from calLF/calRF (dead-end center reading).
// MID_BRAKE_FRAC: mid-cell brake fires when LF or RF exceeds this fraction
// of cal — prevents bump when side opens and robot crosses cell w/o lateral
// reference. TURN_CLEAR_FRAC: must be below to start pivot (nose 50mm fwd
// of axle needs swing clearance). Reverse-pulse until safe.
constexpr float MID_BRAKE_FRAC  = 0.45f;
// 1.15× cal = only fire when robot is closer than the calibration position (~90mm).
// 0.55 caused ensureFrontClearance() to fire every turn (robot at cell center = cal reading).
constexpr float TURN_CLEAR_FRAC = 1.15f;

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

// ── Wheel speed monitor shared state ─────────────────────────────────────────
struct { float rpmL, rpmR; bool stalledL, stalledR; } wheelSpeed = {};
static portMUX_TYPE speedMux = portMUX_INITIALIZER_UNLOCKED;

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

    // Verify and correct: nudge at low speed if settled yaw is off target.
    // Runs up to 3 times; stops each attempt when within 1° or after 400ms.
    for (int attempt = 0; attempt < 3; attempt++) {
        float err = targetDeg - yaw;
        if (fabsf(err) <= TURN_VERIFY_THRESH) break;
        int cdir = (err > 0) ? 1 : -1;
        leftMotor.drive(-cdir * TURN_CORRECT_PWM);
        rightMotor.drive( cdir * TURN_CORRECT_PWM);
        unsigned long ct0 = millis();
        while (millis() - ct0 < 400) {
            updateYaw();
            if (fabsf(targetDeg - yaw) <= 1.0f) break;
        }
        stopMotors();
        unsigned long cs = millis();
        while (millis() - cs < 80) { updateYaw(); }
    }
}

// Forward declarations for chained-run logic.
void senseWalls();
void rotateToHeading(AbsDir target);

// Half-cell advance before pivot: at cell boundary, robot's center is at the
// boundary line (180mm from start of current cell). Pivot axis = robot center.
// Pivoting there leaves robot offset from new heading's lane center. Advance
// half a cell so pivot happens at cell center.
constexpr long CENTER_ADVANCE_TICKS = TICKS_PER_CELL / 2;   // ~90mm forward

void advanceToCellCenter() {
    long refL = leftEnc.getTicks(), refR = rTicks();
    yaw = 0;
    lastImuUs = micros();
    unsigned long t0 = millis();
    while (true) {
        updateYaw();
        sampleIR();
        long avg = ((leftEnc.getTicks() - refL) + (rTicks() - refR)) / 2;
        if (avg >= CENTER_ADVANCE_TICKS) break;
        // Gyro yaw-hold + gentle IR lateral correction using whichever walls exist.
        // Mirrors driveChain's yawBias logic but as direct PWM correction.
        constexpr float IR_CTR_K = 0.10f;
        float irCorr = 0;
        bool wL = irVal[1] > WALL_SIDE_THRESH;
        bool wR = irVal[2] > WALL_SIDE_THRESH;
        if (wL && wR)  irCorr = IR_CTR_K * ((irVal[1]-calL) - (irVal[2]-calR));
        else if (wL)   irCorr = IR_CTR_K *  (irVal[1]-calL);
        else if (wR)   irCorr = IR_CTR_K * -(irVal[2]-calR);
        int corr = constrain((int)(yaw * 3.0f + irCorr), -200, 200);
        leftMotor.drive(constrain(DRIVE_PWM + corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX));
        rightMotor.drive(constrain(DRIVE_PWM - corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX));
        if (millis() - t0 > 1500) break;
    }
    stopMotors();
}

// Drive forward, chaining as many same-heading cells as possible without
// stopping. On each cell-boundary crossing: sense, flood, decide. If next
// best dir == heading, extend target by another cell. Else brake and return.
// On return, robotRow/robotCol have been advanced to the cell the robot
// physically occupies (last cell entered). Caller pivots if needed.
void driveChain() {
    long refL = leftEnc.getTicks(), refR = rTicks();
    yaw = 0;
    lastImuUs = micros();
    long cellBoundary = TICKS_PER_CELL;
    unsigned long startMs = millis();
    unsigned long lastIR = 0;

    constexpr float YAW_KP    = 3.0f;
    constexpr float YAW_KI    = 0.3f;
    constexpr float ENC_KP_BK = 0.2f;
    constexpr int   STRAIGHT_MAX = 200;
    float yawInteg = 0.0f;
    float velIntegL = 0.0f, velIntegR = 0.0f;
    constexpr float IR_ALIGN_K = 0.005f;

    while (true) {
        updateYaw();
        long tL = leftEnc.getTicks() - refL;
        long tR = rTicks() - refR;
        long avg = (tL + tR) / 2;
        float curRpmL, curRpmR;
        bool stalled;
        taskENTER_CRITICAL(&speedMux);
        curRpmL = wheelSpeed.rpmL;
        curRpmR = wheelSpeed.rpmR;
        stalled = wheelSpeed.stalledL || wheelSpeed.stalledR;
        taskEXIT_CRITICAL(&speedMux);

        // Periodic IR sample for crash check + control feedback.
        if (millis() - lastIR > 25) {
            sampleIR();
            lastIR = millis();
        }

        // Imminent-crash brake (both front sensors very close).
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

        // Side-open bump guard: if a front sensor crosses MID_BRAKE_FRAC of
        // cal mid-cell (typical when one side is open and IR PID has no
        // lateral reference), brake NOW and treat as cell-boundary reached.
        // Requires >⅓ cell already traveled to avoid spurious early stops.
        int midBrakeLF = (int)(calLF * MID_BRAKE_FRAC);
        int midBrakeRF = (int)(calRF * MID_BRAKE_FRAC);
        bool frontClose = (irVal[0] > midBrakeLF || irVal[3] > midBrakeRF);
        bool pastThird  = (avg > TICKS_PER_CELL / 3);

        // Cell-boundary crossing: stop first, then advance position + replan.
        if (avg >= cellBoundary || (frontClose && pastThird)) {
            stopMotors();
            delay(100);
            robotRow += DIR_DR[robotHeading];
            robotCol += DIR_DC[robotHeading];
            maze.visited[robotRow][robotCol] = true;
            // Force-clear the wall behind robot — we just drove through it.
            // Prevents stuck "no path" from wall mis-marks on prior visits.
            AbsDir back = (AbsDir)(((int)robotHeading + 2) % 4);
            maze.setWall(robotRow, robotCol, back, false);
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
            // Continue straight. Use current cell's IR readings to bias yaw
            // so next cell's drive corrects toward center.
            //   both walls : align = (irL-calL) - (irR-calR)  → centering
            //   L only     : align = (irL-calL)               → hold L distance
            //   R only     : align = -(irR-calR)              → hold R distance
            //   neither    : align = 0                        → pure gyro
            // align>0 → drifted left → set yaw positive → P-loop steers right.
            float yawBias = 0;
            bool wL = wallLeft();
            bool wR = wallRight();
            int align = 0;
            if (wL && wR)      align = (irVal[1] - calL) - (irVal[2] - calR);
            else if (wL)       align = irVal[1] - calL;
            else if (wR)       align = -(irVal[2] - calR);
            yawBias = constrain(IR_ALIGN_K * (float)align, -5.0f, 5.0f);

            refL = leftEnc.getTicks();
            refR = rTicks();
            cellBoundary = TICKS_PER_CELL;
            yaw = yawBias;
            yawInteg = 0.0f;
            velIntegL = velIntegR = 0.0f;
            lastImuUs = micros();
        }

        // Trapezoidal velocity profile (GreenYe): basePwm = min(accel ramp, decel ramp).
        // Both ramps are linear in encoder ticks. Their minimum naturally forms a
        // triangular profile when ACCEL+DECEL > cell length, trapezoidal otherwise.
        long ticksLeft = cellBoundary - avg;
        int accelPwm = DRIVE_PWM_MIN + (ACCEL_TICKS > 0
            ? (int)((long)(DRIVE_PWM - DRIVE_PWM_MIN) * avg      / ACCEL_TICKS)
            : (DRIVE_PWM - DRIVE_PWM_MIN));
        int decelPwm = DRIVE_PWM_MIN + (DECEL_TICKS > 0
            ? (int)((long)(DRIVE_PWM - DRIVE_PWM_MIN) * ticksLeft / DECEL_TICKS)
            : (DRIVE_PWM - DRIVE_PWM_MIN));
        int basePwm = stalled
            ? constrain(STALL_BOOST_PWM, DRIVE_PWM, MOTOR_PWM_MAX)
            : constrain(min(accelPwm, decelPwm), DRIVE_PWM_MIN, DRIVE_PWM);

        // Velocity PI: target RPM proportional to trapezoid basePwm.
        // Additive correction on top of basePwm — degrades gracefully if mis-tuned.
        float targetRpm = CRUISE_RPM * (float)basePwm / DRIVE_PWM;
        float errL = targetRpm - fabsf(curRpmL);
        float errR = targetRpm - fabsf(curRpmR);
        velIntegL = constrain(velIntegL + errL * 0.02f, -200.0f, 200.0f);
        velIntegR = constrain(velIntegR + errR * 0.02f, -200.0f, 200.0f);
        int vCorrL = constrain((int)(VEL_KP * errL + VEL_KI * velIntegL), -150, 150);
        int vCorrR = constrain((int)(VEL_KP * errR + VEL_KI * velIntegR), -150, 150);

        // Live IR lateral correction.
        float irBias = 0;
        {
            bool wLl = irVal[1] > WALL_SIDE_THRESH;
            bool wRl = irVal[2] > WALL_SIDE_THRESH;
            if (wLl && wRl)   irBias = IR_ALIGN_K * ((irVal[1]-calL) - (irVal[2]-calR));
            else if (wLl)      irBias = IR_ALIGN_K *  (irVal[1]-calL);
            else if (wRl)      irBias = IR_ALIGN_K * -(irVal[2]-calR);
            irBias = constrain(irBias, -5.0f, 5.0f);
        }

        // Gyro yaw PI + encoder-balance correction.
        yawInteg += yaw * 0.001f;
        yawInteg  = constrain(yawInteg, -8.0f, 8.0f);
        int encDiff = (int)(tL - tR);
        int corr = (int)((yaw + irBias) * YAW_KP + yawInteg * YAW_KI) - (int)(encDiff * ENC_KP_BK);
        corr = constrain(corr, -STRAIGHT_MAX, STRAIGHT_MAX);
        int pwmL = constrain(basePwm + vCorrL + corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        int pwmR = constrain(basePwm + vCorrR - corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (millis() - startMs > (unsigned long)(TIMEOUT_MS * 10)) { stopMotors(); return; }
    }
}

void turnRight()  { doTurn(-90); }    // R = yaw negative
void turnLeft()   { doTurn(+90); }
// 180 single-shot consistently undershoots/overshoots — split into two 90s
// with brief settle. Same direction (CW) keeps yaw sign consistent.
void turnAround() {
    doTurn(-180);
    // delay(120);
    // doTurn(-90);
}

void speedTask(void*) {
    vTaskDelay(pdMS_TO_TICKS(200));
    long prevL = leftEnc.getTicks(), prevR = rightEnc.getTicks();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));
        long nowL = leftEnc.getTicks(), nowR = rightEnc.getTicks();
        long dL = nowL - prevL, dR = nowR - prevR;
        prevL = nowL; prevR = nowR;
        // dL ticks / 20ms * (1000ms/s) / TICKS_PER_REV * 60s/min = RPM
        taskENTER_CRITICAL(&speedMux);
        wheelSpeed.rpmL     = (float)dL / TICKS_PER_REV * 3000.0f;
        wheelSpeed.rpmR     = (float)dR / TICKS_PER_REV * 3000.0f;
        wheelSpeed.stalledL = (abs(dL) < 2);
        wheelSpeed.stalledR = (abs(dR) < 2);
        taskEXIT_CRITICAL(&speedMux);
    }
}

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
// Set both present AND absent so flood-fill sees open passages, not just walls.
void senseWalls() {
    AbsDir leftDir  = (AbsDir)(((int)robotHeading + 3) % 4);
    AbsDir rightDir = (AbsDir)(((int)robotHeading + 1) % 4);
    maze.setWall(robotRow, robotCol, robotHeading, wallFront());
    maze.setWall(robotRow, robotCol, leftDir,      wallLeft());
    maze.setWall(robotRow, robotCol, rightDir,     wallRight());
}

// Reverse-pulse if front wall too close for safe pivot. Threshold derived
// from calLF/calRF — nose (sensor) is 50mm fwd of axle, needs clearance to
// arc through 90°. Below TURN_CLEAR_FRAC × cal = safe.
void ensureFrontClearance() {
    int safeLF = min((int)(calLF * TURN_CLEAR_FRAC), 4090);
    int safeRF = min((int)(calRF * TURN_CLEAR_FRAC), 4090);
    sampleIR();
    if (irVal[0] < safeLF && irVal[3] < safeRF) return;
    // Slow reverse, capped at ~20mm to avoid retreating far from cell center.
    // ~2 ticks/mm at TICKS_PER_REV=210, WHEEL_DIA=33.4mm → 40 ticks ≈ 20mm.
    constexpr long BACKUP_LIMIT_TICKS = 40;
    long startL = leftEnc.getTicks();
    long startR = rTicks();
    leftMotor.drive(-DRIVE_PWM_MIN);
    rightMotor.drive(-DRIVE_PWM_MIN);
    unsigned long t0 = millis();
    while (millis() - t0 < 500) {
        sampleIR();
        long dL = abs(leftEnc.getTicks() - startL);
        long dR = abs(rTicks() - startR);
        if ((dL + dR) / 2 >= BACKUP_LIMIT_TICKS) break;
        if (irVal[0] < safeLF - 150 && irVal[3] < safeRF - 150) break;
    }
    stopMotors();
    delay(60);
}

void rotateToHeading(AbsDir target) {
    int diff = ((int)target - (int)robotHeading + 4) % 4;
    if (diff == 0) return;
    ensureFrontClearance();
    if      (diff == 1) { turnRight();  robotHeading = (AbsDir)(((int)robotHeading + 1) % 4); }
    else if (diff == 3) { turnLeft();   robotHeading = (AbsDir)(((int)robotHeading + 3) % 4); }
    else if (diff == 2) { turnAround(); robotHeading = (AbsDir)(((int)robotHeading + 2) % 4); }
}

void setupMaze() {
    maze.reset();
    for (int c = 0; c < MAZE_COLS; c++) maze.setWall(MAZE_ROWS - 1, c, DIR_NORTH, true);
    for (int r = 0; r < MAZE_ROWS; r++) maze.setWall(r, MAZE_COLS - 1, DIR_EAST,  true);
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
    xTaskCreatePinnedToCore(speedTask, "speed", 2048, NULL, 2, NULL, 0);
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
                // Recovery: turnAround once, re-sense, re-flood. Pro mice
                // never crash on first no-path — assume wall mis-mark.
                static int noPathRetries = 0;
                if (noPathRetries < 2) {
                    noPathRetries++;
                    stopMotors();
                    turnAround();
                    robotHeading = (AbsDir)(((int)robotHeading + 2) % 4);
                    break;  // re-enter RUN, sense again with new heading
                }
                noPathRetries = 0;
                stopMotors();
                crashFlag = true;
                crashRow = robotRow; crashCol = robotCol;
                crashHeading = robotHeading;
                for (int i = 0; i < 4; i++) crashIR[i] = irVal[i];
                crashReason = "no path x3";
                crashDrawn = false;
                robotState = CRASH;
                break;
            }

            // If a turn is needed, advance to cell center first so pivot
            // axis aligns with cell center (clean exit lane post-turn).
            if (best != robotHeading) {
                advanceToCellCenter();
            }
            rotateToHeading(best);
            driveChain();   // chains straight cells, updates robotRow/Col
            if (crashFlag) { crashDrawn = false; robotState = CRASH; }
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
            // crashDrawn reset each time we enter CRASH (set in RUN/no-path paths).
            if (!crashDrawn) { oledCrash(); crashDrawn = true; }
            if (buttonEdge()) {
                crashFlag = false;
                crashDrawn = false;
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE;
            }
            break;
        }
    }
}
