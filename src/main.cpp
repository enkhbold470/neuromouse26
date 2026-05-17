// src/main.cpp — Micromouse26 flood-fill solver (SOTA motion stack)
//
// Motion: cascaded velocity PI (speed loop on (vL+vR)/2, straightness loop
// on (curL-curR) tick mismatch) with shared feed-forward derived from
// per-wheel kV. Gyro yaw + IR side-wall lateral bias layered on top.
// Calibrated globals live in PinConfig.h (VPID_*, KV_L/R, RIGHT_ENC_SCALE,
// MOTOR_PWM_FREQ_HZ=200). Re-run on-device CAL in test/velocity-pid-ble.cpp
// to refresh after a hardware change, then copy the values back into
// PinConfig.h.
//
// Turn: trapezoidal angular-velocity profile + Kff + PID on integrated
// MPU-6500 yaw. Surface-independent.
//
// Menu:
//   Cal IR       — capture LF/L/R/RF in dead-end (centered)
//   Cal Gyro     — capture gz bias (300 samples, STILL)
//   Test Motor   — short fwd/rev each motor
//   Test Encoder — live L/R tick counts
//   Test IR      — live 4-bar graph
//   START        — flood-fill solver

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "IRCalibration.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseMaze.h"

// ── OLED ─────────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ── Maze geometry ────────────────────────────────────────────────────────────
constexpr uint8_t MAZE_ROWS = 6;
constexpr uint8_t MAZE_COLS = 3;
constexpr uint8_t GOAL_ROW  = 5;
constexpr uint8_t GOAL_COL  = 2;

// ── Hardware ─────────────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);
MicromouseMaze    maze;

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ── IR ───────────────────────────────────────────────────────────────────────
// 30° outboard mount — see PinConfig.h geometry note.
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

// IR calibration captured in dead-end (centered, all 4 walls present).
// Defaults loaded from PinConfig.h; "Cal IR" menu or BLE-cal updates RAM copy.
static int calLF = IR_CAL_LF;
static int calL  = IR_CAL_L;
static int calR  = IR_CAL_R;
static int calRF = IR_CAL_RF;

static inline bool wallFront() {
    return irVal[0] > WALL_FRONT_THRESH || irVal[3] > WALL_FRONT_THRESH;
}
static inline bool wallLeft()  { return irVal[1] > WALL_SIDE_THRESH; }
static inline bool wallRight() { return irVal[2] > WALL_SIDE_THRESH; }

// ── Robot pose ───────────────────────────────────────────────────────────────
uint8_t robotRow = 0;
uint8_t robotCol = 0;
AbsDir  robotHeading = DIR_NORTH;

// ── Crash report ─────────────────────────────────────────────────────────────
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
// Gyro full-scale ±1000 dps → 32.8 LSB/(°/s). Must exceed TURN_PEAK_OMEGA_DPS.
#define GYRO_FS_SEL       0x10
#define GYRO_SCALE        32.8f

struct ImuRaw { int16_t ax, ay, az, temp, gx, gy, gz; };

static float gyroBiasZ = 0.0f;
static float yaw = 0.0f;
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

// ── Battery voltage (for FF compensation) ────────────────────────────────────
static float readVbat() {
    int raw = analogRead(BAT_V_SENSE);
    return (raw / 4095.0f) * 3.3f * BAT_VDIV_MULT;
}

// ── Turn (trapezoidal ω + PID on yaw) ────────────────────────────────────────
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
            w = 0;
            a = target;
        }
        angle_des = sign * a;
        omega_des = sign * w;
    }
};

void stopMotors()    { leftMotor.brake(); rightMotor.brake(); }
void encodersReset() { leftEnc.reset(); rightEnc.reset(); }

void doTurn(float targetDeg) {
    yaw = 0;
    lastImuUs = micros();
    TurnProfile prof; prof.init(targetDeg);

    unsigned long t0     = millis();
    unsigned long lastUs = micros();
    float prevErr  = 0;
    float peakErr  = 0;
    unsigned long inBandStart = 0;

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

        float pwm = TURN_KFF_PWM_PER_DPS * omega_des
                  + TURN_KP_PWM_PER_DEG  * err
                  + TURN_KD_PWM_PER_DPS  * derr;

        bool holding = (t >= prof.t_tot);
        if (holding && fabsf(pwm) > 1.0f && fabsf(pwm) < TURN_MIN_HOLD_PWM) {
            pwm = (pwm > 0) ? TURN_MIN_HOLD_PWM : -TURN_MIN_HOLD_PWM;
        }
        if (pwm >  TURN_PWM) pwm =  TURN_PWM;
        if (pwm < -TURN_PWM) pwm = -TURN_PWM;

        leftMotor.drive(-(int)pwm);
        rightMotor.drive( (int)pwm);

        if (fabsf(err) > peakErr) peakErr = fabsf(err);

        if (holding && fabsf(targetDeg - yaw) <= TURN_DEADBAND_DEG) {
            if (inBandStart == 0) inBandStart = nowMs;
            if (nowMs - inBandStart >= TURN_HOLD_MS) break;
        } else {
            inBandStart = 0;
        }
        if (nowMs - t0 > TURN_TIMEOUT_MS) break;
    }
    stopMotors();
    unsigned long settleStart = millis();
    while (millis() - settleStart < TURN_SETTLE_MS) { updateYaw(); }
    Serial.printf("[TURN] tgt=%+.1f fin=%+.2f err=%+.2f peakProf=%.2f\n",
                  targetDeg, yaw, targetDeg - yaw, peakErr);
}

void turnRight() { doTurn(-90); }
void turnLeft()  { doTurn(+90); }
void turnAround(){ doTurn(-180); }

// ── Cascaded velocity PI — SOTA inner-loop drive ─────────────────────────────
// One iteration of the cascade. Caller manages encoder reset, target velocity,
// and termination condition (e.g. cell-boundary reached).
//
// State is in/out: `intSpeed`, `velL_ema`, `velR_ema`, `prevL`, `prevR`.
// `extraLeftBias` = IR + yaw lateral correction (positive → steer right).
struct VpidState {
    float intSpeed;
    float velL_ema;
    float velR_ema;
    long  prevL;
    long  prevR;
    float vScale;        // NOMINAL_VBAT / Vbat at run start
    void reset(float vbat) {
        intSpeed = 0;
        velL_ema = velR_ema = 0;
        prevL = prevR = 0;
        float vb = (vbat > 5.5f) ? vbat : NOMINAL_VBAT;
        vScale = NOMINAL_VBAT / vb;
    }
};

static int vpidFeedforward(float target) {
    if (target <= 0.0f) return 0;
    float kV_avg  = 0.5f * (KV_L + KV_R);
    float off_avg = 0.5f * (OFF_L + OFF_R);
    if (kV_avg <= 0.0f) return 0;
    float pwm = (target / kV_avg) + off_avg;
    return (int)constrain(pwm, 0.0f, (float)MOTOR_PWM_MAX);
}

// One 5 ms control step. Sets pwmL/pwmR from current encoder state + target.
// `lateralBias` adds to L pwm (positive = steer right, by speeding L vs R).
static void vpidStep(VpidState& s, float target, long curL, long curR,
                     int lateralBias, int& pwmL, int& pwmR) {
    float dt = VPID_LOOP_US / 1000000.0f;
    float instL = ((curL - s.prevL) * MM_PER_TICK) / dt;
    float instR = ((curR - s.prevR) * MM_PER_TICK) / dt;
    s.prevL = curL; s.prevR = curR;
    s.velL_ema += VPID_EMA_ALPHA * (instL - s.velL_ema);
    s.velR_ema += VPID_EMA_ALPHA * (instR - s.velR_ema);

    float velAvg   = 0.5f * (s.velL_ema + s.velR_ema);
    float errSpeed = target - velAvg;
    s.intSpeed = constrain(s.intSpeed + errSpeed * dt,
                           -VPID_INTEG_LIM, VPID_INTEG_LIM);
    int   pidSpeed = (int)(VPID_LOOP_KP * errSpeed + VPID_LOOP_KI * s.intSpeed);

    long  straightErr = curL - curR;
    int   pidStraight = (int)constrain(
        (float)(VPID_STRAIGHT_KP * straightErr),
        (float)-VPID_STRAIGHT_MAX, (float)VPID_STRAIGHT_MAX);

    int ffBase = (int)(vpidFeedforward(target) * s.vScale);

    int basePwm = ffBase + pidSpeed;
    pwmL = constrain(basePwm - pidStraight + L_PWM_BIAS + lateralBias,
                     0, MOTOR_PWM_MAX);
    pwmR = constrain(basePwm + pidStraight + R_PWM_BIAS - lateralBias,
                     0, MOTOR_PWM_MAX);
}

// ── Wall sensing ─────────────────────────────────────────────────────────────
void senseWalls() {
    AbsDir leftDir  = (AbsDir)(((int)robotHeading + 3) % 4);
    AbsDir rightDir = (AbsDir)(((int)robotHeading + 1) % 4);
    maze.setWall(robotRow, robotCol, robotHeading, wallFront());
    maze.setWall(robotRow, robotCol, leftDir,      wallLeft());
    maze.setWall(robotRow, robotCol, rightDir,     wallRight());
}

void ensureFrontClearance() {
    int safeLF = min((int)(calLF * TURN_CLEAR_FRAC), 4090);
    int safeRF = min((int)(calRF * TURN_CLEAR_FRAC), 4090);
    sampleIR();
    if (irVal[0] < safeLF && irVal[3] < safeRF) return;
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

// ── Half-cell advance before pivot (cascade-driven) ──────────────────────────
constexpr long CENTER_ADVANCE_TICKS = TICKS_PER_CELL / 2;

void advanceToCellCenter() {
    leftEnc.reset(); rightEnc.reset();
    long refL = leftEnc.getTicks(), refR = rTicks();
    yaw = 0; lastImuUs = micros();

    VpidState st; st.reset(readVbat());
    unsigned long nextUs = micros();
    unsigned long t0     = millis();

    while (true) {
        while ((long)(micros() - nextUs) < 0) {}
        nextUs += VPID_LOOP_US;
        updateYaw();
        sampleIR();

        long curL = leftEnc.getTicks();
        long curR = rTicks();
        long avg  = ((curL - refL) + (curR - refR)) / 2;
        if (avg >= CENTER_ADVANCE_TICKS) break;
        if (millis() - t0 > 1500) break;

        // Yaw + IR lateral bias (positive → steer right)
        constexpr float IR_CTR_K = 0.10f;
        constexpr float YAW_K    = 3.0f;
        float irCorr = 0;
        bool wL = irVal[1] > WALL_SIDE_THRESH;
        bool wR = irVal[2] > WALL_SIDE_THRESH;
        if (wL && wR)  irCorr = IR_CTR_K * ((irVal[1]-calL) - (irVal[2]-calR));
        else if (wL)   irCorr = IR_CTR_K *  (irVal[1]-calL);
        else if (wR)   irCorr = IR_CTR_K * -(irVal[2]-calR);
        int bias = constrain((int)(yaw * YAW_K + irCorr), -200, 200);

        int pwmL, pwmR;
        vpidStep(st, (float)CELL_TARGET_MMS, curL, curR, bias, pwmL, pwmR);
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);
    }
    stopMotors();
}

// ── Drive forward, chaining same-heading cells (cascade-driven) ──────────────
// Replaces the old bespoke trapezoid + per-wheel PI. Inner loop is the
// cascaded velocity PI; cell-boundary detection, mid-cell brake, crash
// guard, and lateral bias from IR/yaw stay the same.
void driveChain() {
    leftEnc.reset(); rightEnc.reset();
    long refL = leftEnc.getTicks(), refR = rTicks();
    yaw = 0; lastImuUs = micros();

    VpidState st; st.reset(readVbat());
    unsigned long nextUs = micros();
    long cellBoundary  = TICKS_PER_CELL;
    unsigned long startMs = millis();
    unsigned long lastIR  = 0;

    constexpr float YAW_KP     = 3.0f;
    constexpr float YAW_KI     = 0.3f;
    constexpr float IR_ALIGN_K = 0.005f;
    constexpr int   LATERAL_MAX = 200;
    float yawInteg = 0.0f;

    while (true) {
        while ((long)(micros() - nextUs) < 0) {}
        nextUs += VPID_LOOP_US;
        updateYaw();

        long curL = leftEnc.getTicks();
        long curR = rTicks();
        long tL   = curL - refL;
        long tR   = curR - refR;
        long avg  = (tL + tR) / 2;

        // Periodic IR sample (every 25 ms).
        if (millis() - lastIR > 25) {
            sampleIR();
            lastIR = millis();
        }

        // Imminent-crash brake — both front sensors very close.
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

        // Mid-cell bump guard: front sensor crosses MID_BRAKE_FRAC of cal
        // (typical when a side opens and lateral reference is lost).
        int midBrakeLF = (int)(calLF * MID_BRAKE_FRAC);
        int midBrakeRF = (int)(calRF * MID_BRAKE_FRAC);
        bool frontClose = (irVal[0] > midBrakeLF || irVal[3] > midBrakeRF);
        bool pastThird  = (avg > TICKS_PER_CELL / 3);

        if (avg >= cellBoundary || (frontClose && pastThird)) {
            stopMotors();
            delay(100);
            robotRow += DIR_DR[robotHeading];
            robotCol += DIR_DC[robotHeading];
            maze.visited[robotRow][robotCol] = true;
            AbsDir back = (AbsDir)(((int)robotHeading + 2) % 4);
            maze.setWall(robotRow, robotCol, back, false);
            sampleIR();
            senseWalls();
            maze.floodFill();

            if (maze.isGoal(robotRow, robotCol)) return;
            uint8_t bd;
            AbsDir best = maze.bestDirectionBiased(robotRow, robotCol, robotHeading, bd);
            if (bd == FLOOD_INFINITY || best != robotHeading) {
                return;  // RUN handles turn / no-path
            }

            // Continue straight — seed next cell's yaw bias from IR center error.
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
            lastImuUs = micros();
            // Reset cascade state too — new cell, fresh integrator.
            st.reset(readVbat());
            st.prevL = curL; st.prevR = curR;
        }

        // Lateral bias (IR centering + yaw hold), summed to pwmL+/pwmR-.
        float irBias = 0;
        {
            bool wLl = irVal[1] > WALL_SIDE_THRESH;
            bool wRl = irVal[2] > WALL_SIDE_THRESH;
            if (wLl && wRl)    irBias = IR_ALIGN_K * ((irVal[1]-calL) - (irVal[2]-calR));
            else if (wLl)      irBias = IR_ALIGN_K *  (irVal[1]-calL);
            else if (wRl)      irBias = IR_ALIGN_K * -(irVal[2]-calR);
            irBias = constrain(irBias, -5.0f, 5.0f);
        }
        yawInteg = constrain(yawInteg + yaw * 0.001f, -8.0f, 8.0f);
        int lateralBias = (int)((yaw + irBias) * YAW_KP + yawInteg * YAW_KI);
        lateralBias = constrain(lateralBias, -LATERAL_MAX, LATERAL_MAX);

        int pwmL, pwmR;
        vpidStep(st, (float)CELL_TARGET_MMS, curL, curR, lateralBias, pwmL, pwmR);
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (millis() - startMs > (unsigned long)(TIMEOUT_MS * 10)) {
            stopMotors();
            return;
        }
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
constexpr long ENC_PER_MENU_STEP = 20;

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
void setupMaze() {
    maze.reset();
    for (int c = 0; c < MAZE_COLS; c++) maze.setWall(MAZE_ROWS - 1, c, DIR_NORTH, true);
    for (int r = 0; r < MAZE_ROWS; r++) maze.setWall(r, MAZE_COLS - 1, DIR_EAST,  true);
    maze.setGoalSingle(GOAL_ROW, GOAL_COL);
    maze.floodFill();
}

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

    uint8_t who = 0;
    mpuRead(REG_WHO_AM_I, &who, 1);
    mpuWrite(REG_PWR_MGMT_1, 0x00); delay(50);
    mpuWrite(REG_GYRO_CFG,   GYRO_FS_SEL);
    mpuWrite(REG_ACCEL_CFG,  0x00);
    lastImuUs = micros();
    Serial.printf("[INIT] MPU WHO=0x%02X\n", who);

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
                static int noPathRetries = 0;
                if (noPathRetries < 2) {
                    noPathRetries++;
                    stopMotors();
                    turnAround();
                    robotHeading = (AbsDir)(((int)robotHeading + 2) % 4);
                    break;
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

            if (best != robotHeading) {
                advanceToCellCenter();
            }
            rotateToHeading(best);
            driveChain();
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
