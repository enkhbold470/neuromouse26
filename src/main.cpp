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
// Menu (gyro bias auto-captured every boot, IR cal baked into PinConfig.h):
//   Test Motor   — short fwd/rev each motor
//   Test Encoder — live L/R tick counts
//   Test IR      — live 4-bar graph
//   START        — flood-fill solver

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include "PinConfig.h"
#include "IRCalibration.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseMaze.h"

// ── Persistent maze (NVS) ────────────────────────────────────────────────────
// Saves the 16x16 wall bitmask grid (256 bytes) under namespace "mm26".
// GOAL screen → button = save. Menu "FAST" loads + runs without re-exploring.
static Preferences prefs;
static constexpr const char* NVS_NS         = "mm26";
static constexpr const char* NVS_KEY_WALLS  = "walls";

static bool hasSavedMaze() {
    if (!prefs.begin(NVS_NS, true)) return false;
    bool found = prefs.isKey(NVS_KEY_WALLS);
    prefs.end();
    return found;
}
static bool saveMazeFlash(const MicromouseMaze& m) {
    if (!prefs.begin(NVS_NS, false)) return false;
    size_t n = prefs.putBytes(NVS_KEY_WALLS, m.walls, sizeof(m.walls));
    prefs.end();
    return n == sizeof(m.walls);
}
static bool loadMazeFlash(MicromouseMaze& m) {
    if (!prefs.begin(NVS_NS, true)) return false;
    bool ok = false;
    if (prefs.isKey(NVS_KEY_WALLS)) {
        size_t n = prefs.getBytes(NVS_KEY_WALLS, m.walls, sizeof(m.walls));
        ok = (n == sizeof(m.walls));
    }
    prefs.end();
    return ok;
}
static bool clearSavedMaze() {
    if (!prefs.begin(NVS_NS, false)) return false;
    bool ok = prefs.remove(NVS_KEY_WALLS);
    prefs.end();
    return ok;
}

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

// Set true on START so first driveChain travels (cell pitch + back-wall
// offset) instead of one cell pitch. Robot is assumed to begin with its
// back pressed against the south wall of cell (0,0); axle ≈ 30 mm from
// wall, so reaching cell (1,0) center needs ~60 mm extra travel.
static bool firstCellOfRun = false;

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

// Mirrors test/mpu6500.cpp doTurn() — same trapezoid+FF+PID structure that
// converges cleanly in standalone test. Symmetric PWM (no kV scaling) and
// holding-only min-PWM clamp; do not add an always-on clamp because it
// overrides the accel ramp and induces oscillation.
// Returns true if turn converged inside deadband and held; false on timeout
// or final-error outside 2× deadband. Caller can flag this as a crash.
bool doTurn(float targetDeg) {
    yaw = 0;
    lastImuUs = micros();
    TurnProfile prof; prof.init(targetDeg);

    // Battery-compensated FF: low pack voltage means each PWM unit produces
    // less torque/ω. Scale the feed-forward term so the trapezoid still hits
    // the commanded peak ω as the battery sags. PID stays unscaled so it
    // remains a stability-domain correction.
    float vbat   = readVbat();
    float vScale = (vbat > 5.5f) ? (NOMINAL_VBAT / vbat) : 1.0f;

    unsigned long t0     = millis();
    unsigned long lastUs = micros();
    float prevErr  = 0;
    float peakErr  = 0;
    unsigned long inBandStart = 0;
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

        // Stiction floor only during hold phase — break it free, don't slam.
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
            if (nowMs - inBandStart >= TURN_HOLD_MS) { converged = true; break; }
        } else {
            inBandStart = 0;
        }
        if (nowMs - t0 > TURN_TIMEOUT_MS) break;
    }
    stopMotors();
    unsigned long settleStart = millis();
    while (millis() - settleStart < TURN_SETTLE_MS) { updateYaw(); }
    bool ok = converged && fabsf(targetDeg - yaw) <= 2.0f * TURN_DEADBAND_DEG;
    Serial.printf("[TURN] tgt=%+.1f fin=%+.2f err=%+.2f peakProf=%.2f %s\n",
                  targetDeg, yaw, targetDeg - yaw, peakErr,
                  ok ? "OK" : "FAIL");
    return ok;
}

// ── Pro-technique helpers ────────────────────────────────────────────────────

// Quick gyro re-bias before each turn. Bias drifts with temperature/time;
// even ~0.1°/s of drift across a 0.85 s turn = 0.085° error. ~80 ms of
// standstill sampling re-zeros it cheaply.
static void quickGyroRecal(unsigned ms = 80) {
    stopMotors();
    unsigned long t0 = millis();
    float sum = 0;
    int   n   = 0;
    while (millis() - t0 < ms) {
        ImuRaw d;
        if (imuReadAll(d)) { sum += d.gz / GYRO_SCALE; n++; }
        delayMicroseconds(500);
    }
    if (n > 0) gyroBiasZ = sum / n;
    yaw = 0;
    lastImuUs = micros();
}

// Square robot perpendicular to the front wall using LF/RF reading
// difference. At 30° outboard mount, if robot rotated left of perpendicular,
// LF beam strikes wall closer to normal → higher reading; RF reads less.
// Pivot until |LF - RF| < tolerance. Side benefit: zeros yaw, corrects
// approach-pose drift before the next turn or drive.
static void squareToFrontWall(unsigned long timeoutMs = 1200) {
    constexpr float SQUARE_KP        = 0.30f;  // pwm per IR-unit diff
    constexpr int   SQUARE_MAX_PWM   = 120;
    constexpr int   SQUARE_MIN_PWM   = 70;     // motor deadband
    constexpr int   SQUARE_TOL       = 40;     // |LF-RF| accept band
    constexpr unsigned long HOLD_MS  = 150;

    unsigned long t0 = millis();
    unsigned long inBand = 0;
    while (millis() - t0 < timeoutMs) {
        sampleIR();
        // Bail if front wall no longer visible (drifted away).
        if (irVal[0] < WALL_FRONT_THRESH || irVal[3] < WALL_FRONT_THRESH) break;
        int diff = irVal[0] - irVal[3];   // +: LF closer → robot rotated left
        if (abs(diff) <= SQUARE_TOL) {
            if (inBand == 0) inBand = millis();
            if (millis() - inBand >= HOLD_MS) { stopMotors(); break; }
            stopMotors();
            delay(10);
            continue;
        } else {
            inBand = 0;
        }
        int pwm = (int)constrain(SQUARE_KP * (float)diff,
                                 (float)-SQUARE_MAX_PWM, (float)SQUARE_MAX_PWM);
        if (pwm > 0 && pwm < SQUARE_MIN_PWM) pwm = SQUARE_MIN_PWM;
        if (pwm < 0 && pwm > -SQUARE_MIN_PWM) pwm = -SQUARE_MIN_PWM;
        // +pwm = CW pivot (R wheel reverse, L wheel forward) corrects left
        // rotation. Same kV scaling as doTurn for symmetric pivot.
        constexpr float kV_avg = 0.5f * (KV_L + KV_R);
        leftMotor.drive( (int)( pwm * (kV_avg / KV_L)));
        rightMotor.drive((int)(-pwm * (kV_avg / KV_R)));
        delay(8);
    }
    stopMotors();
    yaw = 0;
    lastImuUs = micros();
}

bool turnRight() { quickGyroRecal();  return doTurn(-90); }
bool turnLeft()  { quickGyroRecal();  return doTurn(+90); }
// 180° as two chained 90°s — single 180° doubles momentum/error budget and
// is the least-reliable single move. Two 90°s with a brief settle between
// gives independent error budgets and lets gyro re-zero mid-spin.
//
// Carry residual from first 90° into the second 90°'s target so total =
// -180° regardless of first-turn undershoot. Without this, two +2°
// undershoots produce -176° → robot drives at 4° angle into wall.
bool turnAround() {
    quickGyroRecal();
    bool ok1 = doTurn(-90);
    float residual = -90.0f - yaw;          // captured BEFORE the re-zero
    delay(80);
    quickGyroRecal();                       // re-zeros bias AND yaw
    bool ok2 = doTurn(-90.0f + residual);   // compensates first-turn error
    return ok1 && ok2;
}

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
// GEOMETRY NOTE — side IR sensors aim 30° forward-outward and read a wall
// point ~150 mm ahead of robot center. At cell-center stop they're aimed
// into cell N+2, so reading there is the wrong cell. Side walls must be
// sensed via look-ahead during driveChain traversal (avg in 17%–100% of
// cell tick window = sensor scanning cell N+1's side). Only the front
// sensor reads the correct cell at cell-center stop.
void senseWallsFrontOnly() {
    maze.setWall(robotRow, robotCol, robotHeading, wallFront());
}
void senseWalls() {
    AbsDir leftDir  = (AbsDir)(((int)robotHeading + 3) % 4);
    AbsDir rightDir = (AbsDir)(((int)robotHeading + 1) % 4);
    maze.setWall(robotRow, robotCol, robotHeading, wallFront());
    maze.setWall(robotRow, robotCol, leftDir,      wallLeft());
    maze.setWall(robotRow, robotCol, rightDir,     wallRight());
}

// ── Side-wall look-ahead (populates next cell's side walls during approach)
// Updated every control loop while sensor is aimed past the current cell's
// far edge into the next cell. At cell-boundary stop, `applyLookaheadSides`
// writes maze walls for the cell we just entered.
struct SideLook {
    int  Lhigh, Llow;
    int  Rhigh, Rlow;
    int  n;
    void reset() { Lhigh = 0; Llow = INT_MAX; Rhigh = 0; Rlow = INT_MAX; n = 0; }
    void update(int L, int R) {
        if (L > Lhigh) Lhigh = L;  if (L < Llow) Llow = L;
        if (R > Rhigh) Rhigh = R;  if (R < Rlow) Rlow = R;
        n++;
    }
};
static SideLook sideLook = { 0, INT_MAX, 0, INT_MAX, 0 };

static void applyLookaheadSides() {
    AbsDir leftDir  = (AbsDir)(((int)robotHeading + 3) % 4);
    AbsDir rightDir = (AbsDir)(((int)robotHeading + 1) % 4);
    // Need a few clean look-ahead samples to trust the call. Approach to a
    // close front wall poisons late samples (front wall enters 30° side
    // cone) so we may end up with very few valid ones; in that case leave
    // the side walls at their prior state rather than writing phantoms.
    constexpr int MIN_LOOKAHEAD_SAMPLES = 3;
    if (sideLook.n >= MIN_LOOKAHEAD_SAMPLES) {
        bool hasL = (sideLook.Lhigh > WALL_SIDE_THRESH) &&
                    (sideLook.Llow  > WALL_SIDE_THRESH / 2);
        bool hasR = (sideLook.Rhigh > WALL_SIDE_THRESH) &&
                    (sideLook.Rlow  > WALL_SIDE_THRESH / 2);
        maze.setWall(robotRow, robotCol, leftDir,  hasL);
        maze.setWall(robotRow, robotCol, rightDir, hasR);
    }
    sideLook.reset();
}

void ensureFrontClearance() {
    int safeLF = min((int)(calLF * TURN_CLEAR_FRAC), 4090);
    int safeRF = min((int)(calRF * TURN_CLEAR_FRAC), 4090);
    sampleIR();
    if (irVal[0] < safeLF && irVal[3] < safeRF) return;
    // 60 mm of backup room — old 40-tick (~20 mm) cap was below the
    // actual dead-end clearance need. With cell-stop at front IR ~50 mm
    // sensor-to-wall the pivot radius needs the robot moved back another
    // ~30 mm for the front corners to clear during a 90°/180°.
    constexpr long BACKUP_LIMIT_TICKS = 120;
    long startL = leftEnc.getTicks();
    long startR = rTicks();
    leftMotor.drive(-DRIVE_PWM_MIN);
    rightMotor.drive(-DRIVE_PWM_MIN);
    unsigned long t0 = millis();
    while (millis() - t0 < 900) {
        sampleIR();
        long dL = abs(leftEnc.getTicks() - startL);
        long dR = abs(rTicks() - startR);
        if ((dL + dR) / 2 >= BACKUP_LIMIT_TICKS) break;
        // Exit only when BOTH front sensors drop well below the safe
        // threshold (≥150 below). Previous AND-of-margin was fine but
        // hit the tick cap first because of the 20 mm budget.
        if (irVal[0] < safeLF - 150 && irVal[3] < safeRF - 150) break;
    }
    stopMotors();
    delay(60);
}

// Returns false if any underlying doTurn() failed to converge — caller
// should NOT issue driveChain() in that case (heading is unreliable).
// robotHeading is still advanced because the maze logic depends on it
// matching the commanded direction; the crash handler will surface the
// failure so the run can be aborted rather than silently driving askew.
bool rotateToHeading(AbsDir target) {
    int diff = ((int)target - (int)robotHeading + 4) % 4;
    if (diff == 0) return true;
    ensureFrontClearance();
    bool ok = true;
    if      (diff == 1) { ok = turnRight();  robotHeading = (AbsDir)(((int)robotHeading + 1) % 4); }
    else if (diff == 3) { ok = turnLeft();   robotHeading = (AbsDir)(((int)robotHeading + 3) % 4); }
    else if (diff == 2) { ok = turnAround(); robotHeading = (AbsDir)(((int)robotHeading + 2) % 4); }
    return ok;
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

    constexpr float POS_KP_PWM     = 0.04f;
    constexpr float YAW_KP_PWM     = 4.0f;
    constexpr int   POS_BIAS_MAX   = 120;
    constexpr int   YAW_BIAS_MAX   = 150;
    constexpr int   LATERAL_MAX    = 220;
    constexpr float CRASH_BRAKE_MM = 30.0f;

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

        // Front safety — stop early if a wall is suddenly close.
        if (wallFront() && IRCal::estimateFrontDistMM(irVal[0], irVal[3]) <= CRASH_BRAKE_MM) {
            break;
        }

        bool wL = irVal[1] > WALL_SIDE_THRESH;
        bool wR = irVal[2] > WALL_SIDE_THRESH;
        int posErr = 0;
        if (wL && wR)  posErr = (irVal[1] - calL) - (irVal[2] - calR);
        else if (wL)   posErr =  (irVal[1] - calL);
        else if (wR)   posErr = -(irVal[2] - calR);

        int posBias = (int)constrain(POS_KP_PWM * (float)posErr,
                                     (float)-POS_BIAS_MAX, (float)POS_BIAS_MAX);
        int yawBias = (int)constrain(YAW_KP_PWM * yaw,
                                     (float)-YAW_BIAS_MAX, (float)YAW_BIAS_MAX);
        int bias = constrain(posBias + yawBias, -LATERAL_MAX, LATERAL_MAX);

        int pwmL, pwmR;
        vpidStep(st, (float)CELL_TARGET_MMS, curL, curR, bias, pwmL, pwmR);
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);
    }
    stopMotors();
}

// ── Drive forward, chaining same-heading cells (cascade-driven) ──────────────
// Cell boundary is IR-anchored when a front wall is present (front-LUT
// distance ≤ CELL_ANCHOR_MM ≈ robot center at cell center); falls back to
// encoder tick count when no front wall. Hard crash brake at ≤ 30 mm.
//
// Lateral hold is direct PWM bias from side-IR position error + gyro yaw,
// no longer routed through the (too-weak) YAW_KP*yaw chain.
void driveChain() {
    leftEnc.reset(); rightEnc.reset();
    long refL = leftEnc.getTicks(), refR = rTicks();
    yaw = 0; lastImuUs = micros();
    sideLook.reset();

    VpidState st; st.reset(readVbat());
    unsigned long nextUs  = micros();
    // First cell after START is longer — robot back against wall puts robot
    // center 40 mm short of cell-0 center (user-measured). Reaching cell 1
    // center therefore needs cell pitch + 40 mm.
    constexpr long FIRST_CELL_EXTRA = (long)(40.0f / MM_PER_TICK);
    long          cellBoundary = firstCellOfRun
        ? TICKS_PER_CELL + FIRST_CELL_EXTRA
        : TICKS_PER_CELL;
    // Look-ahead becomes valid once sensor has scanned past current cell's
    // far edge into the next cell — geometrically ≈ 17% of the cell.
    const long LOOKAHEAD_START_TICKS = TICKS_PER_CELL / 5;
    unsigned long startMs = millis();

    // Lateral controller — direct PWM units.
    constexpr float POS_KP_PWM   = 0.04f;   // pwm per IR-unit position error
    constexpr float YAW_KP_PWM   = 4.0f;    // pwm per degree yaw error
    constexpr int   POS_BIAS_MAX = 120;
    constexpr int   YAW_BIAS_MAX = 150;
    constexpr int   LATERAL_MAX  = 220;

    // IR-anchored stop / cell thresholds (sensor-to-wall, mm).
    // Per IRCal LUT: avg ≥ 3914 → 10 mm; 3138 → 40 mm; 2552 → 50 mm.
    constexpr float CRASH_BRAKE_MM  = 30.0f;
    constexpr float CELL_ANCHOR_MM  = 50.0f;

    while (true) {
        while ((long)(micros() - nextUs) < 0) {}
        nextUs += VPID_LOOP_US;
        updateYaw();

        long curL = leftEnc.getTicks();
        long curR = rTicks();
        long avg  = ((curL - refL) + (curR - refR)) / 2;

        // Sample IR every control loop (5 ms). Side-sensor wall→opening
        // transition lasts ~30 ms at 300 mm/s; per-loop sampling catches it.
        sampleIR();

        // ── Front-wall handling: IR is ground truth when wall present. ────────
        bool  frontWall = wallFront();
        float frontMM   = frontWall
            ? IRCal::estimateFrontDistMM(irVal[0], irVal[3])
            : 999.0f;

        // Build look-ahead estimate of NEXT cell's side walls. Sensor is
        // geometrically aimed into the next cell once we're past the first
        // ~17% of the current cell.
        //
        // POISON FILTER: at front distances <60 mm the 30° side sensors
        // pick up the FRONT wall in their forward-bias cone and report it
        // as a "side wall". Reject samples in that range — they'd corrupt
        // the look-ahead and cause a phantom side wall in the next cell.
        constexpr float SIDE_POISON_MM = 60.0f;
        if (avg > LOOKAHEAD_START_TICKS && frontMM > SIDE_POISON_MM) {
            sideLook.update(irVal[1], irVal[2]);
        }

        bool crashBrake = frontWall && frontMM <= CRASH_BRAKE_MM;
        bool irAnchor   = frontWall && frontMM <= CELL_ANCHOR_MM;
        bool encAnchor  = (avg >= cellBoundary);

        if (crashBrake || irAnchor || encAnchor) {
            stopMotors();
            if (CELL_STOP_DELAY_MS > 0) delay(CELL_STOP_DELAY_MS);
            robotRow += DIR_DR[robotHeading];
            robotCol += DIR_DC[robotHeading];
            maze.visited[robotRow][robotCol] = true;
            AbsDir back = (AbsDir)(((int)robotHeading + 2) % 4);
            maze.setWall(robotRow, robotCol, back, false);
            sampleIR();
            // Front-wall squaring: if we stopped because of a wall ahead,
            // pivot until LF == RF → robot perpendicular → yaw zeroed →
            // both lateral pose and angle are calibrated for free. Pro
            // technique: front wall is the most reliable pose reference in
            // the maze.
            //
            // Skip squaring at very close range — under ~35 mm the 30°
            // sensors saturate and LF-RF diff is noise, so pivoting on it
            // INJECTS yaw error instead of correcting it. The next
            // ensureFrontClearance() will back the robot up before the
            // turn, removing the need.
            if (wallFront()) {
                float fdMM = IRCal::estimateFrontDistMM(irVal[0], irVal[3]);
                if (fdMM > 35.0f && fdMM < 80.0f) {
                    squareToFrontWall();
                    sampleIR();   // refresh after pivot
                }
            }
            // Front wall is sampled correctly at cell-center. Sides use
            // look-ahead data captured during the approach.
            senseWallsFrontOnly();
            applyLookaheadSides();
            firstCellOfRun = false;
            maze.floodFill();

            if (maze.isGoal(robotRow, robotCol)) return;
            uint8_t bd;
            AbsDir best = maze.bestDirectionBiased(robotRow, robotCol, robotHeading, bd);
            if (bd == FLOOD_INFINITY || best != robotHeading) {
                return;
            }
            // If we stopped because wall is right ahead, can't go straight.
            if (crashBrake) return;

            // Continue straight — seed next-cell yaw bias from IR center error.
            float seed = 0;
            int   align = 0;
            bool  wLn = wallLeft();
            bool  wRn = wallRight();
            if (wLn && wRn) align = (irVal[1] - calL) - (irVal[2] - calR);
            else if (wLn)   align =  (irVal[1] - calL);
            else if (wRn)   align = -(irVal[2] - calR);
            seed = constrain(0.005f * (float)align, -5.0f, 5.0f);

            refL = leftEnc.getTicks();
            refR = rTicks();
            cellBoundary = TICKS_PER_CELL;
            yaw          = seed;
            lastImuUs    = micros();
            st.reset(readVbat());
            st.prevL = curL; st.prevR = curR;
            continue;
        }

        // ── Lateral bias (direct PWM units) ───────────────────────────────────
        // posErr > 0  ↔ drifted left (L sensor closer to wall, R farther).
        // Positive bias → L faster, R slower → robot steers right. ✓
        bool wL = irVal[1] > WALL_SIDE_THRESH;
        bool wR = irVal[2] > WALL_SIDE_THRESH;
        int posErr = 0;
        if (wL && wR)   posErr = (irVal[1] - calL) - (irVal[2] - calR);
        else if (wL)    posErr =  (irVal[1] - calL);
        else if (wR)    posErr = -(irVal[2] - calR);

        int posBias = (int)constrain(POS_KP_PWM * (float)posErr,
                                     (float)-POS_BIAS_MAX, (float)POS_BIAS_MAX);
        int yawBias = (int)constrain(YAW_KP_PWM * yaw,
                                     (float)-YAW_BIAS_MAX, (float)YAW_BIAS_MAX);
        int lateralBias = constrain(posBias + yawBias, -LATERAL_MAX, LATERAL_MAX);

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
enum State { IDLE, TEST_MOTOR, TEST_ENC, TEST_IR, RUN, GOAL, CRASH };
State robotState = IDLE;

enum MenuItem {
    M_TEST_MOTOR = 0, M_TEST_ENC, M_TEST_IR,
    M_START, M_FAST, M_CLEAR_SAVE,
    M_COUNT
};
static const char* MENU_LABELS[M_COUNT] = {
    "Test Motor", "Test Encoder", "Test IR",
    "START", "FAST (load)", "Clear Save"
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
// 2S LiPo: empty 6.0V (3.0/cell), full 8.4V (4.2/cell). Fill bar maps to %.
static void drawBatteryIcon(int x, int y) {
    float vb  = readVbat();
    float pct = constrain((vb - 6.0f) / (8.4f - 6.0f), 0.0f, 1.0f);
    constexpr int W = 14, H = 6;
    oled.drawFrame(x, y, W, H);
    oled.drawBox(x + W, y + 2, 2, H - 4);
    int fill = (int)(pct * (W - 2) + 0.5f);
    if (fill > 0) oled.drawBox(x + 1, y + 1, fill, H - 2);
}

static void drawBatteryHeader(int yBaseline) {
    char vbuf[8];
    snprintf(vbuf, sizeof(vbuf), "%.1fV", readVbat());
    int vw = oled.getStrWidth(vbuf);
    int iconX = 128 - 16;          // 14W + 2 tip
    int iconY = yBaseline - 7;
    drawBatteryIcon(iconX, iconY);
    oled.drawStr(iconX - vw - 2, yBaseline, vbuf);
}

void oledMenu() {
    const int VIS = 5;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > M_COUNT - VIS) top = M_COUNT - VIS;
    if (top < 0) top = 0;

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "MM26");
    drawBatteryHeader(8);
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
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "RUN");
    drawBatteryHeader(8);
    oled.setFont(u8g2_font_6x12_tf);
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

// FAST mode setup — load saved walls from NVS, clear visited/flood, re-flood.
// Falls back to fresh setupMaze() if no save exists.
bool setupMazeFast() {
    if (!loadMazeFlash(maze)) {
        setupMaze();
        return false;
    }
    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            maze.flood[r][c]   = FLOOD_INFINITY;
            maze.visited[r][c] = false;
        }
    }
    maze.setGoalSingle(GOAL_ROW, GOAL_COL);
    maze.floodFill();
    return true;
}

// Tiny centered-text screen for save/load feedback.
void oledShortMsg(const char* line) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_8x13B_tf);
    int w = oled.getStrWidth(line);
    oled.drawStr((128 - w) / 2, 38, line);
    oled.sendBuffer();
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
                    case M_TEST_MOTOR: robotState = TEST_MOTOR; break;
                    case M_TEST_ENC:   leftEnc.reset(); rightEnc.reset();
                                       oledEncoderTest();
                                       robotState = TEST_ENC; break;
                    case M_TEST_IR:    sampleIR(); oledBars();
                                       robotState = TEST_IR; break;
                    case M_START:      for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(333); }
                                       robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
                                       firstCellOfRun = true;
                                       setupMaze();
                                       oledRunStatus("go");
                                       robotState = RUN; break;
                    case M_FAST: {
                                       bool loaded = setupMazeFast();
                                       oledShortMsg(loaded ? "LOADED" : "NO SAVE");
                                       delay(700);
                                       if (!loaded) { oledMenu(); break; }
                                       for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(333); }
                                       robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
                                       firstCellOfRun = true;
                                       oledRunStatus("FAST");
                                       robotState = RUN;
                                       break;
                                  }
                    case M_CLEAR_SAVE: {
                                       bool ok = clearSavedMaze();
                                       oledShortMsg(ok ? "CLEARED" : "EMPTY");
                                       delay(700);
                                       oledMenu();
                                       break;
                                  }
                }
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
            // At cell-center, only the front sensor reads the correct cell.
            // Side walls of the current cell were populated by the previous
            // driveChain's look-ahead window during approach.
            senseWallsFrontOnly();
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
            bool turnOk = rotateToHeading(best);
            if (!turnOk) {
                // Turn failed to converge — heading is unreliable. Abort
                // the run rather than driving askew into a wall.
                stopMotors();
                crashFlag = true;
                crashRow = robotRow; crashCol = robotCol;
                crashHeading = robotHeading;
                for (int i = 0; i < 4; i++) crashIR[i] = irVal[i];
                crashReason = "turn fail";
                crashDrawn = false;
                robotState = CRASH;
                break;
            }
            driveChain();
            if (crashFlag) { crashDrawn = false; robotState = CRASH; }
            break;
        }

        case GOAL: {
            stopMotors();
            static bool drawn = false;
            if (!drawn) { oledRunStatus("GOAL btn=save"); drawn = true; }
            if (buttonEdge()) {
                drawn = false;
                bool ok = saveMazeFlash(maze);
                oledShortMsg(ok ? "MAZE SAVED" : "SAVE FAIL");
                delay(900);
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
