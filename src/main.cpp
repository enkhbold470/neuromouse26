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
#include "IRCalibration.h"

// ═══════════════════════════════════════════════════════════════════════════
//                  ★★★  ALL TUNABLE KNOBS LIVE HERE  ★★★
// ═══════════════════════════════════════════════════════════════════════════
// If a value affects motion, speed, power, alignment, or geometry, it should
// be in this block. No other tuning constants in the file — search ends here.
// Grouped sections:
//
//   [A] ROBOT + MAZE GEOMETRY    — chassis, wheel, cell pitch
//   [B] FORWARD MOTION           — trapezoidal velocity, FF, PID
//   [C] PIVOT / SPOT TURNS       — yaw PID, power, settle, stall
//   [D] IR CENTERING             — keep robot mid-corridor while moving
//   [E] DEAD-END HANDLING        — front-wall align + 180° exit script
//   [F] SCRIPT GEOMETRY          — cell tick count + boot pose offset
//   [G] FWD_TO_WALL PHASE        — legacy primitive, kept for future use
//   [H] DEBUG FLAGS              — Serial telemetry + sub-system enables
// ═══════════════════════════════════════════════════════════════════════════

// ── [A] ROBOT + MAZE GEOMETRY (mm) ───────────────────────────────────────────
// Chassis 95×85, wheel center 55 mm from front edge. Cell pitch 180 mm
// centre-to-centre, wall-to-wall opening 170 mm (10 mm of shared wall
// thickness between adjacent cells). Robot 85 mm wide in 170 mm cell →
// 42.5 mm side clearance each side.
constexpr float ROBOT_LEN_MM        =  95.0f;
constexpr float ROBOT_WIDTH_MM      =  85.0f;
constexpr float WHEEL_FRONT_OFF_MM  =  55.0f;
constexpr float CELL_PITCH_MM       = 180.0f;
constexpr float CELL_INNER_MM       = 170.0f;
constexpr float CELL_SIDE_GAP_MM    =  42.5f;
// Motor reference (informational, not consumed by code):
//   N20 brushed DC, 1:30 gearbox, 500 RPM no-load @ 6V, 0.39 kg·cm stall.

// ── [B0] ★★★ MAIN POWER KNOB ★★★ ─────────────────────────────────────────────
// Single source-of-truth for motor breakaway PWM on THIS robot at the CURRENT
// MOTOR_PWM_FREQ_HZ. Every stiction floor and PWM cap below is derived from
// this as a fixed ratio (× 10 / 10 form keeps it constexpr-integer-only).
// Bump this ONE number and the robot gets stronger everywhere proportionally.
//
// Things NOT derived (independent physics, leave alone):
//   POS_KP / POS_KD / YAW_KP / YAW_KD     — closed-loop PID gains
//   FWD_KV_SLOPE                          — velocity-FF slope (PWM per tps)
//   FWD_V_CRUISE_TPS / FWD_ACCEL_TPS2     — trapezoidal profile
//
// Tuning recipe:
//   1. Start with a low FAST_RUN_CRUISE_TPS (≈ 600).
//   2. Watch wheels cold-start. Buzz w/o rotation → BASE too low.
//      Wheels jerk hard / overshoot     → BASE too high.
//   3. Add 20 % headroom over the threshold so dirty wheels still go.
//   4. Changing MOTOR_PWM_FREQ_HZ invalidates BASE — re-tune (higher freq
//      = less peak current per cycle = needs HIGHER BASE).
constexpr int      BASE_BREAKAWAY_PWM    = 110;

// Derived ratios. `(BASE × N) / 10` is an integer-constexpr way to write
// N/10× without floats. Caps inline-clamped to MOTOR_PWM_MAX (1023, 10-bit).
constexpr int      FWD_STICTION_FF     = (BASE_BREAKAWAY_PWM * 10) / 10;   // 1.0×
constexpr int      ALIGN_PWM           = (BASE_BREAKAWAY_PWM * 18) / 10;   // 1.8× — slow creep needs breakaway headroom
constexpr int      POS_STICTION_PWM    = (BASE_BREAKAWAY_PWM * 12) / 10;   // 1.2× — non-FWD pos floors
constexpr int      YAW_STICTION_PWM    = (BASE_BREAKAWAY_PWM * 15) / 10;   // 1.5× — turns drag more
constexpr int      POS_MAX_PWM         = MOTOR_PWM_MAX;   // hard cap; speed actually limited by dynMax in PH_FORWARD
constexpr int      YAW_MAX_PWM         = ((BASE_BREAKAWAY_PWM * 16) / 10 < MOTOR_PWM_MAX)
                                          ? ((BASE_BREAKAWAY_PWM * 16) / 10) : MOTOR_PWM_MAX;   // 1.6× cap — slow turns = cleaner gyro integration

// ── [B] FORWARD MOTION ───────────────────────────────────────────────────────
// Trapezoidal velocity profile drives a smoothly-moving position setpoint.
// Position-PID on (setpoint − measured) tracks it; velocity FF supplies the
// steady-state PWM so the PID only handles tracking error, not driving force.
//   - Raise FWD_V_CRUISE_TPS for higher top speed.
//   - Raise FWD_ACCEL_TPS2 if accel feels sluggish (lower if it skids).
//   - FWD_KV_SLOPE is the PWM-per-(ticks/s) feedforward slope; raise it if
//     wheels under-shoot the commanded cruise speed.
//   - Stiction-related values (FWD_STICTION_FF, POS_*_PWM) are DERIVED from
//     BASE_BREAKAWAY_PWM above — do not edit here, edit the master knob.
constexpr float    FWD_V_CRUISE_TPS    = 150.0f;  // EXPLORE cruise (slow for sense reliability)
// FAST RUN cruise speed is RUNTIME-ADJUSTABLE via the "Fast Speed" menu and
// persisted to NVS (key "fspeed"). Default == FWD_V_CRUISE_TPS. EXPLORE mode
// ignores this knob — explore always uses FWD_V_CRUISE_TPS for sense
// reliability (high speed corrupts IR centering + wall detection).
constexpr float    FAST_RUN_CRUISE_TPS_DEFAULT = FWD_V_CRUISE_TPS;
constexpr float    FAST_RUN_CRUISE_TPS_MIN     =  400.0f;
constexpr float    FAST_RUN_CRUISE_TPS_MAX     = 3000.0f;
constexpr float    FAST_RUN_CRUISE_TPS_STEP    =  100.0f;
// "Rough" cell-to-cell: high accel/decel flattens the trapezoid — near
// instant kick, brief hold, near instant brake. At 1400 ticks/cell + cruise
// 5000 tps, accel 20 000 finishes a cell in ~0.45 s vs ~1 s at accel 5 000.
// Side effects: wheels may chirp at phase boundaries, IR centering has less
// time mid-cell. Lower if straightness degrades.
constexpr float    FWD_ACCEL_TPS2      = 500.0f;
constexpr float    FWD_DECEL_TPS2      = 500.0f;
constexpr float    FWD_KV_SLOPE        =    0.12f;
constexpr float    POS_KP              =    0.80f;
constexpr float    POS_KD              =    0.10f;
constexpr int      POS_FRICTION_ZONE   =   10;
constexpr int      POS_STK_SOFT_BAND   =   30;
constexpr int      POS_HOLD_BAND       =   20;   // |posErr| < this → settle candidate
constexpr uint32_t POS_SETTLE_MS       =   80;
constexpr int      POS_STOP_BIAS       =    0;
constexpr float    POS_STALL_VEL       =   30.0f;
constexpr uint32_t POS_STALL_MS        =  200;
constexpr int      POS_STALL_ERR_MAX   =   40;
constexpr float    BALANCE_KP          =    0.03f;  // (tL−tR) × this added as bias

// ── [C] PIVOT / SPOT TURNS (yaw-IMU controller) ──────────────────────────────
// YAW_STICTION_PWM / YAW_MAX_PWM are DERIVED from BASE_BREAKAWAY_PWM in [B0].
// INVARIANT: YAW_FRICTION_ZONE ≤ YAW_HOLD_BAND (else dead-zone, see [B0]).
constexpr float    YAW_KP              =    6.0f;
constexpr float    YAW_KD              =    0.3f;
constexpr float    YAW_FRICTION_ZONE   =    3.0f;  // HEAD value — soft band fills the gap to HOLD_BAND
constexpr float    YAW_STK_SOFT_BAND   =    2.0f;  // wide blend → smooth floor ramp 0→full over 2°
constexpr float    YAW_HOLD_BAND       =    1.5f;
constexpr uint32_t YAW_SETTLE_MS       =   80;
constexpr float    YAW_STALL_VEL       =    5.0f;
constexpr uint32_t YAW_STALL_MS        =  250;
constexpr float    YAW_STALL_ERR_MAX   =    4.0f;
constexpr float    YAW_HOLD_KP         =    5.0f;   // forward-leg heading hold
constexpr float    YAW_HOLD_KD         =    0.8f;   // damps yaw overshoot during forward (PWM per °/s)
constexpr float    PIVOT_90_DEG        =   90.0f;
constexpr float    SPOT_180_DEG        =  180.0f;

// ── [D] IR CENTERING (forward phase only) ────────────────────────────────────
// Trims left/right PWM bias so the robot stays mid-corridor by matching the
// L/R side-IR raw count to IR_CAL_L / IR_CAL_R from PinConfig.h.
constexpr float    IR_CENTER_KP        =    1.0f;
constexpr float    IR_CENTER_KI        =    0.0f;
constexpr float    IR_CENTER_KD        =    2.0f;
constexpr int      IR_CENTER_MAX       =   15;

// ── [E] DEAD-END HANDLING (PH_ALIGN_FRONT + 180° exit) ───────────────────────
// Before exiting a dead-end, robot creeps fwd/rev until LF≈ALIGN_LF_TARGET
// and RF≈ALIGN_RF_TARGET. These are empirical raw counts at a 37.5 mm
// front-wall gap on THIS hardware (2026-05-19) — re-capture if front IR pair
// is touched. After align: SPOT 180° → reverse DEADEND_REVERSE_MM →
// forward DEADEND_FWD_MM.
constexpr int      ALIGN_LF_TARGET     = 3660;
constexpr int      ALIGN_RF_TARGET     = 2940;
constexpr int      ALIGN_TOL           =  150;
// ALIGN_PWM is DERIVED from BASE_BREAKAWAY_PWM in [B0].
constexpr long     ALIGN_MAX_TICKS     =  800;
constexpr uint32_t ALIGN_SETTLE_MS     =   80;
constexpr float    DEADEND_REVERSE_MM  =   20.0f;
constexpr float    DEADEND_FWD_MM      =  180.0f;

// ── [F] SCRIPT GEOMETRY ──────────────────────────────────────────────────────
// CELL_TICKS = encoder ticks to traverse one 180 mm cell pitch.
// Hand-measured 2026-05-19, both wheels. Do NOT derive from `TICKS_PER_REV`.
// START_OFFSET_TICKS = extra ticks on first forward leg because robot starts
// pressed against the back wall of cell (0,0) — its center sits ~41 mm
// behind the cell-(0,0) centre.
constexpr long     CELL_TICKS              = 1400;
constexpr long     START_OFFSET_TICKS      =  322;
constexpr long     PIVOT_TICKS_FALLBACK    =  900;   // used only if USE_IMU=false
constexpr long     SPOT180_TICKS_FALLBACK  =  906;   // used only if USE_IMU=false

// ── [G] FWD_TO_WALL (legacy primitive — not used by main flow) ───────────────
// Slow open-loop forward that stops when IR-estimated front distance drops
// below FWD2WALL_TOUCH_MM. Kept as a building block; live dead-end path uses
// PH_ALIGN_FRONT instead.
constexpr float    BACKUP_OFFSET_MM     =    0.0f;   // PH_REVERSE_TO_BACK
constexpr float    FWD2WALL_TOUCH_MM    =   35.0f;
constexpr long     FWD2WALL_MAX_TICKS   = 1600;

// ── [H] DEBUG FLAGS ──────────────────────────────────────────────────────────
constexpr bool     USE_IMU             = true;   // false = encoder-tick turns
constexpr bool     USE_IR              = true;   // false = no IR centering
constexpr bool     TELEMETRY           = true;   // Serial.printf control prints

// ═══════════════════════════════════════════════════════════════════════════
//                       END TUNABLE BLOCK
// ═══════════════════════════════════════════════════════════════════════════

// ── Maze + robot state ──────────────────────────────────────────────────────
constexpr uint8_t MAZE_ROWS = 6;
constexpr uint8_t MAZE_COLS = 3;
constexpr uint8_t START_ROW = 0;
constexpr uint8_t START_COL = 0;
constexpr uint8_t GOAL_ROW  = 0;
constexpr uint8_t GOAL_COL  = 2;


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

// Continuous-yaw bookkeeping. `yawDeg` is no longer reset at phase boundaries;
// each turn phase advances `yawTargetDeg` by its signed degree target, and
// each phase snapshots its starting yaw for progress calculation.
static float    yawTargetDeg     = 0.0f;
static float    phaseStartYawDeg = 0.0f;

// Per-phase encoder offsets. Replace `leftEnc.reset()/rightEnc.reset()` between
// phases so the PCNT hardware counter stays continuous and we just compute
// (tL - phaseStartTL) inside the PID.
static long     phaseStartTL = 0;
static long     phaseStartTR = 0;

// Wall-clock at phase entry — drives the trapezoidal position trajectory
// generator for PH_FORWARD.
static uint32_t phaseStartUs = 0;

// IR centering EMA state — pulled out of the loop's static-inside-block so
// scriptKick() can flip `irFirstSample = true` and re-seed the EMA at the
// start of every new script.
static float    irLSm     = 0.0f, irRSm   = 0.0f;
static float    irLPrev   = 0.0f, irRPrev = 0.0f;
static bool     irFirstSample = true;

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

// All tunables now live in the constexpr block at top of file.

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
        float out = IR_CENTER_KP * err + IR_CENTER_KI * integral + IR_CENTER_KD * deriv;
        return constrain(out, -(float)IR_CENTER_MAX, (float)IR_CENTER_MAX);
    }
    void reset() { integral = 0; prevError = 0; prevUs = 0; }
} pid;

// ── Trapezoidal trajectory generator ────────────────────────────────────────
// Position + velocity profiles as a function of time `t` (seconds) since
// phase entry. `dist` is the total distance to travel (ticks, unsigned),
// `vMax` cruise speed, `a` symmetric accel/decel. Once t > tEnd, position
// holds at dist and velocity returns 0 — the position PID then closes the
// final settle. Triangular fallback handles short moves where 2·dAccel > dist
// (no cruise plateau).
static inline float trapezoidPos(float t, float dist, float vMax, float a) {
    if (t < 0.0f || dist <= 0.0f || vMax <= 0.0f || a <= 0.0f) return 0.0f;
    float tA = vMax / a;
    float dA = 0.5f * vMax * tA;
    if (2.0f * dA >= dist) {
        float dPk = dist * 0.5f;
        float vPk = sqrtf(2.0f * a * dPk);
        float tPk = vPk / a;
        if (t <  tPk)        return 0.5f * a * t * t;
        if (t <  2.0f * tPk) {
            float td = t - tPk;
            return dPk + vPk * td - 0.5f * a * td * td;
        }
        return dist;
    }
    float tCru = (dist - 2.0f * dA) / vMax;
    if (t <  tA)        return 0.5f * a * t * t;
    if (t <  tA + tCru) return dA + vMax * (t - tA);
    float tEnd = tA + tCru + tA;
    if (t <  tEnd) {
        float td = t - tA - tCru;
        return dA + vMax * tCru + vMax * td - 0.5f * a * td * td;
    }
    return dist;
}
static inline float trapezoidVel(float t, float dist, float vMax, float a) {
    if (t < 0.0f || dist <= 0.0f || vMax <= 0.0f || a <= 0.0f) return 0.0f;
    float tA = vMax / a;
    float dA = 0.5f * vMax * tA;
    if (2.0f * dA >= dist) {
        float dPk = dist * 0.5f;
        float vPk = sqrtf(2.0f * a * dPk);
        float tPk = vPk / a;
        if (t <  tPk)        return a * t;
        if (t <  2.0f * tPk) return vPk - a * (t - tPk);
        return 0.0f;
    }
    float tCru = (dist - 2.0f * dA) / vMax;
    if (t <  tA)        return a * t;
    if (t <  tA + tCru) return vMax;
    float tEnd = tA + tCru + tA;
    if (t <  tEnd)      return vMax - a * (t - tA - tCru);
    return 0.0f;
}

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


static MicromouseMaze maze;
static uint8_t robotRow    = START_ROW;
static uint8_t robotCol    = START_COL;
static uint8_t robotHeading = DIR_NORTH;

static uint8_t plannedRow     = START_ROW;
static uint8_t plannedCol     = START_COL;
static uint8_t plannedHeading = DIR_NORTH;

static bool    exploreMode = false;
static bool    fastRunMode = false;

// Runtime-adjustable fast-run cruise speed. Edit via Fast Speed menu, persists
// to NVS "fspeed". EXPLORE mode does not read this.
static float   fastRunCruiseTps = FAST_RUN_CRUISE_TPS_DEFAULT;

// Ticks to add to the NEXT forward leg so the robot lands at the next
// cell-center even though it currently sits at "−4.5 cm offset" from the
// current cell center (back-against-wall reference pose). Set to
// `START_OFFSET_TICKS` at Explore start and after every 180° wall re-anchor;
// reset to 0 once consumed by a forward leg.
static long    pendingOffsetTicks = 0;

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

// ── NVS persistence (walls + fast-run speed) ───────────────────────────────
static Preferences prefs;
static const char* NVS_NS     = "mm26";
static const char* NVS_WALLS  = "walls";
static const char* NVS_FSPEED = "fspeed";

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
static void nvsLoadFastSpeed() {
    if (!prefs.begin(NVS_NS, true)) return;
    if (prefs.isKey(NVS_FSPEED)) {
        fastRunCruiseTps = prefs.getFloat(NVS_FSPEED, FAST_RUN_CRUISE_TPS_DEFAULT);
        if (fastRunCruiseTps < FAST_RUN_CRUISE_TPS_MIN) fastRunCruiseTps = FAST_RUN_CRUISE_TPS_MIN;
        if (fastRunCruiseTps > FAST_RUN_CRUISE_TPS_MAX) fastRunCruiseTps = FAST_RUN_CRUISE_TPS_MAX;
    }
    prefs.end();
}
static void nvsSaveFastSpeed() {
    if (!prefs.begin(NVS_NS, false)) return;
    prefs.putFloat(NVS_FSPEED, fastRunCruiseTps);
    prefs.end();
}

// ── Phase script ────────────────────────────────────────────────────────────
enum TurnDir   { TURN_NONE, TURN_RIGHT, TURN_LEFT };
//   PH_FORWARD          — tick-target forward (signed, supports reverse)
//   PH_PIVOT            — single-wheel pivot (yaw-degree or tick target)
//   PH_SPOT             — both-wheels-opposite spot rotation (yaw-deg/tick)
//   PH_FWD_TO_WALL      — open-loop slow forward, stops on IR-distance ≤ threshold
//   PH_REVERSE_TO_BACK  — used after 180°. At phase entry, robot reads front
//                         IR distance and computes a reverse target of
//                         (frontMm + backupOffsetMm) so the rear bumps the
//                         wall behind. Then PID handles it as a reverse
//                         forward-phase (target is negative ticks).
//   PH_ALIGN_FRONT      — creeps fwd/rev until front IR LF/RF read the 4 cm
//                         target row of IRCal::IR_DIST_TABLE. Used at
//                         dead-ends to square against the front wall before
//                         the 180° spin (cell-center re-anchor).
enum RunPhase  { PH_FORWARD, PH_PIVOT, PH_SPOT, PH_FWD_TO_WALL, PH_REVERSE_TO_BACK, PH_ALIGN_FRONT };

struct PhaseStep {
    RunPhase phase;
    long     target;
    TurnDir  dir;
};

constexpr int MAX_SCRIPT = 8;
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
    scriptPush(PH_FORWARD, ticks + (long)POS_STOP_BIAS, TURN_NONE);
}
static void scriptPushSpot(TurnDir d, float deg) {
    long target = USE_IMU ? (long)(deg + 0.5f) : SPOT180_TICKS_FALLBACK;
    scriptPush(PH_SPOT, target, d);
}
static void scriptPushPivot(TurnDir d, float deg) {
    long target = USE_IMU ? (long)(deg + 0.5f) : PIVOT_TICKS_FALLBACK;
    scriptPush(PH_PIVOT, target, d);
}
static void scriptPushFwdToWall() {
    scriptPush(PH_FWD_TO_WALL, FWD2WALL_MAX_TICKS, TURN_NONE);
}
// Reverse-to-back-wall. Target is set at phase activation (front IR sample).
static void scriptPushReverseToBack() {
    scriptPush(PH_REVERSE_TO_BACK, 0, TURN_NONE);
}
// Front-IR alignment to the 4 cm row of IR_DIST_TABLE. Used at dead-ends.
static void scriptPushAlignFront() {
    scriptPush(PH_ALIGN_FRONT, 0, TURN_NONE);
}

// Called when a new script step is activated (kick or advance). Computes
// any deferred targets (e.g. PH_REVERSE_TO_BACK reads front IR right here).
// If the phase is re-classified into PH_FORWARD below, the standard PID
// path takes over from the very next loop iteration.
static void onPhaseActivate() {
    if (runPhase == PH_REVERSE_TO_BACK) {
        sampleIR();
        float frontMm   = IRCal::estimateFrontDistMM(irVal[0], irVal[3]);
        float distMm    = frontMm + BACKUP_OFFSET_MM;
        float ticksPerMm = (float)CELL_TICKS / 180.0f;
        long  ticks      = -(long)(distMm * ticksPerMm + 0.5f);
        Serial.printf("[BACKUP] frontMm=%.1f offsetMm=%.1f → target=%ld ticks (reverse)\n",
                      frontMm, BACKUP_OFFSET_MM, ticks);
        runTarget = ticks;
        runPhase  = PH_FORWARD;    // hand off to standard PID (handles negative target)
    }
}

// Phase-entry: per-phase yaw zero. Each phase gets its own clean coordinate
// frame (yawDeg=0 at start), preventing cross-phase drift accumulation.
// Previously yawDeg was continuous across phases — settle-band imprecision
// (±1.5°/turn) compounded across chained turns. Resetting per phase bounds
// the gyro error to ONE phase duration (max ~1.5 s of integration drift).
static void phaseEnter() {
    phaseStartTL     = leftEnc.getTicks();
    phaseStartTR     = rTicks();
    phaseStartUs     = micros();
    yawDeg           = 0.0f;                      // per-phase reset
    phaseStartYawDeg = 0.0f;
    if (runPhase == PH_SPOT || runPhase == PH_PIVOT) {
        float deg = (float)runTarget;             // turn phases store degrees in runTarget when useImu
        float signedDeg = (runTurnDir == TURN_RIGHT) ? -deg : +deg;
        yawTargetDeg = signedDeg;                 // turn target ±deg from new zero
    } else {
        yawTargetDeg = 0.0f;                      // forward holds current heading (= new zero)
    }
    onPhaseActivate();                            // handles PH_REVERSE_TO_BACK target computation
}

// ── State ───────────────────────────────────────────────────────────────────
enum State { IDLE, ENC_TEST, IR_TEST, FAST_SPEED_EDIT, EXPLORE_THINK, RUN, GOAL, CRASH };
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
    M_FAST_SPEED,
    M_ENC,
    M_IR_TEST,
    M_NVS_CLR,
    M_COUNT
};
static const char* MENU_LABELS[M_COUNT] = {
    "Explore",
    "Fast Run",
    "Fast Speed",
    "Encoder Test",
    "IR Test",
    "Clear NVS"
};
static int  menuSel    = M_EXPLORE;
static long menuEncRef = 0;
constexpr long ENC_PER_STEP = 80;
constexpr uint32_t COUNTDOWN_DELAY_MS = 500;

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
    const char* phN = (runPhase == PH_FORWARD)      ? "FWD"
                    : (runPhase == PH_PIVOT)        ? "PIV"
                    : (runPhase == PH_ALIGN_FRONT)  ? "ALN"
                    : (runPhase == PH_FWD_TO_WALL)  ? "F2W"
                    : (runPhase == PH_REVERSE_TO_BACK) ? "RTB"
                    : "SPOT";
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

void oledIrTest() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "IR Test");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_6x10_tf);
    char buf[32];
    snprintf(buf, sizeof(buf), "LF %4d  RF %4d", irVal[0], irVal[3]);
    oled.drawStr(0, 24, buf);
    snprintf(buf, sizeof(buf), "L  %4d  R  %4d", irVal[1], irVal[2]);
    oled.drawStr(0, 36, buf);
    float frontMm = IRCal::estimateFrontDistMM(irVal[0], irVal[3]);
    snprintf(buf, sizeof(buf), "front %.0f mm", frontMm);
    oled.drawStr(0, 48, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "btn=back");
    oled.sendBuffer();
}

void oledCountdown(int n) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "STARTING");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_logisoso42_tn);
    char buf[4]; snprintf(buf, sizeof(buf), "%d", n);
    int w = oled.getStrWidth(buf);
    oled.drawStr((128 - w) / 2, 60, buf);
    oled.sendBuffer();
}

// Auto-calibrate gyro bias at the start of every run. Replaces the menu-only
// "Cal Gyro" step so the user doesn't have to remember to do it (they were
// getting confused about when it needed to run). Robot must be still during
// the ~600 ms sample window — the post-button-press settle delay handles
// that. Hand vibration from the button release dies in ~200 ms.
static void autoCalGyroBeforeStart() {
    if (!imuReady) {
        Serial.println("[GCAL/AUTO] skipped (no IMU)");
        return;
    }
    oledTerminal("CAL GYRO", "stay still");
    delay(300);                         // settle vibration from button press
    calibrateGyroBias(300, 2);          // 300 samples × ~2 ms ≈ 600 ms
    yawDeg       = 0.0f;
    yawTargetDeg = 0.0f;
    Serial.printf("[GCAL/AUTO] bias=%.4f deg/s\n", gyroBiasZ);
}

void oledFastSpeedEdit() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "Fast Speed");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "%d tps", (int)fastRunCruiseTps);
    oled.drawStr(0, 30, buf);
    float mmps = fastRunCruiseTps * 180.0f / (float)CELL_TICKS;
    snprintf(buf, sizeof(buf), "%d mm/s", (int)mmps);
    oled.drawStr(0, 48, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "spin=adj  btn=save");
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

// Build a one-cell-move script: optional spot turn, then forward 1 cell.
// Two refinements baked in:
//   * `pendingOffsetTicks` is added to the forward target, then cleared.
//     Lets the first forward leg from start (or after a 180° re-anchor)
//     compensate for the −4.5 cm position offset.
//   * 180° turn (diff==2) is followed by an automatic wall re-anchor:
//     spot 180 → drive to wall → back up 1.5 cm → forward (cellTicks + offset).
//     After the re-anchor the robot is at "−4.5 cm" relative to the cell
//     it's about to leave, so the forward leg gets the full offset added.
static void buildMoveScript(AbsDir bestDir) {
    int diff = ((int)bestDir - (int)robotHeading + 4) % 4;
    scriptReset();
    if (diff == 1) {
        // Spot at current cell center. Pivot would translate the robot
        // diagonally — wrong target for a single-cell turn. Pivot is used
        // only mid-chain (see chain-extension below) where a pre-FWD
        // positions the pivot point 4 cm before the turn-cell center.
        scriptPushSpot(TURN_RIGHT, 90.0f);
    } else if (diff == 3) {
        scriptPushSpot(TURN_LEFT, 90.0f);
    } else if (diff == 2) {
        // Dead-end exit. Robot is facing the dead-end wall, currently somewhere
        // near cell-center. Sequence:
        //   1. PH_ALIGN_FRONT — creep until front IR reads the 4 cm row of
        //      IRCal::IR_DIST_TABLE (LF≈3281, RF≈2935). Anchors the pose to
        //      a known offset from the dead-end wall.
        //   2. SPOT 180° — rotate about wheel center; dead-end wall is now
        //      behind us.
        //   3. Reverse DEADEND_REVERSE_MM (2 cm) at low speed.
        //   4. Forward DEADEND_FWD_MM (18 cm = 1 cell pitch).
        // No pendingOffsetTicks involvement — the 2 cm reverse + 18 cm fwd
        // is the new re-anchor primitive, not the "boot pose" offset.
        float ticksPerMm = (float)CELL_TICKS / 180.0f;
        long  revTicks   = (long)(DEADEND_REVERSE_MM * ticksPerMm + 0.5f);
        long  fwdTicks   = (long)(DEADEND_FWD_MM     * ticksPerMm + 0.5f);
        scriptPushAlignFront();
        scriptPushSpot(TURN_RIGHT, 180.0f);
        scriptPushFwd(-revTicks);
        scriptPushFwd( fwdTicks);
        pendingOffsetTicks = 0;
        plannedHeading = bestDir;
        plannedRow     = robotRow + DIR_DR[bestDir];
        plannedCol     = robotCol + DIR_DC[bestDir];
        return;                  // skip trailing generic fwd push + chain ext
    }
    long fwd = CELL_TICKS + pendingOffsetTicks;
    pendingOffsetTicks = 0;
    scriptPushFwd(fwd);

    plannedHeading = bestDir;
    plannedRow     = robotRow + DIR_DR[bestDir];
    plannedCol     = robotCol + DIR_DC[bestDir];

    // ── Fast-run chain extension (straights + mid-chain pivots) ──────────────
    // In FAST mode the maze is fully known and we want unbroken motion. Walk
    // the planned path forward from `plannedRow/Col` in `plannedHeading`:
    //   - while next cell continues straight  → extend last FWD by 1 cell;
    //   - when next cell needs a 90° turn     → shorten approach by 4 cm,
    //     push PIVOT, push fresh FWD covering "post-pivot to next-cell center".
    // After a pivot the robot's center is ~`WHEEL_TRACK_MM/2` past the
    // turn-cell center in the new heading direction, so the post-pivot FWD is
    // (cellTicks − pivotFwdOffset). Lateral ~half-track displacement is
    // corrected by IR centering during the post-pivot FWD. 180° turns can't
    // be pivoted (no in-place rotation), so the chain breaks there.
    if (fastRunMode && scriptLen > 0 && script[scriptLen - 1].phase == PH_FORWARD) {
        int rr = plannedRow, cc = plannedCol;
        AbsDir hh = (AbsDir)plannedHeading;
        long pivotFwdOffset = (long)((WHEEL_TRACK_MM * 0.5f)
                                     * ((float)CELL_TICKS / 180.0f) + 0.5f);
        long postPivotShort = CELL_TICKS - pivotFwdOffset;
        int extraCells = 0, pivots = 0;
        int safetyCap = MAZE_ROWS * MAZE_COLS;
        while (safetyCap-- > 0) {
            if (rr == GOAL_ROW && cc == GOAL_COL) break;
            if (maze.hasWall(rr, cc, hh)) break;
            int nr = rr + DIR_DR[hh];
            int nc = cc + DIR_DC[hh];
            if (nr < 0 || nr >= MAZE_ROWS || nc < 0 || nc >= MAZE_COLS) break;
            uint8_t d;
            AbsDir next = maze.bestDirectionBiased(nr, nc, hh, d);
            if (d == FLOOD_INFINITY) break;

            if (next == hh) {
                // Straight: extend current FWD by 1 cell.
                script[scriptLen - 1].target += CELL_TICKS;
                rr = nr; cc = nc;
                extraCells++;
                continue;
            }

            int turnDiff = ((int)next - (int)hh + 4) % 4;
            if (turnDiff != 1 && turnDiff != 3) break;  // 180° not pivot-able
            if (scriptLen + 2 > MAX_SCRIPT) break;       // need room for PIVOT + FWD
            // Need the diagonal cell to exist + not be wall-blocked after the pivot.
            int diagR = nr + DIR_DR[next];
            int diagC = nc + DIR_DC[next];
            if (diagR < 0 || diagR >= MAZE_ROWS || diagC < 0 || diagC >= MAZE_COLS) break;
            if (maze.hasWall(nr, nc, next)) break;

            // Approach: extend last FWD by 14 cm so pivot point is 4 cm
            // before (nr,nc) center.
            script[scriptLen - 1].target += postPivotShort;
            TurnDir td = (turnDiff == 1) ? TURN_RIGHT : TURN_LEFT;
            scriptPushPivot(td, 90.0f);
            // Post-pivot FWD: covers (nr,nc) center → (diagR,diagC) center,
            // less the ~4 cm the pivot already advanced in the new heading.
            scriptPushFwd(postPivotShort);

            hh = next;
            rr = diagR;
            cc = diagC;
            pivots++;
        }

        plannedRow     = rr;
        plannedCol     = cc;
        plannedHeading = hh;

        if (extraCells > 0 || pivots > 0) {
            Serial.printf("[FAST] chained +%d cells, %d pivots -> (%d,%d) hd=%d\n",
                          extraCells, pivots, rr, cc, (int)hh);
        }
    }
}

static void scriptKick() {
    if (scriptLen == 0) return;
    scriptIdx  = 0;
    runPhase   = script[0].phase;
    runTarget  = script[0].target;
    runTurnDir = script[0].dir;
    pid.reset();
    irFirstSample = true;       // re-seed IR centering EMA on each new script
    phaseEnter();
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
        yawDeg       = 0.0f;
        yawTargetDeg = 0.0f;
        Serial.printf("[IMU] bias=%.4f deg/s\n", gyroBiasZ);
    } else {
        Serial.println("[IMU] mpu6500 NOT detected");
    }

    setupMaze();
    nvsLoadFastSpeed();
    Serial.printf("[FSPEED] loaded = %.0f tps (%.0f mm/s)\n",
                  fastRunCruiseTps,
                  fastRunCruiseTps * 180.0f / (float)CELL_TICKS);

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
                pendingOffsetTicks = START_OFFSET_TICKS;
                exploreMode = true; fastRunMode = false;
                autoCalGyroBeforeStart();
                for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(COUNTDOWN_DELAY_MS); }
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
                pendingOffsetTicks = START_OFFSET_TICKS;
                exploreMode = false; fastRunMode = true;
                autoCalGyroBeforeStart();
                for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(COUNTDOWN_DELAY_MS); }
                Serial.println("--- FAST RUN START ---");
                state = EXPLORE_THINK;
                break;
            case M_FAST_SPEED:
                menuEncRef = rightEnc.getTicks();
                state = FAST_SPEED_EDIT;
                oledFastSpeedEdit();
                break;
            case M_ENC:
                leftEnc.reset(); rightEnc.reset();
                state = ENC_TEST;
                oledEncTest();
                break;
            case M_IR_TEST:
                sampleIR();
                state = IR_TEST;
                oledIrTest();
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

    case IR_TEST: {
        static uint32_t last = 0;
        if (millis() - last > 100) {
            sampleIR();
            oledIrTest();
            Serial.printf("[IR] LF=%d L=%d R=%d RF=%d frontMm=%.1f\n",
                          irVal[0], irVal[1], irVal[2], irVal[3],
                          IRCal::estimateFrontDistMM(irVal[0], irVal[3]));
            last = millis();
        }
        if (buttonEdge()) {
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
        }
        break;
    }

    case FAST_SPEED_EDIT: {
        long delta = rightEnc.getTicks() - menuEncRef;
        if (delta >= ENC_PER_STEP) {
            fastRunCruiseTps += FAST_RUN_CRUISE_TPS_STEP;
            if (fastRunCruiseTps > FAST_RUN_CRUISE_TPS_MAX) fastRunCruiseTps = FAST_RUN_CRUISE_TPS_MAX;
            menuEncRef += ENC_PER_STEP;
            oledFastSpeedEdit();
        } else if (delta <= -ENC_PER_STEP) {
            fastRunCruiseTps -= FAST_RUN_CRUISE_TPS_STEP;
            if (fastRunCruiseTps < FAST_RUN_CRUISE_TPS_MIN) fastRunCruiseTps = FAST_RUN_CRUISE_TPS_MIN;
            menuEncRef -= ENC_PER_STEP;
            oledFastSpeedEdit();
        }
        if (buttonEdge()) {
            nvsSaveFastSpeed();
            Serial.printf("[FSPEED] saved = %.0f tps (%.0f mm/s)\n",
                          fastRunCruiseTps,
                          fastRunCruiseTps * 180.0f / (float)CELL_TICKS);
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
        }
        break;
    }

    case EXPLORE_THINK: {
        // Gyro drift fix: zero heading reference at every cell arrival.
        // Continuous yawDeg accumulation lost ~1–2°/turn from settle-band
        // imprecision; after ~3 turns the drift exceeded a single 90° turn's
        // tolerance. Reset bounds the error to one move at a time.
        yawDeg       = 0.0f;
        yawTargetDeg = 0.0f;
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

        // Encoders run continuously; phaseEnter() captured the absolute count
        // at activation, so the phase-local "tL/tR" is the delta against that.
        long tLAbs = leftEnc.getTicks();
        long tRAbs = rTicks();
        long tL = tLAbs - phaseStartTL;
        long tR = tRAbs - phaseStartTR;

        // ── PH_FWD_TO_WALL: open-loop slow forward, IR-stop. ─────────────
        // Used after 180° to re-anchor on the new front wall. Stops as soon
        // as the IR-estimated front distance drops below `wallTouchDistMm`.
        // Falls through to standard PID/end on safety cap.
        if (runPhase == PH_FWD_TO_WALL) {
            sampleIR();
            float frontMm = IRCal::estimateFrontDistMM(irVal[0], irVal[3]);
            long  avgTicks = (tL + tR) / 2;
            static long     posAvgPrev_w = 0;
            static uint32_t posPrevUs_w  = 0;
            static uint32_t settleStart_w = 0;
            // End conditions: wall touch, or safety travel cap.
            auto endNow = [&](const char* reason) {
                stopMotors();
                Serial.printf("--- STEP END idx=%d/%d ph=FWD_TO_WALL reason=%s avg=%ld frontMm=%.1f tL=%ld tR=%ld ---\n",
                              scriptIdx + 1, scriptLen, reason, avgTicks, frontMm, tL, tR);
                posAvgPrev_w = 0; posPrevUs_w = 0; settleStart_w = 0;
                if (scriptIdx + 1 >= scriptLen) {
                    robotRow = plannedRow; robotCol = plannedCol; robotHeading = plannedHeading;
                    runTurnDir = TURN_NONE;
                    Serial.printf("--- MOVE DONE pos=(%d,%d,%c) ---\n",
                                  robotRow, robotCol, "NESW"[robotHeading]);
                    if (exploreMode || fastRunMode) state = EXPLORE_THINK;
                    else { menuEncRef = rightEnc.getTicks(); oledMenu(); state = IDLE; }
                    return;
                }
                scriptIdx++;
                PhaseStep& next = script[scriptIdx];
                runPhase   = next.phase;
                runTarget  = next.target;
                runTurnDir = next.dir;
                pid.reset();
                phaseEnter();
            };

            if (frontMm <= FWD2WALL_TOUCH_MM) { endNow("WALL_TOUCH"); break; }
            if (avgTicks >= runTarget)        { endNow("NO_WALL_CAP"); break; }

            // Drive forward gently with yaw hold against the commanded heading.
            int throttle = POS_STICTION_PWM;
            int yawBias  = (USE_IMU && imuReady)
                            ? (int)(-YAW_HOLD_KP * (yawDeg - yawTargetDeg) - YAW_HOLD_KD * gzFilt) : 0;
            int pwmL = constrain(throttle - yawBias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            int pwmR = constrain(throttle + yawBias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            leftMotor.drive(pwmL);
            rightMotor.drive(pwmR);

            static uint32_t lastOled = 0;
            if (millis() - lastOled > 150) {
                oledRun(avgTicks, runTarget, tL, tR);
                lastOled = millis();
            }
            if (TELEMETRY) {
                static uint32_t lastTel = 0;
                if (millis() - lastTel > 80) {
                    Serial.printf("t=%lu ph=FWD2WALL avg=%ld frontMm=%.1f thr=%d yaw=%+.2f tL=%ld tR=%ld\n",
                                  (unsigned long)millis(), avgTicks, frontMm, throttle, yawDeg, tL, tR);
                    lastTel = millis();
                }
            }
            break;
        }

        // ── PH_ALIGN_FRONT: creep until LF≈3281 / RF≈2935 (4 cm row). ────
        // Used at dead-ends. Direction is set per-loop from the larger-magnitude
        // sensor error so both LF and RF converge to their targets. Yaw-held
        // against the current commanded heading throughout the creep.
        if (runPhase == PH_ALIGN_FRONT) {
            sampleIR();
            int errLF = irVal[0] - ALIGN_LF_TARGET;
            int errRF = irVal[3] - ALIGN_RF_TARGET;
            // Average distance error: drives fwd/rev. Old code used the
            // larger-magnitude sensor which overshot whichever was farther off.
            int errMean = (errLF + errRF) / 2;
            // LF/RF difference: angular error against the dead-end wall.
            // LF>RF (positive diff) → LF wall closer → robot rotated CW
            // (RF side farther from wall) → spin CCW (left wheel back, right
            // wheel forward) to flatten. Use as additional differential bias.
            int errDiff = errLF - errRF;
            long avgTicks = (tL + tR) / 2;
            static uint32_t alignSettleStart = 0;

            auto endNow = [&](const char* reason) {
                stopMotors();
                Serial.printf("--- STEP END idx=%d/%d ph=ALIGN reason=%s LF=%d RF=%d tL=%ld tR=%ld ---\n",
                              scriptIdx + 1, scriptLen, reason, irVal[0], irVal[3], tL, tR);
                alignSettleStart = 0;
                if (scriptIdx + 1 >= scriptLen) {
                    robotRow = plannedRow; robotCol = plannedCol; robotHeading = plannedHeading;
                    runTurnDir = TURN_NONE;
                    Serial.printf("--- MOVE DONE pos=(%d,%d,%c) ---\n",
                                  robotRow, robotCol, "NESW"[robotHeading]);
                    if (exploreMode || fastRunMode) state = EXPLORE_THINK;
                    else { menuEncRef = rightEnc.getTicks(); oledMenu(); state = IDLE; }
                    return;
                }
                scriptIdx++;
                PhaseStep& next = script[scriptIdx];
                runPhase   = next.phase;
                runTarget  = next.target;
                runTurnDir = next.dir;
                pid.reset();
                phaseEnter();
            };

            // Settle: both sensors in tolerance for ALIGN_SETTLE_MS.
            if (abs(errLF) <= ALIGN_TOL && abs(errRF) <= ALIGN_TOL) {
                stopMotors();
                if (alignSettleStart == 0) alignSettleStart = millis();
                if (millis() - alignSettleStart > ALIGN_SETTLE_MS) {
                    endNow("ALIGN_OK");
                }
                break;
            }
            alignSettleStart = 0;
            // Safety: if we've traveled ALIGN_MAX_TICKS without converging, bail.
            if (labs(avgTicks) >= ALIGN_MAX_TICKS) { endNow("ALIGN_CAP"); break; }

            // errMean > 0 → both IR brighter than target → wall closer than
            // target → REVERSE. errMean < 0 → wall farther → FORWARD.
            int dir = (errMean > 0) ? -1 : +1;
            int throttle = dir * ALIGN_PWM;
            // Angular correction from front-sensor mismatch. Scale: errDiff
            // of 500 raw counts ≈ ~5° rotation off-perpendicular at 4 cm gap;
            // KP=0.05 PWM/count → ~25 PWM differential for 500-count diff.
            constexpr float ALIGN_DIFF_KP = 0.05f;
            int angleBias = (int)(ALIGN_DIFF_KP * (float)errDiff);
            int yawBias  = (USE_IMU && imuReady)
                            ? (int)(-YAW_HOLD_KP * (yawDeg - yawTargetDeg) - YAW_HOLD_KD * gzFilt) : 0;
            // Combine: yawBias holds heading, angleBias rotates to square
            // against the dead-end wall (overrides gyro hold for this phase
            // since the wall is the truth reference).
            int bias = angleBias + yawBias / 4;  // de-weight gyro vs wall
            int pwmL = constrain(throttle - bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            int pwmR = constrain(throttle + bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            leftMotor.drive(pwmL);
            rightMotor.drive(pwmR);

            static uint32_t lastOled = 0;
            if (millis() - lastOled > 150) {
                oledRun(avgTicks, ALIGN_MAX_TICKS, tL, tR);
                lastOled = millis();
            }
            if (TELEMETRY) {
                static uint32_t lastTel = 0;
                if (millis() - lastTel > 80) {
                    Serial.printf("t=%lu ph=ALIGN LF=%d RF=%d errLF=%+d errRF=%+d dir=%+d tL=%ld tR=%ld\n",
                                  (unsigned long)millis(), irVal[0], irVal[3], errLF, errRF, dir, tL, tR);
                    lastTel = millis();
                }
            }
            break;
        }

        bool imuMode = USE_IMU && imuReady && (runPhase != PH_FORWARD);

        float avg;
        if (runPhase == PH_FORWARD) {
            avg = (float)((tL + tR) / 2);
        } else if (imuMode) {
            // Yaw is no longer reset between phases; measure progress against
            // the snapshot taken at phaseEnter() instead of against absolute 0.
            float dy = yawDeg - phaseStartYawDeg;
            avg = (runTurnDir == TURN_RIGHT) ? -dy : +dy;
        } else if (runPhase == PH_PIVOT) {
            avg = (float)((runTurnDir == TURN_RIGHT) ? tL : tR);
        } else {
            avg = (float)((runTurnDir == TURN_RIGHT) ? (tL - tR) / 2 : (tR - tL) / 2);
        }

        float    effKp        = imuMode ? YAW_KP           : POS_KP;
        float    effKd        = imuMode ? YAW_KD           : POS_KD;
        int      effMaxPwm    = imuMode ? YAW_MAX_PWM       : POS_MAX_PWM;
        int      effStkPwm    = imuMode ? YAW_STICTION_PWM  : POS_STICTION_PWM;
        float    effFz        = imuMode ? YAW_FRICTION_ZONE : (float)POS_FRICTION_ZONE;
        float    effHb        = imuMode ? YAW_HOLD_BAND     : (float)POS_HOLD_BAND;
        uint32_t effSettleMs  = imuMode ? YAW_SETTLE_MS     : POS_SETTLE_MS;
        float    effStallVel  = imuMode ? YAW_STALL_VEL     : POS_STALL_VEL;
        uint32_t effStallMs   = imuMode ? YAW_STALL_MS      : POS_STALL_MS;
        float    effStallEmax = imuMode ? YAW_STALL_ERR_MAX  : (float)POS_STALL_ERR_MAX;
        float    effStkSoft   = imuMode ? YAW_STK_SOFT_BAND  : (float)POS_STK_SOFT_BAND;

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

        // Fast-run continuous-roll flag: while we're inside a FWD phase of a
        // FAST RUN, skip stopMotors at SETTLED transitions so wheels don't
        // brake on every cell boundary. Stall/timeout exits still stop.
        bool fastFwdRoll = fastRunMode && runPhase == PH_FORWARD;

        auto endPhase = [&](const char* reason) {
            bool isSettle = (reason && reason[0] == 'S' && reason[1] == 'E');  // "SETTLED"
            if (!(fastFwdRoll && isSettle)) stopMotors();
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
            resetPidState();
            pid.reset();
            phaseEnter();
        };

        if (fabsf(posErr) < effHb) {
            if (fastFwdRoll) {
                // Fast run: no brake, no settle dwell — end the phase
                // immediately so the next cell's script kicks in without a
                // visible pause at the cell boundary.
                endPhase("SETTLED");
                break;
            }
            stopMotors();
            if (settleStart == 0) settleStart = millis();
            if (millis() - settleStart > effSettleMs) {
                endPhase("SETTLED");
            }
            break;
        }
        settleStart = 0;

        uint32_t nowUs = micros();
        if (posPrevUs == 0) {
            // First iteration after a phase reset: seed posAvgPrev with the
            // current avg so the first computed velocity is 0 instead of a
            // huge spike scaled by an arbitrary default dt.
            posAvgPrev = avg;
            posPrevUs  = nowUs;
            // velFilt keeps its prior filtered value (resetPidState set it to 0).
        } else {
            float dt = constrain((nowUs - posPrevUs) / 1e6f, 1e-4f, 0.05f);
            posPrevUs = nowUs;
            if (imuMode) {
                velFilt = (runTurnDir == TURN_RIGHT) ? -gzFilt : +gzFilt;
                posAvgPrev = avg;
            } else {
                float velRaw = (avg - posAvgPrev) / dt;
                posAvgPrev   = avg;
                velFilt      = 0.7f * velFilt + 0.3f * velRaw;
            }
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

        int throttle;
        float vCmdSigned = 0.0f;            // exposed for telemetry print below
        float xCmdSigned = (float)runTarget;
        if (runPhase == PH_FORWARD) {
            // POSITION-BASED trapezoidal velocity command (was time-based;
            // time-based overshot when wheels lagged the setpoint, then PID
            // couldn't reverse out at sub-stiction PWM and the robot stuck).
            //   vAccel = sqrt(2·a·xDone)  → smooth ramp-up from rest
            //   vDecel = sqrt(2·d·xRem)   → vCmd → 0 as remaining → 0
            //   vAbsCmd = min(vCruise, vAccel, vDecel)
            // Closed loop on actual distance, so wheel lag/slip can't push the
            // robot past target — vCmd just stays low while distance still has
            // remaining. xDone is bootstrapped to ≥ 30 ticks so vAccel isn't
            // zero at phase entry (otherwise wheels never start).
            //
            // FF uses commanded vAbsCmd; PID closes lag on posErr (= runTarget
            // − avg). When vAbsCmd → 0 near settle, residual floor kicks in
            // to push at breakaway PWM if PID alone is sub-stiction — that
            // covers the "robot stopped 50 ticks short of target" case where
            // POS_KP·50 = 50 PWM < 500 stiction. Disabled inside POS_HOLD_BAND
            // so the settle path can stop the wheels cleanly.
            float dist    = fabsf((float)runTarget);
            float absAvg  = fabsf(avg);
            float xRem    = (dist > absAvg) ? (dist - absAvg) : 0.0f;
            float xDoneEff = absAvg < 30.0f ? 30.0f : absAvg;  // bootstrap
            float vCruise = fastRunMode ? fastRunCruiseTps : FWD_V_CRUISE_TPS;
            float vAccel  = sqrtf(2.0f * FWD_ACCEL_TPS2 * xDoneEff);
            float vDecel  = sqrtf(2.0f * FWD_DECEL_TPS2 * xRem);
            float vAbsCmd = vCruise;
            if (vAccel < vAbsCmd) vAbsCmd = vAccel;
            if (vDecel < vAbsCmd) vAbsCmd = vDecel;
            float dirSign = ((float)runTarget - avg >= 0) ? +1.0f : -1.0f;
            vCmdSigned    = dirSign * vAbsCmd;
            xCmdSigned    = (float)runTarget;  // setpoint = true target (PID closes the loop)
            float velErr  = vCmdSigned - velFilt;
            float uFF     = (vAbsCmd > 5.0f) ? dirSign * ((float)FWD_STICTION_FF
                                                       + FWD_KV_SLOPE * vAbsCmd) : 0.0f;
            float u       = uFF + POS_KP * posErr + POS_KD * velErr;
            // Residual stiction floor: trapezoid is at v=0 but robot is still
            // outside the settle band. PID alone gives sub-breakaway PWM →
            // wheels buzz. Floor at FWD_STICTION_FF in the direction of posErr.
            if (vAbsCmd <= 5.0f && fabsf(posErr) > (float)POS_HOLD_BAND
                && fabsf(u) < (float)FWD_STICTION_FF) {
                u = ((posErr >= 0) ? +1.0f : -1.0f) * (float)FWD_STICTION_FF;
            }
            // Dynamic cap: PWM ≤ FF(vCmd) + small PID headroom. Previously
            // POS_KP·posErr saturated POS_MAX_PWM (275) for the whole cell
            // because the setpoint was the FINAL target — robot ran at max
            // PWM regardless of vCruise. Cap to FF + headroom so commanded
            // velocity is actually enforced.
            float ffMag = (vAbsCmd > 5.0f) ? ((float)FWD_STICTION_FF + FWD_KV_SLOPE * vAbsCmd) : 0.0f;
            int   dynMax = (int)ffMag + 40;
            if (dynMax > POS_MAX_PWM) dynMax = POS_MAX_PWM;
            if (dynMax < (int)FWD_STICTION_FF + 20) dynMax = (int)FWD_STICTION_FF + 20;
            throttle      = constrain((int)u, -dynMax, dynMax);
        } else {
            // PIVOT/SPOT path: position PID with soft stiction floor (unchanged).
            float u   = effKp * posErr - effKd * velFilt;
            int   mag = (int)fabsf(u);
            if (mag > effMaxPwm) mag = effMaxPwm;
            float errAbs = fabsf(posErr);
            if (errAbs > effFz) {
                float tBlend = constrain((errAbs - effFz) / effStkSoft, 0.0f, 1.0f);
                int softFloor = (int)(tBlend * (float)effStkPwm);
                if (mag < softFloor) mag = softFloor;
            }
            throttle = (u >= 0) ? mag : -mag;
        }

        // IR centering (forward phase only).
        float corr = 0.0f;
        if (runPhase == PH_FORWARD && USE_IR) {
            sampleIR();
            constexpr float IR_CONF_LO    = 200.0f;
            constexpr float IR_CONF_HI    = 800.0f;
            constexpr float IR_EDGE_DELTA = 300.0f;

            // EMA + edge state are file-scope globals so scriptKick() can
            // re-seed `irFirstSample = true` and re-prime the filter at the
            // start of every script.
            if (irFirstSample) {
                irLSm = irVal[1]; irRSm = irVal[2];
                irLPrev = irLSm;  irRPrev = irRSm;
                irFirstSample = false;
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
            // Hold against the commanded heading, not against an absolute 0
            // that no longer means "straight ahead" after the first turn.
            int yawBias    = (USE_IMU && imuReady)
                              ? (int)(-YAW_HOLD_KP * (yawDeg - yawTargetDeg) - YAW_HOLD_KD * gzFilt) : 0;
            int encBalance = (int)((tL - tR) * BALANCE_KP);
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
        if (TELEMETRY) {
            static uint32_t lastTel = 0;
            if (millis() - lastTel > 80) {
                const char* phn = (runPhase == PH_FORWARD) ? "FWD"
                                : (runPhase == PH_PIVOT)   ? "PIV" : "SPOT";
                if (runPhase == PH_FORWARD) {
                    Serial.printf("t=%lu ph=FWD/TRAP tgt=%ld xCmd=%+.0f vCmd=%+.0f avg=%+.1f err=%+.1f v=%+.1f thr=%+d pwmL=%+d pwmR=%+d yaw=%+.2f tL=%ld tR=%ld\n",
                                  (unsigned long)millis(), runTarget,
                                  xCmdSigned, vCmdSigned, avg, posErr, velFilt,
                                  throttle, pwmL, pwmR, yawDeg, tL, tR);
                } else {
                    Serial.printf("t=%lu ph=%s%s tgt=%ld avg=%+.1f err=%+.1f v=%+.1f thr=%+d pwmL=%+d pwmR=%+d yaw=%+.2f tL=%ld tR=%ld\n",
                                  (unsigned long)millis(), phn, imuMode ? "/IMU" : "",
                                  runTarget, avg, posErr, velFilt, throttle, pwmL, pwmR, yawDeg, tL, tR);
                }
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
