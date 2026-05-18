// test/wall-follow-encoder-pcnt.cpp
//
// Fork of wall-follow-encoder-count-cell.cpp that swaps the rising-edge ISR
// encoder for the ESP32-S3 PCNT hardware quadrature decoder. 4x decode means
// ~4× the tick rate vs. the legacy driver, so TICKS_PER_CELL, BALANCE_KP and
// the menu-scroll step are retuned locally (PinConfig values stay untouched).
//
// First run procedure:
//   1. Pick "Encoder Test" from menu.
//   2. Roll the robot one full wheel revolution by hand → note tick count.
//      Expect ≈ 4 × physical PPR × gear = 4 × 14 × 30 = ~1680 ideal, but the
//      magnetic encoder's real PPR varies; capture the measured value.
//   3. Roll exactly one cell pitch (180 mm) along a wall → note ticks.
//      Update TICKS_PER_CELL_PCNT below to that measured value.
//   4. If a wheel's tick count decreases when the robot rolls forward, set
//      its `inverted` flag in the ctor below to true.
//
// Menu (right encoder = scroll, BUTTON_1 = select):
//   Calibrate
//   Encoder Test
//   Run 1..5 cells
//   Live IR

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoderPCNT.h"

// ── MPU-6500 (gyro for stable pivots/spot turns) ─────────────────────────────
// Shared I2C bus with OLED (OLED_SDA / OLED_SCL @ 400 kHz).
// Yaw is the integral of gz with bias subtracted; bias is captured once at
// boot and re-captured during each phase transition's brake/settle window.
// DLPF is configured for motor-vibration rejection (≤41 Hz bandwidth).
#define MPU_ADDR        0x68
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_XOUT_H 0x3B
// Gyro full-scale ±1000 dps. Scale = 32.8 LSB/(deg/s). Headroom for 360 dps peak.
#define GYRO_FS_SEL     0x10
#define GYRO_SCALE      32.8f
// DLPF_CFG=3 → 41 Hz BW, 5.9 ms group delay. Filters out high-freq motor noise.
#define DLPF_CFG_VAL    0x03

static float    gyroBiasZ   = 0.0f;
static float    yawDeg      = 0.0f;
static float    gzFilt      = 0.0f;     // low-passed gz, deg/s
static uint32_t lastYawUs   = 0;
static bool     imuReady    = false;

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

// Reads just gz (saves ~6 bytes of I2C per call vs full burst).
static bool readGzRaw(int16_t& out) {
    uint8_t b[2];
    if (!mpuReadN(0x47, b, 2)) return false;     // GYRO_ZOUT_H
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

// Average N gz samples. Robot MUST be still during call.
static void calibrateGyroBias(int N = 300, int sampleDelayMs = 2) {
    float sum = 0;
    int   good = 0;
    for (int i = 0; i < N; i++) {
        int16_t raw;
        if (readGzRaw(raw)) { sum += raw / GYRO_SCALE; good++; }
        delay(sampleDelayMs);
    }
    if (good > 0) gyroBiasZ = sum / good;
    gzFilt    = 0.0f;
    lastYawUs = micros();
}

// Integrate yaw from gz with bias subtraction + LPF + small deadband.
// Call frequently (every loop iteration).
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
    gzFilt = 0.7f * gzFilt + 0.3f * gz;        // light LPF for PID derivative
    yawDeg += gz * dt;
}

// ── PCNT-quadrature tuning (local; supersedes PinConfig ISR-encoder values) ──
constexpr long  ENC_PER_STEP = 80;   // 4× 80, menu-scroll detents

// ── Live tuning (settable over Serial; see handleCmd) ────────────────────────
// throttle = KP·err − KD·velocity. (Velocity-damped form — sign is correct
// regardless of approach direction. Reverses on overshoot via signed PWM.)
//   - Stiction floor STK applied only when |err| > FZ (avoids bang-bang near 0).
//   - Hold band HB widened to ~3 mm; below this, brake + settle exit.
struct Tuning {
    // Position PID (forward/reverse to target ticks)
    // INVARIANT: frictionZone <= holdBand. Otherwise robot stalls in the
    // dead zone (no stiction floor, but not inside settle band either).
    float    kp           = 0.20f;   // PWM per tick of position error
    float    kd           = 0.05f;   // PWM per (tick/sec) of forward velocity (lower → less brake jitter)
    int      maxPwm       = 200;     // lower cruise PWM → less momentum to dump at brake
    int      stictionPwm  = 110;     // min PWM applied when |err| > frictionZone
    int      frictionZone = 10;      // ticks; below this no stiction floor (must be ≤ hb)
    int      holdBand     = 20;      // ticks; ±~2.5 mm settle band
    uint32_t settleMs     = 80;      // shortened: chain steps with no inter-phase delay
    int      stopBias     = 0;       // ticks; added to target. + → robot overshoots target, − → undershoots

    // Stall-escape settle (handles "stuck just outside hb" case)
    float    stallVel     = 30.0f;   // ticks/sec; below this considered stalled
    uint32_t stallMs      = 200;     // how long stalled before accepting current pos
    int      stallErrMax  = 40;      // ticks; max err to accept stall settle (~5 mm)

    // Steering bias terms (added on top of throttle)
    float    balanceKp    = 0.03f;   // (tL−tR) gain — secondary; IMU heading hold is primary
    float    yawHoldKp    = 5.0f;    // PWM per deg of heading drift during forward (IMU-based)
    float    centerKp     = 0.04f;   // IR centering proportional gain
    float    centerKi     = 0.0f;
    float    centerKd     = 0.0f;    // disabled; derivative kicks on noisy irErr
    int      centerMax    = 15;      // clamp |IR corr|; small enough that L/R diff can't yaw robot

    // Misc
    long     ticksPerCell         = 1405;  // measured 2026-05-18 hand-roll 180 mm
    long     turnForwardTicks     = 1093;  // 14 cm — turn from cell-aligned start
    long     turnForwardTicksAfter= 858;   // 11 cm — turn following another turn (3.5 cm offset)
    long     turnPivotTicks       = 900;   // single-wheel arc for ~90° pivot (used if useImu=false)
    long     spot180Ticks         = 906;   // per-wheel arc for 180° spot (used if useImu=false)

    // IMU-based pivot/spot tuning (used when useImu=true)
    bool     useImu               = true;  // pivot/spot stop on yaw angle, not encoder ticks
    float    pivot90Deg           = 90.0f; // pivot target angle
    float    spot180Deg           = 180.0f;// spot target angle
    float    yawKp                = 6.0f;  // PWM per degree of yaw error
    float    yawKd                = 0.3f;  // PWM per (deg/s) of yaw rate
    int      yawMaxPwm            = 230;
    int      yawStictionPwm       = 130;   // overcome stiction in pivot/spot
    float    yawFrictionZone      = 3.0f;  // deg; no stiction floor inside this
    float    yawHoldBand          = 1.5f;  // deg; settle band
    uint32_t yawSettleMs          = 80;    // shortened: chain pivots/spots with no inter-phase delay
    float    yawStallVel          = 5.0f;  // deg/s; below this considered stalled
    uint32_t yawStallMs           = 250;
    float    yawStallErrMax       = 4.0f;  // deg; max err to accept stall settle
    bool     yawRecalEachPhase    = false; // skip per-phase recal so steps chain with no delay; boot + end-of-seq cal carry bias
    bool     telemetry       = true;  // CSV print during RUN (`tel 0` to silence)
    bool     useIr           = false; // encoder-only run when false (no IR sample, no centering, no front-wall stop)
};
static Tuning T;

// ── OLED ─────────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ── Hardware ─────────────────────────────────────────────────────────────────
MicromouseMotor       leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor       rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
// inverted=true on both: MOTOR_*_INV flips IN1/IN2, so the magnetic encoder's
// "native +1 direction" ended up being mechanical reverse. Flip here so
// driveForward → positive ticks. Confirmed 2026-05-18 on this chassis.
MicromouseEncoderPCNT leftEnc   (PCNT_UNIT_0, ENC_L_A, ENC_L_B, /*inverted=*/true);
MicromouseEncoderPCNT rightEnc  (PCNT_UNIT_1, ENC_R_A, ENC_R_B, /*inverted=*/true);

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ── IR ───────────────────────────────────────────────────────────────────────
// PAIRS: 0=LF, 1=L, 2=R, 3=RF
struct IRPair { uint8_t emit, rx; };
static IRPair PAIRS[4] = {
    { EMIT_L,  RX_L  },
    { EMIT_LF, RX_LF },
    { EMIT_RF, RX_RF },
    { EMIT_R,  RX_R  },
};
static int irVal[4] = {0,0,0,0};

constexpr int WALL_SIDE_PRESENT = 400;
constexpr int WALL_FRONT_STOP   = 1500;

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

// ── Calibration ──────────────────────────────────────────────────────────────
static int calL = 1500;
static int calR = 1500;

// ── PID ──────────────────────────────────────────────────────────────────────
// IR centering gains + clamp moved into the Tuning struct above (live-tunable).

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

// ── Battery ──────────────────────────────────────────────────────────────────
// Reads BAT_V_SENSE through the resistor divider, returns volts + 0..100% pct.
// 2S LiPo: ~8.4 V full, ~6.4 V "empty" (cutoff before damage). Linear mapping.
static float readVbat() {
    int raw = analogRead(BAT_V_SENSE);
    return (raw / 4095.0f) * 3.3f * BAT_VDIV_MULT;
}
static int batPct() {
    float v = readVbat();
    if (v < 6.4f) return 0;
    if (v > 8.4f) return 100;
    return (int)((v - 6.4f) * 50.0f + 0.5f);   // (v-6.4)/2.0 × 100
}

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

// ── State / menu ─────────────────────────────────────────────────────────────
enum State { IDLE, CAL, RUN, LIVE, ENC_TEST, GYRO_CAL };
State state = IDLE;

enum MenuItem {
    M_ENC = 0,
    M_GYRO_CAL,
    M_TURN_R,
    M_TURN_L,
    M_SEQ,
    M_COUNT
};
static const char* MENU_LABELS[M_COUNT] = {
    "Encoder Test",
    "Cal Gyro",
    "Turn Right",
    "Turn Left",
    "Sequence"
};
static int  menuSel    = M_ENC;
static long menuEncRef = 0;

// ── Run state ────────────────────────────────────────────────────────────────
// Movement is driven by a `script` — an array of PhaseStep records. Each step
// names a phase type, a tick target, and a direction (for pivot/spot steps).
// The runtime resets encoders at every phase boundary so each phase counts
// progress from 0.
//
// Phase semantics:
//   PH_FORWARD  → both wheels drive forward. Progress = (tL + tR) / 2.
//   PH_PIVOT    → inner wheel braked, outer wheel drives. Right turn → outer=L.
//                 Progress = outer wheel's ticks.
//   PH_SPOT     → wheels drive opposite. Right spot → L forward + R reverse.
//                 Progress = (tL − tR)/2 for right, (tR − tL)/2 for left.
//
// `lastWasTurn` tracks whether the previous routine ended in a turn. Used by
// the standalone Turn Right/Left menu items to pick a shortened forward leg.
enum TurnDir  { TURN_NONE, TURN_RIGHT, TURN_LEFT };
enum RunPhase { PH_FORWARD, PH_PIVOT, PH_SPOT };

struct PhaseStep {
    RunPhase phase;
    long     target;
    TurnDir  dir;
};

constexpr int MAX_SCRIPT = 20;
static PhaseStep script[MAX_SCRIPT];
static int       scriptLen   = 0;
static bool      scriptAutoCalAtEnd = false;   // true → re-cal gyro bias when last step completes
static int       scriptIdx   = 0;

static TurnDir   runTurnDir  = TURN_NONE;
static RunPhase  runPhase    = PH_FORWARD;
static long      runTarget   = 0;
static bool      lastWasTurn = false;

// ── Serial tuner ─────────────────────────────────────────────────────────────
// Line protocol @115200 baud, '\n' terminated. Examples:
//   ?            show all
//   kp 0.4 | kd 0.15 | max 220 | stk 100 | fz 60 | hb 25 | sms 300
//   bal 0.5      tL−tR steering-bias gain
//   tpc 1405     ticks per cell (refresh after measurement)
//   tel 1|0      CSV telemetry during RUN
//   run 1..5     start N-cell run
//   stop
//   tick         print live tL,tR,avg
//   reset        zero encoders now
//   help
static String rxBuf;
void oledMenu();          // fwd decl — defined below in OLED screens section
void scriptLoadTurn(TurnDir d, bool postTurn);
void scriptLoadSequence();
void scriptKick();

void serialDump() {
    Serial.printf("kp=%.3f kd=%.3f max=%d stk=%d fz=%d hb=%d sms=%lu bias=%d\n"
                  "stallV=%.1f stallMs=%lu stallEmax=%d\n"
                  "bal=%.3f ckp=%.3f cki=%.3f ckd=%.3f cmax=%d\n"
                  "tpc=%ld fwd=%ld fwda=%ld piv=%ld spt=%ld lwt=%d\n"
                  "tel=%d ir=%d vbat=%.2fV (%d%%)\n",
        T.kp, T.kd, T.maxPwm, T.stictionPwm, T.frictionZone,
        T.holdBand, (unsigned long)T.settleMs, T.stopBias,
        T.stallVel, (unsigned long)T.stallMs, T.stallErrMax,
        T.balanceKp, T.centerKp, T.centerKi, T.centerKd, T.centerMax,
        T.ticksPerCell, T.turnForwardTicks, T.turnForwardTicksAfter,
        T.turnPivotTicks, T.spot180Ticks, lastWasTurn ? 1 : 0,
        T.telemetry ? 1 : 0, T.useIr ? 1 : 0,
        readVbat(), batPct());
}

static int parseInt(const String& a, int dflt) {
    return a.length() ? a.toInt() : dflt;
}
static float parseFloat(const String& a, float dflt) {
    return a.length() ? a.toFloat() : dflt;
}

void handleCmd(String s) {
    s.trim();
    if (s.length() == 0) return;
    int sp = s.indexOf(' ');
    String cmd = (sp < 0) ? s : s.substring(0, sp);
    String arg = (sp < 0) ? String("") : s.substring(sp + 1);
    cmd.toLowerCase();

    if (cmd == "?" || cmd == "show")        { serialDump(); }
    else if (cmd == "help") {
        Serial.println("pos: kp kd max stk fz hb sms bias");
        Serial.println("stall: stv stms stem");
        Serial.println("steer: bal ckp cki ckd cmax");
        Serial.println("moves: tr tl seq   (fwd fwda piv spt tpc lwt)");
        Serial.println("misc: tel ir stop tick reset show");
    }
    else if (cmd == "kp")   { T.kp = parseFloat(arg, T.kp); serialDump(); }
    else if (cmd == "kd")   { T.kd = parseFloat(arg, T.kd); serialDump(); }
    else if (cmd == "max")  { T.maxPwm = parseInt(arg, T.maxPwm); serialDump(); }
    else if (cmd == "stk")  { T.stictionPwm = parseInt(arg, T.stictionPwm); serialDump(); }
    else if (cmd == "fz")   { T.frictionZone = parseInt(arg, T.frictionZone); serialDump(); }
    else if (cmd == "hb")   { T.holdBand = parseInt(arg, T.holdBand); serialDump(); }
    else if (cmd == "sms")  { T.settleMs = (uint32_t)parseInt(arg, T.settleMs); serialDump(); }
    else if (cmd == "bias") { T.stopBias = parseInt(arg, T.stopBias); serialDump(); }
    else if (cmd == "stv")  { T.stallVel = parseFloat(arg, T.stallVel); serialDump(); }
    else if (cmd == "stms") { T.stallMs = (uint32_t)parseInt(arg, T.stallMs); serialDump(); }
    else if (cmd == "stem") { T.stallErrMax = parseInt(arg, T.stallErrMax); serialDump(); }
    else if (cmd == "bal")  { T.balanceKp = parseFloat(arg, T.balanceKp); serialDump(); }
    else if (cmd == "yhkp") { T.yawHoldKp = parseFloat(arg, T.yawHoldKp); serialDump(); }
    else if (cmd == "ckp")  { T.centerKp = parseFloat(arg, T.centerKp); serialDump(); }
    else if (cmd == "cki")  { T.centerKi = parseFloat(arg, T.centerKi); serialDump(); }
    else if (cmd == "ckd")  { T.centerKd = parseFloat(arg, T.centerKd); serialDump(); }
    else if (cmd == "cmax") { T.centerMax = parseInt(arg, T.centerMax); serialDump(); }
    else if (cmd == "tpc")  { T.ticksPerCell = parseInt(arg, T.ticksPerCell); serialDump(); }
    else if (cmd == "tel")  { T.telemetry = parseInt(arg, T.telemetry ? 1 : 0) != 0; serialDump(); }
    else if (cmd == "ir")   { T.useIr = parseInt(arg, T.useIr ? 1 : 0) != 0; serialDump(); }
    else if (cmd == "tick") {
        long tL = leftEnc.getTicks(), tR = rTicks();
        Serial.printf("tL=%ld tR=%ld avg=%ld\n", tL, tR, (tL + tR) / 2);
    }
    else if (cmd == "reset") {
        leftEnc.reset(); rightEnc.reset();
        Serial.println("enc reset");
    }
    else if (cmd == "stop") {
        stopMotors();
        state = IDLE;
        oledMenu();
        Serial.println("STOP");
    }
    else if (cmd == "tr" || cmd == "tl") {
        TurnDir d = (cmd == "tr") ? TURN_RIGHT : TURN_LEFT;
        scriptLoadTurn(d, lastWasTurn);
        Serial.printf("--- RUN START kind=TURN dir=%s (%s) ---\n",
                      d == TURN_RIGHT ? "RIGHT" : "LEFT",
                      lastWasTurn ? "POST-TURN, short" : "STANDALONE");
        scriptKick();
        state = RUN;
    }
    else if (cmd == "seq") {
        scriptLoadSequence();
        Serial.println("--- RUN START kind=SEQUENCE (2 cells, R, R-short, 3 cells, 180) ---");
        scriptKick();
        state = RUN;
    }
    else if (cmd == "fwd")  { T.turnForwardTicks      = parseInt(arg, T.turnForwardTicks);      serialDump(); }
    else if (cmd == "fwda") { T.turnForwardTicksAfter = parseInt(arg, T.turnForwardTicksAfter); serialDump(); }
    else if (cmd == "piv")  { T.turnPivotTicks        = parseInt(arg, T.turnPivotTicks);        serialDump(); }
    else if (cmd == "spt")  { T.spot180Ticks          = parseInt(arg, T.spot180Ticks);          serialDump(); }
    else if (cmd == "lwt")  { lastWasTurn = parseInt(arg, lastWasTurn ? 1 : 0) != 0;
                              Serial.printf("lastWasTurn=%d\n", lastWasTurn ? 1 : 0); }
    else {
        Serial.print("? unknown: "); Serial.println(cmd);
    }
}

void serialPoll() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') { handleCmd(rxBuf); rxBuf = ""; }
        else { rxBuf += c; if (rxBuf.length() > 80) rxBuf = ""; }
    }
}

// ── OLED screens ─────────────────────────────────────────────────────────────
// Draws a small battery indicator at the top-right (icon + percentage).
// Layout: "85%" text at x≈108, a 12×6 outline icon to its right with a fill
// bar proportional to percentage.
static void drawBatteryTopRight() {
    int pct = batPct();
    char b[8]; snprintf(b, sizeof(b), "%d%%", pct);
    oled.setFont(u8g2_font_5x7_tf);
    int w = oled.getStrWidth(b);
    int xText = 128 - 16 - w - 1;           // 16 px reserved for icon
    int yText = 7;
    oled.drawStr(xText, yText, b);
    // Icon (outlined battery with tip + fill).
    int xIcon = 128 - 14;
    int yIcon = 1;
    oled.drawFrame(xIcon, yIcon, 12, 6);
    oled.drawBox  (xIcon + 12, yIcon + 2, 2, 2);   // tip
    int fillW = (pct * 10 + 50) / 100;             // 0..10
    if (fillW > 0) oled.drawBox(xIcon + 1, yIcon + 1, fillW, 4);
}

void oledMenu() {
    const int VIS = 5;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > M_COUNT - VIS) top = M_COUNT - VIS;
    if (top < 0) top = 0;

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "wf-pcnt");
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

void oledRun(int err, int corr, long ticks, long tgt, long tL, long tR) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    char hdr[24];
    const char* phName = (runPhase == PH_FORWARD) ? "FWD"
                       : (runPhase == PH_PIVOT)   ? "PIV" : "SPOT";
    const char* dirName = (runTurnDir == TURN_RIGHT) ? "R"
                        : (runTurnDir == TURN_LEFT)  ? "L" : "-";
    snprintf(hdr, sizeof(hdr), "%s/%s %d/%d", phName, dirName, scriptIdx + 1, scriptLen);
    oled.drawStr(0, 8, hdr);
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_5x7_tf);
    oled.setFont(u8g2_font_5x7_tf);
    char buf[28];
    snprintf(buf, sizeof(buf), "IR L%4d R%4d", irVal[1], irVal[2]);
    oled.drawStr(0, 22, buf);
    snprintf(buf, sizeof(buf), "tL%+5ld tR%+5ld", tL, tR);
    oled.drawStr(0, 32, buf);
    snprintf(buf, sizeof(buf), "err%+d c%+d", err, corr);
    oled.drawStr(0, 42, buf);
    snprintf(buf, sizeof(buf), "|avg| %ld/%ld", ticks, tgt);
    oled.drawStr(0, 52, buf);
    int pw = (int)((ticks * 124L) / (tgt > 0 ? tgt : 1));
    if (pw < 0) pw = 0; if (pw > 124) pw = 124;
    oled.drawFrame(0, 54, 126, 8);
    oled.drawBox(1, 55, pw, 6);
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

// ── Gyro-cal screen ──────────────────────────────────────────────────────────
// Shows live IR vs IR_CAL targets so the user can square the robot to a wall
// before triggering bias capture. The cal trigger itself is driven by
// encoder + gyro stillness, NOT IR (IR is just the alignment aid).
static int gcalSampleIRPair(uint8_t emit, uint8_t rx) {
    digitalWrite(emit, LOW);
    delayMicroseconds(80);
    int amb = analogRead(rx);
    digitalWrite(emit, HIGH);
    delayMicroseconds(80);
    int lit = analogRead(rx);
    digitalWrite(emit, LOW);
    int d = amb - lit;
    return d < 0 ? 0 : d;
}
static void oledGyroCal(int irLF, int irRF, bool still, int stillMs, const char* msg) {
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
    snprintf(buf, sizeof(buf), "still=%d  hold=%d/100ms", still ? 1 : 0, stillMs);
    oled.drawStr(0, 44, buf);
    snprintf(buf, sizeof(buf), "yaw=%+.2f bias=%.3f", yawDeg, gyroBiasZ);
    oled.drawStr(0, 54, buf);
    oled.drawStr(0, 63, msg ? msg : "btn=exit");
    oled.sendBuffer();
}

void oledEncTest() {
    long tL = leftEnc.getTicks();
    long tR = rightEnc.getTicks();   // raw, no RIGHT_ENC_SCALE
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "Enc Test");
    drawBatteryTopRight();
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "L %ld", tL);
    oled.drawStr(0, 28, buf);
    snprintf(buf, sizeof(buf), "R %ld", tR);
    oled.drawStr(0, 44, buf);
    oled.setFont(u8g2_font_5x7_tf);
    snprintf(buf, sizeof(buf), "cell tgt %ld", T.ticksPerCell);
    oled.drawStr(0, 56, buf);
    oled.drawStr(0, 63, "btn=reset+back");
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

// ── Script construction ──────────────────────────────────────────────────────
static void scriptReset() { scriptLen = 0; scriptIdx = 0; scriptAutoCalAtEnd = false; }
static void scriptPush(RunPhase ph, long target, TurnDir d = TURN_NONE) {
    if (scriptLen < MAX_SCRIPT) script[scriptLen++] = { ph, target, d };
}
// Forward leg with stopBias compensation.
static void scriptPushFwd  (long ticks)         { scriptPush(PH_FORWARD, ticks + (long)T.stopBias, TURN_NONE); }
static void scriptPushPivot(TurnDir d) {
    long target = T.useImu ? (long)(T.pivot90Deg + 0.5f) : T.turnPivotTicks;
    scriptPush(PH_PIVOT, target, d);
}
static void scriptPushSpot(TurnDir d) {
    long target = T.useImu ? (long)(T.spot180Deg + 0.5f) : T.spot180Ticks;
    scriptPush(PH_SPOT, target, d);
}

// Standard single-turn script. `postTurn`=true uses the shortened forward leg.
void scriptLoadTurn(TurnDir d, bool postTurn) {
    scriptReset();
    scriptPushFwd(postTurn ? T.turnForwardTicksAfter : T.turnForwardTicks);
    scriptPushPivot(d);
}

// Full sequence (13 steps):
//   1. fwd 2 cells
//   2. fwd 14 cm           (approach for turn 1)
//   3. pivot right
//   4. fwd 11 cm           (approach for turn 2, post-turn short)
//   5. pivot right
// 5.5. fwd 14 cm 
//   6. fwd 2 cells
//   7. spot 180°
//   8. fwd 2 cells
//   8.5. fwd 14 cm  
//   9. pivot left
//  10. fwd 11 cm
//  11. pivot left
// 11.5. fwd 14 cm  
//  12. fwd 2 cells
//  13. spot 180°
// calibrate gyro
void scriptLoadSequence() {
    scriptReset();
    scriptAutoCalAtEnd = true;                  // auto gyro recal when sequence done
    // First half (right pivots).
    scriptPushFwd(T.ticksPerCell * 2);
    scriptPushFwd(T.turnForwardTicks);          // step 2: pre-turn
    scriptPushPivot(TURN_RIGHT);
    scriptPushFwd(T.turnForwardTicksAfter);     // step 4: chained
    scriptPushPivot(TURN_RIGHT);
    scriptPushFwd(T.turnForwardTicks);          // step 5.5: finish-cell
    scriptPushFwd(T.ticksPerCell * 2);          // step 6
    scriptPushSpot(TURN_RIGHT);                 // step 7
    // Second half (left pivots).
    scriptPushFwd(T.ticksPerCell * 2);          // step 8
    scriptPushFwd(T.turnForwardTicks);          // step 8.5: pre-turn
    scriptPushPivot(TURN_LEFT);
    scriptPushFwd(T.turnForwardTicksAfter);     // step 10: chained
    scriptPushPivot(TURN_LEFT);
    scriptPushFwd(T.turnForwardTicks);          // step 11.5: finish-cell
    scriptPushFwd(T.ticksPerCell * 2);          // step 12
    scriptPushSpot(TURN_LEFT);                  // step 13
}

// Activate the first step of the loaded script (resets encoders, primes PID).
void scriptKick() {
    if (scriptLen == 0) return;
    scriptIdx  = 0;
    runPhase   = script[0].phase;
    runTarget  = script[0].target;
    runTurnDir = script[0].dir;
    leftEnc.reset();
    rightEnc.reset();
    pid.reset();
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

    // MPU-6500: init + initial bias capture (robot MUST be still at boot).
    imuReady = mpuInit();
    if (imuReady) {
        Serial.println("[IMU] mpu6500 ok, calibrating bias (300 samples)... keep still");
        calibrateGyroBias(300, 2);
        yawDeg = 0.0f;
        Serial.printf("[IMU] bias=%.4f deg/s\n", gyroBiasZ);
    } else {
        Serial.println("[IMU] mpu6500 NOT detected — pivot/spot will fall back to encoder ticks");
    }

    menuEncRef = rightEnc.getTicks();
    oledMenu();

    Serial.println();
    Serial.println("wall-follow-pcnt ready. `help` for cmds, `?` to dump tuning.");
    serialDump();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    updateYaw();          // continuous yaw integration (no-op if IMU absent)
    serialPoll();
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
                if (menuSel == M_ENC) {
                    leftEnc.reset(); rightEnc.reset();
                    lastWasTurn = false;            // re-align: assume cell-aligned start
                    oledEncTest();
                    state = ENC_TEST;
                } else if (menuSel == M_GYRO_CAL) {
                    leftEnc.reset(); rightEnc.reset();
                    oledGyroCal(0, 0, false, 0, "btn=exit");
                    state = GYRO_CAL;
                } else if (menuSel == M_TURN_R || menuSel == M_TURN_L) {
                    TurnDir d = (menuSel == M_TURN_R) ? TURN_RIGHT : TURN_LEFT;
                    scriptLoadTurn(d, lastWasTurn);
                    for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(1000); }
                    Serial.printf("--- RUN START kind=TURN dir=%s (%s) ---\n",
                                  d == TURN_RIGHT ? "RIGHT" : "LEFT",
                                  lastWasTurn ? "POST-TURN, short" : "STANDALONE");
                    serialDump();
                    scriptKick();
                    state = RUN;
                } else if (menuSel == M_SEQ) {
                    scriptLoadSequence();
                    for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(1000); }
                    Serial.println("--- RUN START kind=SEQUENCE (2 cells, R, R-short, 3 cells, 180) ---");
                    serialDump();
                    scriptKick();
                    state = RUN;
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
            // Cal Gyro state. Robot is meant to be pressed against a wall;
            // OLED shows IR LF/RF vs IR_CAL_LF/IR_CAL_RF as a visual aid.
            // Bias capture only triggers once encoders + gyro have been still
            // for STILL_HOLD_MS continuously (auto-detected, no extra button).
            static uint32_t lastUiMs   = 0;
            static uint32_t stillStart = 0;
            static long     prevTL = 0, prevTR = 0;
            constexpr uint32_t STILL_HOLD_MS    = 100;
            constexpr float    STILL_GZ_LIMIT   = 1.0f;     // deg/s
            constexpr long     STILL_TICK_LIMIT = 1;        // per refresh

            if (buttonEdge()) {                              // manual exit
                stopMotors();
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                state = IDLE;
                break;
            }

            if (millis() - lastUiMs > 50) {
                lastUiMs = millis();
                int irLF = gcalSampleIRPair(EMIT_LF, RX_LF);
                int irRF = gcalSampleIRPair(EMIT_RF, RX_RF);
                long tL  = leftEnc.getTicks();
                long tR  = rTicks();
                bool encStill = (labs(tL - prevTL) <= STILL_TICK_LIMIT)
                             && (labs(tR - prevTR) <= STILL_TICK_LIMIT);
                prevTL = tL; prevTR = tR;
                bool gzStill = fabsf(gzFilt) < STILL_GZ_LIMIT;
                bool still   = encStill && gzStill && imuReady;

                if (still) {
                    if (stillStart == 0) stillStart = millis();
                } else {
                    stillStart = 0;
                }
                uint32_t held = (stillStart == 0) ? 0 : (millis() - stillStart);
                if (held > 500) held = 500;

                if (stillStart && (millis() - stillStart) >= STILL_HOLD_MS) {
                    oledGyroCal(irLF, irRF, true, (int)held, "calibrating...");
                    delay(20);
                    calibrateGyroBias(300, 2);
                    yawDeg = 0.0f;
                    Serial.printf("[GCAL] bias=%.4f deg/s\n", gyroBiasZ);
                    oledGyroCal(irLF, irRF, true, (int)held, "DONE — btn=exit");
                    // Wait for button to leave (or press again).
                    while (digitalRead(BUTTON_1) == LOW) { delay(5); }
                    delay(150);
                    // Reset capture state for the next entry.
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

        case LIVE: {
            static uint32_t last = 0;
            if (millis() - last > 100) { sampleIR(); oledBars(); last = millis(); }
            if (buttonEdge()) {
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                state = IDLE;
            }
            break;
        }

        case RUN: {
            if (buttonEdge()) {
                stopMotors();
                Serial.printf("--- RUN END reason=BTN phase=%s tL=%ld tR=%ld ---\n",
                              runPhase == PH_FORWARD ? "FWD" : "PIV",
                              leftEnc.getTicks(), rTicks());
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                state = IDLE;
                break;
            }

            long tL  = leftEnc.getTicks();
            long tR  = rTicks();

            // Pivot/spot can use IMU (yaw degrees) or encoder ticks. FORWARD is
            // always encoder-based. `imuMode` selects gain set + units below.
            bool imuMode = T.useImu && imuReady && (runPhase != PH_FORWARD);

            // Phase-specific progress (avg).
            //   FORWARD:        ticks, average of both wheels
            //   PIVOT (ticks):  outer wheel ticks (inner braked)
            //   SPOT  (ticks):  (tL−tR)/2 signed by direction
            //   PIVOT/SPOT IMU: signed yaw progress in degrees
            float avg;
            if (runPhase == PH_FORWARD) {
                avg = (float)((tL + tR) / 2);
            } else if (imuMode) {
                // yawDeg was reset to 0 at phase start.
                // Right turn → robot rotates CW → yawDeg becomes negative.
                avg = (runTurnDir == TURN_RIGHT) ? -yawDeg : +yawDeg;
            } else if (runPhase == PH_PIVOT) {
                avg = (float)((runTurnDir == TURN_RIGHT) ? tL : tR);
            } else /* PH_SPOT, ticks */ {
                avg = (float)((runTurnDir == TURN_RIGHT) ? (tL - tR) / 2
                                                         : (tR - tL) / 2);
            }

            // Select effective gains / bands by mode.
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

            // ── Position PID throttle (velocity-damped) ─────────────────────
            static float    posAvgPrev   = 0.0f;
            static uint32_t posPrevUs    = 0;
            static float    velFilt      = 0.0f;
            static uint32_t settleStart  = 0;
            static uint32_t stallStart   = 0;

            float posErr = (float)runTarget - avg;

            // Helper: reset per-phase PID state.
            auto resetPidState = [&]() {
                posAvgPrev = 0.0f; posPrevUs = 0; velFilt = 0.0f;
                settleStart = 0; stallStart = 0;
            };

            // Helper: end the current script step. Advance scriptIdx and
            // either prime the next step or exit RUN if the script is done.
            auto endPhase = [&](const char* reason) {
                stopMotors();
                const char* phName = (runPhase == PH_FORWARD) ? "FWD"
                                   : (runPhase == PH_PIVOT)   ? "PIV" : "SPOT";
                Serial.printf("--- STEP END idx=%d/%d phase=%s reason=%s err=%+.2f tL=%ld tR=%ld yaw=%+.2f ---\n",
                              scriptIdx + 1, scriptLen, phName, reason, posErr, tL, tR, yawDeg);

                if (scriptIdx + 1 >= scriptLen) {
                    RunPhase lastPh = script[scriptLen - 1].phase;
                    lastWasTurn = (lastPh == PH_PIVOT || lastPh == PH_SPOT);
                    Serial.printf("--- RUN END lastWasTurn=%d ---\n", lastWasTurn ? 1 : 0);
                    // Auto gyro re-cal at sequence end. Robot is braked + still
                    // here, so this is a clean window for bias capture.
                    if (scriptAutoCalAtEnd && T.useImu && imuReady) {
                        Serial.println("[GCAL] auto re-cal at sequence end (200 samples)");
                        delay(150);                       // let motors fully settle
                        calibrateGyroBias(200, 2);
                        yawDeg = 0.0f;
                        Serial.printf("[GCAL] bias=%.4f deg/s\n", gyroBiasZ);
                    }
                    resetPidState();
                    runTurnDir = TURN_NONE;
                    menuEncRef = rightEnc.getTicks();
                    oledMenu();
                    state = IDLE;
                    return;
                }

                // Advance to next step.
                scriptIdx++;
                PhaseStep& next = script[scriptIdx];
                runPhase   = next.phase;
                runTarget  = next.target;
                runTurnDir = next.dir;
                leftEnc.reset();
                rightEnc.reset();
                resetPidState();
                pid.reset();

                // For rotational phases: refresh bias during the brake window
                // to fight gyro temperature drift. For ALL phases (including
                // forward), zero yaw so per-phase heading reference starts
                // clean — used by forward heading-hold and by pivot/spot.
                bool nextIsRot = (next.phase == PH_PIVOT || next.phase == PH_SPOT);
                if (T.useImu && imuReady) {
                    if (nextIsRot && T.yawRecalEachPhase) {
                        delay(80);
                        calibrateGyroBias(80, 1);
                    }
                    yawDeg    = 0.0f;
                    gzFilt    = 0.0f;
                    lastYawUs = micros();
                }
            };

            // Arrived? Brake-hold and settle.
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
                // Use gyro rate directly; signed by intended rotation direction.
                velFilt = (runTurnDir == TURN_RIGHT) ? -gzFilt : +gzFilt;
                posAvgPrev = avg;
            } else {
                float velRaw = (avg - posAvgPrev) / dt;          // ticks/sec
                posAvgPrev   = avg;
                velFilt      = 0.7f * velFilt + 0.3f * velRaw;
            }

            // Stall-escape: stiction won, accept current position if close enough.
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

            // IR centering (forward phase only; pivot ignores IR).
            int  irErr = 0;
            float corr = 0.0f;
            if (runPhase == PH_FORWARD && T.useIr) {
                sampleIR();
                bool wL = irVal[1] > WALL_SIDE_PRESENT;
                bool wR = irVal[2] > WALL_SIDE_PRESENT;
                bool wF = irVal[0] > WALL_FRONT_STOP || irVal[3] > WALL_FRONT_STOP;
                if (wF && throttle > 0) {
                    stopMotors();
                    Serial.printf("--- RUN END reason=WALL err=%+.2f irLF=%d irRF=%d tL=%ld tR=%ld ---\n",
                                  posErr, irVal[0], irVal[3], tL, tR);
                    runTurnDir = TURN_NONE;
                    menuEncRef = rightEnc.getTicks();
                    oledMenu();
                    state = IDLE;
                    break;
                }
                int errR = wR ? (irVal[2] - calR) : 0;
                int errL = wL ? (irVal[1] - calL) : 0;
                irErr = errR - errL;
                corr = (wL || wR) ? pid.compute((float)irErr) : 0.0f;
            }

            // Motor drive: phase-dependent mixing.
            int pwmL, pwmR;
            if (runPhase == PH_FORWARD) {
                // Heading correction sources (positive bias → steers LEFT,
                // i.e. pwmR > pwmL).
                //   IMU yaw drift × yawHoldKp  (primary, when imuReady)
                //   (tL−tR)        × balanceKp (secondary, encoder symmetry)
                //   IR centering corr          (if useIr)
                // Yaw convention here: right rotation → yawDeg < 0 (matches
                // pivot code that uses −yawDeg as right-turn progress). So a
                // rightward drift gives yawDeg < 0, and we want positive bias
                // to steer left → multiply by −1.
                int yawBias    = (T.useImu && imuReady) ? (int)(-T.yawHoldKp * yawDeg) : 0;
                int encBalance = (int)((tL - tR) * T.balanceKp);
                int bias = (int)corr + encBalance + yawBias;
                pwmL = constrain(throttle - bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
                pwmR = constrain(throttle + bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
                leftMotor.drive(pwmL);
                rightMotor.drive(pwmR);
            } else if (runPhase == PH_PIVOT) {
                // Outer wheel = throttle, inner wheel = brake (holds position).
                if (runTurnDir == TURN_RIGHT) {
                    pwmL = throttle;
                    pwmR = 0;
                    leftMotor.drive(pwmL);
                    rightMotor.brake();
                } else {
                    pwmL = 0;
                    pwmR = throttle;
                    leftMotor.brake();
                    rightMotor.drive(pwmR);
                }
            } else /* PH_SPOT */ {
                // Wheels run opposite — robot rotates around its own center.
                // Right spot: L forward, R reverse. Sign of throttle handles
                // both forward progress (throttle>0) and overshoot recovery.
                int sign = (runTurnDir == TURN_RIGHT) ? +1 : -1;
                pwmL = constrain( sign * throttle, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
                pwmR = constrain(-sign * throttle, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
                leftMotor.drive(pwmL);
                rightMotor.drive(pwmR);
            }

            static uint32_t lastOled = 0;
            if (millis() - lastOled > 150) {
                oledRun(irErr, (int)corr, avg, runTarget, tL, tR);
                lastOled = millis();
            }
            if (T.telemetry) {
                static uint32_t lastTel = 0;
                if (millis() - lastTel > 50) {
                    bool stkActive = (labs(posErr) > T.frictionZone) && ((int)fabsf(u) < T.stictionPwm);
                    const char* phn = (runPhase == PH_FORWARD) ? "FWD"
                                    : (runPhase == PH_PIVOT)   ? "PIV" : "SPOT";
                    Serial.printf(
                        "t=%lu step=%d/%d ph=%s tgt=%ld avg=%ld err=%+ld v=%+.0f u=%+.1f mag=%d stk=%d thr=%+d "
                        "corr=%+.1f pwmL=%+d pwmR=%+d tL=%ld tR=%ld\n",
                        (unsigned long)millis(), scriptIdx + 1, scriptLen, phn,
                        runTarget, avg, posErr, velFilt, u, mag, stkActive ? 1 : 0, throttle,
                        corr, pwmL, pwmR, tL, tR);
                    lastTel = millis();
                }
            }
            break;
        }
    }
}
