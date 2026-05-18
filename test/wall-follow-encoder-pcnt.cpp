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
    float    kd           = 0.08f;   // PWM per (tick/sec) of forward velocity (higher → brakes earlier, less coast)
    int      maxPwm       = 200;     // lower cruise PWM → less momentum to dump at brake
    int      stictionPwm  = 120;     // min PWM applied when |err| > frictionZone
    int      frictionZone = 12;      // ticks; below this no stiction floor (must be ≤ hb)
    int      holdBand     = 15;      // ticks; ±~2 mm settle band
    uint32_t settleMs     = 200;
    int      stopBias     = 0;       // ticks; added to target. + → robot overshoots target, − → undershoots

    // Stall-escape settle (handles "stuck just outside hb" case)
    float    stallVel     = 30.0f;   // ticks/sec; below this considered stalled
    uint32_t stallMs      = 200;     // how long stalled before accepting current pos
    int      stallErrMax  = 40;      // ticks; max err to accept stall settle (~5 mm)

    // Steering bias terms (added on top of throttle)
    float    balanceKp    = 0.10f;   // (tL−tR) gain
    float    centerKp     = 0.04f;   // IR centering proportional gain
    float    centerKi     = 0.0f;
    float    centerKd     = 0.0f;    // disabled; derivative kicks on noisy irErr
    int      centerMax    = 15;      // clamp |IR corr|; small enough that L/R diff can't yaw robot

    // Misc
    long     ticksPerCell = 1405;    // measured 2026-05-18 hand-roll 180 mm
    bool     telemetry    = true;    // CSV print during RUN (`tel 0` to silence)
    bool     useIr        = false;   // encoder-only run when false (no IR sample, no centering, no front-wall stop)
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
    { EMIT_LF, RX_LF },
    { EMIT_L,  RX_L  },
    { EMIT_R,  RX_R  },
    { EMIT_RF, RX_RF },
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
static int calL = 800;
static int calR = 800;

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
enum State { IDLE, CAL, RUN, LIVE, ENC_TEST };
State state = IDLE;

enum MenuItem {
    M_CAL = 0,
    M_ENC,
    M_RUN1, M_RUN2, M_RUN3, M_RUN4, M_RUN5,
    M_LIVE,
    M_COUNT
};
static const char* MENU_LABELS[M_COUNT] = {
    "Calibrate",
    "Encoder Test",
    "Run 1 cell", "Run 2 cells", "Run 3 cells", "Run 4 cells", "Run 5 cells",
    "Live IR"
};
static int  menuSel    = M_ENC;
static long menuEncRef = 0;

// Active run params
static int  runCells   = 1;
static long runTarget  = 0;

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
void oledMenu();   // fwd decl — defined below in OLED screens section

void serialDump() {
    Serial.printf("kp=%.3f kd=%.3f max=%d stk=%d fz=%d hb=%d sms=%lu bias=%d\n"
                  "stallV=%.1f stallMs=%lu stallEmax=%d\n"
                  "bal=%.3f ckp=%.3f cki=%.3f ckd=%.3f cmax=%d\n"
                  "tpc=%ld tel=%d ir=%d\n",
        T.kp, T.kd, T.maxPwm, T.stictionPwm, T.frictionZone,
        T.holdBand, (unsigned long)T.settleMs, T.stopBias,
        T.stallVel, (unsigned long)T.stallMs, T.stallErrMax,
        T.balanceKp, T.centerKp, T.centerKi, T.centerKd, T.centerMax,
        T.ticksPerCell, T.telemetry ? 1 : 0, T.useIr ? 1 : 0);
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
        Serial.println("misc: tpc tel ir run stop tick reset show");
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
    else if (cmd == "run") {
        int n = parseInt(arg, 1);
        if (n < 1) n = 1;
        if (n > 20) n = 20;
        runCells  = n;
        runTarget = T.ticksPerCell * (long)runCells + (long)T.stopBias;
        pid.reset();
        leftEnc.reset(); rightEnc.reset();
        state = RUN;
        Serial.printf("RUN cells=%d target=%ld\n", runCells, runTarget);
    }
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
void oledMenu() {
    const int VIS = 5;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > M_COUNT - VIS) top = M_COUNT - VIS;
    if (top < 0) top = 0;

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "wf-pcnt");
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

void oledRun(int err, int corr, long ticks, long tgt, long tL, long tR) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    char hdr[24];
    snprintf(hdr, sizeof(hdr), "RUN %d cell", runCells);
    oled.drawStr(0, 10, hdr);
    oled.drawHLine(0, 12, 128);
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
    oled.setFont(u8g2_font_6x12_tf);
    char hdr[24];
    snprintf(hdr, sizeof(hdr), "RUN %d cell", runCells);
    oled.drawStr(0, 10, hdr);
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_logisoso42_tn);
    char buf[4]; snprintf(buf, sizeof(buf), "%d", n);
    int w = oled.getStrWidth(buf);
    oled.drawStr((128 - w) / 2, 60, buf);
    oled.sendBuffer();
}

void oledEncTest() {
    long tL = leftEnc.getTicks();
    long tR = rightEnc.getTicks();   // raw, no RIGHT_ENC_SCALE
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "Enc Test (PCNT)");
    oled.drawHLine(0, 12, 128);
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

    menuEncRef = rightEnc.getTicks();
    oledMenu();

    Serial.println();
    Serial.println("wall-follow-pcnt ready. `help` for cmds, `?` to dump tuning.");
    serialDump();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
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
                if (menuSel == M_CAL) {
                    sampleIR(); oledCal(); state = CAL;
                } else if (menuSel == M_ENC) {
                    leftEnc.reset(); rightEnc.reset();
                    oledEncTest();
                    state = ENC_TEST;
                } else if (menuSel == M_LIVE) {
                    sampleIR(); oledBars(); state = LIVE;
                } else {
                    runCells  = menuSel - M_RUN1 + 1;
                    runTarget = T.ticksPerCell * (long)runCells + (long)T.stopBias;
                    for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(1000); }
                    pid.reset();
                    leftEnc.reset(); rightEnc.reset();
                    Serial.printf("--- RUN START cells=%d target=%ld ---\n", runCells, runTarget);
                    serialDump();
                    state = RUN;
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
                state = IDLE;
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
                Serial.printf("--- RUN END reason=BTN tL=%ld tR=%ld ---\n",
                              leftEnc.getTicks(), rTicks());
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                state = IDLE;
                break;
            }

            long tL  = leftEnc.getTicks();
            long tR  = rTicks();
            long avg = (tL + tR) / 2;          // signed forward distance

            // ── Position PID throttle (velocity-damped) ─────────────────────
            //   u = KP·err − KD·vel
            //     err = target − avg  (+ve = drive forward)
            //     vel = LPF(d(avg)/dt) — damps in direction of motion
            //   Stiction floor applied only when |err| > frictionZone, so near
            //   target PWM can drop below stiction and let robot coast/brake.
            static long     posAvgPrev   = 0;
            static uint32_t posPrevUs    = 0;
            static float    velFilt      = 0.0f;
            static uint32_t settleStart  = 0;
            static uint32_t stallStart   = 0;

            long posErr = runTarget - avg;

            // Arrived?  Brake-hold and settle.
            if (labs(posErr) < T.holdBand) {
                stopMotors();
                if (settleStart == 0) settleStart = millis();
                if (millis() - settleStart > T.settleMs) {
                    Serial.printf("--- RUN END reason=SETTLED err=%+ld tL=%ld tR=%ld ---\n",
                                  posErr, tL, tR);
                    posAvgPrev = 0; posPrevUs = 0; velFilt = 0.0f;
                    settleStart = 0; stallStart = 0;
                    menuEncRef = rightEnc.getTicks();
                    oledMenu();
                    state = IDLE;
                }
                break;
            }
            settleStart = 0;

            uint32_t nowUs = micros();
            float dt = (posPrevUs == 0) ? 0.005f
                                        : constrain((nowUs - posPrevUs) / 1e6f, 1e-4f, 0.05f);
            posPrevUs = nowUs;

            float velRaw = (float)(avg - posAvgPrev) / dt;          // ticks/sec
            posAvgPrev   = avg;
            velFilt      = 0.7f * velFilt + 0.3f * velRaw;          // simple LPF

            // Stall-escape: outside holdBand but velocity ~zero (stiction won.
            // Accept current position if close enough).
            if (fabsf(velFilt) < T.stallVel && labs(posErr) < T.stallErrMax) {
                if (stallStart == 0) stallStart = millis();
                if (millis() - stallStart > T.stallMs) {
                    stopMotors();
                    Serial.printf("--- RUN END reason=STALL err=%+ld tL=%ld tR=%ld ---\n",
                                  posErr, tL, tR);
                    posAvgPrev = 0; posPrevUs = 0; velFilt = 0.0f;
                    settleStart = 0; stallStart = 0;
                    menuEncRef = rightEnc.getTicks();
                    oledMenu();
                    state = IDLE;
                    break;
                }
            } else {
                stallStart = 0;
            }

            float u   = T.kp * (float)posErr - T.kd * velFilt;
            int   mag = (int)fabsf(u);
            if (mag > T.maxPwm) mag = T.maxPwm;
            if (labs(posErr) > T.frictionZone && mag < T.stictionPwm) mag = T.stictionPwm;
            int throttle = (u >= 0) ? mag : -mag;

            // IR pipeline (optional — encoder-only when T.useIr == false).
            bool wL = false, wR = false, wF = false;
            int  irErr = 0;
            float corr = 0.0f;
            if (T.useIr) {
                sampleIR();
                wL = irVal[1] > WALL_SIDE_PRESENT;
                wR = irVal[2] > WALL_SIDE_PRESENT;
                wF = irVal[0] > WALL_FRONT_STOP || irVal[3] > WALL_FRONT_STOP;
                if (wF && throttle > 0) {
                    stopMotors();
                    Serial.printf("--- RUN END reason=WALL err=%+ld irLF=%d irRF=%d tL=%ld tR=%ld ---\n",
                                  posErr, irVal[0], irVal[3], tL, tR);
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

            // Steering bias on top of throttle (encoder balance always on).
            int encBalance = (int)((tL - tR) * T.balanceKp);
            int bias = (int)corr + encBalance;
            int pwmL = constrain(throttle - bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            int pwmR = constrain(throttle + bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            leftMotor.drive(pwmL);
            rightMotor.drive(pwmR);

            static uint32_t lastOled = 0;
            if (millis() - lastOled > 150) {
                oledRun(irErr, (int)corr, avg, runTarget, tL, tR);
                lastOled = millis();
            }
            if (T.telemetry) {
                static uint32_t lastTel = 0;
                if (millis() - lastTel > 50) {
                    bool stkActive = (labs(posErr) > T.frictionZone) && ((int)fabsf(u) < T.stictionPwm);
                    Serial.printf(
                        "t=%lu tgt=%ld avg=%ld err=%+ld v=%+.0f u=%+.1f mag=%d stk=%d thr=%+d "
                        "irLF=%d irL=%d irR=%d irRF=%d wL=%d wR=%d wF=%d "
                        "irErr=%+d corr=%+.1f bal=%+d pwmL=%+d pwmR=%+d tL=%ld tR=%ld\n",
                        (unsigned long)millis(), runTarget, avg, posErr, velFilt, u, mag, stkActive ? 1 : 0, throttle,
                        irVal[0], irVal[1], irVal[2], irVal[3], wL ? 1 : 0, wR ? 1 : 0, wF ? 1 : 0,
                        irErr, corr, encBalance, pwmL, pwmR, tL, tR);
                    lastTel = millis();
                }
            }
            break;
        }
    }
}
