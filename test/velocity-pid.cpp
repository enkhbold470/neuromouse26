// test/velocity-pid.cpp
//
// Closed-loop wheel velocity control test.
//
// What this file does:
//   1. Locks motor PWM at 200 Hz (best torque on this rig — empirical).
//   2. CHAR: sweeps PWM 100..1000 step 100, holds each ~1.5 s, measures
//      steady-state mm/s per wheel, linear-fits kV (mm/s per pwm) and
//      offset (pwm dead-band) per wheel. Live progress on OLED.
//   3. TAPE: drives both wheels forward at TAPE_PWM until next button
//      click. Shows L/R ticks + mm on OLED so you can compare to a
//      tape-measure reference.
//   4. RUN: drives both wheels at target mm/s for RUN_MS using
//      feedforward (pwm = target/kV + offset) + per-wheel PI velocity
//      loop + battery-voltage compensation. Live target / vL / vR /
//      pwmL / pwmR on OLED.
//
//   ALL feedback is on the OLED — Serial is intentionally not used.
//
// OLED control:
//   - Scroll right-encoder wheel = move menu cursor.
//   - Click button = fire the selected action.
//   - In "Set target" the right-encoder edits target mm/s; next click locks.
//
// Build:  pio run -e velocity-pid -t upload

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── Configuration ─────────────────────────────────────────────────────────────
constexpr uint8_t  LCH1 = 0, LCH2 = 1, RCH1 = 2, RCH2 = 3;

// 200 Hz PWM — empirically gives best torque for this N20 + DRV8833 rig.
constexpr uint32_t PWM_FREQ_HZ   = 200;
constexpr int      PWM_BITS      = MOTOR_PWM_BITS;
constexpr int      PWM_MAX       = MOTOR_PWM_MAX;

// Right encoder scale. Final calibration:
//   first pass (R_SCALE=1.0) gave avg L/R ~1.067 → set R_SCALE=1.067
//   second pass at 1.067 gave L=11323, R=11591 → L/R=0.977 (R overscaled)
// Effective true ratio = 1.067 × 0.977 = 1.042 → use this.
constexpr float    R_SCALE       = 1.042f;

constexpr float    NOMINAL_VBAT  = 7.4f;        // 2S full-charge target
constexpr unsigned long LOOP_US  = 5000;        // 200 Hz control loop
                                                // matches one PWM cycle, so each
                                                // sample sees a full duty period.

// Characterization
constexpr int      CHAR_PWM_LO    = 100;
constexpr int      CHAR_PWM_HI    = 1000;
constexpr int      CHAR_PWM_STEP  = 100;
constexpr unsigned long CHAR_HOLD_MS   = 1500;
constexpr unsigned long CHAR_SETTLE_MS = 700;

// Motor saturates hard above ~pwm 300 (your data: 200→886, 300→993, 1000→1188).
// Above the knee, slope collapses → fitting full range gives a tiny kV that
// makes FF overshoot 3× for low targets. Restrict fit to the linear region.
constexpr int      CHAR_FIT_PWM_MAX = 300;

// Tape test
constexpr int      TAPE_PWM      = 250;

// Run test
constexpr unsigned long RUN_MS   = 4000;
constexpr int      TARG_MIN      = 200;          // sub-200 uncontrollable
                                                 // at 200 Hz on this rig
constexpr int      TARG_MAX      = 1500;
constexpr int      TARG_STEP     = 25;

// Velocity PI (post-feedforward correction; gains tuned for 200 Hz loop).
// Pushed harder because R motor is physically slower than L — PI integrator
// has to do work to drag R up to target.
constexpr float    LOOP_KP       = 1.00f;       // pwm per (mm/s) error
constexpr float    LOOP_KI       = 1.50f;       // pwm per (mm·s)
constexpr float    INTEG_LIMIT   = 800.0f;      // anti-windup, pwm units
constexpr float    EMA_ALPHA     = 0.50f;       // velocity low-pass

// Straight-line coupling — cuts veering by forcing equal L/R accumulated ticks.
//   straightErr (ticks) = ticksL - ticksR
//   pwmL -= STRAIGHT_KP * straightErr
//   pwmR += STRAIGHT_KP * straightErr
constexpr float    STRAIGHT_KP   = 6.00f;       // pwm per tick of L-R mismatch
constexpr int      STRAIGHT_MAX  = 500;         // clamp coupling authority

// Per-wheel pwm bias added on top of shared FF. R motor is hardware-slower on
// your rig — give it a constant boost so PI starts closer to needed pwm.
// Reduce R_PWM_BIAS to 0 if drifts left after this change.
constexpr int      L_PWM_BIAS    = 0;
constexpr int      R_PWM_BIAS    = 0; //333

// Pre-run finger-release delay — counts down on OLED so you can let go of
// the button before motion starts. Applied at the top of doRunPid and
// doTapeTest. Set to 0 to disable.
constexpr unsigned long PRE_RUN_WAIT_MS = 1000;

// Use shared (averaged) FF for both wheels — guarantees identical baseline pwm,
// so any asymmetry is corrected by per-wheel PI + R_PWM_BIAS, not baked into FF.
constexpr bool     USE_SHARED_FF = true;

// Menu encoder
constexpr long     ENC_PER_STEP  = 20;

// OLED redraw cadence inside long actions
constexpr unsigned long OLED_REDRAW_MS = 150;

// ── Hardware ──────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, LCH1, LCH2, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, RCH1, RCH2, MOTOR_R_INV);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);

static inline long rTicks() { return (long)(rightEnc.getTicks() * R_SCALE); }

// ── State ─────────────────────────────────────────────────────────────────────
// Defaults from your linear-region CHAR (slope across pwm 100..300):
//   L: (886-480)/(200-100) = 4.06, with offset ~19 pwm
//   R: (859-427)/(200-100) = 4.32, with offset ~1  pwm
// Re-run CHAR on ground each session — kV drifts with battery and surface.
static float kV_L  = 4.06f, kV_R  = 4.32f;
static float off_L = 19.0f, off_R = 1.0f;
static bool  charDone = false;

static int   targetMmS = 300;
static char  statusLine[28] = "boot";

static void setStatus(const char* fmt, ...) {
    va_list args; va_start(args, fmt);
    vsnprintf(statusLine, sizeof(statusLine), fmt, args);
    va_end(args);
}

// ── Frequency ─────────────────────────────────────────────────────────────────
static void applyPwmFreq() {
    ledcSetup(LCH1, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(LCH2, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(RCH1, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(RCH2, PWM_FREQ_HZ, PWM_BITS);
}

// ── Battery ───────────────────────────────────────────────────────────────────
static float readVbat() {
    int raw = analogRead(BAT_V_SENSE);
    return (raw / 4095.0f) * 3.3f * BAT_VDIV_MULT;
}

// ── Button edge ───────────────────────────────────────────────────────────────
static bool buttonEdge() {
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

// ── Linear fit: velocity = kV * (pwm - offset) ────────────────────────────────
// Filters out (a) dead-band (vel<5) and (b) saturation knee (pwm above
// CHAR_FIT_PWM_MAX), so the slope reflects the working linear region we
// actually run PID in.
static bool fitLine(const int* pwms, const float* vels, int n,
                    float& kV, float& offset) {
    int    keep = 0;
    double sx = 0, sy = 0, sxy = 0, sxx = 0;
    for (int i = 0; i < n; i++) {
        if (vels[i] < 5.0f)              continue;
        if (pwms[i] > CHAR_FIT_PWM_MAX)  continue;
        keep++;
        sx  += pwms[i];
        sy  += vels[i];
        sxy += (double)pwms[i] * vels[i];
        sxx += (double)pwms[i] * pwms[i];
    }
    if (keep < 2) return false;
    double denom = keep * sxx - sx * sx;
    if (denom == 0.0) return false;
    double slope = (keep * sxy - sx * sy) / denom;
    double inter = (sy - slope * sx) / keep;
    if (slope <= 0.0) return false;
    kV     = (float)slope;
    offset = (float)(-inter / slope);
    if (offset < 0) offset = 0;
    return true;
}

// ── OLED helpers ──────────────────────────────────────────────────────────────
static void drawCharScreen(int pwm, float vL, float vR, float vbat, int rowIdx, int total) {
    char l1[28], l2[28], l3[28], l4[28], l5[28];
    snprintf(l1, sizeof(l1), "CHAR sweep %d/%d", rowIdx, total);
    snprintf(l2, sizeof(l2), "pwm: %d", pwm);
    snprintf(l3, sizeof(l3), "vL : %.0f mm/s", vL);
    snprintf(l4, sizeof(l4), "vR : %.0f mm/s", vR);
    snprintf(l5, sizeof(l5), "vbat %.2f V", vbat);
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0,  9, l1);
    oled.drawHLine(0, 11, 128);
    oled.drawStr(0, 23, l2);
    oled.drawStr(0, 34, l3);
    oled.drawStr(0, 45, l4);
    oled.drawStr(0, 56, l5);
    oled.sendBuffer();
}

static void drawRunScreen(float target, float vL, float vR,
                          int pwmL, int pwmR, float vbat,
                          unsigned long elapsedMs) {
    char l1[28], l2[28], l3[28], l4[28], l5[28];
    snprintf(l1, sizeof(l1), "RUN T=%.0f t%lums", target, elapsedMs);
    snprintf(l2, sizeof(l2), "L %4.0f pw%4d", vL, pwmL);
    snprintf(l3, sizeof(l3), "R %4.0f pw%4d", vR, pwmR);
    snprintf(l4, sizeof(l4), "errL %+.0f", target - vL);
    snprintf(l5, sizeof(l5), "errR %+.0f  V%.2f", target - vR, vbat);
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0,  9, l1);
    oled.drawHLine(0, 11, 128);
    oled.drawStr(0, 23, l2);
    oled.drawStr(0, 34, l3);
    oled.drawStr(0, 45, l4);
    oled.drawStr(0, 56, l5);
    oled.sendBuffer();
}

static void drawTapeScreen(long tL, long tR, float mmL, float mmR) {
    char l1[28], l2[28], l3[28], l4[28];
    snprintf(l1, sizeof(l1), "TAPE click=stop");
    snprintf(l2, sizeof(l2), "L %6ld  %5.0fmm", tL, mmL);
    snprintf(l3, sizeof(l3), "R %6ld  %5.0fmm", tR, mmR);
    snprintf(l4, sizeof(l4), "L/R %.3f", tR > 0 ? (float)tL / (float)tR : 0.0f);
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0,  9, l1);
    oled.drawHLine(0, 11, 128);
    oled.drawStr(0, 25, l2);
    oled.drawStr(0, 38, l3);
    oled.drawStr(0, 56, l4);
    oled.sendBuffer();
}

// ── Pre-run finger-release countdown ──────────────────────────────────────────
// Waits for button release, then counts PRE_RUN_WAIT_MS down on OLED so the
// user can let go of the button before motion starts. Motors must already
// be coasting when called.
static void preRunCountdown(const char* label) {
    while (digitalRead(BUTTON_1) == LOW) {}
    if (PRE_RUN_WAIT_MS == 0) return;

    unsigned long start = millis();
    while (millis() - start < PRE_RUN_WAIT_MS) {
        unsigned long remain = PRE_RUN_WAIT_MS - (millis() - start);
        char l1[28], l2[28];
        snprintf(l1, sizeof(l1), "%s starting", label);
        snprintf(l2, sizeof(l2), "in %lu ms ...", remain);
        oled.clearBuffer();
        oled.setFont(u8g2_font_6x10_tf);
        oled.drawStr(0,  9, l1);
        oled.drawHLine(0, 11, 128);
        oled.drawStr(0, 30, l2);
        oled.drawStr(0, 50, "release button");
        oled.sendBuffer();
        delay(50);
    }
}

// ── CHAR: PWM sweep characterization (OLED progress) ──────────────────────────
static void doCharacterize() {
    setStatus("CHAR running");
    const int N = (CHAR_PWM_HI - CHAR_PWM_LO) / CHAR_PWM_STEP + 1;
    int   pwms[16];
    float vL[16], vR[16];
    int   idx = 0;

    for (int pwm = CHAR_PWM_LO; pwm <= CHAR_PWM_HI; pwm += CHAR_PWM_STEP) {
        leftMotor.drive(pwm);
        rightMotor.drive(pwm);

        delay(CHAR_SETTLE_MS);
        long l0 = leftEnc.getTicks();
        long r0 = rTicks();
        unsigned long t0 = millis();
        unsigned long sampleMs = CHAR_HOLD_MS - CHAR_SETTLE_MS;
        delay(sampleMs);
        long l1 = leftEnc.getTicks();
        long r1 = rTicks();
        float secs = (millis() - t0) / 1000.0f;

        float velL = ((l1 - l0) * MM_PER_TICK) / secs;
        float velR = ((r1 - r0) * MM_PER_TICK) / secs;

        pwms[idx] = pwm;
        vL[idx]   = velL;
        vR[idx]   = velR;
        idx++;

        drawCharScreen(pwm, velL, velR, readVbat(), idx, N);
    }
    leftMotor.coast();
    rightMotor.coast();

    bool okL = fitLine(pwms, vL, idx, kV_L, off_L);
    bool okR = fitLine(pwms, vR, idx, kV_R, off_R);
    charDone = okL && okR;

    if (charDone) setStatus("kV L%.2f R%.2f", kV_L, kV_R);
    else          setStatus("CHAR fit fail");
}

// ── TAPE: drive slow until next click; report measured mm ─────────────────────
static void doTapeTest() {
    preRunCountdown("TAPE");
    leftEnc.reset();
    rightEnc.reset();
    setStatus("TAPE click=stop");
    drawTapeScreen(0, 0, 0.0f, 0.0f);

    leftMotor.drive(TAPE_PWM);
    rightMotor.drive(TAPE_PWM);

    delay(50);

    unsigned long lastDraw = 0;
    while (!buttonEdge()) {
        unsigned long now = millis();
        if (now - lastDraw >= OLED_REDRAW_MS) {
            lastDraw = now;
            long tL = leftEnc.getTicks();
            long tR = rTicks();
            drawTapeScreen(tL, tR, tL * MM_PER_TICK, tR * MM_PER_TICK);
        }
    }

    leftMotor.coast();
    rightMotor.coast();

    long tL = leftEnc.getTicks();
    long tR = rTicks();
    float mmL = tL * MM_PER_TICK;
    float mmR = tR * MM_PER_TICK;
    drawTapeScreen(tL, tR, mmL, mmR);
    setStatus("L%.0f R%.0f mm", mmL, mmR);
}

// ── RUN: velocity PI + FF + battery comp for RUN_MS ──────────────────────────
static int feedforwardPwm(float target, float kV, float offset, float vScale) {
    if (target <= 0.0f || kV <= 0.0f) return 0;
    float pwm = (target / kV) + offset;
    pwm *= vScale;
    return (int)constrain(pwm, 0.0f, (float)PWM_MAX);
}

static void doRunPid() {
    preRunCountdown("RUN");
    leftEnc.reset();
    rightEnc.reset();
    long prevL = 0, prevR = 0;

    float velL_ema = 0.0f, velR_ema = 0.0f;
    // Cascaded controller: one speed integrator (avg vel → target),
    // one straightness integrator (curL-curR → 0). No per-wheel PI.
    float intSpeed = 0.0f;

    float vbat = readVbat();
    if (vbat < 5.5f) vbat = NOMINAL_VBAT;
    float vScale = NOMINAL_VBAT / vbat;

    unsigned long startMs = millis();
    unsigned long nextUs  = micros();
    unsigned long lastDraw = startMs;
    int lastPwmL = 0, lastPwmR = 0;

    setStatus("RUN %d mm/s", targetMmS);
    drawRunScreen((float)targetMmS, 0, 0, 0, 0, vbat, 0);

    while (millis() - startMs < RUN_MS) {
        // 200 Hz cadence
        while ((long)(micros() - nextUs) < 0) {}
        nextUs += LOOP_US;
        float dt = LOOP_US / 1000000.0f;

        long curL = leftEnc.getTicks();
        long curR = rTicks();
        float instL = ((curL - prevL) * MM_PER_TICK) / dt;
        float instR = ((curR - prevR) * MM_PER_TICK) / dt;
        prevL = curL; prevR = curR;

        velL_ema += EMA_ALPHA * (instL - velL_ema);
        velR_ema += EMA_ALPHA * (instR - velR_ema);

        float target = (float)targetMmS;

        // ── Speed loop: drive average velocity to target ──────────────────
        float velAvg   = 0.5f * (velL_ema + velR_ema);
        float errSpeed = target - velAvg;
        intSpeed = constrain(intSpeed + errSpeed * dt, -INTEG_LIMIT, INTEG_LIMIT);
        int   pidSpeed = (int)(LOOP_KP * errSpeed + LOOP_KI * intSpeed);

        // ── Straightness loop: drive accumulated tick mismatch to zero ────
        // Operates on tick mismatch (= integral of velocity diff) so it acts
        // like an I-term on velocity difference. Independent of speed loop.
        long  straightErr = curL - curR;
        int   pidStraight = (int)constrain((float)(STRAIGHT_KP * straightErr),
                                           (float)-STRAIGHT_MAX, (float)STRAIGHT_MAX);

        // Shared feedforward: avg kV / offset of both wheels.
        float kV_avg  = 0.5f * (kV_L + kV_R);
        float off_avg = 0.5f * (off_L + off_R);
        int   ffBase  = feedforwardPwm(target, kV_avg, off_avg, vScale);

        // Mix: speed adds to both, straightness subtracts from leader / adds to lagger.
        int basePwm = ffBase + pidSpeed;
        int pwmL = constrain(basePwm - pidStraight + L_PWM_BIAS, 0, PWM_MAX);
        int pwmR = constrain(basePwm + pidStraight + R_PWM_BIAS, 0, PWM_MAX);

        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);
        lastPwmL = pwmL; lastPwmR = pwmR;

        unsigned long nowMs = millis();
        if (nowMs - lastDraw >= OLED_REDRAW_MS) {
            lastDraw = nowMs;
            drawRunScreen(target, velL_ema, velR_ema, pwmL, pwmR,
                          vbat, nowMs - startMs);
        }
    }

    leftMotor.coast();
    rightMotor.coast();
    drawRunScreen((float)targetMmS, velL_ema, velR_ema, lastPwmL, lastPwmR,
                  vbat, RUN_MS);
    setStatus("done %.0f/%.0f", velL_ema, velR_ema);
}

// ── Menu ──────────────────────────────────────────────────────────────────────
enum Item { I_CHAR = 0, I_TAPE, I_TARG, I_RUN, I_INFO, I_COUNT };
static const char* MENU[I_COUNT] = {
    "Characterize", "Tape test", "Set target", "Run PID", "Show kV"
};
static int  menuSel = 0;
static long menuEncRef = 0;
static bool editingTarget = false;

static void drawMenu() {
    char hdr[28];
    snprintf(hdr, sizeof(hdr), "vbat %.2fV T%d", readVbat(), targetMmS);
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 9, hdr);
    oled.drawHLine(0, 11, 128);

    const int VIS = 4;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > I_COUNT - VIS) top = I_COUNT - VIS;
    if (top < 0) top = 0;

    for (int i = 0; i < VIS && (top + i) < I_COUNT; i++) {
        int idx = top + i;
        int y = 14 + i * 10;
        if (idx == menuSel) {
            oled.drawBox(0, y, 128, 10);
            oled.setDrawColor(0);
            oled.drawStr(3, y + 8, MENU[idx]);
            if (idx == I_TARG && editingTarget) oled.drawStr(96, y + 8, "EDIT");
            oled.setDrawColor(1);
        } else {
            oled.drawStr(3, y + 8, MENU[idx]);
        }
    }
    oled.drawHLine(0, 54, 128);
    oled.drawStr(0, 63, statusLine);
    oled.sendBuffer();
}

static void showKvScreen() {
    char l1[28], l2[28], l3[28], l4[28];
    snprintf(l1, sizeof(l1), "Cal status: %s", charDone ? "OK" : "DEFAULT");
    snprintf(l2, sizeof(l2), "L kV %.3f off %.0f", kV_L, off_L);
    snprintf(l3, sizeof(l3), "R kV %.3f off %.0f", kV_R, off_R);
    snprintf(l4, sizeof(l4), "R_SCALE %.3f", R_SCALE);
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0,  9, l1);
    oled.drawHLine(0, 11, 128);
    oled.drawStr(0, 25, l2);
    oled.drawStr(0, 38, l3);
    oled.drawStr(0, 51, l4);
    oled.sendBuffer();
    // hold screen until next button click
    while (digitalRead(BUTTON_1) == LOW) {}
    delay(50);
    while (!buttonEdge()) {}
}

static void dispatch(int sel) {
    switch (sel) {
        case I_CHAR: doCharacterize(); break;
        case I_TAPE: doTapeTest();     break;
        case I_TARG: editingTarget = !editingTarget;
                     setStatus(editingTarget ? "edit targ" : "lock %d", targetMmS);
                     break;
        case I_RUN:  doRunPid();       break;
        case I_INFO: showKvScreen();
                     setStatus("kV L%.2f R%.2f", kV_L, kV_R);
                     break;
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(BUTTON_1, INPUT_PULLUP);
    analogReadResolution(12);

    leftMotor.begin();
    rightMotor.begin();
    leftEnc.begin();
    rightEnc.begin();
    applyPwmFreq();

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    menuEncRef = rightEnc.getTicks();
    setStatus("ready 200Hz");
    drawMenu();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    long delta = rightEnc.getTicks() - menuEncRef;

    if (editingTarget) {
        if (delta >= ENC_PER_STEP) {
            targetMmS = constrain(targetMmS + TARG_STEP, TARG_MIN, TARG_MAX);
            menuEncRef += ENC_PER_STEP;
            drawMenu();
            return;
        }
        if (delta <= -ENC_PER_STEP) {
            targetMmS = constrain(targetMmS - TARG_STEP, TARG_MIN, TARG_MAX);
            menuEncRef -= ENC_PER_STEP;
            drawMenu();
            return;
        }
    } else {
        if (delta >= ENC_PER_STEP) {
            menuSel = (menuSel + 1) % I_COUNT;
            menuEncRef += ENC_PER_STEP;
            drawMenu();
            return;
        }
        if (delta <= -ENC_PER_STEP) {
            menuSel = (menuSel - 1 + I_COUNT) % I_COUNT;
            menuEncRef -= ENC_PER_STEP;
            drawMenu();
            return;
        }
    }

    if (buttonEdge()) {
        dispatch(menuSel);
        menuEncRef = rightEnc.getTicks();
        drawMenu();
    }
}
