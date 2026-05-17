// test/velocity-pid-ble.cpp
//
// Velocity-PID test with live BLE tuning.
//
// All PID gains, biases, kV, offsets, target, and R_SCALE are runtime-
// tunable via NimBLE NUS commands from tools/ble-debug.html (or any NUS
// terminal). RUN streams real-time telemetry back over BLE.
//
// Local controls (OLED menu) still work — BLE is parallel to OLED, not a
// replacement.
//
// BLE command format: `<key> <value>\n` (newline optional, terminator
// whitespace also OK).
//
//   kp <f>     LOOP_KP
//   ki <f>     LOOP_KI
//   str <f>    STRAIGHT_KP
//   sm <int>   STRAIGHT_MAX
//   il <f>     INTEG_LIMIT
//   ema <f>    EMA_ALPHA   (0..1)
//   lb <int>   L_PWM_BIAS
//   rb <int>   R_PWM_BIAS
//   kvl <f>    kV_L
//   kvr <f>    kV_R
//   ofl <f>    off_L
//   ofr <f>    off_R
//   t <int>    target mm/s
//   rs <f>     R_SCALE      (encoder L/R equalization)
//   runms <int> RUN duration ms
//
// BLE action commands (no arg):
//   run        start RUN with current settings
//   tape       start TAPE test (click button to stop, or send `stop`)
//   char       PWM sweep characterization (writes kV/off)
//   stop       cancel current action
//   ?          dump current params over BLE
//
// Build:  pio run -e velocity-pid-ble -t upload
// Device name: neuromouse

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <NimBLEDevice.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── Fixed configuration ───────────────────────────────────────────────────────
constexpr uint8_t  LCH1 = 0, LCH2 = 1, RCH1 = 2, RCH2 = 3;
constexpr uint32_t PWM_FREQ_HZ   = 200;
constexpr int      PWM_BITS      = MOTOR_PWM_BITS;
constexpr int      PWM_MAX       = MOTOR_PWM_MAX;

constexpr unsigned long LOOP_US  = 5000;        // 200 Hz control loop
                                                // (NOMINAL_VBAT comes from PinConfig.h)

constexpr int      CHAR_PWM_LO    = 100;
constexpr int      CHAR_PWM_HI    = 1000;
constexpr int      CHAR_PWM_STEP  = 100;
constexpr unsigned long CHAR_HOLD_MS   = 1500;
constexpr unsigned long CHAR_SETTLE_MS = 700;
constexpr int      CHAR_FIT_PWM_MAX = 300;

constexpr int      TAPE_PWM      = 250;
constexpr int      TARG_MIN      = 200;
constexpr int      TARG_MAX      = 1500;
constexpr int      TARG_STEP     = 25;
constexpr long     ENC_PER_STEP  = 20;
constexpr unsigned long OLED_REDRAW_MS = 150;
constexpr unsigned long BLE_TELEM_MS   = 100;
constexpr unsigned long PRE_RUN_WAIT_MS = 1000;

// NUS UUIDs
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ── Runtime-tunable state (BLE/OLED can update) ──────────────────────────────
// Defaults locked in from auto-calibration on 2026-05-17:
//   sweep pwm 100/200/300 → kV_L=2.961, kV_R=2.749, R_SCALE=1.0135
//   RUN at T=300 with these values converged cleanly for first 3 s.
// Rerun the on-device Calibrate to refresh after a battery change, motor
// swap, or surface change.
static float    R_SCALE     = 1.0135f;

static float    kV_L        = 2.961f;
static float    kV_R        = 2.749f;
static float    off_L       = 0.0f;
static float    off_R       = 0.0f;
static bool     charDone    = false;

static float    LOOP_KP     = 1.00f;
static float    LOOP_KI     = 1.50f;
static float    INTEG_LIMIT = 800.0f;
static float    EMA_ALPHA   = 0.50f;

static float    STRAIGHT_KP  = 6.00f;
static int      STRAIGHT_MAX = 500;

// Runtime overrides — shadow the constexpr PinConfig L_PWM_BIAS / R_PWM_BIAS
// so BLE can tune them live. Same default as PinConfig.h.
static int      lPwmBias    = 0;
static int      rPwmBias    = 0;

static int                targetMmS = 300;
static unsigned long      RUN_MS    = 4000;

// ── Hardware ──────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, LCH1, LCH2, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, RCH1, RCH2, MOTOR_R_INV);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);

static inline long rTicks() { return (long)(rightEnc.getTicks() * R_SCALE); }

// ── BLE plumbing ──────────────────────────────────────────────────────────────
static NimBLEServer*         pServer       = nullptr;
static NimBLECharacteristic* pTX           = nullptr;
static volatile bool         bleConnected  = false;

// Inbox for BLE commands — set by RX callback, drained in loop.
static volatile bool         bleCmdReady   = false;
static char                  bleCmdBuf[96];

// Action requests — main loop polls and dispatches.
enum BleAction { ACT_NONE = 0, ACT_RUN, ACT_TAPE, ACT_CHAR, ACT_CAL, ACT_STOP, ACT_DUMP };
static volatile int          bleAction     = ACT_NONE;
static volatile bool         stopRequested = false;

// IR state: live ambient-subtracted readings + cal values (per-sensor anchor
// captured in dead-end). irStream toggles continuous telemetry from main loop.
struct IRPair { uint8_t emit, rx; };
static IRPair PAIRS[4] = {
    { EMIT_LF, RX_LF },
    { EMIT_L,  RX_L  },
    { EMIT_R,  RX_R  },
    { EMIT_RF, RX_RF },
};
static int    irVal[4]  = { 0, 0, 0, 0 };
static int    calLF     = IR_CAL_LF;
static int    calL      = IR_CAL_L;
static int    calR      = IR_CAL_R;
static int    calRF     = IR_CAL_RF;
static volatile bool irStream  = false;
static volatile bool irCaptureRequested = false;

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

static void blePrint(const char* str) {
    if (!bleConnected || !pTX) return;
    size_t len = strlen(str);
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = min((size_t)20, len - offset);
        pTX->setValue((uint8_t*)(str + offset), chunk);
        pTX->notify();
        offset += chunk;
        delay(5);
    }
}

static void blePrintf(const char* fmt, ...) {
    char buf[160];
    va_list args; va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    blePrint(buf);
}

// ── Status line ──────────────────────────────────────────────────────────────
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

// ── Button ────────────────────────────────────────────────────────────────────
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

// ── Linear fit (CHAR linear region only) ──────────────────────────────────────
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

// ── OLED screens ─────────────────────────────────────────────────────────────
static void drawCharScreen(int pwm, float vL, float vR, float vbat, int rowIdx, int total) {
    char l1[28], l2[28], l3[28], l4[28], l5[28];
    snprintf(l1, sizeof(l1), "CHAR %d/%d %s", rowIdx, total, bleConnected ? "BLE" : "");
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
    snprintf(l1, sizeof(l1), "RUN T%.0f %lums %s",
             target, elapsedMs, bleConnected ? "BLE" : "");
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
    snprintf(l1, sizeof(l1), "TAPE %s", bleConnected ? "BLE" : "click=stop");
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

// ── Pre-run countdown (skipped if BLE-initiated) ─────────────────────────────
static void preRunCountdown(const char* label, bool fromBle) {
    while (!fromBle && digitalRead(BUTTON_1) == LOW) {}
    if (PRE_RUN_WAIT_MS == 0 || fromBle) return;
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

// ── Auto-calibration ─────────────────────────────────────────────────────────
// One-button routine: sweep PWM 100/200/300 (each 1.5 s = 4.5 s total),
// accumulate per-wheel ticks across the whole sweep. Outputs:
//   - kV_L, off_L, kV_R, off_R   (from linear fit on the 3 points)
//   - R_SCALE adjusted to equalize total L vs R travel
//
// Live OLED + BLE telemetry during sweep so you can watch convergence.
// Runs ~5 s, then returns to menu. No second button press needed.
static void doAutoCalibrate(bool fromBle) {
    preRunCountdown("CAL", fromBle);
    setStatus("CAL running");
    blePrintf("cal start\n");
    stopRequested = false;

    leftEnc.reset();
    rightEnc.reset();

    constexpr int CAL_STEPS[]    = { 100, 200, 300 };
    constexpr int N              = sizeof(CAL_STEPS) / sizeof(CAL_STEPS[0]);
    constexpr unsigned long PER  = 1500;
    constexpr unsigned long SETT = 500;

    int   pwms[N];
    float vL[N], vR[N];
    long  totRawTL = 0, totRawTR = 0;             // raw L ticks and raw R ticks
                                                  // (R BEFORE applying R_SCALE)

    for (int i = 0; i < N; i++) {
        if (stopRequested) break;
        int pwm = CAL_STEPS[i];
        leftMotor.drive(pwm);
        rightMotor.drive(pwm);

        delay(SETT);
        long lStart = leftEnc.getTicks();
        long rStart = rightEnc.getTicks();         // raw, no scale
        unsigned long t0 = millis();
        unsigned long sampleMs = PER - SETT;

        // live update during sample window
        unsigned long lastDraw = 0;
        while (millis() - t0 < sampleMs) {
            unsigned long now = millis();
            if (now - lastDraw >= OLED_REDRAW_MS) {
                lastDraw = now;
                long curL  = leftEnc.getTicks();
                long curRr = rightEnc.getTicks();   // raw R
                long dL    = curL - lStart;
                long dRr   = curRr - rStart;
                float ratio = (dRr != 0) ? (float)dL / (float)dRr : 0.0f;
                char l1[28], l2[28], l3[28], l4[28], l5[28];
                snprintf(l1, sizeof(l1), "CAL %d/%d  pwm %d", i+1, N, pwm);
                snprintf(l2, sizeof(l2), "dL  %ld", dL);
                snprintf(l3, sizeof(l3), "dR  %ld (raw)", dRr);
                snprintf(l4, sizeof(l4), "L/Rraw %.3f", ratio);
                snprintf(l5, sizeof(l5), "rs cur %.3f", R_SCALE);
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
        }

        long lEnd  = leftEnc.getTicks();
        long rEnd  = rightEnc.getTicks();          // raw
        float secs = (millis() - t0) / 1000.0f;
        long dL    = lEnd - lStart;
        long dRr   = rEnd - rStart;
        // scaled R for velocity measurement (consistent with runtime)
        float dR_scaled = dRr * R_SCALE;

        pwms[i] = pwm;
        vL[i]   = (dL        * MM_PER_TICK) / secs;
        vR[i]   = (dR_scaled * MM_PER_TICK) / secs;

        totRawTL += dL;
        totRawTR += dRr;

        blePrintf("cal pwm=%d vL=%.0f vR=%.0f dL=%ld dRraw=%ld\n",
                  pwm, vL[i], vR[i], dL, dRr);
    }
    leftMotor.coast();
    rightMotor.coast();

    // Fit kV / offset on the 3 linear-region points.
    bool okL = fitLine(pwms, vL, N, kV_L, off_L);
    bool okR = fitLine(pwms, vR, N, kV_R, off_R);
    charDone = okL && okR;

    // Recompute R_SCALE: target = totRawTL / totRawTR (raw ratio).
    // This makes rTicks() == leftEnc.getTicks() for equal travel next time.
    if (totRawTR > 0) {
        R_SCALE = (float)totRawTL / (float)totRawTR;
    }

    // Final result screen (held briefly so user can read).
    char l1[28], l2[28], l3[28], l4[28], l5[28];
    snprintf(l1, sizeof(l1), "CAL done");
    snprintf(l2, sizeof(l2), "L kV %.2f of %.0f", kV_L, off_L);
    snprintf(l3, sizeof(l3), "R kV %.2f of %.0f", kV_R, off_R);
    snprintf(l4, sizeof(l4), "rs new %.4f", R_SCALE);
    snprintf(l5, sizeof(l5), "ticks L%ld R%ld", totRawTL, totRawTR);
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0,  9, l1);
    oled.drawHLine(0, 11, 128);
    oled.drawStr(0, 23, l2);
    oled.drawStr(0, 34, l3);
    oled.drawStr(0, 45, l4);
    oled.drawStr(0, 56, l5);
    oled.sendBuffer();
    delay(2500);

    setStatus("kV %.1f/%.1f", kV_L, kV_R);
    blePrintf("cal done kvl=%.3f kvr=%.3f ofl=%.1f ofr=%.1f rs=%.4f okfit=%d\n",
              kV_L, kV_R, off_L, off_R, R_SCALE, charDone);
}

// ── CHAR (full sweep, kept for advanced use via BLE 'char' cmd) ──────────────
static void doCharacterize(bool fromBle) {
    setStatus("CHAR running");
    blePrintf("char start\n");
    const int N = (CHAR_PWM_HI - CHAR_PWM_LO) / CHAR_PWM_STEP + 1;
    int   pwms[16];
    float vL[16], vR[16];
    int   idx = 0;
    stopRequested = false;

    for (int pwm = CHAR_PWM_LO; pwm <= CHAR_PWM_HI; pwm += CHAR_PWM_STEP) {
        if (stopRequested) break;
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
        blePrintf("char pwm=%d vL=%.0f vR=%.0f\n", pwm, velL, velR);
    }
    leftMotor.coast();
    rightMotor.coast();

    bool okL = fitLine(pwms, vL, idx, kV_L, off_L);
    bool okR = fitLine(pwms, vR, idx, kV_R, off_R);
    charDone = okL && okR;

    if (charDone) {
        setStatus("kV L%.2f R%.2f", kV_L, kV_R);
        blePrintf("char done kvl=%.3f ofl=%.1f kvr=%.3f ofr=%.1f\n",
                  kV_L, off_L, kV_R, off_R);
    } else {
        setStatus("CHAR fit fail");
        blePrintf("char fit fail\n");
    }
}

// ── TAPE ──────────────────────────────────────────────────────────────────────
static void doTapeTest(bool fromBle) {
    preRunCountdown("TAPE", fromBle);
    leftEnc.reset();
    rightEnc.reset();
    setStatus("TAPE running");
    drawTapeScreen(0, 0, 0.0f, 0.0f);
    blePrintf("tape start\n");
    stopRequested = false;

    leftMotor.drive(TAPE_PWM);
    rightMotor.drive(TAPE_PWM);

    delay(50);
    unsigned long lastDraw = 0;
    unsigned long lastBle  = 0;
    while (true) {
        if (buttonEdge() || stopRequested) break;
        unsigned long now = millis();
        long tL = leftEnc.getTicks();
        long tR = rTicks();

        if (now - lastDraw >= OLED_REDRAW_MS) {
            lastDraw = now;
            drawTapeScreen(tL, tR, tL * MM_PER_TICK, tR * MM_PER_TICK);
        }
        if (now - lastBle >= BLE_TELEM_MS) {
            lastBle = now;
            blePrintf("tape tL=%ld tR=%ld\n", tL, tR);
        }
    }

    leftMotor.coast();
    rightMotor.coast();

    long tL = leftEnc.getTicks();
    long tR = rTicks();
    float mmL = tL * MM_PER_TICK;
    float mmR = tR * MM_PER_TICK;
    float ratio = tR > 0 ? (float)tL / (float)tR : 0.0f;
    drawTapeScreen(tL, tR, mmL, mmR);
    setStatus("L%.0f R%.0f mm", mmL, mmR);
    blePrintf("tape done tL=%ld tR=%ld mmL=%.0f mmR=%.0f ratio=%.4f rs=%.4f\n",
              tL, tR, mmL, mmR, ratio, R_SCALE);
}

// ── RUN (cascaded speed + straightness loop) ─────────────────────────────────
static int feedforwardPwm(float target, float kV, float offset, float vScale) {
    if (target <= 0.0f || kV <= 0.0f) return 0;
    float pwm = (target / kV) + offset;
    pwm *= vScale;
    return (int)constrain(pwm, 0.0f, (float)PWM_MAX);
}

static void doRunPid(bool fromBle) {
    preRunCountdown("RUN", fromBle);
    leftEnc.reset();
    rightEnc.reset();
    long prevL = 0, prevR = 0;

    float velL_ema = 0.0f, velR_ema = 0.0f;
    float intSpeed = 0.0f;

    float vbat = readVbat();
    if (vbat < 5.5f) vbat = NOMINAL_VBAT;
    float vScale = NOMINAL_VBAT / vbat;

    unsigned long startMs  = millis();
    unsigned long nextUs   = micros();
    unsigned long lastDraw = startMs;
    unsigned long lastBle  = startMs;
    int lastPwmL = 0, lastPwmR = 0;
    stopRequested = false;

    setStatus("RUN %d mm/s", targetMmS);
    drawRunScreen((float)targetMmS, 0, 0, 0, 0, vbat, 0);
    blePrintf("run start T=%d ms=%lu kp=%.2f ki=%.2f str=%.2f lb=%d rb=%d kvl=%.2f kvr=%.2f\n",
              targetMmS, RUN_MS, LOOP_KP, LOOP_KI, STRAIGHT_KP,
              lPwmBias, rPwmBias, kV_L, kV_R);

    while (millis() - startMs < RUN_MS && !stopRequested) {
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
        float velAvg = 0.5f * (velL_ema + velR_ema);
        float errSpeed = target - velAvg;
        intSpeed = constrain(intSpeed + errSpeed * dt, -INTEG_LIMIT, INTEG_LIMIT);
        int   pidSpeed = (int)(LOOP_KP * errSpeed + LOOP_KI * intSpeed);

        long  straightErr = curL - curR;
        int   pidStraight = (int)constrain((float)(STRAIGHT_KP * straightErr),
                                           (float)-STRAIGHT_MAX, (float)STRAIGHT_MAX);

        float kV_avg  = 0.5f * (kV_L + kV_R);
        float off_avg = 0.5f * (off_L + off_R);
        int   ffBase  = feedforwardPwm(target, kV_avg, off_avg, vScale);

        int basePwm = ffBase + pidSpeed;
        int pwmL = constrain(basePwm - pidStraight + lPwmBias, 0, PWM_MAX);
        int pwmR = constrain(basePwm + pidStraight + rPwmBias, 0, PWM_MAX);

        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);
        lastPwmL = pwmL; lastPwmR = pwmR;

        unsigned long nowMs = millis();
        if (nowMs - lastDraw >= OLED_REDRAW_MS) {
            lastDraw = nowMs;
            drawRunScreen(target, velL_ema, velR_ema, pwmL, pwmR,
                          vbat, nowMs - startMs);
        }
        if (nowMs - lastBle >= BLE_TELEM_MS) {
            lastBle = nowMs;
            float vbatNow = readVbat();
            blePrintf("rt t=%lu vL=%.0f vR=%.0f pL=%d pR=%d sE=%ld vb=%.2f\n",
                      nowMs - startMs, velL_ema, velR_ema,
                      pwmL, pwmR, straightErr, vbatNow);
        }
    }

    leftMotor.coast();
    rightMotor.coast();
    drawRunScreen((float)targetMmS, velL_ema, velR_ema, lastPwmL, lastPwmR,
                  vbat, RUN_MS);
    setStatus("done %.0f/%.0f", velL_ema, velR_ema);
    blePrintf("run done vL=%.0f vR=%.0f pL=%d pR=%d\n",
              velL_ema, velR_ema, lastPwmL, lastPwmR);
}

// ── BLE command parser ────────────────────────────────────────────────────────
static void dumpParams() {
    blePrintf("p kp=%.2f ki=%.2f il=%.0f ema=%.2f\n", LOOP_KP, LOOP_KI, INTEG_LIMIT, EMA_ALPHA);
    blePrintf("p str=%.2f sm=%d\n", STRAIGHT_KP, STRAIGHT_MAX);
    blePrintf("p lb=%d rb=%d\n", lPwmBias, rPwmBias);
    blePrintf("p kvl=%.3f kvr=%.3f ofl=%.1f ofr=%.1f\n", kV_L, kV_R, off_L, off_R);
    blePrintf("p t=%d rs=%.4f runms=%lu\n", targetMmS, R_SCALE, RUN_MS);
    blePrintf("p clf=%d cl=%d cr=%d crf=%d\n", calLF, calL, calR, calRF);
}

static void handleBleCommand(const char* cmd) {
    // tokenize: key [value]
    char key[16] = {0};
    char val[48] = {0};
    int n = sscanf(cmd, "%15s %47s", key, val);
    if (n < 1 || key[0] == 0) return;

    // action keywords
    if      (!strcmp(key, "run"))      { bleAction = ACT_RUN;  return; }
    else if (!strcmp(key, "cal"))      { bleAction = ACT_CAL;  return; }
    else if (!strcmp(key, "tape"))     { bleAction = ACT_TAPE; return; }
    else if (!strcmp(key, "char"))     { bleAction = ACT_CHAR; return; }
    else if (!strcmp(key, "stop"))     { stopRequested = true; irStream = false; return; }
    else if (!strcmp(key, "?"))        { dumpParams(); return; }
    else if (!strcmp(key, "irstream")) { irStream = true;  blePrintf("ir stream on\n");  return; }
    else if (!strcmp(key, "irstop"))   { irStream = false; blePrintf("ir stream off\n"); return; }
    else if (!strcmp(key, "ircal"))    { irCaptureRequested = true; return; }

    if (n < 2) { blePrintf("err: need value for %s\n", key); return; }
    float fv = atof(val);
    int   iv = atoi(val);

    if      (!strcmp(key, "kp"))    LOOP_KP      = fv;
    else if (!strcmp(key, "ki"))    LOOP_KI      = fv;
    else if (!strcmp(key, "il"))    INTEG_LIMIT  = fv;
    else if (!strcmp(key, "ema"))   EMA_ALPHA    = constrain(fv, 0.01f, 1.0f);
    else if (!strcmp(key, "str"))   STRAIGHT_KP  = fv;
    else if (!strcmp(key, "sm"))    STRAIGHT_MAX = iv;
    else if (!strcmp(key, "lb"))    lPwmBias     = iv;
    else if (!strcmp(key, "rb"))    rPwmBias     = iv;
    else if (!strcmp(key, "kvl"))   kV_L         = fv;
    else if (!strcmp(key, "kvr"))   kV_R         = fv;
    else if (!strcmp(key, "ofl"))   off_L        = fv;
    else if (!strcmp(key, "ofr"))   off_R        = fv;
    else if (!strcmp(key, "t"))     targetMmS    = constrain(iv, TARG_MIN, TARG_MAX);
    else if (!strcmp(key, "rs"))    R_SCALE      = fv;
    else if (!strcmp(key, "runms")) RUN_MS       = (unsigned long)iv;
    else if (!strcmp(key, "clf"))   calLF        = iv;
    else if (!strcmp(key, "cl"))    calL         = iv;
    else if (!strcmp(key, "cr"))    calR         = iv;
    else if (!strcmp(key, "crf"))   calRF        = iv;
    else { blePrintf("err: unknown %s\n", key); return; }

    blePrintf("ok %s=%s\n", key, val);
}

// ── BLE callbacks ─────────────────────────────────────────────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& connInfo) override {
        bleConnected = true;
        blePrintf("hello neuromouse\n");
    }
    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) override {
        bleConnected = false;
        NimBLEDevice::startAdvertising();
    }
};

class RXCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        std::string val = c->getValue();
        if (val.empty()) return;
        size_t n = min(val.size(), sizeof(bleCmdBuf) - 1);
        memcpy((void*)bleCmdBuf, val.data(), n);
        ((char*)bleCmdBuf)[n] = 0;
        bleCmdReady = true;
    }
};

// ── OLED menu (one-hand: two items only) ─────────────────────────────────────
enum Item { I_CAL = 0, I_RUN, I_COUNT };
static const char* MENU[I_COUNT] = {
    "Calibrate", "Run PID"
};
static int  menuSel = 0;
static long menuEncRef = 0;

static void drawMenu() {
    char hdr[28];
    snprintf(hdr, sizeof(hdr), "%.2fV T%d %s", readVbat(), targetMmS,
             bleConnected ? "BLE" : "");
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
            oled.setDrawColor(1);
        } else {
            oled.drawStr(3, y + 8, MENU[idx]);
        }
    }
    // Bottom: live params line so user has feedback w/o extra menu items.
    char ln[28];
    snprintf(ln, sizeof(ln), "kV%.1f/%.1f rs%.3f", kV_L, kV_R, R_SCALE);
    oled.drawStr(0, 53, ln);
    oled.drawHLine(0, 54, 128);
    oled.drawStr(0, 63, statusLine);
    oled.sendBuffer();
}

static void dispatchLocal(int sel) {
    switch (sel) {
        case I_CAL: doAutoCalibrate(false); break;
        case I_RUN: doRunPid(false);        break;
    }
}

// ── BLE init ──────────────────────────────────────────────────────────────────
#define BLE_DEVICE_NAME "neuromouse"

static void bleInit() {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(NUS_SERVICE_UUID);
    pTX = pService->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic* pRX = pService->createCharacteristic(
        NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pRX->setCallbacks(new RXCallbacks());

    pService->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->setName(BLE_DEVICE_NAME);
    pAdv->addServiceUUID(NUS_SERVICE_UUID);
    pAdv->enableScanResponse(true);
    pAdv->start();
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

    // IR emitters/receivers — same pattern as main.cpp.
    for (auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT);
        digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    bleInit();

    menuEncRef = rightEnc.getTicks();
    setStatus("ready BLE");
    drawMenu();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // 1) drain BLE inbox
    if (bleCmdReady) {
        bleCmdReady = false;
        handleBleCommand((const char*)bleCmdBuf);
    }

    // 1b) IR stream — continuously sample + push when armed.
    static unsigned long lastIrBle = 0;
    if (irStream) {
        unsigned long now = millis();
        if (now - lastIrBle >= 100) {
            lastIrBle = now;
            sampleIR();
            blePrintf("ir LF=%d L=%d R=%d RF=%d\n",
                      irVal[0], irVal[1], irVal[2], irVal[3]);
        }
    }
    // 1c) IR cal capture: take a fresh sample, latch as new cal values, echo.
    if (irCaptureRequested) {
        irCaptureRequested = false;
        sampleIR();
        calLF = irVal[0]; calL = irVal[1]; calR = irVal[2]; calRF = irVal[3];
        blePrintf("ircal done clf=%d cl=%d cr=%d crf=%d\n",
                  calLF, calL, calR, calRF);
        setStatus("IR cal %d/%d/%d/%d", calLF, calL, calR, calRF);
        drawMenu();
    }

    // 2) BLE-triggered actions (run from main loop so they own motors safely)
    int act = bleAction;
    if (act != ACT_NONE) {
        bleAction = ACT_NONE;
        switch (act) {
            case ACT_RUN:  doRunPid(true);        break;
            case ACT_CAL:  doAutoCalibrate(true); break;
            case ACT_TAPE: doTapeTest(true);      break;
            case ACT_CHAR: doCharacterize(true);  break;
            default: break;
        }
        drawMenu();
        return;
    }

    // 3) OLED menu via right encoder wheel
    long delta = rightEnc.getTicks() - menuEncRef;
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

    if (buttonEdge()) {
        dispatchLocal(menuSel);
        menuEncRef = rightEnc.getTicks();
        drawMenu();
    }
}
