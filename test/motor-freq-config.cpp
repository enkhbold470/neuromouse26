// test/motor-freq-config.cpp
//
// Motor PWM frequency × speed sweep test.
// OLED menu: scroll right-encoder to change focused field, button click cycles
// focus FREQ → PWM → RUN → FREQ. Click on RUN drives both motors forward at the
// selected PWM and frequency for RUN_MS, then coasts and reports tick counts +
// mm/s + RPM so you can compare torque/efficiency tradeoffs.
//
// Build: pio run -e motor-freq -t upload

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── Sweep options ─────────────────────────────────────────────────────────────
static const uint32_t FREQS[] = {
    200, 500, 1000, 2000, 5000, 10000, 20000, 30000
};
static const int FREQ_COUNT = sizeof(FREQS) / sizeof(FREQS[0]);

static const int PWMS[] = {
    150, 250, 400, 600, 800, 1000
};
static const int PWM_COUNT = sizeof(PWMS) / sizeof(PWMS[0]);

constexpr unsigned long RUN_MS = 2000;        // run duration per test
constexpr long          ENC_PER_STEP = 80;    // scroll sensitivity

// LEDC channels used by motor objects (must match constructors below).
constexpr uint8_t LCH1 = 0, LCH2 = 1, RCH1 = 2, RCH2 = 3;

// ── Hardware ──────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, LCH1, LCH2, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, RCH1, RCH2, MOTOR_R_INV);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ── UI state ──────────────────────────────────────────────────────────────────
enum Focus { F_FREQ = 0, F_PWM, F_RUN, F_COUNT };
static Focus focus     = F_FREQ;
static int   freqIdx   = 1;    // default 500 Hz
static int   pwmIdx    = 1;    // default 250
static long  menuEncRef = 0;
static char  statusLine[28] = "ready";

static void setStatus(const char* fmt, ...) {
    va_list args; va_start(args, fmt);
    vsnprintf(statusLine, sizeof(statusLine), fmt, args);
    va_end(args);
}

// ── Button (same edge logic as motor-ble-drive.cpp) ───────────────────────────
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

// ── Frequency reconfigure ─────────────────────────────────────────────────────
// Re-runs ledcSetup on all 4 motor channels. Channel re-attach not needed —
// pin↔channel binding survives a setup() call that only changes freq/bits.
static void applyFreq(uint32_t hz) {
    ledcSetup(LCH1, hz, MOTOR_PWM_BITS);
    ledcSetup(LCH2, hz, MOTOR_PWM_BITS);
    ledcSetup(RCH1, hz, MOTOR_PWM_BITS);
    ledcSetup(RCH2, hz, MOTOR_PWM_BITS);
}

// ── Test run ──────────────────────────────────────────────────────────────────
static void runTest() {
    uint32_t hz  = FREQS[freqIdx];
    int      pwm = PWMS[pwmIdx];

    applyFreq(hz);
    leftEnc.reset();
    rightEnc.reset();

    setStatus("RUN %luHz pwm%d", (unsigned long)hz, pwm);
    leftMotor.drive(pwm);
    rightMotor.drive(pwm);

    unsigned long t0 = millis();
    while (millis() - t0 < RUN_MS) {
        // tight wait; nothing to do — encoders count via ISR.
    }

    leftMotor.coast();
    rightMotor.coast();

    long tL = leftEnc.getTicks();
    long tR = rTicks();
    long avg = (tL + tR) / 2;

    // mm/s = (ticks / RUN_s) * MM_PER_TICK
    float secs = RUN_MS / 1000.0f;
    float mmps = (avg / secs) * MM_PER_TICK;
    // output-shaft RPM = (ticks/sec) / TICKS_PER_REV * 60
    float rpm  = (avg / secs) / TICKS_PER_REV * 60.0f;

    Serial.printf("[FREQ %lu Hz][PWM %d] L=%ld R=%ld avg=%ld | %.0f mm/s | %.0f RPM\n",
                  (unsigned long)hz, pwm, tL, tR, avg, mmps, rpm);

    setStatus("L%ld R%ld %.0fmm/s", tL, tR, mmps);
}

// ── OLED render ───────────────────────────────────────────────────────────────
static void drawRow(int y, const char* label, const char* val, bool selected) {
    if (selected) {
        oled.drawBox(0, y, 128, 12);
        oled.setDrawColor(0);
    }
    oled.drawStr(3, y + 9, label);
    oled.drawStr(60, y + 9, val);
    if (selected) oled.setDrawColor(1);
}

static void oledMenu() {
    char fbuf[16], pbuf[16];
    snprintf(fbuf, sizeof(fbuf), "%lu Hz", (unsigned long)FREQS[freqIdx]);
    snprintf(pbuf, sizeof(pbuf), "%d", PWMS[pwmIdx]);

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 9, "Motor freq sweep");
    oled.drawHLine(0, 11, 128);

    drawRow(14, "FREQ", fbuf, focus == F_FREQ);
    drawRow(27, "PWM",  pbuf, focus == F_PWM);
    drawRow(40, "->",   focus == F_RUN ? "RUN!" : "run", focus == F_RUN);

    oled.drawHLine(0, 54, 128);
    oled.drawStr(0, 63, statusLine);
    oled.sendBuffer();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(50);
    pinMode(BUTTON_1, INPUT_PULLUP);

    leftMotor.begin();
    rightMotor.begin();
    leftEnc.begin();
    rightEnc.begin();

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    applyFreq(FREQS[freqIdx]);
    menuEncRef = rightEnc.getTicks();
    setStatus("ready");
    oledMenu();
    Serial.println("[INIT] motor-freq-config — scroll=change, click=focus/run");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    long delta = rightEnc.getTicks() - menuEncRef;

    auto bumpFocused = [](int dir) {
        switch (focus) {
            case F_FREQ:
                freqIdx = (freqIdx + dir + FREQ_COUNT) % FREQ_COUNT;
                applyFreq(FREQS[freqIdx]);
                break;
            case F_PWM:
                pwmIdx = (pwmIdx + dir + PWM_COUNT) % PWM_COUNT;
                break;
            case F_RUN:
            default: break;
        }
    };

    if (delta >= ENC_PER_STEP) {
        bumpFocused(+1);
        menuEncRef += ENC_PER_STEP;
        oledMenu();
        return;
    }
    if (delta <= -ENC_PER_STEP) {
        bumpFocused(-1);
        menuEncRef -= ENC_PER_STEP;
        oledMenu();
        return;
    }

    if (buttonEdge()) {
        if (focus == F_RUN) {
            runTest();
        } else {
            focus = (Focus)((focus + 1) % F_COUNT);
        }
        menuEncRef = rightEnc.getTicks();
        oledMenu();
    }
}
