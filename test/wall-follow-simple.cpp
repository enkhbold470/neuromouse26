// test/wall-follow-simple.cpp — wall-follow PID test w/ on-board calibration
//
// Menu (right encoder = scroll, BUTTON_1 = select):
//   Calibrate    — capture L/R sensor readings at centered position
//   Run          — drive forward, wall-follow PID
//   Live IR      — 4-bar graph
//
// Sign convention:
//   err = (irR - calR) - (irL - calL)
//   drifted right → irR > calR → err positive → corr positive
//   pwmL = BASE - corr (slower L) ; pwmR = BASE + corr (faster R)
//   → robot turns LEFT, away from right wall.

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── OLED ─────────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ── Hardware ─────────────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);

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

constexpr int WALL_SIDE_PRESENT = 400;   // sensor must see at least this much to count as "wall here"
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

// ── Calibration (persists for session) ───────────────────────────────────────
static int calL = 800;  // sensible defaults, overwritten by Calibrate
static int calR = 800;

// ── PID ──────────────────────────────────────────────────────────────────────
constexpr float CENTER_KP = 0.12f;
constexpr float CENTER_KI = 0.0f;
constexpr float CENTER_KD = 0.03f;
constexpr int   MAX_CORR  = 250;
constexpr int   BASE_PWM_WF = DRIVE_PWM;

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

void stopMotors() { leftMotor.brake(); rightMotor.brake(); }

// Mechanical keyswitch, no debounce cap. BUTTON_HOLD_MS in PinConfig.h.
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
enum State { IDLE, CAL, RUN, LIVE };
State state = IDLE;

enum MenuItem { M_CAL = 0, M_RUN, M_LIVE, M_COUNT };
static const char* MENU_LABELS[M_COUNT] = { "Calibrate", "Run", "Live IR" };
static int  menuSel    = M_RUN;
static long menuEncRef = 0;
constexpr long ENC_PER_STEP = 80;

// ── OLED screens ─────────────────────────────────────────────────────────────
void oledMenu() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "Wall-follow");
    oled.drawHLine(0, 12, 128);
    const int Y0 = 26, LH = 12;
    for (int i = 0; i < M_COUNT; i++) {
        int y = Y0 + i * LH;
        if (i == menuSel) {
            oled.drawBox(0, y - 10, 128, LH);
            oled.setDrawColor(0);
            oled.drawStr(4, y, MENU_LABELS[i]);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(4, y, MENU_LABELS[i]);
        }
    }
    oled.setFont(u8g2_font_5x7_tf);
    char buf[28];
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

void oledRun(int err, int corr, int pwmL, int pwmR) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "RUN  btn=stop");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_6x10_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "L%4d R%4d", irVal[1], irVal[2]);
    oled.drawStr(0, 24, buf);
    snprintf(buf, sizeof(buf), "LF%4d RF%4d", irVal[0], irVal[3]);
    oled.drawStr(0, 36, buf);
    snprintf(buf, sizeof(buf), "err%+d c%+d", err, corr);
    oled.drawStr(0, 48, buf);
    snprintf(buf, sizeof(buf), "pwm L%d R%d", pwmL, pwmR);
    oled.drawStr(0, 60, buf);
    oled.sendBuffer();
}

void oledCountdown(int n) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "Wall-follow RUN");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_logisoso42_tn);  // big digit font
    char buf[4]; snprintf(buf, sizeof(buf), "%d", n);
    int w = oled.getStrWidth(buf);
    oled.drawStr((128 - w) / 2, 60, buf);
    oled.sendBuffer();
}

void oledBars() {
    static const uint8_t order[4] = { 1, 0, 3, 2 };  // L LF RF R
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
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
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
                    case M_CAL:  sampleIR(); oledCal();  state = CAL;  break;
                    case M_RUN:  for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(1000); }
                                 pid.reset();
                                 leftEnc.reset(); rightEnc.reset();
                                 state = RUN;  break;
                    case M_LIVE: sampleIR(); oledBars(); state = LIVE; break;
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
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                state = IDLE;
                break;
            }

            sampleIR();
            bool wL = irVal[1] > WALL_SIDE_PRESENT;
            bool wR = irVal[2] > WALL_SIDE_PRESENT;
            bool wF = irVal[0] > WALL_FRONT_STOP || irVal[3] > WALL_FRONT_STOP;

            if (wF) {
                stopMotors();
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                state = IDLE;
                break;
            }

            // ── Sign-correct centering ─────────────────────────────────────
            // err > 0 when robot is closer to RIGHT wall (irR > calR).
            // Positive corr → slow L wheel, boost R wheel → turn LEFT (away from R wall).
            int errR = wR ? (irVal[2] - calR) : 0;
            int errL = wL ? (irVal[1] - calL) : 0;
            int err  = errR - errL;
            float corr = (wL || wR) ? pid.compute((float)err) : 0.0f;

            int pwmL, pwmR;
            if (wL || wR) {
                pwmL = constrain(BASE_PWM_WF - (int)corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
                pwmR = constrain(BASE_PWM_WF + (int)corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
            } else {
                // No walls — encoder balance.
                long tL = leftEnc.getTicks();
                long tR = rTicks();
                int  encErr = (int)(tL - tR);
                pwmL = constrain(BASE_PWM_WF - (int)(encErr * BALANCE_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
                pwmR = constrain(BASE_PWM_WF + (int)(encErr * BALANCE_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
            }

            leftMotor.drive(pwmL);
            rightMotor.drive(pwmR);

            static uint32_t lastOled = 0;
            if (millis() - lastOled > 150) {
                oledRun(err, (int)corr, pwmL, pwmR);
                lastOled = millis();
            }
            break;
        }
    }
}
