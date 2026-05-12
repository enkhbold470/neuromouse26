// test/side-open-turn.cpp — opening-detect + gyro turn demo
//
// Robot drives forward at DRIVE_PWM with IR centering. Tracks side sensors.
// When L or R drops from wall→open, robot remembers which side. Continues
// driving until ONE cell of forward travel completes (TICKS_PER_CELL).
// At cell boundary, resamples IR:
//   - if side still open AND front wall present (LF or RF > FRONT) → gyro
//     turn 90° toward that side, then drives one more cell forward
//   - else: stop, show status
//
// Menu:
//   Cal IR     — dead-end capture
//   Cal Gyro   — bias capture
//   Run        — start demo (3,2,1 countdown)
//   Live IR    — bar graph
//
// Wiring: MPU shared I2C with OLED (SDA=OLED_SDA, SCL=OLED_SCL, addr 0x68).

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── OLED + hardware ──────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ── Thresholds ───────────────────────────────────────────────────────────────
constexpr int WALL_SIDE_THRESH  = 1000;
constexpr int WALL_FRONT_THRESH = 1500;

// ── Turn overshoot ───────────────────────────────────────────────────────────
constexpr float TURN_OVERSHOOT_90 = 10.0f;

// ── IR ───────────────────────────────────────────────────────────────────────
struct IRPair { uint8_t emit, rx; };
static IRPair PAIRS[4] = {
    { EMIT_LF, RX_LF },
    { EMIT_L,  RX_L  },
    { EMIT_R,  RX_R  },
    { EMIT_RF, RX_RF },
};
static int irVal[4] = {0,0,0,0};
static int calL = 2000, calR = 2000, calLF = 2000, calRF = 2000;

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
static inline bool wallL()      { return irVal[1] > WALL_SIDE_THRESH; }
static inline bool wallR()      { return irVal[2] > WALL_SIDE_THRESH; }
static inline bool wallFront()  { return irVal[0] > WALL_FRONT_THRESH || irVal[3] > WALL_FRONT_THRESH; }

// ── MPU-6500 ─────────────────────────────────────────────────────────────────
#define MPU_ADDR          0x68
#define REG_WHO_AM_I      0x75
#define REG_PWR_MGMT_1    0x6B
#define REG_GYRO_CFG      0x1B
#define REG_ACCEL_CFG     0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define GYRO_SCALE        131.0f
struct ImuRaw { int16_t ax, ay, az, temp, gx, gy, gz; };
static float gyroBiasZ = 0, yaw = 0;
static unsigned long lastImuUs = 0;

static bool mpuWrite(uint8_t r, uint8_t v) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(r); Wire.write(v);
    return Wire.endTransmission() == 0;
}
static bool mpuRead(uint8_t r, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(r);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }
    return true;
}
static int16_t to16(uint8_t hi, uint8_t lo) { return (int16_t)((hi << 8) | lo); }
static bool imuAll(ImuRaw& d) {
    uint8_t b[14];
    if (!mpuRead(REG_ACCEL_XOUT_H, b, 14)) return false;
    d.ax=to16(b[0],b[1]); d.ay=to16(b[2],b[3]); d.az=to16(b[4],b[5]);
    d.temp=to16(b[6],b[7]);
    d.gx=to16(b[8],b[9]); d.gy=to16(b[10],b[11]); d.gz=to16(b[12],b[13]);
    return true;
}
static void updateYaw() {
    ImuRaw d;
    if (!imuAll(d)) return;
    unsigned long now = micros();
    float dt = (lastImuUs == 0) ? 0.001f : (now - lastImuUs) / 1e6f;
    if (dt > 0.05f) dt = 0.05f;
    lastImuUs = now;
    float gz = d.gz / GYRO_SCALE - gyroBiasZ;
    if (fabsf(gz) < 0.05f) gz = 0;
    yaw += gz * dt;
}

// ── Motion ───────────────────────────────────────────────────────────────────
static void stopMotors() { leftMotor.brake(); rightMotor.brake(); }
static void encodersReset() { leftEnc.reset(); rightEnc.reset(); }

// PID for centering during forward drive.
struct PID {
    float integral = 0, prevError = 0;
    unsigned long prevUs = 0;
    float compute(float err) {
        unsigned long now = micros();
        float dt = (prevUs == 0) ? 0.001f
                                 : constrain((now - prevUs) / 1e6f, 0.0001f, 0.05f);
        prevUs = now;
        integral += err * dt;
        integral = constrain(integral, -2000.0f, 2000.0f);
        float deriv = (err - prevError) / dt;
        prevError = err;
        float out = 0.12f * err + 0.03f * deriv;
        return constrain(out, -250.0f, 250.0f);
    }
    void reset() { integral = 0; prevError = 0; prevUs = 0; }
} pid;

void doTurn90(int dir) {
    yaw = 0; lastImuUs = micros();
    float target = dir * 90.0f;
    float stopAt = (target > 0) ? (target - TURN_OVERSHOOT_90)
                                : (target + TURN_OVERSHOOT_90);
    leftMotor.drive(-dir * TURN_PWM);
    rightMotor.drive( dir * TURN_PWM);
    unsigned long t0 = millis();
    while (true) {
        updateYaw();
        if (dir > 0 ? yaw >= stopAt : yaw <= stopAt) break;
        if (millis() - t0 > 3000) break;
    }
    stopMotors();
    unsigned long s = millis();
    while (millis() - s < 200) updateYaw();
}

// Drive one cell with centering. Returns side that opened mid-cell (0=none, -1=L, +1=R).
// Brake at TICKS_PER_CELL − COAST_COMP_TICKS.
int driveOneCellTrackOpening() {
    encodersReset();
    pid.reset();
    long target = (long)TICKS_PER_CELL - COAST_COMP_TICKS;
    sampleIR();
    bool startedWithL = wallL();
    bool startedWithR = wallR();
    int  openSide = 0;
    unsigned long t0 = millis();

    while (true) {
        long avg = (leftEnc.getTicks() + rTicks()) / 2;
        if (avg >= target) { stopMotors(); return openSide; }

        sampleIR();
        bool cur_wL = wallL();
        bool cur_wR = wallR();
        // Latch first opening seen this cell.
        if (openSide == 0 && avg > TICKS_PER_CELL / 4) {
            if (startedWithL && !cur_wL) openSide = -1;
            else if (startedWithR && !cur_wR) openSide = +1;
        }

        // Centering PID (only if a wall is currently present on either side).
        int errR = cur_wR ? (irVal[2] - calR) : 0;
        int errL = cur_wL ? (irVal[1] - calL) : 0;
        int err = errR - errL;
        float corr = (cur_wL || cur_wR) ? pid.compute((float)err) : 0;

        int pwmL = constrain(DRIVE_PWM - (int)corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        int pwmR = constrain(DRIVE_PWM + (int)corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        if (!cur_wL && !cur_wR) {
            int encErr = (int)(leftEnc.getTicks() - rTicks());
            pwmL = constrain(DRIVE_PWM - (int)(encErr * BALANCE_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
            pwmR = constrain(DRIVE_PWM + (int)(encErr * BALANCE_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        }
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        if (millis() - t0 > (unsigned long)TIMEOUT_MS) { stopMotors(); return openSide; }
    }
}

// ── Button ───────────────────────────────────────────────────────────────────
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

// ── OLED ─────────────────────────────────────────────────────────────────────
static char statusLine[24] = "ready";

void oledMsg(const char* title, const char* l1, const char* l2 = nullptr) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, title);
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    if (l1) oled.drawStr(0, 32, l1);
    if (l2) oled.drawStr(0, 50, l2);
    oled.sendBuffer();
}

// ── Menu ─────────────────────────────────────────────────────────────────────
enum { M_CAL_IR = 0, M_CAL_GYRO, M_RUN, M_LIVE, M_COUNT };
static const char* MENU[M_COUNT] = { "Cal IR", "Cal Gyro", "Run", "Live IR" };
static int menuSel = M_RUN;
static long menuEncRef = 0;
constexpr long ENC_PER_STEP = 80;

void oledMenu() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "side-open");
    char hdr[20]; snprintf(hdr, sizeof(hdr), "%d/%d", menuSel + 1, M_COUNT);
    oled.drawStr(96, 8, hdr);
    oled.drawHLine(0, 10, 128);
    const int LH = 10;
    for (int i = 0; i < M_COUNT; i++) {
        int y = 12 + i * LH;
        if (i == menuSel) {
            oled.drawBox(0, y, 128, LH);
            oled.setDrawColor(0);
            oled.drawStr(3, y + 8, MENU[i]);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(3, y + 8, MENU[i]);
        }
    }
    oled.drawHLine(0, 64 - 10, 128);
    oled.setFont(u8g2_font_5x7_tf);
    char buf[32];
    snprintf(buf, sizeof(buf), "%s gz%+.2f", statusLine, gyroBiasZ);
    oled.drawStr(0, 63, buf);
    oled.sendBuffer();
}

void oledCal() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "CAL IR: dead-end");
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

void oledLive() {
    static const uint8_t order[4] = { 1, 0, 3, 2 };
    static const char*   lbl[4]   = { "L", "LF", "RF", "R" };
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

void oledCountdown(int n) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "Run in...");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_logisoso42_tn);
    char b[4]; snprintf(b, sizeof(b), "%d", n);
    int w = oled.getStrWidth(b);
    oled.drawStr((128 - w) / 2, 60, b);
    oled.sendBuffer();
}

// ── Cal funcs ────────────────────────────────────────────────────────────────
void doCalIR() {
    unsigned long t = millis();
    while (millis() - t < 1500) {   // 1.5s live preview
        sampleIR();
        oledCal();
        if (buttonEdge()) break;
        delay(50);
    }
    sampleIR();
    calLF = irVal[0]; calL = irVal[1]; calR = irVal[2]; calRF = irVal[3];
    snprintf(statusLine, sizeof(statusLine), "cal IR ok");
}

void doCalGyro() {
    constexpr int N = 300;
    float sum = 0; int good = 0;
    for (int i = 0; i < N; i++) {
        ImuRaw d;
        if (imuAll(d)) { sum += d.gz / GYRO_SCALE; good++; }
        if ((i & 0x3F) == 0) {
            char b[12]; snprintf(b, sizeof(b), "%d/%d", i, N);
            oledMsg("CAL GYRO", "STILL", b);
        }
        delay(2);
    }
    gyroBiasZ = good > 0 ? sum / good : 0;
    yaw = 0; lastImuUs = micros();
    snprintf(statusLine, sizeof(statusLine), "cal gz ok");
}

// ── Main run ─────────────────────────────────────────────────────────────────
void doRun() {
    for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(1000); }

    // Cell 1: drive forward, track opening
    int openSide = driveOneCellTrackOpening();
    delay(150);

    // Resample to confirm
    sampleIR();
    bool fWall = wallFront();
    bool stillOpenL = !wallL();
    bool stillOpenR = !wallR();

    if (openSide == -1 && fWall && stillOpenL) {
        snprintf(statusLine, sizeof(statusLine), "open L → L90");
        oledMenu();
        delay(300);
        doTurn90(+1);    // L = positive
        delay(150);
        driveOneCellTrackOpening();
    } else if (openSide == +1 && fWall && stillOpenR) {
        snprintf(statusLine, sizeof(statusLine), "open R → R90");
        oledMenu();
        delay(300);
        doTurn90(-1);    // R = negative
        delay(150);
        driveOneCellTrackOpening();
    } else {
        snprintf(statusLine, sizeof(statusLine), "open=%d fW%d sL%d sR%d",
                 openSide, fWall, stillOpenL, stillOpenR);
    }
    stopMotors();
}

// ── State ────────────────────────────────────────────────────────────────────
enum State { IDLE, CAL_IR_LIVE, LIVE_IR };
State state = IDLE;

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_1, INPUT_PULLUP);

    leftMotor.begin();
    rightMotor.begin();
    leftEnc.begin();
    rightEnc.begin();
    for (auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT); digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }
    analogReadResolution(12);

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    // MPU init
    uint8_t who = 0;
    mpuRead(REG_WHO_AM_I, &who, 1);
    mpuWrite(REG_PWR_MGMT_1, 0x00); delay(50);
    mpuWrite(REG_GYRO_CFG, 0x00);
    mpuWrite(REG_ACCEL_CFG, 0x00);
    lastImuUs = micros();

    menuEncRef = rightEnc.getTicks();
    oledMenu();
    Serial.printf("[INIT] side-open MPU WHO=0x%02X\n", who);
}

void loop() {
    switch (state) {
        case IDLE: {
            long delta = rightEnc.getTicks() - menuEncRef;
            if (delta >= ENC_PER_STEP) {
                menuSel = (menuSel + 1) % M_COUNT;
                menuEncRef += ENC_PER_STEP;
                oledMenu();
                return;
            }
            if (delta <= -ENC_PER_STEP) {
                menuSel = (menuSel - 1 + M_COUNT) % M_COUNT;
                menuEncRef -= ENC_PER_STEP;
                oledMenu();
                return;
            }
            if (buttonEdge()) {
                switch (menuSel) {
                    case M_CAL_IR:   doCalIR();    oledMenu(); break;
                    case M_CAL_GYRO: doCalGyro();  oledMenu(); break;
                    case M_RUN:      doRun();      oledMenu(); break;
                    case M_LIVE:     state = LIVE_IR; break;
                }
                menuEncRef = rightEnc.getTicks();
            }
            break;
        }

        case LIVE_IR: {
            static uint32_t last = 0;
            if (millis() - last > 100) { sampleIR(); oledLive(); last = millis(); }
            if (buttonEdge()) {
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                state = IDLE;
            }
            break;
        }

        default: break;
    }
}
