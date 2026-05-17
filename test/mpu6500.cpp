// test/mpu6500.cpp — IMU-based turn practice
// MPU-6500 on shared I2C bus with OLED (SDA=OLED_SDA, SCL=OLED_SCL).
// Menu:
//   Cal Gyro     — 300 samples bias capture (keep robot STILL)
//   Turn R 90    — pivot until yaw >= +90°
//   Turn L 90    — pivot until yaw <= -90°
//   Turn R 180   — pivot until yaw >= +180°
//   Turn L 180   — pivot until yaw <= -180°
//   Live IMU     — yaw + gz live
//   Reset Yaw    — zero integration
//
// Pivot: left=+, right=- on left motor; mirrored on right.
// Closed-loop: trapezoidal ω profile + Kff + PID on integrated yaw.
// Tune TURN_PEAK_OMEGA_DPS / TURN_ACCEL_DPS2 / TURN_KP/KD / TURN_KFF.

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── Turn closed-loop tuning ──────────────────────────────────────────────────
// Trapezoidal angular velocity profile + feedforward + PID on integrated yaw.
// Surface-independent: PID closes the loop on actual yaw, profile shape

// ── MPU-6500 ─────────────────────────────────────────────────────────────────
#define MPU_ADDR          0x68
#define REG_WHO_AM_I      0x75
#define REG_PWR_MGMT_1    0x6B
#define REG_GYRO_CFG      0x1B
#define REG_ACCEL_CFG     0x1C
#define REG_ACCEL_XOUT_H  0x3B
// Gyro full-scale: 0x00=±250(131), 0x08=±500(65.5), 0x10=±1000(32.8), 0x18=±2000(16.4)
#define GYRO_FS_SEL       0x10     // ±1000 dps — must exceed TURN_PEAK_OMEGA_DPS
#define GYRO_SCALE        32.8f    // LSB/(°/s) matched to GYRO_FS_SEL
const int TURN_MIN_PWM = 200;
struct RawData { int16_t ax, ay, az, temp, gx, gy, gz; };

static float gyroBiasZ = 0.0f;
static float yaw = 0.0f;
static unsigned long lastUpdateUs = 0;

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
static int16_t to16(uint8_t hi, uint8_t lo) { return (int16_t)((hi << 8) | lo); }

static bool readAll(RawData& d) {
    uint8_t b[14];
    if (!mpuRead(REG_ACCEL_XOUT_H, b, 14)) return false;
    d.ax=to16(b[0],b[1]); d.ay=to16(b[2],b[3]); d.az=to16(b[4],b[5]);
    d.temp=to16(b[6],b[7]);
    d.gx=to16(b[8],b[9]); d.gy=to16(b[10],b[11]); d.gz=to16(b[12],b[13]);
    return true;
}

// Integrate yaw from gz. Call frequently.
static void updateYaw() {
    RawData d;
    if (!readAll(d)) return;
    unsigned long now = micros();
    float dt = (lastUpdateUs == 0) ? 0.001f : (now - lastUpdateUs) / 1e6f;
    if (dt > 0.05f) dt = 0.05f;
    lastUpdateUs = now;
    float gz = d.gz / GYRO_SCALE - gyroBiasZ;
    if (fabsf(gz) < 0.05f) gz = 0;
    yaw += gz * dt;
}

// ── OLED + motors ────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);   // for menu scroll only

static void stopMotors() { leftMotor.brake(); rightMotor.brake(); }

// Mechanical keyswitch debounce.
static bool buttonEdge() {
    static unsigned long pressStart = 0;
    static bool armed = true;
    bool low = (digitalRead(BUTTON_1) == LOW);
    unsigned long now = millis();
    if (!low) { pressStart = 0; armed = true; return false; }
    if (pressStart == 0) pressStart = now;
    if (armed && (now - pressStart >= BUTTON_HOLD_MS)) { armed = false; return true; }
    return false;
}

// ── Turn logic (trapezoid + PID) ─────────────────────────────────────────────
// Trapezoidal angular-velocity profile generator. Triangle if target small.
struct TurnProfile {
    float accel;    // deg/s^2, positive
    float peak;     // deg/s, positive
    float target;   // deg, positive
    float t_acc, t_cru, t_tot;
    float d_acc, d_cru;
    int   sign;

    void init(float targetDeg) {
        sign   = (targetDeg >= 0) ? 1 : -1;
        target = fabsf(targetDeg);
        accel  = TURN_ACCEL_DPS2;
        peak   = TURN_PEAK_OMEGA_DPS;
        float d_full_acc = (peak * peak) / (2.0f * accel);
        if (2.0f * d_full_acc >= target) {        // triangular
            peak  = sqrtf(accel * target);
            d_acc = target * 0.5f;
            d_cru = 0;
        } else {                                  // trapezoid
            d_acc = d_full_acc;
            d_cru = target - 2.0f * d_full_acc;
        }
        t_acc = peak / accel;
        t_cru = (peak > 0) ? d_cru / peak : 0;
        t_tot = 2.0f * t_acc + t_cru;
    }

    // Desired signed angle and omega at time t (seconds since start).
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

static void settle(unsigned long ms) {
    unsigned long t0 = millis();
    while (millis() - t0 < ms) { updateYaw(); }
}

static void doTurn(float targetDeg) {
    yaw = 0;
    lastUpdateUs = micros();
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
        if (dt < 1.0e-4f) continue;                // skip ultra-fast iters
        lastUs = nowUs;

        float angle_des, omega_des;
        prof.at(t, angle_des, omega_des);

        float err  = angle_des - yaw;
        float derr = (err - prevErr) / dt;
        prevErr    = err;

        float pwm = TURN_KFF_PWM_PER_DPS * omega_des
                  + TURN_KP_PWM_PER_DEG  * err
                  + TURN_KD_PWM_PER_DPS  * derr;

        // Stiction floor only during hold phase — break it free, don't slam.
        bool holding = (t >= prof.t_tot);
        if (holding && fabsf(pwm) > 1.0f && fabsf(pwm) < TURN_MIN_PWM) {
            pwm = (pwm > 0) ? TURN_MIN_PWM : -TURN_MIN_PWM;
        }
        if (pwm >  TURN_PWM) pwm =  TURN_PWM;
        if (pwm < -TURN_PWM) pwm = -TURN_PWM;

        leftMotor.drive(-(int)pwm);
        rightMotor.drive( (int)pwm);

        if (fabsf(err) > peakErr) peakErr = fabsf(err);

        // Exit: in deadband continuously for HOLD_MS after profile end.
        if (holding && fabsf(targetDeg - yaw) <= TURN_DEADBAND_DEG) {
            if (inBandStart == 0) inBandStart = nowMs;
            if (nowMs - inBandStart >= TURN_HOLD_MS) break;
        } else {
            inBandStart = 0;
        }
        if (nowMs - t0 > TURN_TIMEOUT_MS) break;
    }
    stopMotors();
    settle(TURN_SETTLE_MS);
    Serial.printf("[TURN] target=%+.1f final=%+.2f err=%+.2f peakProfErr=%.2f\n",
                  targetDeg, yaw, targetDeg - yaw, peakErr);
}

// ── Calibration ──────────────────────────────────────────────────────────────
static char calStatus[24] = "";

static void oledMsg(const char* title, const char* l1, const char* l2 = nullptr) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, title);
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    if (l1) oled.drawStr(0, 32, l1);
    if (l2) oled.drawStr(0, 50, l2);
    oled.sendBuffer();
}

static void calibrateGyro() {
    constexpr int N = 300;
    float sum = 0;
    int good = 0;
    char buf[24];
    for (int i = 0; i < N; i++) {
        RawData d;
        if (readAll(d)) { sum += d.gz / GYRO_SCALE; good++; }
        if ((i & 0x3F) == 0) {
            snprintf(buf, sizeof(buf), "%d/%d", i, N);
            oledMsg("CAL GYRO", "STILL", buf);
        }
        delay(2);
    }
    gyroBiasZ = (good > 0) ? sum / good : 0;
    snprintf(calStatus, sizeof(calStatus), "bias=%.3f", gyroBiasZ);
    yaw = 0;
    lastUpdateUs = micros();
}

// ── Menu ─────────────────────────────────────────────────────────────────────
enum { M_CAL = 0, M_TR90, M_TL90, M_TR180, M_TL180, M_LIVE, M_RST, M_COUNT };
static const char* MENU[M_COUNT] = {
    "Cal Gyro", "Turn R 90", "Turn L 90", "Turn R 180", "Turn L 180",
    "Live IMU", "Reset Yaw"
};
static int menuSel = M_CAL;
static long menuEncRef = 0;
constexpr long ENC_PER_STEP = 80;

static void oledMenu() {
    const int VIS = 5;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > M_COUNT - VIS) top = M_COUNT - VIS;
    if (top < 0) top = 0;

    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "IMU turn");
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
            oled.drawStr(3, y + 8, MENU[idx]);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(3, y + 8, MENU[idx]);
        }
    }
    oled.drawHLine(0, 64 - 10, 128);
    oled.setFont(u8g2_font_5x7_tf);
    char buf[32];
    snprintf(buf, sizeof(buf), "yaw=%+.1f  %s", yaw, calStatus);
    oled.drawStr(0, 63, buf);
    oled.sendBuffer();
}

static void oledLive() {
    RawData d;
    readAll(d);
    float gz = d.gz / GYRO_SCALE - gyroBiasZ;
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, "Live IMU");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "yaw %+.1f", yaw);
    oled.drawStr(0, 32, buf);
    snprintf(buf, sizeof(buf), "gz  %+.2f", gz);
    oled.drawStr(0, 50, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "btn=back");
    oled.sendBuffer();
}

// ── State ────────────────────────────────────────────────────────────────────
enum State { IDLE, LIVE };
State state = IDLE;

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_1, INPUT_PULLUP);

    leftMotor.begin();
    rightMotor.begin();
    rightEnc.begin();

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    // MPU init
    uint8_t who = 0;
    if (!mpuRead(REG_WHO_AM_I, &who, 1)) {
        oledMsg("MPU-6500", "I2C FAIL");
        while (1) delay(1000);
    }
    snprintf(calStatus, sizeof(calStatus), "WHO=0x%02X", who);
    mpuWrite(REG_PWR_MGMT_1, 0x00); delay(50);
    mpuWrite(REG_GYRO_CFG,   GYRO_FS_SEL);
    mpuWrite(REG_ACCEL_CFG,  0x00);
    lastUpdateUs = micros();

    menuEncRef = rightEnc.getTicks();
    oledMenu();
    Serial.println("[INIT] imu turn ready");
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    updateYaw();   // always integrate

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
                    case M_CAL:   calibrateGyro(); oledMenu(); break;
                    case M_TR90:  doTurn(-90);  oledMenu(); break;  // R = yaw negative
                    case M_TL90:  doTurn(+90);  oledMenu(); break;
                    case M_TR180: doTurn(-180); oledMenu(); break;
                    case M_TL180: doTurn(+180); oledMenu(); break;
                    case M_LIVE:  state = LIVE; break;
                    case M_RST:   yaw = 0; oledMenu(); break;
                }
                menuEncRef = rightEnc.getTicks();
            }
            // Periodic refresh for yaw footer
            static uint32_t lastRefresh = 0;
            if (millis() - lastRefresh > 200) { oledMenu(); lastRefresh = millis(); }
            break;
        }

        case LIVE: {
            static uint32_t last = 0;
            if (millis() - last > 100) { oledLive(); last = millis(); }
            if (buttonEdge()) {
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                state = IDLE;
            }
            break;
        }
    }
}
