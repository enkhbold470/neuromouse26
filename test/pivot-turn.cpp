// test/pivot-turn.cpp — standalone pivot-turn diagnostic
//
// Repeats a single pivot turn primitive on each button press so the user
// can measure post-pivot center position vs cell geometry.
//
// Pivot mechanics mirror src/main.cpp::PH_PIVOT exactly:
//   - one wheel braked (= "pivot point"), other wheel drives forward
//   - gyro yaw integrated continuously; turn target = ±90° (or ±180°)
//   - PID on (target − progress) with no-reverse clamp on active wheel
//
// Menu (rotate spinner = right encoder; press button to select):
//   Cal Gyro       — 300-sample bias capture (keep still)
//   Pivot R 90     — single right pivot
//   Pivot L 90     — single left pivot
//   Pivot R 180    — right 180° (= two stacked right pivots in code, single
//                                  phase here)
//   Pivot L 180    — left 180°
//   Live IMU       — show yaw + gz
//   Reset Yaw      — yawDeg=0
//
// Wire as the PH_PIVOT entry in main.cpp. Tune values are pulled from
// PinConfig.h (TURN_KP/KD/TURN_PEAK_OMEGA_DPS/etc.).
//
// platformio.ini env block (add to existing file):
//   [env:pivot-turn]
//   extends = common
//   build_src_filter = +<../test/pivot-turn.cpp>

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoderPCNT.h"

// ── MPU-6500 ─────────────────────────────────────────────────────────────────
#define MPU_ADDR          0x68
#define REG_WHO_AM_I      0x75
#define REG_PWR_MGMT_1    0x6B
#define REG_CONFIG        0x1A
#define REG_GYRO_CFG      0x1B
#define REG_ACCEL_CFG     0x1C
#define GYRO_FS_SEL       0x10        // ±1000 dps
#define GYRO_SCALE        32.8f
#define DLPF_CFG_VAL      0x03        // 41 Hz BW

// ── Pivot tuning (same as src/main.cpp values) ───────────────────────────────
constexpr float YAW_KP_T              =    6.0f;
constexpr float YAW_KD_T              =    0.3f;
constexpr float YAW_FRICTION_ZONE_T   =    3.0f;
constexpr float YAW_STK_SOFT_BAND_T   =    2.0f;
constexpr float YAW_HOLD_BAND_T       =    1.5f;
constexpr uint32_t YAW_SETTLE_MS_T    =   80;
constexpr float YAW_STALL_VEL_T       =    5.0f;
constexpr uint32_t YAW_STALL_MS_T     =  250;
constexpr float YAW_STALL_ERR_MAX_T   =    4.0f;

constexpr int   BASE_BREAKAWAY_T      =  110;
constexpr int   YAW_STICTION_PWM_T    = (BASE_BREAKAWAY_T * 15) / 10;          // 1.5×
constexpr int   YAW_MAX_PWM_T         = (BASE_BREAKAWAY_T * 16) / 10;          // 1.6× cap

// ── Globals ──────────────────────────────────────────────────────────────────
static float    gyroBiasZ = 0.0f;
static float    yawDeg    = 0.0f;
static float    gzFilt    = 0.0f;
static uint32_t lastYawUs = 0;
static bool     imuReady  = false;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
MicromouseMotor       leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor       rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoderPCNT leftEnc   (PCNT_UNIT_0, ENC_L_A, ENC_L_B, /*inverted=*/true);
MicromouseEncoderPCNT rightEnc  (PCNT_UNIT_1, ENC_R_A, ENC_R_B, /*inverted=*/true);

// ── MPU helpers ──────────────────────────────────────────────────────────────
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
static void calibrateGyroBias(int N = 300, int dly = 2) {
    float sum = 0; int good = 0;
    for (int i = 0; i < N; i++) {
        int16_t raw;
        if (readGzRaw(raw)) { sum += raw / GYRO_SCALE; good++; }
        delay(dly);
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

static void stopMotors() { leftMotor.brake(); rightMotor.brake(); }

// ── Pivot turn primitive ─────────────────────────────────────────────────────
// dir: +1 = right (left wheel drives, right wheel braked).
//      -1 = left  (right wheel drives, left wheel braked).
// targetDeg: absolute magnitude (90 or 180); sign comes from dir.
// Blocks until settled, stalled, or timed out (~5 s).
static void doPivot(int dir, float targetDeg) {
    float startYaw   = yawDeg;
    float signedTgt  = (dir > 0) ? -targetDeg : +targetDeg;  // right turn → yawDeg goes negative
    uint32_t startMs = millis();

    uint32_t settleStart = 0, stallStart = 0;
    Serial.printf("[PIVOT] dir=%+d target=%.1f startYaw=%.2f\n",
                  dir, targetDeg, startYaw);

    while (true) {
        updateYaw();

        float dy       = yawDeg - startYaw;
        float progress = (dir > 0) ? -dy : +dy;          // progress always positive
        float posErr   = targetDeg - progress;

        // Settle band — same exit criterion as PH_PIVOT
        if (fabsf(posErr) < YAW_HOLD_BAND_T) {
            stopMotors();
            if (settleStart == 0) settleStart = millis();
            if (millis() - settleStart > YAW_SETTLE_MS_T) {
                Serial.printf("[PIVOT] SETTLED progress=%.2f err=%.2f endYaw=%.2f\n",
                              progress, posErr, yawDeg);
                return;
            }
            continue;
        }
        settleStart = 0;

        // Stall detect
        if (fabsf(gzFilt) < YAW_STALL_VEL_T && fabsf(posErr) < YAW_STALL_ERR_MAX_T) {
            if (stallStart == 0) stallStart = millis();
            if (millis() - stallStart > YAW_STALL_MS_T) {
                stopMotors();
                Serial.printf("[PIVOT] STALL progress=%.2f err=%.2f\n", progress, posErr);
                return;
            }
        } else {
            stallStart = 0;
        }

        // Timeout safety
        if (millis() - startMs > 5000) {
            stopMotors();
            Serial.printf("[PIVOT] TIMEOUT progress=%.2f err=%.2f\n", progress, posErr);
            return;
        }

        // PID — same form as production PH_PIVOT path
        float u   = YAW_KP_T * posErr - YAW_KD_T * gzFilt;
        int   mag = (int)fabsf(u);
        if (mag > YAW_MAX_PWM_T) mag = YAW_MAX_PWM_T;

        // Soft stiction floor outside friction zone
        float errAbs = fabsf(posErr);
        if (errAbs > YAW_FRICTION_ZONE_T) {
            float tBlend = constrain((errAbs - YAW_FRICTION_ZONE_T) / YAW_STK_SOFT_BAND_T,
                                     0.0f, 1.0f);
            int softFloor = (int)(tBlend * (float)YAW_STICTION_PWM_T);
            if (mag < softFloor) mag = softFloor;
        }

        int throttle = (u >= 0) ? mag : -mag;
        // No-reverse clamp: active wheel only drives forward; if PID would
        // command reverse (overshoot), brake both wheels.
        int pivotThrottle = (throttle > 0) ? throttle : 0;

        if (dir > 0) {
            // Right pivot: left wheel drives forward, right wheel braked.
            if (pivotThrottle > 0) leftMotor.drive(pivotThrottle);
            else                    leftMotor.brake();
            rightMotor.brake();
        } else {
            // Left pivot: right wheel drives forward, left wheel braked.
            leftMotor.brake();
            if (pivotThrottle > 0) rightMotor.drive(pivotThrottle);
            else                    rightMotor.brake();
        }

        // Slow telemetry @ ~50 Hz to avoid spamming
        static uint32_t lastTel = 0;
        if (millis() - lastTel > 20) {
            Serial.printf("t=%lu yaw=%+.2f gz=%+.2f prog=%+.2f err=%+.2f thr=%d\n",
                          (unsigned long)millis(), yawDeg, gzFilt,
                          progress, posErr, throttle);
            lastTel = millis();
        }
    }
}

// ── OLED menu ────────────────────────────────────────────────────────────────
enum Item {
    M_CAL = 0,
    M_R90,
    M_L90,
    M_R180,
    M_L180,
    M_LIVE,
    M_RESET,
    M_COUNT
};
static const char* LABELS[M_COUNT] = {
    "Cal Gyro", "Pivot R 90", "Pivot L 90", "Pivot R 180",
    "Pivot L 180", "Live IMU", "Reset Yaw"
};
static int  sel        = M_R90;
static long encRef     = 0;
constexpr long ENC_PER_STEP = 80;

static void drawMenu() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "pivot-turn test");
    oled.drawHLine(0, 10, 128);
    const int VIS = 5, LH = 10;
    int top = sel - VIS / 2; if (top < 0) top = 0;
    if (top > M_COUNT - VIS) top = M_COUNT - VIS;
    if (top < 0) top = 0;
    for (int i = 0; i < VIS; i++) {
        int idx = top + i;
        if (idx >= M_COUNT) break;
        int y = 12 + i * LH;
        if (idx == sel) {
            oled.drawBox(0, y, 128, LH);
            oled.setDrawColor(0);
            oled.drawStr(3, y + 8, LABELS[idx]);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(3, y + 8, LABELS[idx]);
        }
    }
    oled.sendBuffer();
}

static void showLive() {
    char b[24];
    while (true) {
        updateYaw();
        oled.clearBuffer();
        oled.setFont(u8g2_font_6x10_tf);
        oled.drawStr(0, 8, "Live IMU");
        oled.drawHLine(0, 10, 128);
        oled.setFont(u8g2_font_8x13B_tf);
        snprintf(b, sizeof(b), "yaw %+.1f", yawDeg);
        oled.drawStr(0, 28, b);
        snprintf(b, sizeof(b), "gz  %+.1f", gzFilt);
        oled.drawStr(0, 46, b);
        oled.setFont(u8g2_font_5x7_tf);
        oled.drawStr(0, 63, "btn=back");
        oled.sendBuffer();
        if (buttonEdge()) return;
        delay(50);
    }
}

static void showWaiting(const char* msg) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "pivot-turn test");
    oled.drawHLine(0, 10, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    oled.drawStr(0, 36, msg);
    oled.sendBuffer();
}

// ── Setup / loop ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_1, INPUT_PULLUP);

    leftMotor.begin(); rightMotor.begin();
    leftEnc.begin();   rightEnc.begin();

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    imuReady = mpuInit();
    if (imuReady) {
        Serial.println("[IMU] mpu6500 ok, calibrating bias (keep still)...");
        showWaiting("CAL GYRO");
        delay(300);
        calibrateGyroBias(300, 2);
        yawDeg = 0.0f;
        Serial.printf("[IMU] bias=%.4f deg/s\n", gyroBiasZ);
    } else {
        Serial.println("[IMU] NOT detected");
    }

    encRef = rightEnc.getTicks();
    drawMenu();
    Serial.println("pivot-turn ready");
}

void loop() {
    updateYaw();

    long delta = rightEnc.getTicks() - encRef;
    if (delta >=  ENC_PER_STEP) { sel = (sel + 1) % M_COUNT;            encRef += ENC_PER_STEP; drawMenu(); }
    if (delta <= -ENC_PER_STEP) { sel = (sel - 1 + M_COUNT) % M_COUNT;  encRef -= ENC_PER_STEP; drawMenu(); }

    if (!buttonEdge()) return;

    switch (sel) {
    case M_CAL:
        showWaiting("CAL GYRO");
        delay(300);
        calibrateGyroBias(300, 2);
        yawDeg = 0.0f;
        Serial.printf("[IMU] bias=%.4f deg/s\n", gyroBiasZ);
        encRef = rightEnc.getTicks();
        drawMenu();
        break;
    case M_R90:   showWaiting("PIV R 90");  doPivot(+1,  90.0f); drawMenu(); encRef = rightEnc.getTicks(); break;
    case M_L90:   showWaiting("PIV L 90");  doPivot(-1,  90.0f); drawMenu(); encRef = rightEnc.getTicks(); break;
    case M_R180:  showWaiting("PIV R 180"); doPivot(+1, 180.0f); drawMenu(); encRef = rightEnc.getTicks(); break;
    case M_L180:  showWaiting("PIV L 180"); doPivot(-1, 180.0f); drawMenu(); encRef = rightEnc.getTicks(); break;
    case M_LIVE:  showLive();               drawMenu(); encRef = rightEnc.getTicks(); break;
    case M_RESET: yawDeg = 0.0f; Serial.println("[IMU] yawDeg=0"); break;
    }
}
