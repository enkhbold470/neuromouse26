// test/wall-follow-simple.cpp — wall-follow PID test
//
// Uses same proven scaffolding as src/main.cpp:
//   inline IR sample (amb - lit, 80µs settle, threshold 1500)
//   motor inv flags from PinConfig
//   OLED feedback, BUTTON_1 = start/stop
//   no FreeRTOS task, no tone, no Serial cmds
//
// Behavior:
//   button → toggle run/stop
//   running → drive forward at BASE_PWM, PID centering when L or R wall seen,
//             encoder balance when no walls

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

// ── IR (same pattern as main.cpp) ────────────────────────────────────────────
// PAIRS index: 0=LF, 1=L, 2=R, 3=RF
struct IRPair { uint8_t emit, rx; };
static IRPair PAIRS[4] = {
    { EMIT_LF, RX_LF },
    { EMIT_L,  RX_L  },
    { EMIT_R,  RX_R  },
    { EMIT_RF, RX_RF },
};
static int irVal[4] = {0,0,0,0};

constexpr int WALL_SIDE  = 1500;
constexpr int WALL_FRONT = 1500;

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

static void sampleIR() {
    for (int i = 0; i < 4; i++) irVal[i] = readIR(PAIRS[i]);
}

// ── Tuning ───────────────────────────────────────────────────────────────────
// PID gains for centering. Inputs are raw IR deltas (0..~3900).
// error = irL - irR  (positive = closer to L wall = drifted left → steer right)
constexpr float CENTER_KP   = 0.10f;
constexpr float CENTER_KI   = 0.0f;
constexpr float CENTER_KD   = 0.02f;
constexpr int   MAX_CORR    = 200;
constexpr int   BASE_PWM_WF = DRIVE_PWM;
constexpr float ENC_BAL_KP  = (float)BALANCE_KP;  // when no walls

// ── PID ──────────────────────────────────────────────────────────────────────
struct PID {
    float integral  = 0;
    float prevError = 0;
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

// ── Button ───────────────────────────────────────────────────────────────────
bool buttonEdge() {
    static bool last = HIGH;
    bool cur = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && cur == LOW);
    last = cur;
    return edge;
}

// ── Motors ───────────────────────────────────────────────────────────────────
void stopMotors() { leftMotor.brake(); rightMotor.brake(); }

// ── OLED screen ──────────────────────────────────────────────────────────────
static bool running = false;
static int  lastErr  = 0;
static int  lastCorr = 0;

void oledShow() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 10, running ? "wall-follow RUN" : "wall-follow STOP");
    oled.drawHLine(0, 12, 128);
    char buf[24];
    snprintf(buf, sizeof(buf), "L%4d R%4d", irVal[1], irVal[2]);
    oled.drawStr(0, 26, buf);
    snprintf(buf, sizeof(buf), "LF%4d RF%4d", irVal[0], irVal[3]);
    oled.drawStr(0, 40, buf);
    snprintf(buf, sizeof(buf), "err%+d corr%+d", lastErr, lastCorr);
    oled.drawStr(0, 54, buf);
    oled.setFont(u8g2_font_5x7_tf);
    oled.drawStr(0, 63, "btn = toggle");
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

    sampleIR();
    oledShow();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    if (buttonEdge()) {
        running = !running;
        if (running) {
            pid.reset();
            leftEnc.reset();
            rightEnc.reset();
        } else {
            stopMotors();
        }
        sampleIR();
        oledShow();
        delay(30);
    }

    if (!running) return;

    sampleIR();

    bool wL = irVal[1] > WALL_SIDE;
    bool wR = irVal[2] > WALL_SIDE;
    bool wF = irVal[0] > WALL_FRONT || irVal[3] > WALL_FRONT;

    if (wF) {
        // Front wall — stop, exit run.
        stopMotors();
        running = false;
        lastErr = 0; lastCorr = 0;
        oledShow();
        return;
    }

    int  pwmL, pwmR;
    if (wL || wR) {
        // Centering: error = L - R. Positive = closer to L → steer right
        // → reduce left wheel, boost right.
        int  err  = irVal[1] - irVal[2];
        float corr = pid.compute((float)err);
        lastErr  = err;
        lastCorr = (int)corr;
        pwmL = constrain(BASE_PWM_WF - (int)corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        pwmR = constrain(BASE_PWM_WF + (int)corr, DRIVE_PWM_MIN, MOTOR_PWM_MAX);
    } else {
        // No walls — encoder balance only.
        long tL = leftEnc.getTicks();
        long tR = rTicks();
        int  encErr = (int)(tL - tR);
        lastErr  = encErr;
        lastCorr = 0;
        pwmL = constrain(BASE_PWM_WF - (int)(encErr * ENC_BAL_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
        pwmR = constrain(BASE_PWM_WF + (int)(encErr * ENC_BAL_KP), DRIVE_PWM_MIN, MOTOR_PWM_MAX);
    }

    leftMotor.drive(pwmL);
    rightMotor.drive(pwmR);

    // OLED at low rate, but only while running. Throttle to avoid I2C hogging.
    static uint32_t lastOled = 0;
    if (millis() - lastOled > 150) {
        oledShow();
        lastOled = millis();
    }
}
