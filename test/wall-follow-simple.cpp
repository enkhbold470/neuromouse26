// test/wall-follow-simple.cpp — NO Serial, NO BLE, NO delays in loop.
// Button press = start. Press again = stop.

#include <Arduino.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"

// ── IR calibration (real hardware measurements) ───────────────────────────────
#define L45_CENTER  865
#define R45_CENTER  477
#define L45_THRESH  433
#define R45_THRESH  238

// ── Tuning ────────────────────────────────────────────────────────────────────
#define BASE_PWM    250
#define CENTER_KP   300.0f
#define CENTER_KI   50.0f   // raised: integrator kills steady-state drift fast
#define CENTER_KD   50.0f
#define MAX_CORR    400
// Trim: positive value nudges robot LEFT. Increase in steps of 0.05 until centered.
#define ERROR_TRIM  0.05f

// ── Hardware ──────────────────────────────────────────────────────────────────
MicromouseMotor leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "L");
MicromouseMotor rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "R");

static const uint8_t EMIT_PINS[4] = { EMIT_LF, EMIT_L45, EMIT_R45, EMIT_RF };
static const uint8_t RX_PINS[4]   = { RX_LF,   RX_L45,   RX_R45,   RX_RF  };

int irRead(int idx) {
    int amb = analogRead(RX_PINS[idx]);
    digitalWrite(EMIT_PINS[idx], HIGH);
    delayMicroseconds(50);
    int lit = analogRead(RX_PINS[idx]);
    digitalWrite(EMIT_PINS[idx], LOW);
    return max(0, lit - amb);
}

float normL45(int raw) { return constrain((float)raw / L45_CENTER, 0.0f, 2.0f); }
float normR45(int raw) { return constrain((float)raw / R45_CENTER, 0.0f, 2.0f); }

void stopMotors() {
    leftMotor.brake(); rightMotor.brake();
    delay(80);
    leftMotor.coast(); rightMotor.coast();
}

// ── Centering PID ─────────────────────────────────────────────────────────────
struct CenterPID {
    float integral  = 0;
    float prevError = 0;
    unsigned long prevUs = 0;

    float compute(float error) {
        unsigned long now = micros();
        float dt = (prevUs == 0) ? 0.001f : constrain((now - prevUs) / 1e6f, 0.0001f, 0.05f);
        prevUs = now;
        integral += error * dt;
        integral  = constrain(integral, -2.0f, 2.0f);
        float deriv = (error - prevError) / dt;
        prevError = error;
        float out = CENTER_KP * error + CENTER_KI * integral + CENTER_KD * deriv;
        return constrain(out, -(float)MAX_CORR, (float)MAX_CORR);
    }

    void reset() { integral = 0; prevError = 0; prevUs = 0; }
} pid;

// ── Button ────────────────────────────────────────────────────────────────────
bool buttonEdge() {
    static bool last = HIGH;
    bool now  = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && now == LOW);
    last = now;
    return edge;
}

bool running = false;

void setup() {
    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    leftMotor.begin();
    rightMotor.begin();

    for (int i = 0; i < 4; i++) {
        pinMode(EMIT_PINS[i], OUTPUT);
        digitalWrite(EMIT_PINS[i], LOW);
        pinMode(RX_PINS[i], INPUT);
    }
}

void loop() {
    if (buttonEdge()) {
        running = !running;
        if (!running) stopMotors();
        else          pid.reset();
        delay(30);
    }

    if (!running) return;

    int   l45 = irRead(1);
    int   r45 = irRead(2);
    float nL  = normL45(l45);
    float nR  = normR45(r45);
    bool  wL  = l45 > L45_THRESH;
    bool  wR  = r45 > R45_THRESH;

    float error = 0.0f;
    if (wL && wR)   error =  nR - nL;
    else if (wR)    error =  nR - 1.0f;
    else if (wL)    error = -(nL - 1.0f);

    // Positive trim shifts setpoint left. Tune if robot consistently off-center.
    error += ERROR_TRIM;

    float corr = (wL || wR) ? pid.compute(error) : 0.0f;

    int pwmL = constrain(BASE_PWM - (int)corr, -1023, 1023);
    int pwmR = constrain(BASE_PWM + (int)corr, -1023, 1023);

    leftMotor.drive(pwmL);
    rightMotor.drive(pwmR);
}
