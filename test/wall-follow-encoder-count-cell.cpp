// test/wall-follow-encoder.cpp
// Wall follow (IR centering) + encoder straight-keeping + continuous cell tracking.
// Button press = start. Press again = stop.
// Robot moves 1 cell at a time via encoder, never stops between cells.
//
// Cell math:
//   WHEEL_DIAMETER = 33.4mm → circumference = π × 33.4 ≈ 104.93mm
//   TICKS_PER_REV  = 210 (7 PPR × 30 gear, single-channel RISING)
//   ticks/mm = 210 / 104.93 ≈ 2.001
//   1 cell = 180mm → TICKS_PER_CELL = 180 × 2.001 ≈ 360

#include <Arduino.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── IR calibration ────────────────────────────────────────────────────────────
#define L45_CENTER  865
#define R45_CENTER  477
#define L45_THRESH  433
#define R45_THRESH  238

// ── Drive tuning ──────────────────────────────────────────────────────────────
#define BASE_PWM        250

// Wall-centering PID (IR-based, same as wall-follow-simple)
#define WALL_KP         500.0f
#define WALL_KI         50.0f
#define WALL_KD         300.0f
#define WALL_MAX_CORR   400
#define ERROR_TRIM      0.10f

// Encoder straight-keeping PID (differential tick error → motor correction)
// Tune ENC_KP first; KI/KD can be zeroed initially.
#define ENC_KP          6.0f
#define ENC_KI          0.8f
#define ENC_KD          0.5f
#define ENC_MAX_CORR    120

// Cell distance in encoder ticks (see math above)
#define TICKS_PER_CELL  360L
#define MAX_CELLS       5
#define CELL_PAUSE_MS   500

// ── Hardware ──────────────────────────────────────────────────────────────────
MicromouseMotor leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "L");
MicromouseMotor rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "R");

MicromouseEncoder encLeft (ENC_L_A, ENC_L_B);
MicromouseEncoder encRight(ENC_R_A, ENC_R_B);

void IRAM_ATTR isrLeft()  { encLeft.handleInterrupt();  }
void IRAM_ATTR isrRight() { encRight.handleInterrupt(); }

static const uint8_t EMIT_PINS[4] = { EMIT_LF, EMIT_L45, EMIT_R45, EMIT_RF };
static const uint8_t RX_PINS[4]   = { RX_LF,   RX_L45,   RX_R45,   RX_RF  };

// ── IR read ───────────────────────────────────────────────────────────────────
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

// ── Generic PID ───────────────────────────────────────────────────────────────
struct PID {
    float kp, ki, kd, maxOut;
    float integral  = 0;
    float prevError = 0;
    unsigned long prevUs = 0;

    PID(float p, float i, float d, float mx) : kp(p), ki(i), kd(d), maxOut(mx) {}

    float compute(float error) {
        unsigned long now = micros();
        float dt = (prevUs == 0) ? 0.001f
                                 : constrain((now - prevUs) / 1e6f, 0.0001f, 0.05f);
        prevUs = now;
        integral  += error * dt;
        integral   = constrain(integral, -2.0f, 2.0f);
        float deriv = (error - prevError) / dt;
        prevError = error;
        return constrain(kp * error + ki * integral + kd * deriv, -maxOut, maxOut);
    }

    void reset() { integral = 0; prevError = 0; prevUs = 0; }
};

PID wallPid(WALL_KP, WALL_KI, WALL_KD, (float)WALL_MAX_CORR);
PID encPid (ENC_KP,  ENC_KI,  ENC_KD,  (float)ENC_MAX_CORR);

// ── Button ────────────────────────────────────────────────────────────────────
bool buttonEdge() {
    static bool last = HIGH;
    bool cur  = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && cur == LOW);
    last = cur;
    return edge;
}

void stopMotors() {
    leftMotor.brake(); rightMotor.brake();
    delay(80);
    leftMotor.coast(); rightMotor.coast();
}

// ── State ─────────────────────────────────────────────────────────────────────
bool running   = false;
long cellCount = 0;    // total cells completed (informational)
long cellBaseL = 0;    // encoder snapshot at start of current cell
long cellBaseR = 0;

void resetCellBase() {
    cellBaseL = encLeft.getTicks();
    cellBaseR = encRight.getTicks();
}

void setup() {
    Serial.begin(115200);

    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    leftMotor.begin();
    rightMotor.begin();

    encLeft.begin(isrLeft);
    encRight.begin(isrRight);

    for (int i = 0; i < 4; i++) {
        pinMode(EMIT_PINS[i], OUTPUT);
        digitalWrite(EMIT_PINS[i], LOW);
        pinMode(RX_PINS[i], INPUT);
    }

    Serial.println("[ENC-WALL] ready. Press button to start.");
}

void loop() {
    if (buttonEdge()) {
        running = !running;
        if (!running) {
            stopMotors();
            Serial.printf("[ENC-WALL] stopped after %ld cells\n", cellCount);
        } else {
            wallPid.reset();
            encPid.reset();
            encLeft.reset();
            encRight.reset();
            cellCount = 0;
            resetCellBase();
            Serial.println("[ENC-WALL] running");
        }
        delay(30);
    }

    if (!running) return;

    // ── IR wall centering ─────────────────────────────────────────────────────
    int   l45 = irRead(1);
    int   r45 = irRead(2);
    float nL  = normL45(l45);
    float nR  = normR45(r45);
    bool  wL  = l45 > L45_THRESH;
    bool  wR  = r45 > R45_THRESH;

    float wallErr = 0.0f;
    if      (wL && wR) wallErr =  nR - nL;
    else if (wR)       wallErr =  nR - 1.0f;
    else if (wL)       wallErr = -(nL - 1.0f);
    wallErr += ERROR_TRIM;

    float wallCorr = (wL || wR) ? wallPid.compute(wallErr) : 0.0f;

    // ── Encoder straight-keeping ──────────────────────────────────────────────
    // ticks traveled within the current cell for each wheel
    long tL = encLeft.getTicks()  - cellBaseL;
    long tR = encRight.getTicks() - cellBaseR;

    // Positive encErr → left ran further → robot drifted right → slow left / speed right
    float encErr  = (float)(tL - tR);
    float encCorr = encPid.compute(encErr);

    // ── Cell boundary ─────────────────────────────────────────────────────────
    long avgTicks = (tL + tR) / 2;
    if (avgTicks >= TICKS_PER_CELL) {
        cellCount++;
        Serial.printf("[ENC-WALL] cell %ld/%d complete  encL=%ld  encR=%ld  diff=%ld\n",
                      cellCount, MAX_CELLS, tL, tR, tL - tR);

        stopMotors();
        delay(CELL_PAUSE_MS);

        if (cellCount >= MAX_CELLS) {
            running = false;
            Serial.println("[ENC-WALL] all cells done — stopped");
            return;
        }

        // Resume next cell
        resetCellBase();
        wallPid.reset();
        encPid.reset();
    }

    // ── Motor output ──────────────────────────────────────────────────────────
    // Positive correction → left slows, right speeds (steer left to correct right drift)
    float totalCorr = wallCorr + encCorr;
    int pwmL = constrain(BASE_PWM - (int)totalCorr, -1023, 1023);
    int pwmR = constrain(BASE_PWM + (int)totalCorr, -1023, 1023);

    leftMotor.drive(pwmL);
    rightMotor.drive(pwmR);
}
