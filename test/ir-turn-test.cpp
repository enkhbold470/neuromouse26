// ir-turn-test.cpp — drive straight, turn on wall openings
//
// Logic per cell boundary:
//   right opening (no R wall) → turn right
//   left  opening (no L wall) AND right wall present → turn left
//   else  → go straight
//
// Sensors used every loop tick:
//   IR  — wall detection + yaw-less centering when both walls present
//   Gyro — yaw hold during straight drive + pivot turns
//   Enc  — speed PID + straight-line PI
//
// Flash: pio run -e ir-turn-test -t upload
// Button hold: start / stop

#include <Arduino.h>
#include <Wire.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"

// ── Hardware ─────────────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ── IR ───────────────────────────────────────────────────────────────────────
struct IRPair { uint8_t emit, rx; };
static const IRPair PAIRS[4] = {
    { EMIT_LF, RX_LF },   // 0 = LF
    { EMIT_L,  RX_L  },   // 1 = L
    { EMIT_R,  RX_R  },   // 2 = R
    { EMIT_RF, RX_RF },   // 3 = RF
};
static int irVal[4];

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

static inline bool wallFront() { return irVal[0] > WALL_FRONT_THRESH || irVal[3] > WALL_FRONT_THRESH; }
static inline bool wallLeft()  { return irVal[1] > WALL_SIDE_THRESH; }
static inline bool wallRight() { return irVal[2] > WALL_SIDE_THRESH; }

// ── Gyro (MPU-6500) ──────────────────────────────────────────────────────────
#define MPU_ADDR       0x68
#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_CFG   0x1B
#define GYRO_FS_SEL    0x10   // ±1000 dps
#define GYRO_SCALE     32.8f  // LSB/(dps) for ±1000 dps range

static float gyroBias = 0.0f;
static float yaw      = 0.0f;
static unsigned long lastGyroUs = 0;

static bool mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}

static float readGz() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);  // GYRO_ZOUT_H
    if (Wire.endTransmission(false) != 0) return 0;
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return 0;
    int16_t raw = (Wire.read() << 8) | Wire.read();
    return raw / GYRO_SCALE;
}

static void updateYaw() {
    float gz = readGz() - gyroBias;
    if (fabsf(gz) < 0.3f) gz = 0;
    unsigned long now = micros();
    float dt = (lastGyroUs == 0) ? 0.001f : (now - lastGyroUs) / 1e6f;
    if (dt > 0.05f) dt = 0.05f;
    lastGyroUs = now;
    yaw += gz * dt;
}

static void calibrateGyro() {
    Serial.print("[CAL] gyro bias ");
    const int N = 300;
    float sum = 0;
    for (int i = 0; i < N; i++) { sum += readGz(); delay(2); }
    gyroBias = sum / N;
    yaw = 0; lastGyroUs = micros();
    Serial.printf("%.3f dps\n", gyroBias);
}

// ── Motion primitives ─────────────────────────────────────────────────────────
static void stopMotors() { leftMotor.brake(); rightMotor.brake(); }

// Straight drive: velocity PID + encoder straight PI + gyro yaw hold.
// Runs for `ticks` encoder counts (average L+R), then stops.
static void driveTicks(long ticks, int targetMms = 220) {
    leftEnc.reset(); rightEnc.reset();
    yaw = 0; lastGyroUs = micros();

    // Velocity PID state
    float intSpeed = 0, velL_ema = 0, velR_ema = 0;
    long  prevL = 0, prevR = 0;
    float vbat  = analogRead(BAT_V_SENSE) / 4095.0f * 3.3f * BAT_VDIV_MULT;
    float vScale = NOMINAL_VBAT / (vbat > 5.5f ? vbat : NOMINAL_VBAT);
    float kV_avg  = 0.5f * (KV_L + KV_R);
    float off_avg = 0.5f * (OFF_L + OFF_R);

    constexpr float YAW_KP  = 4.0f;   // pwm per degree
    constexpr int   YAW_MAX = 150;

    unsigned long nextUs = micros();
    unsigned long t0     = millis();

    while (true) {
        while ((long)(micros() - nextUs) < 0) {}
        nextUs += VPID_LOOP_US;

        updateYaw();
        sampleIR();

        long curL = leftEnc.getTicks();
        long curR = rTicks();
        long avg  = (curL + curR) / 2;

        // Stop conditions
        bool frontCrash = wallFront() && irVal[0] + irVal[3] > 2000;
        if (avg >= ticks || frontCrash || millis() - t0 > 5000) break;

        // Velocity PID
        float dt      = VPID_LOOP_US / 1000000.0f;
        float instL   = (curL - prevL) * MM_PER_TICK / dt;
        float instR   = (curR - prevR) * MM_PER_TICK / dt;
        prevL = curL; prevR = curR;
        velL_ema += VPID_EMA_ALPHA * (instL - velL_ema);
        velR_ema += VPID_EMA_ALPHA * (instR - velR_ema);

        float velAvg   = 0.5f * (velL_ema + velR_ema);
        float errSpeed = (float)targetMms - velAvg;
        intSpeed = constrain(intSpeed + errSpeed * dt, -VPID_INTEG_LIM, VPID_INTEG_LIM);
        int pidSpeed = (int)(VPID_LOOP_KP * errSpeed + VPID_LOOP_KI * intSpeed);

        // Straight PI from encoder mismatch
        long straightErr = curL - curR;
        int pidStr = (int)constrain((float)(VPID_STRAIGHT_KP * straightErr),
                                    (float)-VPID_STRAIGHT_MAX, (float)VPID_STRAIGHT_MAX);

        // Gyro yaw hold (deadband ±1°)
        float yawErr = fabsf(yaw) > 1.0f ? yaw : 0.0f;
        int yawBias  = (int)constrain(YAW_KP * yawErr, (float)-YAW_MAX, (float)YAW_MAX);

        // IR centering — only when both walls present
        int irBias = 0;
        if (wallLeft() && wallRight()) {
            int posErr = (irVal[1] - IR_CAL_L) - (irVal[2] - IR_CAL_R);
            irBias = (int)constrain(0.04f * (float)posErr, -80.0f, 80.0f);
        }

        int ff      = (int)((kV_avg > 0 ? (targetMms / kV_avg) + off_avg : 0) * vScale);
        int base    = ff + pidSpeed;
        int lateral = constrain(yawBias + irBias, -200, 200);
        int pwmL    = constrain(base - pidStr + L_PWM_BIAS + lateral, 0, MOTOR_PWM_MAX);
        int pwmR    = constrain(base + pidStr + R_PWM_BIAS - lateral, 0, MOTOR_PWM_MAX);

        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        Serial.printf("avg=%ld vL=%.0f vR=%.0f pL=%d pR=%d yaw=%.1f str=%ld ir=%d\n",
                      avg, velL_ema, velR_ema, pwmL, pwmR, yaw, straightErr, irBias);
    }
    stopMotors();
    delay(100);
}

// Gyro-based pivot turn. deg>0=left, deg<0=right.
static void doTurn(float targetDeg) {
    yaw = 0; lastGyroUs = micros();
    int dir = (targetDeg > 0) ? 1 : -1;

    constexpr float PEAK_DPS    = 300.0f;
    constexpr float ACCEL_DPS2  = 1500.0f;
    constexpr float KFF         = 0.9f;    // pwm per dps
    constexpr float KP          = 8.0f;    // pwm per degree error
    constexpr float KD          = 0.5f;    // pwm per dps
    constexpr float DECEL_DEG   = 30.0f;   // start slowing this many deg before target

    float omega     = 0;
    float prevErr   = targetDeg;
    unsigned long prevUs = micros();
    unsigned long t0     = millis();

    while (millis() - t0 < 3000) {
        unsigned long now = micros();
        float dt = (now - prevUs) / 1e6f;
        prevUs = now;

        updateYaw();
        float remaining = targetDeg - yaw;
        if (fabsf(remaining) < 1.0f) break;

        // Trapezoid omega profile
        float absRem  = fabsf(remaining);
        float omegaTgt = (absRem < DECEL_DEG)
            ? PEAK_DPS * absRem / DECEL_DEG
            : PEAK_DPS;
        omegaTgt = constrain(omegaTgt, 40.0f, PEAK_DPS);

        // Simple accel ramp
        float dOmega = ACCEL_DPS2 * dt;
        if (omega < omegaTgt) omega = min(omega + dOmega, omegaTgt);
        else                  omega = max(omega - dOmega, omegaTgt);

        float errDeg = remaining;
        float dErr   = (errDeg - prevErr) / dt;
        prevErr = errDeg;

        int pwm = (int)(KFF * omega + KP * errDeg + KD * dErr);
        pwm = constrain(abs(pwm), 60, TURN_PWM);

        leftMotor.drive(-dir * pwm);
        rightMotor.drive( dir * pwm);

        Serial.printf("[TURN] yaw=%.1f rem=%.1f omega=%.0f pwm=%d\n",
                      yaw, remaining, omega, pwm);
    }
    stopMotors();
    delay(200);
    updateYaw();
    Serial.printf("[TURN] done: yaw=%.1f target=%.1f err=%.1f\n",
                  yaw, targetDeg, targetDeg - yaw);
    yaw = 0; lastGyroUs = micros();
}

// ── Button ────────────────────────────────────────────────────────────────────
static bool buttonPressed() {
    static bool last = HIGH;
    bool cur = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && cur == LOW);
    last = cur;
    return edge;
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────
static bool running = false;

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
    mpuWrite(REG_PWR_MGMT_1, 0x00); delay(50);
    mpuWrite(REG_GYRO_CFG, GYRO_FS_SEL);
    lastGyroUs = micros();

    calibrateGyro();
    Serial.println("[READY] hold button to start/stop");
}

void loop() {
    delay(10);

    if (buttonPressed()) {
        running = !running;
        if (!running) { stopMotors(); Serial.println("[STOP]"); return; }
        Serial.println("[START]");
        delay(1000);
    }

    if (!running) return;

    // ── One cell cycle ────────────────────────────────────────────────────────
    // Drive one full cell, sampling IR at each control tick.
    // At cell end, decide: turn right, turn left, or continue straight.

    driveTicks(TICKS_PER_CELL);

    if (!running) return;

    // Read walls after stopping at cell center
    sampleIR();
    bool wL = wallLeft();
    bool wR = wallRight();
    bool wF = wallFront();

    Serial.printf("[CELL] wF=%d wL=%d wR=%d IR: %d %d %d %d\n",
                  wF, wL, wR, irVal[0], irVal[1], irVal[2], irVal[3]);

    // Decision: right turn preferred, else left, else straight, else dead-end
    if (!wR) {
        Serial.println("[TURN] right (opening on right)");
        doTurn(-90.0f);
    } else if (!wL) {
        Serial.println("[TURN] left (opening on left)");
        doTurn(90.0f);
    } else if (!wF) {
        Serial.println("[STRAIGHT] continuing");
        // no turn needed, just keep going
    } else {
        // Dead end — turn around
        Serial.println("[TURN] dead end, 180");
        doTurn(90.0f);
        doTurn(90.0f);
    }
}
