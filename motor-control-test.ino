#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseIMU.h"
#include "PinConfig.h"

// --- Hardware Objects ---
MicromouseEncoder leftEnc(ENC_L_A, ENC_L_B);
MicromouseMotor   leftMotor(MOTOR_L_IN1, MOTOR_L_IN2);
MicromouseIMU     imu;

// ISR wrapper — must be a free function, calls into the encoder object
void IRAM_ATTR leftISR() { leftEnc.handleInterrupt(); }

// ---------------------------------------------------------------------------
// PID Configuration
// ---------------------------------------------------------------------------
// Speed unit: ticks/second (single-channel RISING encoder)
// Motor free-run at 6V ≈ 500 RPM → 500/60 * 210 ticks/rev ≈ 1750 ticks/sec
// Start with a modest target well below free-run to leave PID headroom.
// ---------------------------------------------------------------------------
float Kp = 1.5;   // Tuning start point. Raise until motor responds, back off if it oscillates.
float Ki = 0.8;   // Removes steady-state error. Raise slowly after Kp is set.
float Kd = 0.05;  // Light derivative to dampen overshoot. Keep low due to encoder quantization.

float target_speed = 800.0; // ticks/sec  (~23% of free-run — safe starting point)

// --- PID State ---
float         current_speed = 0.0;
long          prev_ticks    = 0;
unsigned long prev_time     = 0;
float         integral      = 0.0;
float         prev_error    = 0.0;

// Integral windup clamp: max integral contribution = MAX_PWM / Ki
// This means integral can never push output beyond ±1023 on its own.
const float INTEGRAL_MAX = 1023.0f / 0.8f; // ≈ 1278 — recalculate if you change Ki

const unsigned long PID_INTERVAL_US = 20000; // 20 ms → 50 Hz

void setup() {
    Serial.begin(115200);

    // Wake up DRV8833 driver (active HIGH)
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    leftMotor.begin();
    leftEnc.begin(leftISR);
    imu.begin(IMU_SDA, IMU_SCL);

    prev_time = micros();
    Serial.println("=== Motor PID Online ===");
    Serial.println("Target(t/s)\tSpeed(t/s)\tYaw");
}

void loop() {
    // 1. Keep IMU integrated
    imu.update();

    // 2. Fixed-interval PID (50 Hz)
    unsigned long now = micros();
    if (now - prev_time >= PID_INTERVAL_US) {
        float dt = (float)(now - prev_time) / 1000000.0f;
        prev_time = now;

        // --- Measure speed ---
        long  ticks = leftEnc.getTicks();
        float raw_speed = (float)(ticks - prev_ticks) / dt;
        prev_ticks = ticks;

        // Low-pass filter to smooth encoder quantization noise (α=0.5 — balanced)
        current_speed = (0.5f * current_speed) + (0.5f * raw_speed);

        // --- PID ---
        float error = target_speed - current_speed;

        integral  += error * dt;
        integral   = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

        float derivative = (error - prev_error) / dt;
        prev_error = error;

        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        leftMotor.drive((int)output);
    }

    // 3. Serial telemetry (tab-separated for Serial Plotter, ~20 Hz)
    static unsigned long last_print = 0;
    if (millis() - last_print >= 50) {
        last_print = millis();
        Serial.printf("%.1f\t%.1f\t%.1f\n", target_speed, current_speed, imu.getYaw());
    }
}
