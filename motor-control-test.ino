#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseIMU.h"
#include "PinConfig.h"

// Instantiate hardware using PinConfig.h definitions
MicromouseEncoder leftEnc(ENC_L_A, ENC_L_B);
MicromouseMotor leftMotor(MOTOR_L_IN1, MOTOR_L_IN2);
MicromouseIMU imu;

// Callback wrappers for interrupts
void IRAM_ATTR leftISR() { leftEnc.handleInterrupt(); }

// --- PID Control Variables ---
float Kp = 0.2;  // Proportional gain (reduced drastically to stop shaking)
float Ki = 0.05; // Integral gain
float Kd = 0.0;  // Derivative gain (Disabled: causes massive spikes due to tick quantization noise)

float target_speed = 500.0; // Desired ticks per second
float current_speed = 0.0;  // Measured ticks per second

// State tracking for PID
long prev_ticks = 0;
unsigned long prev_time = 0;
float integral = 0.0;
float prev_error = 0.0;
const unsigned long PID_INTERVAL = 20000; // 20ms (50Hz) to collect more ticks per loop

void setup() {
    Serial.begin(115200);

    // Wake up motor driver
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    // Initialize subsystems
    leftMotor.begin();
    leftEnc.begin(leftISR);
    imu.begin(IMU_SDA, IMU_SCL);

    Serial.println("Sensors & Motors Online");
    prev_time = micros();
}

void loop() {
    // 1. Keep IMU updated
    imu.update(); 
    
    // 2. Fixed-Interval PID Loop (100Hz)
    unsigned long current_time = micros();
    if (current_time - prev_time >= PID_INTERVAL) {
        // Calculate dynamic dt (should be close to 0.010s)
        float dt = (current_time - prev_time) / 1000000.0;
        prev_time = current_time;

        // Estimate current speed (ticks per sec)
        long current_ticks = leftEnc.getTicks();
        float raw_speed = (float)(current_ticks - prev_ticks) / dt;
        prev_ticks = current_ticks;

        // Apply Low-Pass Filter to smooth out quantization noise from low encoder resolution
        current_speed = (0.7 * current_speed) + (0.3 * raw_speed);

        // PID Error math
        float error = target_speed - current_speed;
        
        // Integral with windup clamping
        integral += error * dt;
        integral = constrain(integral, -1000, 1000); // Adjust bounds as needed
        
        // Derivative
        float derivative = (error - prev_error) / dt;
        prev_error = error;

        // Compute control signal
        float control_signal = (Kp * error) + (Ki * integral) + (Kd * derivative);

        // Apply to motor
        leftMotor.drive((int)control_signal);
    }

    // 3. Serial Telemetry (Non-blocking, ~20Hz)
    static unsigned long last_print = 0;
    if (millis() - last_print >= 50) {
        last_print = millis();
        // Format works beautifully in Arduino Serial Plotter
        Serial.printf("Target:%.2f Speed:%.2f Yaw:%.2f\n", target_speed, current_speed, imu.getYaw());
    }
}