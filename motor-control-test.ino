#include "MicromouseMotor.h"
// 1. Define your locked-in pins
#define MOTOR1_IN1 15
#define MOTOR1_IN2 16
#define MOTOR2_IN3 17
#define MOTOR2_IN4 18
#define DRV_SLEEP_PIN 41

// 2. Instantiate the classes
MicromouseMotor leftMotor(MOTOR1_IN1, MOTOR1_IN2);
MicromouseMotor rightMotor(MOTOR2_IN3, MOTOR2_IN4);

void setup() {
    // 3. Wake up the DRV8833 driver
    Serial.begin(115200);
    Serial.println("START...");
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH); // HIGH = Awake, LOW = Sleep

    // 4. Initialize the hardware PWM timers
    leftMotor.begin();
    rightMotor.begin();
    
    // Optional delay to let hardware settle
    delay(10);
}

void loop() {
    // --- Example PID Implementation ---
    // Let's assume your PID calculates we need to steer right.
    // Base speed is 500, steering correction is 150.
    
    // int base_speed = 500;
    // int turn_correction = 150;
    
    // int left_pwm = base_speed + turn_correction;  // 650
    // int right_pwm = base_speed - turn_correction; // 350
    
    leftMotor.drive(1023);
    // rightMotor.drive(right_pwm);
    
    // To instantly stop at a wall:
    // leftMotor.brake();
    // rightMotor.brake();
}