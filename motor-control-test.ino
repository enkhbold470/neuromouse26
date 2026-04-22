// --- Encoder Test for Motor 1 ---
#include "MicromouseMotor.h"

// Pin Definitions
#define MOTOR1_IN1 15
#define MOTOR1_IN2 16
#define ENC1_A 14
#define ENC1_B 21
#define DRV_SLEEP_PIN 41

MicromouseMotor motor(MOTOR1_IN1, MOTOR1_IN2);

// Volatile variables for Interrupt Service Routine (ISR)
volatile long pulseCount = 0;
long lastTime = 0;
double currentSpeed = 0; // Pulses per second

// ISR: This runs every time Phase A changes state
void IRAM_ATTR handleEncoder() {
    // Check Phase B to determine direction
    if (digitalRead(ENC1_B) == HIGH) {
        pulseCount++;
    } else {
        pulseCount--;
    }
}

void setup() {
    Serial.begin(115200);
    
    // Enable Motor Driver
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);
    
    // Setup Encoder Pins
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    
    // Attach Interrupt to Phase A
    attachInterrupt(digitalPinToInterrupt(ENC1_A), handleEncoder, RISING);
    
    motor.begin();
    Serial.println("Encoder Test Initialized...");
}

void loop() {
    // Run motor at a steady test speedmotor.drive(1023); 
    // delay(10); // 10ms burst to break friction

    // motor.drive(1023); 
    // delay(10); // 10ms burst to break friction
    motor.drive(400); 

    // Every 100ms, calculate the speed
    if (millis() - lastTime >= 100) {
        noInterrupts(); // Temporarily disable to read volatile long safely
        long currentCount = pulseCount;
        pulseCount = 0; // Reset for next interval
        interrupts();

        // Speed in Pulses per Second
        currentSpeed = currentCount / 0.1; 

        Serial.print("Target PWM: 400 | Current Pulses/Sec: ");
        Serial.println(currentSpeed);
        
        lastTime = millis();
    }
}