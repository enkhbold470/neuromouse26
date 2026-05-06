#include <Arduino.h>
#include <FastLED.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseMaze.h"
#include "PID.h"

// ── Globals ──────────────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoder encLeft   (ENC_L_A, ENC_L_B);
MicromouseEncoder encRight  (ENC_R_A, ENC_R_B);
MicromouseMaze    maze;

#define NUM_LEDS 8
CRGB leds[NUM_LEDS];

void IRAM_ATTR isrLeft()  { encLeft.handleInterrupt();  }
void IRAM_ATTR isrRight() { encRight.handleInterrupt(); }

void IRAM_ATTR MicromouseEncoder::handleInterrupt() {
    if (digitalRead(pinB)) count++; else count--;
}

static const uint8_t EMIT_PINS[4] = { EMIT_LF, EMIT_L45, EMIT_R45, EMIT_RF };
static const uint8_t RX_PINS[4]   = { RX_LF,   RX_L45,   RX_R45,   RX_RF  };

#define IR_LF  0
#define IR_L45 1
#define IR_R45 2
#define IR_RF  3

PID wallPid(WALL_KP, WALL_KI, WALL_KD, (float)WALL_MAX_CORR);
PID encPid (ENC_KP,  ENC_KI,  ENC_KD,  (float)ENC_MAX_CORR);

enum State { IDLE, RUN, GOAL, STOP };
State robotState = IDLE;

uint8_t robotRow = 0;
uint8_t robotCol = 0;
AbsDir  robotHeading = DIR_NORTH;

// ── Hardware Helpers ─────────────────────────────────────────────────────────
int irRead(int idx) {
    int amb = analogRead(RX_PINS[idx]);
    digitalWrite(EMIT_PINS[idx], HIGH);
    delayMicroseconds(50);
    int lit = analogRead(RX_PINS[idx]);
    digitalWrite(EMIT_PINS[idx], LOW);
    return max(0, lit - amb);
}

void beep(int ms) {
    tone(BUZZER_PIN, BUZZER_FREQ, ms);
    delay(ms);
}

void beepGoal() { for (int i = 0; i < 3; i++) { beep(120); delay(100); } }

bool buttonEdge() {
    static bool last = HIGH;
    bool cur  = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && cur == LOW);
    last = cur;
    return edge;
}

void stopMotors() {
    leftMotor.brake(); rightMotor.brake();
    delay(100); 
    leftMotor.coast(); rightMotor.coast();
}

void updateBatteryIndicator() {
    int raw = analogRead(BAT_V_SENSE);
    float vBat = (raw / 4095.0f) * 3.3f * BAT_VDIV_MULT;
    
    // Physics: 2S LiPo range 6.8V (Dead) to 8.4V (Full)
    float pct = (vBat - 6.8f) / (8.4f - 6.8f);
    pct = constrain(pct, 0.0f, 1.0f);
    
    // LED 7 (Index 6) Green (Full) -> Red (Low)
    leds[6] = CRGB(255 * (1.0f - pct), 255 * pct, 0);
    FastLED.show();
}

// ── Movement with Physics ────────────────────────────────────────────────────
void driveWithRamp(int targetPWM, long targetTicks, bool forward) {
    long startTicksL = encLeft.getTicks();
    long currentTicks = 0;
    int currentPWM = 30; // Start slightly higher to ensure movement
    
    wallPid.reset();
    encPid.reset();

    while (currentTicks < targetTicks) {
        currentTicks = abs(encLeft.getTicks() - startTicksL);
        
        if (currentPWM < targetPWM) currentPWM += 2;
        if (currentTicks > (targetTicks * 3 / 4)) {
            if (currentPWM > 40) currentPWM -= 4;
        }

        if (forward && (irRead(IR_LF) > LF_THRESH || irRead(IR_RF) > RF_THRESH)) break;

        float wallCorr = 0;
        if (forward) {
            int l45 = irRead(IR_L45);
            int r45 = irRead(IR_R45);
            if (l45 > L45_THRESH && r45 > R45_THRESH) {
                wallCorr = wallPid.compute((float)(l45 - L45_CENTER) - (float)(r45 - R45_CENTER));
            } else if (l45 > L45_THRESH) {
                wallCorr = wallPid.compute((float)(l45 - L45_CENTER) * 2.0f);
            } else if (r45 > R45_THRESH) {
                wallCorr = wallPid.compute((float)-(r45 - R45_CENTER) * 2.0f);
            }
        }

        int dir = forward ? 1 : -1;
        leftMotor.drive(dir * (currentPWM - (int)wallCorr));
        rightMotor.drive(dir * (currentPWM + (int)wallCorr));
        
        static unsigned long lastBatt = 0;
        if (millis() - lastBatt > 500) { lastBatt = millis(); updateBatteryIndicator(); }
        delay(10);
    }
    stopMotors();
}

void turnRight() {
    leftMotor.drive(TURN_PWM);
    rightMotor.drive(-TURN_PWM);
    long startL = encLeft.getTicks();
    unsigned long startMs = millis();
    while (abs(encLeft.getTicks() - startL) < TICKS_PER_90) { 
        if (millis() - startMs > 2000) break;
        delay(1); 
    }
    stopMotors();
    robotHeading = (AbsDir)(((int)robotHeading + 1) % 4);
}

void turnLeft() {
    leftMotor.drive(-TURN_PWM);
    rightMotor.drive(TURN_PWM);
    long startL = encLeft.getTicks();
    unsigned long startMs = millis();
    while (abs(encLeft.getTicks() - startL) < TICKS_PER_90) { 
        if (millis() - startMs > 2000) break;
        delay(1); 
    }
    stopMotors();
    robotHeading = (AbsDir)(((int)robotHeading + 3) % 4);
}

void turnAround() { turnRight(); delay(100); turnRight(); }

void moveForwardOneCell() {
    driveWithRamp(BASE_PWM, TICKS_PER_CELL, true);
    robotRow += DIR_DR[robotHeading];
    robotCol += DIR_DC[robotHeading];
}

// ── Logic ────────────────────────────────────────────────────────────────────
void senseWalls() {
    if (irRead(IR_LF) > LF_THRESH || irRead(IR_RF) > RF_THRESH)
        maze.setWall(robotRow, robotCol, robotHeading, true);
    
    AbsDir left = (AbsDir)(((int)robotHeading + 3) % 4);
    if (irRead(IR_L45) > L45_THRESH)
        maze.setWall(robotRow, robotCol, left, true);
    
    AbsDir right = (AbsDir)(((int)robotHeading + 1) % 4);
    if (irRead(IR_R45) > R45_THRESH)
        maze.setWall(robotRow, robotCol, right, true);
}

void setup() {
    Serial.begin(115200);
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);
    
    FastLED.addLeds<WS2812B, WS2812_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(20);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    leftMotor.begin();
    rightMotor.begin();
    encLeft.begin(isrLeft);
    encRight.begin(isrRight);
    
    for (int i=0; i<4; i++) {
        pinMode(EMIT_PINS[i], OUTPUT);
        digitalWrite(EMIT_PINS[i], LOW);
    }
    
    pinMode(BUTTON_1, INPUT_PULLUP);
    
    // Startup Diagnostic: Pulse each motor FORWARD
    beep(100);
    updateBatteryIndicator();
    Serial.println("DIAGNOSTIC: Left Motor Forward...");
    leftMotor.drive(40); delay(200); leftMotor.coast();
    delay(500);
    Serial.println("DIAGNOSTIC: Right Motor Forward...");
    rightMotor.drive(40); delay(200); rightMotor.coast();
    
    maze.reset();
}

void loop() {
    updateBatteryIndicator();
    switch (robotState) {
        case IDLE:
            if (buttonEdge()) {
                beep(200);
                delay(1000);
                robotState = RUN;
            }
            break;
            
        case RUN: {
            maze.visited[robotRow][robotCol] = true;
            if (maze.isGoal(robotRow, robotCol)) {
                robotState = GOAL;
                break;
            }
            senseWalls();
            maze.floodFill();
            uint8_t d;
            AbsDir next = maze.bestDirectionBiased(robotRow, robotCol, robotHeading, d);
            int turn = ((int)next - (int)robotHeading + 4) % 4;
            if (turn == 1) turnRight();
            else if (turn == 2) turnAround();
            else if (turn == 3) turnLeft();
            moveForwardOneCell();
            delay(CELL_PAUSE_MS);
            break;
        }
        case GOAL: beepGoal(); robotState = STOP; break;
        case STOP:
            if (buttonEdge()) {
                robotRow = 0; robotCol = 0;
                robotHeading = DIR_NORTH;
                maze.reset();
                robotState = IDLE;
            }
            break;
    }
}
