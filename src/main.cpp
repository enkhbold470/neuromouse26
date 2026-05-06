#include <Arduino.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseMaze.h"
#include "PID.h"

// ── Globals ──────────────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3);
MicromouseEncoder encLeft   (ENC_L_A, ENC_L_B);
MicromouseEncoder encRight  (ENC_R_A, ENC_R_B);
MicromouseMaze    maze;

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
    delay(100); // Allow physical momentum to dissipate
    leftMotor.coast(); rightMotor.coast();
}

// ── Movement with Physics (Acceleration/Deceleration) ───────────────────────
void driveWithRamp(int targetPWM, long targetTicks, bool forward) {
    long startTicks = encLeft.getTicks();
    long currentTicks = 0;
    int currentPWM = 20; // Start just above stall to respect inertia
    
    wallPid.reset();
    encPid.reset();

    while (currentTicks < targetTicks) {
        currentTicks = abs(encLeft.getTicks() - startTicks);
        
        // Physics: Acceleration Ramp
        if (currentPWM < targetPWM) currentPWM += 5; // ramp up
        
        // Physics: Deceleration Ramp (start slowing down in last 25% of movement)
        if (currentTicks > (targetTicks * 3 / 4)) {
            if (currentPWM > 40) currentPWM -= 8; // ramp down
        }

        // Physics: Emergency Front Wall Stop
        if (forward && (irRead(IR_LF) > LF_THRESH || irRead(IR_RF) > RF_THRESH)) {
            break; // Stop immediately if we are about to hit a front wall
        }

        float wallCorr = 0;
        if (forward) {
            int l45 = irRead(IR_L45);
            int r45 = irRead(IR_R45);
            
            // Proportional Centering Physics:
            if (l45 > L45_THRESH && r45 > R45_THRESH) {
                // Both walls: Center between them
                wallCorr = wallPid.compute((float)(l45 - L45_CENTER) - (float)(r45 - R45_CENTER));
            } else if (l45 > L45_THRESH) {
                // Only left wall: Keep distance from it
                wallCorr = wallPid.compute((float)(l45 - L45_CENTER) * 2.0f);
            } else if (r45 > R45_THRESH) {
                // Only right wall: Keep distance from it
                wallCorr = wallPid.compute((float)-(r45 - R45_CENTER) * 2.0f);
            }
        }

        // Forward/Backward multiplier
        int dir = forward ? 1 : -1;
        leftMotor.drive(dir * (currentPWM - (int)wallCorr));
        rightMotor.drive(dir * (currentPWM + (int)wallCorr));
        delay(10);
    }
    stopMotors();
}

void turnRight() {
    leftMotor.drive(TURN_PWM);
    rightMotor.drive(-TURN_PWM);
    long startL = encLeft.getTicks();
    // 90 degree turn is a specific physical distance on the wheel track
    while (abs(encLeft.getTicks() - startL) < TICKS_PER_90) { delay(1); }
    stopMotors();
    robotHeading = (AbsDir)(((int)robotHeading + 1) % 4);
}

void turnLeft() {
    leftMotor.drive(-TURN_PWM);
    rightMotor.drive(TURN_PWM);
    long startL = encLeft.getTicks();
    while (abs(encLeft.getTicks() - startL) < TICKS_PER_90) { delay(1); }
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
    
    leftMotor.begin();
    rightMotor.begin();
    encLeft.begin(isrLeft);
    encRight.begin(isrRight);
    
    for (int i=0; i<4; i++) {
        pinMode(EMIT_PINS[i], OUTPUT);
        digitalWrite(EMIT_PINS[i], LOW);
    }
    
    pinMode(BUTTON_1, INPUT_PULLUP);
    beep(100);
    maze.reset();
}

void loop() {
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
            
        case GOAL:
            beepGoal();
            robotState = STOP;
            break;
            
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
