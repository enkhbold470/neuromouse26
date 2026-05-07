#include <Arduino.h>
#include <FastLED.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseMaze.h"
#include "PID.h"
#include "WifiDebug.h"

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

TuningConfig tuning;
PID wallPid(tuning.wallKp, tuning.wallKi, tuning.wallKd, (float)tuning.wallMaxCorr);
PID encPid (tuning.encKp,  tuning.encKi,  tuning.encKd,  (float)tuning.encMaxCorr);

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
    
    // Physics: 2S LiPo range 6.8V (Dead) to 8.2V (100%)
    float pct = (vBat - 6.8f) / (8.2f - 6.8f);
    pct = constrain(pct, 0.0f, 1.0f);
    
    // LED 7 (Index 6) Green (Full) -> Red (Low)
    leds[6] = CRGB(255 * (1.0f - pct), 255 * pct, 0);
    FastLED.show();
}

// ── Encoder Drive ─────────────────────────────────────────────────────────────
// Resets both encoders, drives until avg abs ticks >= targetTicks or 4s timeout.
// abs() on ticks works regardless of pinB wiring direction.
void driveEncoder(long targetTicks, int pwm, bool forward) {
    encLeft.reset();
    encRight.reset();
    encPid.reset();
    unsigned long startMs = millis();
    unsigned long lastLog = 0;
    int dir = forward ? 1 : -1;

    Serial.printf("[ENC] START target=%ld pwm=%d\n", targetTicks, pwm);

    while (true) {
        long tL     = abs(encLeft.getTicks());
        long tR     = abs(encRight.getTicks());
        long traveled = (tL + tR) / 2;

        if (millis() - lastLog > 150) {
            lastLog = millis();
            Serial.printf("[ENC] tL=%ld tR=%ld traveled=%ld\n", tL, tR, traveled);
        }

        if (traveled >= targetTicks) { Serial.println("[ENC] DONE"); break; }

        if (millis() - startMs > 4000) {
            Serial.printf("[ENC] TIMEOUT tL=%ld tR=%ld — check encoder wiring / PWM\n", tL, tR);
            break;
        }

        float corr   = encPid.compute((float)(tL - tR));
        int leftPwm  = constrain(pwm - (int)corr, 25, 255);
        int rightPwm = constrain(pwm + (int)corr, 25, 255);
        leftMotor.drive(dir * leftPwm);
        rightMotor.drive(dir * rightPwm);
        delay(5);
    }
    stopMotors();
}

void turnRight() {
    leftMotor.drive(tuning.turnPwm);
    rightMotor.drive(-tuning.turnPwm);
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
    leftMotor.drive(-tuning.turnPwm);
    rightMotor.drive(tuning.turnPwm);
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
    driveEncoder(tuning.ticksPerCell, tuning.basePwm, true);
    robotRow += DIR_DR[robotHeading];
    robotCol += DIR_DC[robotHeading];
}

// ── Logic ────────────────────────────────────────────────────────────────────
void senseWalls() {
    if (irRead(IR_LF) > tuning.lfThresh || irRead(IR_RF) > tuning.rfThresh)
        maze.setWall(robotRow, robotCol, robotHeading, true);

    AbsDir left = (AbsDir)(((int)robotHeading + 3) % 4);
    if (irRead(IR_L45) > tuning.l45Thresh)
        maze.setWall(robotRow, robotCol, left, true);

    AbsDir right = (AbsDir)(((int)robotHeading + 1) % 4);
    if (irRead(IR_R45) > tuning.r45Thresh)
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
    leftMotor.drive(100); delay(300); leftMotor.coast();
    delay(400);
    Serial.println("DIAGNOSTIC: Right Motor Forward...");
    rightMotor.drive(100); delay(300); rightMotor.coast();
    
    maze.reset();
    // Apply default 3x6 practice maze boundaries
    for (int c = 0; c < MAZE_SIZE; c++) maze.setWall(tuning.mazeRows - 1, c, DIR_NORTH, true);
    for (int r = 0; r < MAZE_SIZE; r++) maze.setWall(r, tuning.mazeCols - 1, DIR_EAST,  true);
    maze.setGoalSingle(tuning.goalRow, tuning.goalCol);
    maze.floodFill();
    wifiDebugSetIRReader(irRead);
    wifiDebugInit(&tuning);
}

void loop() {
    updateBatteryIndicator();
    wifiDebugHandle();

    // push snapshot for WiFi debug page
    {
        int raw = analogRead(BAT_V_SENSE);
        float vb = (raw / 4095.0f) * 3.3f * BAT_VDIV_MULT;
        DebugSnapshot snap;
        snap.ir[0]   = irRead(IR_LF);
        snap.ir[1]   = irRead(IR_L45);
        snap.ir[2]   = irRead(IR_R45);
        snap.ir[3]   = irRead(IR_RF);
        snap.encL    = encLeft.getTicks();
        snap.encR    = encRight.getTicks();
        snap.vBat    = vb;
        snap.state   = (int)robotState;
        snap.row     = robotRow;
        snap.col     = robotCol;
        snap.heading = (int)robotHeading;
        snap.wallF   = snap.ir[0] > tuning.lfThresh || snap.ir[3] > tuning.rfThresh;
        snap.wallL   = snap.ir[1] > tuning.l45Thresh;
        snap.wallR   = snap.ir[2] > tuning.r45Thresh;
        memcpy(snap.mazeWalls, maze.walls, sizeof(maze.walls));
        memcpy(snap.mazeFlood, maze.flood, sizeof(maze.flood));
        for (int r = 0; r < MAZE_SIZE; r++)
            for (int c = 0; c < MAZE_SIZE; c++)
                snap.mazeVisited[r][c] = maze.visited[r][c] ? 1 : 0;
        snap.mazeRows = tuning.mazeRows;
        snap.mazeCols = tuning.mazeCols;
        wifiDebugUpdate(snap);
    }

    if (wifiDebugConfigChanged()) {
        wallPid.kp = tuning.wallKp; wallPid.ki = tuning.wallKi;
        wallPid.kd = tuning.wallKd; wallPid.maxOut = (float)tuning.wallMaxCorr;
        wallPid.reset();
        encPid.kp  = tuning.encKp;  encPid.ki  = tuning.encKi;
        encPid.kd  = tuning.encKd;  encPid.maxOut  = (float)tuning.encMaxCorr;
        encPid.reset();
        maze.reset();
        if (tuning.mazeRows < MAZE_SIZE) {
            for (int c = 0; c < MAZE_SIZE; c++)
                maze.setWall(tuning.mazeRows - 1, c, DIR_NORTH, true);
        }
        if (tuning.mazeCols < MAZE_SIZE) {
            for (int r = 0; r < MAZE_SIZE; r++)
                maze.setWall(r, tuning.mazeCols - 1, DIR_EAST, true);
        }
        maze.setGoalSingle(tuning.goalRow, tuning.goalCol);
        maze.floodFill();
        Serial.printf("[Config] Updated: basePwm=%d wallKp=%.3f mazeRows=%d mazeCols=%d goal=(%d,%d)\n",
            tuning.basePwm, tuning.wallKp, tuning.mazeRows, tuning.mazeCols, tuning.goalRow, tuning.goalCol);
    }
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
            delay(tuning.cellPauseMs);
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
