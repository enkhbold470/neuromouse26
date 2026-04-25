// =============================================================================
// main.cpp — Micromouse26 firmware
// ESP32-S3, PlatformIO + Arduino framework
//
// Robot Finite State Machine (FSM)
// ─────────────────────────────────
//   IDLE           Waiting for button press. All motors off.
//   CALIBRATE      Runs IMU gyro bias calibration (robot must be still).
//   EXPLORE        Flood-fill guided left-hand exploration of the maze.
//                  At each cell: read IR, update walls, re-flood, move toward goal.
//   GOAL_REACHED   Arrived at one of the goal cells. Briefly celebrate.
//   RETURN_HOME    Flood-fill from (0,0) back to start (not yet implemented).
//   STOP           Error or finished — motors off, loop prints status.
//
// Cell movement model
// ───────────────────
//   One "cell move" = drive forward one maze cell (18 cm standard cell).
//   One "turn"      = spin in place 90° using IMU yaw feedback.
//   Both are blocking functions with full verbose Serial logging.
//
// Telemetry format (every 100 ms, easy to log/plot)
//   [TELE] step=N  pos=(r,c)  heading=X  yaw=Y.Y°  L_ticks=N  R_ticks=N
//   [TELE] flood_here=N  best_dir=X  front=W  left=W  right=W
// =============================================================================

#include <Arduino.h>
#include "PinConfig.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseIMU.h"
#include "MicromouseIR.h"
#include "MicromouseMaze.h"

// ============================================================================
// Hardware object instantiation
// ============================================================================
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, "LEFT");
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, "RIGHT");
MicromouseEncoder leftEnc   (ENC_L_A,     ENC_L_B,     "LEFT");
MicromouseEncoder rightEnc  (ENC_R_A,     ENC_R_B,     "RIGHT");
MicromouseIMU     imu;
MicromouseIR      ir;
MicromouseMaze    maze;

// ISR wrappers — must be plain functions (no member calls via vtable in ISR)
void IRAM_ATTR leftISR()  { leftEnc.handleInterrupt();  }
void IRAM_ATTR rightISR() { rightEnc.handleInterrupt(); }

// ============================================================================
// Robot position and heading tracking
// ============================================================================
// Heading: absolute compass direction the robot's front faces.
//   0 = NORTH (positive row direction)
//   1 = EAST  (positive col direction)
//   2 = SOUTH (negative row direction)
//   3 = WEST  (negative col direction)
uint8_t robotRow     = 0;   // current cell row (0 = start row)
uint8_t robotCol     = 0;   // current cell col (0 = start col)
AbsDir  robotHeading = DIR_NORTH; // facing north at start

static const char* HEADING_NAME[4] = { "NORTH", "EAST", "SOUTH", "WEST" };

// Step counter — incremented after every cell traversal
uint32_t stepCount = 0;

// ============================================================================
// FSM state
// ============================================================================
enum RobotState : uint8_t {
    STATE_IDLE         = 0,
    STATE_CALIBRATE    = 1,
    STATE_EXPLORE      = 2,
    STATE_GOAL_REACHED = 3,
    STATE_RETURN_HOME  = 4,
    STATE_STOP         = 5
};
RobotState robotState = STATE_IDLE;

static const char* STATE_NAME[6] = {
    "IDLE", "CALIBRATE", "EXPLORE", "GOAL_REACHED", "RETURN_HOME", "STOP"
};

// ============================================================================
// PID parameters for straight-line speed control
// ============================================================================
// Targets are in ticks/second for each encoder.
// N20 free-run ≈ 1750 t/s at 6V. Use ~45% = 800 t/s for exploration.
static const float PID_TARGET_EXPLORE = 800.0f;   // t/s — exploration speed
static const float PID_TARGET_TURN    = 400.0f;   // t/s — turning speed (slower)

float Kp = 1.5f;
float Ki = 0.8f;
float Kd = 0.05f;

// Per-motor PID state
struct PIDState {
    float integral  = 0.0f;
    float prevError = 0.0f;
    long  prevTicks = 0;
};
PIDState pidLeft, pidRight;

static const unsigned long PID_INTERVAL_US = 20000; // 20 ms = 50 Hz

// ============================================================================
// Physical constants
// ============================================================================
// One standard maze cell = 180 mm.
// Ticks per cell = (CELL_MM / (PI * WHEEL_DIAMETER)) * TICKS_PER_REV
//   = (180 / (3.14159 * 32)) * 210
//   = (180 / 100.53) * 210
//   = 1.791 * 210
//   ≈ 376 ticks
static const float CELL_MM         = 180.0f;
static const float MM_PER_TICK     = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;
static const long  TICKS_PER_CELL  = (long)(CELL_MM / MM_PER_TICK);

// Turn 90°:
// Robot track width (distance between wheel contact patches) ≈ 74 mm.
// Arc length for 90° = (PI * TRACK_WIDTH) / 4
// Ticks for 90° = arc / MM_PER_TICK
static const float TRACK_WIDTH_MM  = 74.0f;  // measure your robot!
static const long  TICKS_PER_90DEG = (long)((PI * TRACK_WIDTH_MM / 4.0f) / MM_PER_TICK);

// IMU yaw tolerance: stop turning when abs(yaw) > YAW_TARGET - YAW_TOLERANCE
static const float YAW_90_TARGET   = 90.0f;
static const float YAW_TOLERANCE   = 2.0f;   // degrees

// ============================================================================
// Telemetry timer
// ============================================================================
unsigned long lastTelemetry = 0;
static const unsigned long TELE_INTERVAL_MS = 100;

// ============================================================================
// Forward declarations
// ============================================================================
void transitionTo(RobotState newState);
void doIdle();
void doCalibrate();
void doExplore();
void doGoalReached();
void doReturnHome();
void doStop();

void senseWallsAndUpdate();
void moveForwardOneCell();
void turnLeft();
void turnRight();
void turnAround();
float computePID(PIDState &pid, float target, long curTicks, float dt);
void printTelemetry();
void printRobotPosition();

// ============================================================================
// setup()
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);  // give USB-Serial time to enumerate

    Serial.println(F(""));
    Serial.println(F("============================================================="));
    Serial.println(F("  Micromouse26 firmware  —  verbose test build"));
    Serial.println(F("============================================================="));
    Serial.printf( "  Build date : %s %s\n", __DATE__, __TIME__);
    Serial.printf( "  CPU freq   : %lu MHz\n", (unsigned long)(F_CPU / 1000000));
    Serial.printf( "  TICKS/CELL : %ld\n",  TICKS_PER_CELL);
    Serial.printf( "  TICKS/90°  : %ld\n",  TICKS_PER_90DEG);
    Serial.printf( "  MM/TICK    : %.4f mm\n", MM_PER_TICK);
    Serial.println(F("============================================================="));

    // --- DRV8833 wake ---
    Serial.println(F("[INIT] Waking DRV8833 motor driver (DRV_SLEEP_PIN HIGH)"));
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    // --- Motors ---
    Serial.println(F("[INIT] Initialising motors"));
    leftMotor.begin();
    rightMotor.begin();

    // --- Encoders ---
    Serial.println(F("[INIT] Initialising encoders"));
    leftEnc.begin(leftISR);
    rightEnc.begin(rightISR);

    // --- IR sensors ---
    Serial.println(F("[INIT] Initialising IR sensor array"));
    ir.begin();

    // --- Button ---
    Serial.printf("[INIT] Button GPIO%d → INPUT_PULLUP\n", BUTTON_1);
    pinMode(BUTTON_1, INPUT_PULLUP);

    // --- Buzzer ---
    Serial.printf("[INIT] Buzzer GPIO%d → OUTPUT (LOW)\n", BUZZER_PIN);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // --- Maze ---
    Serial.println(F("[INIT] Initialising maze data structure"));
    maze.reset();

    // --- Initial flood-fill (open maze — no interior walls known yet) ---
    Serial.println(F("[INIT] Running initial flood-fill (empty maze)"));
    maze.floodFill();
    maze.printFlood();

    // --- IMU (last — takes ~400 ms to calibrate) ---
    Serial.println(F("[INIT] Initialising IMU (keep robot still for calibration!)"));
    imu.begin(IMU_SDA, IMU_SCL);

    Serial.println(F("[INIT] All hardware initialised"));
    Serial.println(F("[INIT] Entering IDLE state. Press BUTTON_1 (GPIO42) to start."));
    transitionTo(STATE_IDLE);
}

// ============================================================================
// loop()
// ============================================================================
void loop() {
    // Always update IMU every loop (low overhead after calibration)
    imu.update();

    // Dispatch FSM
    switch (robotState) {
        case STATE_IDLE:         doIdle();         break;
        case STATE_CALIBRATE:    doCalibrate();    break;
        case STATE_EXPLORE:      doExplore();      break;
        case STATE_GOAL_REACHED: doGoalReached();  break;
        case STATE_RETURN_HOME:  doReturnHome();   break;
        case STATE_STOP:         doStop();         break;
    }

    // Periodic telemetry (non-blocking)
    if (millis() - lastTelemetry >= TELE_INTERVAL_MS) {
        lastTelemetry = millis();
        printTelemetry();
    }
}

// ============================================================================
// FSM state transition helper
// ============================================================================
void transitionTo(RobotState newState) {
    Serial.printf("\n[FSM] *** Transition: %s → %s ***\n\n",
                  STATE_NAME[(int)robotState], STATE_NAME[(int)newState]);
    robotState = newState;
}

// ============================================================================
// STATE: IDLE — wait for button press
// ============================================================================
void doIdle() {
    static bool lastBtn = HIGH;
    bool btn = digitalRead(BUTTON_1);

    if (lastBtn == HIGH && btn == LOW) {
        // Falling edge = button pressed
        Serial.println(F("[IDLE] Button pressed — starting calibration"));
        delay(50);  // debounce
        transitionTo(STATE_CALIBRATE);
    }
    lastBtn = btn;
}

// ============================================================================
// STATE: CALIBRATE — re-run IMU calibration with robot on maze start cell
// ============================================================================
void doCalibrate() {
    Serial.println(F("[CALIBRATE] Re-running IMU gyro bias calibration"));
    Serial.println(F("[CALIBRATE] Ensure robot is stationary on start cell!"));

    imu.calibrate();
    imu.resetYaw();
    imu.printStatus();

    // Reset encoder counts for fresh odometry
    leftEnc.reset();
    rightEnc.reset();

    // Reset robot logical position
    robotRow     = 0;
    robotCol     = 0;
    robotHeading = DIR_NORTH;
    stepCount    = 0;

    // Reset maze (re-apply only outer border walls; clear explored data)
    maze.reset();
    maze.floodFill();

    Serial.println(F("[CALIBRATE] Calibration complete. Starting exploration in 2 s..."));
    delay(2000);

    transitionTo(STATE_EXPLORE);
}

// ============================================================================
// STATE: EXPLORE — flood-fill guided cell-by-cell navigation
// ============================================================================
void doExplore() {
    Serial.printf("\n[EXPLORE] === Step %lu at (%d,%d) heading=%s ===\n",
                  (unsigned long)stepCount, robotRow, robotCol,
                  HEADING_NAME[(int)robotHeading]);

    // 1. Mark current cell as visited
    maze.visited[robotRow][robotCol] = true;
    Serial.printf("[EXPLORE] Marked (%d,%d) as visited\n", robotRow, robotCol);

    // 2. Check goal
    if (maze.isGoal(robotRow, robotCol)) {
        Serial.println(F("[EXPLORE] GOAL CELL REACHED!"));
        transitionTo(STATE_GOAL_REACHED);
        return;
    }

    // 3. Read IR sensors and update maze walls
    senseWallsAndUpdate();

    // 4. Recompute flood-fill with new wall knowledge
    maze.floodFill();

    // 5. Print updated maps every 10 steps (to avoid swamping Serial)
    if (stepCount % 10 == 0) {
        maze.printFlood();
        maze.printVisited();
    }

    // 6. Decide next direction (lowest flood neighbour)
    uint8_t bestDist;
    AbsDir  targetDir = maze.bestDirection(robotRow, robotCol, bestDist);

    if (bestDist == FLOOD_INFINITY) {
        Serial.println(F("[EXPLORE] ERROR: all neighbours are FLOOD_INFINITY — maze is unsolvable from here!"));
        transitionTo(STATE_STOP);
        return;
    }

    Serial.printf("[EXPLORE] Moving toward dir=%s (flood dist to goal=%d)\n",
                  HEADING_NAME[(int)targetDir], (int)bestDist);

    // 7. Turn to face targetDir, then move forward one cell
    int turnSteps = ((int)targetDir - (int)robotHeading + 4) % 4;
    Serial.printf("[EXPLORE] Current heading=%s  target heading=%s  turn steps=%d\n",
                  HEADING_NAME[(int)robotHeading], HEADING_NAME[(int)targetDir], turnSteps);

    switch (turnSteps) {
        case 0:
            Serial.println(F("[EXPLORE] No turn needed — already facing target direction"));
            break;
        case 1:
            Serial.println(F("[EXPLORE] Turning RIGHT (90° clockwise)"));
            turnRight();
            break;
        case 2:
            Serial.println(F("[EXPLORE] Turning AROUND (180°)"));
            turnAround();
            break;
        case 3:
            Serial.println(F("[EXPLORE] Turning LEFT (90° counter-clockwise)"));
            turnLeft();
            break;
    }

    // Update logical heading
    robotHeading = targetDir;
    Serial.printf("[EXPLORE] Heading updated → %s\n", HEADING_NAME[(int)robotHeading]);

    // 8. Drive forward one cell
    moveForwardOneCell();

    // 9. Update logical position
    int newRow = (int)robotRow + DIR_DR[(int)robotHeading];
    int newCol = (int)robotCol + DIR_DC[(int)robotHeading];

    Serial.printf("[EXPLORE] Position update: (%d,%d) → (%d,%d)\n",
                  robotRow, robotCol, newRow, newCol);

    robotRow = (uint8_t)newRow;
    robotCol = (uint8_t)newCol;

    stepCount++;
    printRobotPosition();
}

// ============================================================================
// STATE: GOAL_REACHED
// ============================================================================
void doGoalReached() {
    Serial.println(F("[GOAL] Arrived at goal! Motors stopping."));
    leftMotor.brake();
    rightMotor.brake();
    delay(300);
    leftMotor.coast();
    rightMotor.coast();

    // Beep buzzer
    Serial.println(F("[GOAL] Beeping buzzer 3×"));
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        delay(200);
    }

    maze.printFlood();
    maze.printVisited();
    imu.printStatus();

    Serial.println(F("[GOAL] Press BUTTON_1 to return home (not yet implemented — going to STOP)"));
    transitionTo(STATE_STOP);

    // Future: transitionTo(STATE_RETURN_HOME);
}

// ============================================================================
// STATE: RETURN_HOME — placeholder
// ============================================================================
void doReturnHome() {
    // TODO: set goal to (0,0), re-flood, navigate back
    Serial.println(F("[RETURN] Return-home not yet implemented — stopping"));
    transitionTo(STATE_STOP);
}

// ============================================================================
// STATE: STOP
// ============================================================================
void doStop() {
    static unsigned long lastStopPrint = 0;
    leftMotor.coast();
    rightMotor.coast();
    if (millis() - lastStopPrint >= 5000) {
        lastStopPrint = millis();
        Serial.println(F("[STOP] Robot stopped. Reset ESP32 or press BUTTON_1 to restart."));
        printRobotPosition();
        imu.printStatus();
    }
    // Allow button press to restart
    static bool lastBtn = HIGH;
    bool btn = digitalRead(BUTTON_1);
    if (lastBtn == HIGH && btn == LOW) {
        delay(50);
        Serial.println(F("[STOP] Button pressed — restarting from IDLE"));
        transitionTo(STATE_IDLE);
    }
    lastBtn = btn;
}

// ============================================================================
// senseWallsAndUpdate()
// Read IR sensors and register detected walls in the maze data structure.
// Walls are expressed in absolute (compass) directions based on current heading.
// ============================================================================
void senseWallsAndUpdate() {
    Serial.println(F("[SENSE] Reading IR sensors"));
    ir.update();
    ir.printStatus();

    // The robot's "front" faces robotHeading.
    // LEFT  = (robotHeading + 3) % 4  (one step counter-clockwise)
    // RIGHT = (robotHeading + 1) % 4  (one step clockwise)
    AbsDir frontDir = robotHeading;
    AbsDir leftDir  = (AbsDir)(((int)robotHeading + 3) % 4);
    AbsDir rightDir = (AbsDir)(((int)robotHeading + 1) % 4);

    bool wFront = ir.wallFront();
    bool wLeft  = ir.wallLeft();
    bool wRight = ir.wallRight();

    Serial.printf("[SENSE] Wall detections — FRONT=%s(%s)  LEFT=%s(%s)  RIGHT=%s(%s)\n",
                  HEADING_NAME[(int)frontDir], wFront ? "WALL" : "open",
                  HEADING_NAME[(int)leftDir],  wLeft  ? "WALL" : "open",
                  HEADING_NAME[(int)rightDir], wRight ? "WALL" : "open");

    // Register walls. setWall() automatically mirrors to the neighbour cell.
    maze.setWall(robotRow, robotCol, frontDir, wFront);
    maze.setWall(robotRow, robotCol, leftDir,  wLeft);
    maze.setWall(robotRow, robotCol, rightDir, wRight);

    Serial.printf("[SENSE] Wall bits for cell (%d,%d) = 0x%02X\n",
                  robotRow, robotCol, maze.walls[robotRow][robotCol]);
}

// ============================================================================
// computePID() — one PID tick for one motor
// Returns the signed PWM output (-1023 to +1023).
// ============================================================================
float computePID(PIDState &pid, float target, long curTicks, float dt) {
    float rawSpeed = (float)(curTicks - pid.prevTicks) / dt;
    pid.prevTicks  = curTicks;

    // Low-pass filter on speed (α = 0.5)
    static float filteredSpeed = 0.0f;  // NOTE: this static is shared — fine for single-motor call
    filteredSpeed = 0.5f * filteredSpeed + 0.5f * rawSpeed;

    float error      = target - filteredSpeed;
    pid.integral    += error * dt;
    pid.integral     = constrain(pid.integral, -1278.0f, 1278.0f);  // anti-windup
    float derivative = (error - pid.prevError) / dt;
    pid.prevError    = error;

    float output = (Kp * error) + (Ki * pid.integral) + (Kd * derivative);

    Serial.printf("[PID] target=%.0f  speed=%.1f  err=%.1f  I=%.2f  D=%.3f  out=%.1f\n",
                  target, filteredSpeed, error, pid.integral, derivative, output);

    return output;
}

// ============================================================================
// moveForwardOneCell()
// Drive both motors forward until TICKS_PER_CELL encoder ticks are accumulated.
// Uses independent PID on each motor to maintain equal speed.
// ============================================================================
void moveForwardOneCell() {
    Serial.printf("[MOVE] moveForwardOneCell() — target=%ld ticks (%.1f mm)\n",
                  TICKS_PER_CELL, CELL_MM);

    leftEnc.reset();
    rightEnc.reset();
    imu.resetYaw();

    // Reset PID integrators
    pidLeft  = PIDState();
    pidRight = PIDState();

    unsigned long lastPID = micros();
    long ticksL = 0, ticksR = 0;
    unsigned long moveStart = millis();

    Serial.println(F("[MOVE] Starting forward drive..."));

    while (ticksL < TICKS_PER_CELL && ticksR < TICKS_PER_CELL) {
        // Safety timeout: if stuck >5 s, abort
        if (millis() - moveStart > 5000) {
            Serial.println(F("[MOVE] WARNING: forward move timeout (5 s) — aborting!"));
            break;
        }

        unsigned long now = micros();
        if (now - lastPID >= PID_INTERVAL_US) {
            float dt = (float)(now - lastPID) / 1e6f;
            lastPID  = now;

            ticksL = leftEnc.getTicks();
            ticksR = rightEnc.getTicks();

            // Compute remaining ticks and ramp down near end
            long remainL = TICKS_PER_CELL - ticksL;
            long remainR = TICKS_PER_CELL - ticksR;

            // Slow down in last 20% of cell to avoid overshooting
            float speedTarget = PID_TARGET_EXPLORE;
            if (remainL < TICKS_PER_CELL * 0.20f || remainR < TICKS_PER_CELL * 0.20f) {
                speedTarget = PID_TARGET_EXPLORE * 0.5f;
                Serial.println(F("[MOVE] Ramp-down zone — reducing target speed 50%"));
            }

            float outL = computePID(pidLeft,  speedTarget, ticksL, dt);
            float outR = computePID(pidRight, speedTarget, ticksR, dt);

            // Yaw correction: if robot drifts, slow the faster-side motor
            float yaw = imu.getYaw();
            float yawCorr = yaw * 0.5f;  // proportional yaw correction gain
            Serial.printf("[MOVE] ticksL=%ld  ticksR=%ld  yaw=%.2f°  yawCorr=%.2f\n",
                          ticksL, ticksR, yaw, yawCorr);

            leftMotor.drive( (int)(outL - yawCorr));
            rightMotor.drive((int)(outR + yawCorr));
        }
    }

    // Hard stop
    leftMotor.brake();
    rightMotor.brake();
    delay(80);
    leftMotor.coast();
    rightMotor.coast();

    Serial.printf("[MOVE] moveForwardOneCell() complete — finalL=%ld  finalR=%ld  yaw=%.2f°\n",
                  leftEnc.getTicks(), rightEnc.getTicks(), imu.getYaw());
}

// ============================================================================
// turnRight() — rotate clockwise 90° using IMU yaw feedback
// Left motor forward, right motor reverse.
// ============================================================================
void turnRight() {
    Serial.println(F("[TURN] turnRight() — clockwise 90° by IMU"));
    imu.resetYaw();

    leftMotor.drive( (int)PID_TARGET_TURN);
    rightMotor.drive(-(int)PID_TARGET_TURN);

    unsigned long tStart = millis();
    while (true) {
        imu.update();
        float yaw = imu.getYaw();
        Serial.printf("[TURN-R] yaw=%.2f° (target=%.1f°)\n", yaw, YAW_90_TARGET);

        // Yaw decreases when turning right (clockwise = negative yaw convention on Z-down IMU)
        // Adjust sign based on your IMU orientation.
        if (fabsf(yaw) >= YAW_90_TARGET - YAW_TOLERANCE) break;
        if (millis() - tStart > 3000) {
            Serial.println(F("[TURN-R] WARNING: 3 s timeout — aborting turn"));
            break;
        }
        delay(2);
    }

    leftMotor.brake();
    rightMotor.brake();
    delay(80);
    leftMotor.coast();
    rightMotor.coast();
    Serial.printf("[TURN] turnRight() done — finalYaw=%.2f°\n", imu.getYaw());
}

// ============================================================================
// turnLeft() — rotate counter-clockwise 90° using IMU yaw feedback
// Left motor reverse, right motor forward.
// ============================================================================
void turnLeft() {
    Serial.println(F("[TURN] turnLeft() — counter-clockwise 90° by IMU"));
    imu.resetYaw();

    leftMotor.drive( -(int)PID_TARGET_TURN);
    rightMotor.drive( (int)PID_TARGET_TURN);

    unsigned long tStart = millis();
    while (true) {
        imu.update();
        float yaw = imu.getYaw();
        Serial.printf("[TURN-L] yaw=%.2f° (target=%.1f°)\n", yaw, YAW_90_TARGET);

        if (fabsf(yaw) >= YAW_90_TARGET - YAW_TOLERANCE) break;
        if (millis() - tStart > 3000) {
            Serial.println(F("[TURN-L] WARNING: 3 s timeout — aborting turn"));
            break;
        }
        delay(2);
    }

    leftMotor.brake();
    rightMotor.brake();
    delay(80);
    leftMotor.coast();
    rightMotor.coast();
    Serial.printf("[TURN] turnLeft() done — finalYaw=%.2f°\n", imu.getYaw());
}

// ============================================================================
// turnAround() — rotate 180° = two right turns
// ============================================================================
void turnAround() {
    Serial.println(F("[TURN] turnAround() — executing two consecutive right turns"));
    turnRight();
    delay(100);
    turnRight();
    Serial.println(F("[TURN] turnAround() complete"));
}

// ============================================================================
// printTelemetry() — compact status line every 100 ms
// ============================================================================
void printTelemetry() {
    Serial.printf("[TELE] step=%lu  pos=(%d,%d)  heading=%s  yaw=%.1f°  Lticks=%ld  Rticks=%ld\n",
                  (unsigned long)stepCount,
                  robotRow, robotCol,
                  HEADING_NAME[(int)robotHeading],
                  imu.getYaw(),
                  leftEnc.getTicks(),
                  rightEnc.getTicks());

    uint8_t bestDist;
    AbsDir  bestDir = maze.bestDirection(robotRow, robotCol, bestDist);
    Serial.printf("[TELE] flood_here=%d  best_dir=%s(%d)  state=%s\n",
                  (int)maze.flood[robotRow][robotCol],
                  HEADING_NAME[(int)bestDir], (int)bestDist,
                  STATE_NAME[(int)robotState]);
}

// ============================================================================
// printRobotPosition() — verbose position dump
// ============================================================================
void printRobotPosition() {
    Serial.println(F("[POS] ================================================="));
    Serial.printf( "[POS]  Step     : %lu\n",     (unsigned long)stepCount);
    Serial.printf( "[POS]  Position : row=%d  col=%d\n", robotRow, robotCol);
    Serial.printf( "[POS]  Heading  : %s (%d)\n", HEADING_NAME[(int)robotHeading], (int)robotHeading);
    Serial.printf( "[POS]  Flood    : %d (distance to goal)\n", (int)maze.flood[robotRow][robotCol]);
    Serial.printf( "[POS]  Walls    : 0x%02X  (N=%d E=%d S=%d W=%d)\n",
                   maze.walls[robotRow][robotCol],
                   maze.hasWall(robotRow, robotCol, DIR_NORTH) ? 1 : 0,
                   maze.hasWall(robotRow, robotCol, DIR_EAST)  ? 1 : 0,
                   maze.hasWall(robotRow, robotCol, DIR_SOUTH) ? 1 : 0,
                   maze.hasWall(robotRow, robotCol, DIR_WEST)  ? 1 : 0);
    Serial.printf( "[POS]  Visited  : %s\n",      maze.visited[robotRow][robotCol] ? "YES" : "no");
    Serial.println(F("[POS] ================================================="));
}
