// src/main.cpp — Micromouse26 entry point.
//
// Domain code lives in include/* headers. This file owns:
//   * Hardware object instances (motors, encoders, maze, IR-centering PID).
//   * Helpers tied to that hardware (rTicks, stopMotors, buttonEdge).
//   * Phase activation hooks that reach into hardware (onPhaseActivate,
//     phaseEnter, scriptKick).
//   * setup() and the loop() state machine.
//
//   Tuning.h         every tunable constant (sections [A]-[F], [H])
//   IMU.h            MPU-6500 + yaw integration
//   IRSensors.h      4-sensor IR array + EMA state
//   Battery.h        Vbat ADC + 6.4-8.4 V → 0-100 %
//   Pose.h           robot row/col/heading, mode flags, fast cruise speed
//   MotionScript.h   PhaseStep array + simple pushers
//   Persistence.h    NVS save/load for walls + fast speed
//   Planner.h        setupMaze, senseAndStoreWalls, buildMoveScript
//   OLED.h           menu/run/diag screens + auto gyro cal

#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include "PinConfig.h"
#include "Tuning.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoderPCNT.h"
#include "MicromouseMaze.h"
#include "IRCalibration.h"
#include "IMU.h"
#include "IRSensors.h"
#include "Battery.h"
#include "Pose.h"
#include "MotionScript.h"

// ── Hardware object instances ───────────────────────────────────────────────
MicromouseMaze        maze;
MicromouseMotor       leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor       rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoderPCNT leftEnc   (PCNT_UNIT_0, ENC_L_A, ENC_L_B, /*inverted=*/false);
MicromouseEncoderPCNT rightEnc  (PCNT_UNIT_1, ENC_R_A, ENC_R_B, /*inverted=*/false);

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ── Onboard RGB LED (single WS2812 on rgb_pin) ──────────────────────────────
// Used as a visual status indicator: green celebration on GOAL, red SOS on
// CRASH. Driven non-blocking from inside the loop().
static CRGB rgbLed[1];
static uint32_t rgbStateT0 = 0;     // millis snapshot when entering GOAL/CRASH

static void rgbOff() {
    rgbLed[0] = CRGB::Black;
    FastLED.show();
}

// Green flash → rainbow chase → off → repeat. Non-blocking; call every loop.
static void rgbBlinkGoal(uint32_t elapsedMs) {
    constexpr uint32_t FLASH_PERIOD_MS = 400;
    constexpr uint32_t FLASH_PHASE_MS  = 2000;
    constexpr uint32_t RAIN_STEP_MS    = 200;
    constexpr uint32_t RAIN_PHASE_MS   = 1600;
    constexpr uint32_t TOTAL_MS        = FLASH_PHASE_MS + RAIN_PHASE_MS + 400;
    uint32_t t = elapsedMs % TOTAL_MS;
    if (t < FLASH_PHASE_MS) {
        bool on = (t % FLASH_PERIOD_MS) < (FLASH_PERIOD_MS / 2);
        rgbLed[0] = on ? CRGB::Green : CRGB::Black;
    } else if (t < FLASH_PHASE_MS + RAIN_PHASE_MS) {
        uint8_t step = (t - FLASH_PHASE_MS) / RAIN_STEP_MS;
        rgbLed[0] = CHSV((step * 32) & 0xFF, 255, 255);
    } else {
        rgbLed[0] = CRGB::Black;
    }
    FastLED.show();
}

// Red SOS blink. Non-blocking; call every loop.
static void rgbBlinkCrash(uint32_t elapsedMs) {
    bool on = (elapsedMs % 600) < 300;
    rgbLed[0] = on ? CRGB::Red : CRGB::Black;
    FastLED.show();
}

// ── IR-centering PID (consumed by the PH_FORWARD executor below) ────────────
struct PID {
    float integral = 0, prevError = 0;
    unsigned long prevUs = 0;
    float compute(float err) {
        unsigned long now = micros();
        float dt = (prevUs == 0) ? 0.001f
                                 : constrain((now - prevUs) / 1e6f, 0.0001f, 0.05f);
        prevUs = now;
        integral += err * dt;
        integral  = constrain(integral, -2000.0f, 2000.0f);
        float deriv = (err - prevError) / dt;
        prevError = err;
        float out = IR_CENTER_KP * err + IR_CENTER_KI * integral + IR_CENTER_KD * deriv;
        return constrain(out, -(float)IR_CENTER_MAX, (float)IR_CENTER_MAX);
    }
    void reset() { integral = 0; prevError = 0; prevUs = 0; }
} pid;

// Headers that reach into `maze` or want `oled`/encoders defined above.
#include "Persistence.h"
#include "Planner.h"
#include "OLED.h"

// ── Common helpers tied to hardware ─────────────────────────────────────────
void stopMotors() { leftMotor.brake(); rightMotor.brake(); }

bool buttonEdge() {
    static unsigned long pressStart = 0;
    static bool armed = true;
    bool low = (digitalRead(BUTTON_1) == LOW);
    unsigned long now = millis();
    if (!low) { pressStart = 0; armed = true; return false; }
    if (pressStart == 0) pressStart = now;
    if (armed && (now - pressStart >= BUTTON_HOLD_MS)) { armed = false; return true; }
    return false;
}

// ── Phase activation hooks (reach into IR + encoders + pid) ─────────────────
// onPhaseActivate handles deferred phase targets (PH_REVERSE_TO_BACK samples
// front IR right at activation and reclassifies itself as a reverse-FWD).
// phaseEnter snapshots encoders + yaw at every phase entry.
// scriptKick arms the first step of a freshly-built script.
static void onPhaseActivate() {
    if (runPhase == PH_REVERSE_TO_BACK) {
        sampleIR();
        float frontMm    = IRCal::estimateFrontDistMM(irVal[0], irVal[3]);
        float distMm     = frontMm + BACKUP_OFFSET_MM;
        float ticksPerMm = (float)CELL_TICKS / 180.0f;
        long  ticks      = -(long)(distMm * ticksPerMm + 0.5f);
        Serial.printf("[BACKUP] frontMm=%.1f offsetMm=%.1f → target=%ld ticks (reverse)\n",
                      frontMm, BACKUP_OFFSET_MM, ticks);
        runTarget = ticks;
        runPhase  = PH_FORWARD;
    }
}

static bool midCellSensed = false;  // reset by phaseEnter, fired once per FWD leg

static void phaseEnter() {
    phaseStartTL     = leftEnc.getTicks();
    phaseStartTR     = rTicks();
    phaseStartUs     = micros();
    yawDeg           = 0.0f;
    phaseStartYawDeg = 0.0f;
    midCellSensed    = false;
    if (runPhase == PH_SPOT || runPhase == PH_PIVOT) {
        float deg = (float)runTarget;
        float signedDeg = (runTurnDir == TURN_RIGHT) ? -deg : +deg;
        yawTargetDeg = signedDeg;
    } else {
        yawTargetDeg = 0.0f;
    }
    onPhaseActivate();
}

static void scriptKick() {
    if (scriptLen == 0) return;
    scriptIdx  = 0;
    runPhase   = script[0].phase;
    runTarget  = script[0].target;
    runTurnDir = script[0].dir;
    pid.reset();
    irFirstSample = true;   // re-seed IR centering EMA on each new script
    phaseEnter();
}

// ── State machine ───────────────────────────────────────────────────────────
enum State { IDLE, ENC_TEST, IR_TEST, CAL_IR, MOTOR_TEST, FAST_SPEED_EDIT, EXPLORE_THINK, RUN, GOAL, CRASH };
static State state = IDLE;

// ── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_1, INPUT_PULLUP);

    pinMode(MOTOR_SLEEP, OUTPUT);
    digitalWrite(MOTOR_SLEEP, HIGH);

    leftMotor.begin(); rightMotor.begin();
    leftEnc.begin();   rightEnc.begin();

    for (auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT);
        digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }
    analogReadResolution(12);

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    FastLED.addLeds<WS2812B, rgb_pin, GRB>(rgbLed, 1);
    FastLED.setBrightness(20);
    rgbOff();

    imuReady = mpuInit();
    if (imuReady) {
        Serial.println("[IMU] mpu6500 ok, calibrating bias... keep still");
        calibrateGyroBias(300, 2);
        yawDeg       = 0.0f;
        yawTargetDeg = 0.0f;
        Serial.printf("[IMU] bias=%.4f deg/s\n", gyroBiasZ);
    } else {
        Serial.println("[IMU] mpu6500 NOT detected");
    }

    setupMaze();
    nvsLoadFastSpeed();
    Serial.printf("[FSPEED] loaded = %.0f tps (%.0f mm/s)\n",
                  fastRunCruiseTps,
                  fastRunCruiseTps * 180.0f / (float)CELL_TICKS);

    menuEncRef = rightEnc.getTicks();
    oledMenu();

    Serial.println();
    Serial.println("mm26 flood ready");
}

// ── Loop ────────────────────────────────────────────────────────────────────
void loop() {
    updateYaw();

    switch (state) {

    case IDLE: {
        long delta = rightEnc.getTicks() - menuEncRef;
        if (delta >= ENC_PER_STEP) {
            menuSel = (menuSel + 1) % M_COUNT;
            menuEncRef += ENC_PER_STEP;
            oledMenu();
        } else if (delta <= -ENC_PER_STEP) {
            menuSel = (menuSel - 1 + M_COUNT) % M_COUNT;
            menuEncRef -= ENC_PER_STEP;
            oledMenu();
        }
        if (buttonEdge()) {
            switch (menuSel) {
            case M_EXPLORE:
                setupMaze();
                robotRow = START_ROW; robotCol = START_COL; robotHeading = DIR_NORTH;
                pendingOffsetTicks = START_OFFSET_TICKS;
                exploreMode = true; fastRunMode = false; returnHomeMode = false; finalTurnPending = false;
                autoCalGyroBeforeStart();
                for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(COUNTDOWN_DELAY_MS); }
                Serial.println("--- EXPLORE START (round trip) ---");
                state = EXPLORE_THINK;
                break;
            case M_FAST:
                setupMaze();
                if (!nvsLoadWalls()) {
                    Serial.println("[FAST] no saved walls");
                    oledTerminal("FAST", "no walls");
                    delay(800); oledMenu();
                    break;
                }
                robotRow = START_ROW; robotCol = START_COL; robotHeading = DIR_NORTH;
                pendingOffsetTicks = START_OFFSET_TICKS;
                exploreMode = false; fastRunMode = true; returnHomeMode = false; finalTurnPending = false;
                autoCalGyroBeforeStart();
                for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(COUNTDOWN_DELAY_MS); }
                Serial.println("--- FAST RUN START ---");
                state = EXPLORE_THINK;
                break;
            case M_FAST_SPEED:
                menuEncRef = rightEnc.getTicks();
                state = FAST_SPEED_EDIT;
                oledFastSpeedEdit();
                break;
            case M_ENC:
                leftEnc.reset(); rightEnc.reset();
                state = ENC_TEST;
                oledEncTest();
                break;
            case M_IR_TEST:
                sampleIR();
                state = IR_TEST;
                oledIrTest();
                break;
            case M_CAL_IR:
                oledTerminal("Cal IR", "center robot\nbtn=start");
                state = CAL_IR;
                break;
            case M_MOTOR_TEST: {
                leftEnc.reset(); rightEnc.reset();
                menuEncRef = rightEnc.getTicks();
                state = MOTOR_TEST;
                // motorTestStep and motorTestTimer initialised in the state case
                break;
            }
            case M_NVS_CLR:
                nvsClearWalls();
                Serial.println("[NVS] walls cleared");
                oledTerminal("NVS", "cleared");
                delay(600); oledMenu();
                break;
            case M_DUMP_MAZE:
                Serial.println("[MAZE DUMP] walls + flood (hex), * = visited");
                maze.floodFill();
                maze.dump();
                oledTerminal("Maze", "dumped->serial");
                delay(600); oledMenu();
                break;
            }
        }
        break;
    }

    case ENC_TEST: {
        static uint32_t last = 0;
        if (millis() - last > 100) { oledEncTest(); last = millis(); }
        if (buttonEdge()) {
            leftEnc.reset(); rightEnc.reset();
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
        }
        break;
    }

    case IR_TEST: {
        static uint32_t last = 0;
        if (millis() - last > 100) {
            // Read amb+lit individually for serial diagnostic
            int amb[4], lit[4], delta[4];
            for (int i = 0; i < 4; i++) {
                digitalWrite(PAIRS[i].emit, LOW);  delayMicroseconds(80);
                amb[i] = analogRead(PAIRS[i].rx);
                digitalWrite(PAIRS[i].emit, HIGH); delayMicroseconds(80);
                lit[i] = analogRead(PAIRS[i].rx);
                digitalWrite(PAIRS[i].emit, LOW);
                int d = lit[i] - amb[i];
                delta[i] = d < 0 ? 0 : d;
                irVal[i] = delta[i];
            }
            oledIrTest();
            // PAIRS: 0=LF 1=L45 2=R45 3=RF
            Serial.printf("[IR] LF  emit=IO%-2d rx=IO%-2d amb=%4d lit=%4d d=%4d\n", PAIRS[0].emit, PAIRS[0].rx, amb[0], lit[0], delta[0]);
            Serial.printf("[IR] RF  emit=IO%-2d rx=IO%-2d amb=%4d lit=%4d d=%4d\n", PAIRS[3].emit, PAIRS[3].rx, amb[3], lit[3], delta[3]);
            Serial.printf("[IR] R45 emit=IO%-2d rx=IO%-2d amb=%4d lit=%4d d=%4d\n", PAIRS[2].emit, PAIRS[2].rx, amb[2], lit[2], delta[2]);
            Serial.printf("[IR] L45 emit=IO%-2d rx=IO%-2d amb=%4d lit=%4d d=%4d\n", PAIRS[1].emit, PAIRS[1].rx, amb[1], lit[1], delta[1]);
            Serial.printf("[IR] front=%.1f mm\n---\n", IRCal::estimateFrontDistMM(delta[0], delta[3]));
            last = millis();
        }
        if (buttonEdge()) {
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
        }
        break;
    }

    case CAL_IR: {
        // Wait for button press → sample 32 reads → update calL/calR → display result.
        static bool calDone = false;
        if (!calDone && buttonEdge()) {
            // 32-sample mean of L45 (PAIRS[1]) and R45 (PAIRS[2])
            long sumL = 0, sumR = 0;
            for (int n = 0; n < 32; n++) {
                digitalWrite(PAIRS[1].emit, LOW);  delayMicroseconds(80);
                int aL = analogRead(PAIRS[1].rx);
                digitalWrite(PAIRS[1].emit, HIGH); delayMicroseconds(80);
                int lL = analogRead(PAIRS[1].rx);
                digitalWrite(PAIRS[1].emit, LOW);
                sumL += max(0, lL - aL);

                digitalWrite(PAIRS[2].emit, LOW);  delayMicroseconds(80);
                int aR = analogRead(PAIRS[2].rx);
                digitalWrite(PAIRS[2].emit, HIGH); delayMicroseconds(80);
                int lR = analogRead(PAIRS[2].rx);
                digitalWrite(PAIRS[2].emit, LOW);
                sumR += max(0, lR - aR);

                delay(5);
            }
            calL = (int)(sumL / 32);
            calR = (int)(sumR / 32);
            calDone = true;
            oledCalIrResult(calL, calR);
            Serial.printf("[CAL IR] L45=%d  R45=%d  (paste into PinConfig.h IR_CAL_L45/R45)\n", calL, calR);
        } else if (calDone && buttonEdge()) {
            calDone = false;
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
        }
        break;
    }

    case MOTOR_TEST: {
        // Button press advances to next step; after all 4, exits to menu.
        static const struct { const char* label; int l; int r; } MT_STEPS[] = {
            {"L fwd      ", +400,    0},
            {"L rev      ", -400,    0},
            {"R fwd      ",    0, +400},
            {"R rev      ",    0, -400},
        };
        static int mtStep = -1;  // -1 = uninitialised sentinel

        if (mtStep == -1) {
            mtStep = 0;
            leftMotor.drive(MT_STEPS[0].l);
            rightMotor.drive(MT_STEPS[0].r);
            Serial.printf("[MOTOR TEST] step 0: %s\n", MT_STEPS[0].label);
            oledTerminal("MotorTest", MT_STEPS[0].label);
        }

        if (buttonEdge()) {
            leftMotor.brake(); rightMotor.brake();
            mtStep++;
            if (mtStep >= 4) {
                mtStep = -1;
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                state = IDLE;
            } else {
                leftMotor.drive(MT_STEPS[mtStep].l);
                rightMotor.drive(MT_STEPS[mtStep].r);
                Serial.printf("[MOTOR TEST] step %d: %s\n", mtStep, MT_STEPS[mtStep].label);
                oledTerminal("MotorTest", MT_STEPS[mtStep].label);
            }
        }
        break;
    }

    case FAST_SPEED_EDIT: {
        long delta = rightEnc.getTicks() - menuEncRef;
        if (delta >= ENC_PER_STEP) {
            fastRunCruiseTps += FAST_RUN_CRUISE_TPS_STEP;
            if (fastRunCruiseTps > FAST_RUN_CRUISE_TPS_MAX) fastRunCruiseTps = FAST_RUN_CRUISE_TPS_MAX;
            menuEncRef += ENC_PER_STEP;
            oledFastSpeedEdit();
        } else if (delta <= -ENC_PER_STEP) {
            fastRunCruiseTps -= FAST_RUN_CRUISE_TPS_STEP;
            if (fastRunCruiseTps < FAST_RUN_CRUISE_TPS_MIN) fastRunCruiseTps = FAST_RUN_CRUISE_TPS_MIN;
            menuEncRef -= ENC_PER_STEP;
            oledFastSpeedEdit();
        }
        if (buttonEdge()) {
            nvsSaveFastSpeed();
            Serial.printf("[FSPEED] saved = %.0f tps (%.0f mm/s)\n",
                          fastRunCruiseTps,
                          fastRunCruiseTps * 180.0f / (float)CELL_TICKS);
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
        }
        break;
    }

    case EXPLORE_THINK: {
        // Final celebration spin completed → transition to GOAL.
        if (finalTurnPending) {
            finalTurnPending = false;
            stopMotors();
            oledTerminal("DONE!", returnHomeMode ? "home" : "goal");
            rgbStateT0 = millis();
            state = GOAL;
            break;
        }
        // Zero heading reference at every cell arrival to bound gyro drift
        // accumulation to a single move.
        yawDeg       = 0.0f;
        yawTargetDeg = 0.0f;
        if (exploreMode) {
            if (!maze.visited[robotRow][robotCol]) {
                senseAndStoreWalls();
            }
            if (atDeadEnd()) {
                AbsDir back = (AbsDir)((robotHeading + 2) % 4);
                Serial.printf("[DEADEND] (%d,%d,%c) → 180 + exit cell\n",
                              robotRow, robotCol, "NESW"[robotHeading]);
                maze.visited[robotRow][robotCol] = true;
                // Spin 180° then drive back one full cell. EXPLORE_THINK
                // re-senses from the previous cell where L/R branches exist.
                scriptReset();
                scriptPushSpot(TURN_RIGHT, 180.0f);
                long fwd = CELL_TICKS + pendingOffsetTicks;
                pendingOffsetTicks = 0;
                scriptPushFwd(fwd);
                plannedHeading = (uint8_t)back;
                plannedRow     = robotRow + DIR_DR[back];
                plannedCol     = robotCol + DIR_DC[back];
                scriptKick();
                state = RUN;
                break;
            }
        }
        maze.visited[robotRow][robotCol] = true;
        if (maze.isGoal(robotRow, robotCol)) {
            if (exploreMode && !returnHomeMode) {
                // First leg done — flip the goal to home (0,0) and keep
                // exploring back. Walls accumulated on the return leg fill
                // in anything missed on the way out.
                //
                // Save NVS immediately so a crash during return-home doesn't
                // lose the forward map. The home-arrival save below will
                // overwrite with the more complete round-trip map.
                Serial.printf("--- FWD GOAL reached (%d,%d), returning home ---\n",
                              robotRow, robotCol);
                if (nvsSaveWalls()) Serial.println("[NVS] walls saved (forward leg done)");
                maze.setGoalSingle(START_ROW, START_COL);
                returnHomeMode = true;
                // Fall through into floodFill / planning with the new goal.
            } else if (exploreMode && returnHomeMode) {
                // Back home — check for any unvisited cells before stopping.
                int ur = -1, uc = -1, bestManhattan = 9999;
                for (int r = 0; r < MAZE_ROWS; r++) {
                    for (int c = 0; c < MAZE_COLS; c++) {
                        if (!maze.visited[r][c]) {
                            int d = abs(r - (int)robotRow) + abs(c - (int)robotCol);
                            if (d < bestManhattan) { bestManhattan = d; ur = r; uc = c; }
                        }
                    }
                }
                if (ur >= 0) {
                    Serial.printf("--- home reached, unvisited (%d,%d) remain, targeting (%d,%d) ---\n",
                                  MAZE_ROWS * MAZE_COLS - /* rough */ 0, 0, ur, uc);
                    maze.setGoalSingle(ur, uc);
                    // returnHomeMode stays true; we'll re-check each time we hit a goal
                    // Fall through to floodFill / planning below.
                } else {
                    // All reachable cells visited — wrap up.
                    maze.setGoalCentre4();
                    if (nvsSaveWalls()) Serial.println("[NVS] walls saved (full explore done)");
                    Serial.printf("--- FULLY EXPLORED, spinning 180 ---\n");
                    oledTerminal("DONE!", "full");
                    scriptReset();
                    scriptPushSpot(TURN_RIGHT, 180.0f);
                    plannedRow     = robotRow;
                    plannedCol     = robotCol;
                    plannedHeading = (robotHeading + 2) % 4;
                    finalTurnPending = true;
                    scriptKick();
                    state = RUN;
                    break;
                }
            } else {
                // Fast-run goal reached or explore forward goal reached (handled above).
                Serial.printf("--- DONE at (%d,%d), spinning 180 ---\n",
                              robotRow, robotCol);
                oledTerminal("DONE!", "spin");
                scriptReset();
                scriptPushSpot(TURN_RIGHT, 180.0f);
                plannedRow     = robotRow;
                plannedCol     = robotCol;
                plannedHeading = (robotHeading + 2) % 4;
                finalTurnPending = true;
                scriptKick();
                state = RUN;
                break;
            }
        }
        // Explore: flood from all unvisited cells (return to branches after dead
        // ends). Fast run / return-home: flood toward goal. Open-wall IR clears
        // keep floodFillExplore from ping-ponging on a stale map.
        bool routeToUnvisited = exploreMode && !returnHomeMode;
        if (routeToUnvisited)
            maze.floodFillExplore(MAZE_ROWS, MAZE_COLS);
        else
            maze.floodFill();
        uint8_t bestDist;
        AbsDir bestDir = maze.bestDirectionBiased(robotRow, robotCol,
                                                  (AbsDir)robotHeading, bestDist,
                                                  routeToUnvisited);
        if (bestDist == FLOOD_INFINITY) {
            if (exploreMode) {
                // False walls isolated us — force 180° and keep exploring.
                Serial.printf("[RECOVER] (%d,%d,%c) flood=INF, forcing 180\n",
                              robotRow, robotCol, "NESW"[robotHeading]);
                AbsDir backDir = (AbsDir)((robotHeading + 2) % 4);
                buildMoveScript(backDir);
                scriptKick();
                state = RUN;
                break;
            }
            stopMotors();
            // Re-sample so OLED shows current raw values
            sampleIR();
            bool dbgF = irVal[0] > WALL_FRONT_THRESH && irVal[3] > WALL_FRONT_THRESH;
            bool dbgL = irVal[1] > WALL_SIDE_THRESH;
            bool dbgR = irVal[2] > WALL_SIDE_THRESH;
            // Print flood distances and wall bits for all 4 neighbours
            int nr_N = robotRow + DIR_DR[DIR_NORTH], nc_N = robotCol + DIR_DC[DIR_NORTH];
            int nr_E = robotRow + DIR_DR[DIR_EAST],  nc_E = robotCol + DIR_DC[DIR_EAST];
            int nr_S = robotRow + DIR_DR[DIR_SOUTH], nc_S = robotCol + DIR_DC[DIR_SOUTH];
            int nr_W = robotRow + DIR_DR[DIR_WEST],  nc_W = robotCol + DIR_DC[DIR_WEST];
            Serial.printf("[CRASH] boxed (%d,%d,%c) walls=0x%02X\n",
                          robotRow, robotCol, "NESW"[robotHeading], maze.walls[robotRow][robotCol]);
            Serial.printf("[CRASH] floodN=%d floodE=%d floodS=%d floodW=%d\n",
                          (nr_N>=0&&nr_N<MAZE_SIZE)?maze.flood[nr_N][nc_N]:255,
                          (nr_E>=0&&nc_E<MAZE_SIZE)?maze.flood[nr_E][nc_E]:255,
                          (nr_S>=0&&nr_S>=0)?maze.flood[nr_S][nc_S]:255,
                          (nr_W>=0&&nc_W>=0)?maze.flood[nr_W][nc_W]:255);
            Serial.printf("[CRASH] IR F=%d(LF=%d RF=%d) L=%d(%d) R=%d(%d) thr=%d\n",
                          dbgF, irVal[0], irVal[3],
                          dbgL, irVal[1], dbgR, irVal[2], WALL_FRONT_THRESH);
            // OLED: walls bitmask + flood distances of neighbours
            {
                char line1[24], line2[24], line3[24];
                snprintf(line1, sizeof(line1), "BOX(%d,%d)%c w=%02X f=%d", robotRow, robotCol, "NESW"[robotHeading], maze.walls[robotRow][robotCol], maze.flood[robotRow][robotCol]);
                uint8_t fN = (nr_N>=0&&nr_N<MAZE_SIZE) ? maze.flood[nr_N][nc_N] : 255;
                uint8_t fS = (nr_S>=0&&nr_S<MAZE_SIZE) ? maze.flood[nr_S][nc_S] : 255;
                uint8_t fE = (nc_E>=0&&nc_E<MAZE_SIZE) ? maze.flood[nr_E][nc_E] : 255;
                snprintf(line2, sizeof(line2), "fN=%d fS=%d fE=%d", fN, fS, fE);
                snprintf(line3, sizeof(line3), "w4=%02X L=%d R=%d", (nr_S>=0&&nr_S<MAZE_SIZE)?maze.walls[nr_S][nc_S]:0, irVal[1], irVal[2]);
                oled.clearBuffer();
                oled.setFont(u8g2_font_5x7_tf);
                oled.drawStr(0, 8,  line1);
                oled.drawHLine(0, 10, 128);
                oled.drawStr(0, 22, line2);
                oled.drawStr(0, 34, line3);
                oled.drawStr(0, 63, "btn=back");
                oled.sendBuffer();
            }
            rgbStateT0 = millis();
            state = CRASH;
            break;
        }
        if (!fastRunMode) {
            Serial.printf("[PLAN] (%d,%d,%c) dist=%d uv=%d fHere=%d → %c\n",
                          robotRow, robotCol, "NESW"[robotHeading], bestDist,
                          routeToUnvisited ? maze.countUnvisited(MAZE_ROWS, MAZE_COLS) : 0,
                          maze.flood[robotRow][robotCol], "NESW"[bestDir]);
        }
        buildMoveScript(bestDir);
        scriptKick();
        state = RUN;
        break;
    }

    case RUN: {
        if (buttonEdge()) {
            stopMotors();
            Serial.println("--- RUN aborted ---");
            exploreMode = false; fastRunMode = false; returnHomeMode = false;
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
            break;
        }

        // Encoders run continuously; phaseEnter() captured the absolute count
        // at activation, so the phase-local "tL/tR" is the delta against that.
        long tLAbs = leftEnc.getTicks();
        long tRAbs = rTicks();
        long tL = tLAbs - phaseStartTL;
        long tR = tRAbs - phaseStartTR;

        // ── PH_ALIGN_FRONT: creep until LF/RF at ALIGN_*_TARGET (dead-end). ──
        // Direction set per-loop from the mean error; yaw-held against the
        // current commanded heading.
        if (runPhase == PH_ALIGN_FRONT) {
            sampleIR();
            int errLF = irVal[0] - ALIGN_LF_TARGET;
            int errRF = irVal[3] - ALIGN_RF_TARGET;
            int errMean = (errLF + errRF) / 2;
            // LF/RF diff: angular error against the dead-end wall.
            // LF > RF → LF closer → robot rotated CW → spin CCW to flatten.
            int errDiff = errLF - errRF;
            long avgTicks = (tL + tR) / 2;
            static uint32_t alignSettleStart = 0;

            auto endNow = [&](const char* reason) {
                stopMotors();
                if (!fastRunMode) {
                    Serial.printf("--- STEP END idx=%d/%d ph=ALIGN reason=%s LF=%d RF=%d tL=%ld tR=%ld ---\n",
                                  scriptIdx + 1, scriptLen, reason, irVal[0], irVal[3], tL, tR);
                }
                alignSettleStart = 0;
                if (scriptIdx + 1 >= scriptLen) {
                    robotRow = plannedRow; robotCol = plannedCol; robotHeading = plannedHeading;
                    runTurnDir = TURN_NONE;
                    if (!fastRunMode) {
                        Serial.printf("--- MOVE DONE pos=(%d,%d,%c) ---\n",
                                      robotRow, robotCol, "NESW"[robotHeading]);
                    }
                    if (exploreMode || fastRunMode) state = EXPLORE_THINK;
                    else { menuEncRef = rightEnc.getTicks(); oledMenu(); state = IDLE; }
                    return;
                }
                scriptIdx++;
                PhaseStep& next = script[scriptIdx];
                runPhase   = next.phase;
                runTarget  = next.target;
                runTurnDir = next.dir;
                pid.reset();
                phaseEnter();
            };

            if (abs(errLF) <= ALIGN_TOL && abs(errRF) <= ALIGN_TOL) {
                stopMotors();
                if (alignSettleStart == 0) alignSettleStart = millis();
                if (millis() - alignSettleStart > ALIGN_SETTLE_MS) {
                    endNow("ALIGN_OK");
                }
                break;
            }
            alignSettleStart = 0;
            if (labs(avgTicks) >= ALIGN_MAX_TICKS) { endNow("ALIGN_CAP"); break; }

            // errMean > 0 → both IR brighter than target → wall closer than
            // target → REVERSE. errMean < 0 → wall farther → FORWARD.
            int dir = (errMean > 0) ? -1 : +1;
            int throttle = dir * ALIGN_PWM;
            // Angular correction from front-sensor mismatch.
            constexpr float ALIGN_DIFF_KP = 0.05f;
            int angleBias = (int)(ALIGN_DIFF_KP * (float)errDiff);
            int yawBias  = (USE_IMU && imuReady)
                            ? (int)(-YAW_HOLD_KP * (yawDeg - yawTargetDeg) - YAW_HOLD_KD * gzFilt) : 0;
            // angleBias overrides gyro hold for this phase (wall is the truth).
            int bias = angleBias + yawBias / 4;
            int pwmL = constrain(throttle - bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            int pwmR = constrain(throttle + bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            leftMotor.drive(pwmL);
            rightMotor.drive(pwmR);

            static uint32_t lastOled = 0;
            if (millis() - lastOled > 150) {
                oledRun(avgTicks, ALIGN_MAX_TICKS, tL, tR);
                lastOled = millis();
            }
            if (TELEMETRY && !fastRunMode) {
                static uint32_t lastTel = 0;
                if (millis() - lastTel > 80) {
                    Serial.printf("t=%lu ph=ALIGN LF=%d RF=%d errLF=%+d errRF=%+d dir=%+d tL=%ld tR=%ld\n",
                                  (unsigned long)millis(), irVal[0], irVal[3], errLF, errRF, dir, tL, tR);
                    lastTel = millis();
                }
            }
            break;
        }

        // ── Position / yaw PID for PH_FORWARD / PH_PIVOT / PH_SPOT ──────────
        bool imuMode = USE_IMU && imuReady && (runPhase != PH_FORWARD);

        float avg;
        if (runPhase == PH_FORWARD) {
            avg = (float)((tL + tR) / 2);
            // Mid-cell side-wall sense: fires once per forward leg when the robot
            // is ~halfway through the cell. At this distance the 45° sensors are
            // clear of any front wall, so L45/R45 read true side walls of the
            // destination cell without contamination.
            if (!fastRunMode && !midCellSensed
                    && runTarget > 0
                    && avg >= (float)(CELL_TICKS * 3 / 4)
                    && !maze.visited[plannedRow][plannedCol]) {
                senseSideWallsMidCell(plannedRow, plannedCol, plannedHeading);
                midCellSensed = true;
            }
        } else if (imuMode) {
            // Per-phase yaw zero — measure progress against the phase-entry snapshot.
            float dy = yawDeg - phaseStartYawDeg;
            avg = (runTurnDir == TURN_RIGHT) ? -dy : +dy;
        } else if (runPhase == PH_PIVOT) {
            avg = (float)((runTurnDir == TURN_RIGHT) ? tL : tR);
        } else {
            avg = (float)((runTurnDir == TURN_RIGHT) ? (tL - tR) / 2 : (tR - tL) / 2);
        }

        float    effKp        = imuMode ? YAW_KP            : POS_KP;
        float    effKd        = imuMode ? YAW_KD            : POS_KD;
        int      effMaxPwm    = imuMode ? YAW_MAX_PWM       : POS_MAX_PWM;
        int      effStkPwm    = imuMode ? YAW_STICTION_PWM  : POS_STICTION_PWM;
        float    effFz        = imuMode ? YAW_FRICTION_ZONE : (float)POS_FRICTION_ZONE;
        float    effHb        = imuMode ? YAW_HOLD_BAND     : (float)POS_HOLD_BAND;
        // Fast run cuts the FWD-settle dwell — trapezoid decel already
        // parks velocity at ~0 by the time |posErr| < holdBand, and the
        // 80 ms wait shows up as a visible "pause" at every turn cell.
        // Rotation phases (PH_SPOT) keep the full YAW_SETTLE_MS dwell so
        // the heading doesn't commit before the gyro has actually settled.
        uint32_t effSettleMs  = imuMode             ? YAW_SETTLE_MS
                              : fastRunMode         ? 0u
                                                    : POS_SETTLE_MS;
        float    effStallVel  = imuMode ? YAW_STALL_VEL     : POS_STALL_VEL;
        uint32_t effStallMs   = imuMode ? YAW_STALL_MS      : POS_STALL_MS;
        float    effStallEmax = imuMode ? YAW_STALL_ERR_MAX : (float)POS_STALL_ERR_MAX;
        float    effStkSoft   = imuMode ? YAW_STK_SOFT_BAND : (float)POS_STK_SOFT_BAND;

        static float    posAvgPrev  = 0.0f;
        static uint32_t posPrevUs   = 0;
        static float    velFilt     = 0.0f;
        static uint32_t settleStart = 0;
        static uint32_t stallStart  = 0;

        float posErr = (float)runTarget - avg;

        auto resetPidState = [&]() {
            posAvgPrev = 0.0f; posPrevUs = 0; velFilt = 0.0f;
            settleStart = 0; stallStart = 0;
        };

        auto endPhase = [&](const char* reason) {
            stopMotors();
            const char* phN = (runPhase == PH_FORWARD) ? "FWD"
                            : (runPhase == PH_PIVOT)   ? "PIV" : "SPOT";
            if (!fastRunMode) {
                Serial.printf("--- STEP END idx=%d/%d ph=%s reason=%s err=%+.2f tL=%ld tR=%ld yaw=%+.2f ---\n",
                              scriptIdx + 1, scriptLen, phN, reason, posErr, tL, tR, yawDeg);
            }

            if (scriptIdx + 1 >= scriptLen) {
                robotRow = plannedRow; robotCol = plannedCol; robotHeading = plannedHeading;
                resetPidState();
                runTurnDir = TURN_NONE;
                if (!fastRunMode) {
                    Serial.printf("--- MOVE DONE pos=(%d,%d,%c) ---\n",
                                  robotRow, robotCol, "NESW"[robotHeading]);
                }
                if (exploreMode || fastRunMode) {
                    state = EXPLORE_THINK;
                } else {
                    menuEncRef = rightEnc.getTicks();
                    oledMenu();
                    state = IDLE;
                }
                return;
            }
            scriptIdx++;
            PhaseStep& next = script[scriptIdx];
            runPhase   = next.phase;
            runTarget  = next.target;
            runTurnDir = next.dir;
            resetPidState();
            pid.reset();
            phaseEnter();
        };

        if (fabsf(posErr) < effHb) {
            stopMotors();
            if (settleStart == 0) settleStart = millis();
            if (millis() - settleStart > effSettleMs) {
                endPhase("SETTLED");
            }
            break;
        }
        settleStart = 0;

        uint32_t nowUs = micros();
        if (posPrevUs == 0) {
            // First iteration after a phase reset: seed posAvgPrev so the
            // first computed velocity is 0 instead of a huge spike.
            posAvgPrev = avg;
            posPrevUs  = nowUs;
        } else {
            float dt = constrain((nowUs - posPrevUs) / 1e6f, 1e-4f, 0.05f);
            posPrevUs = nowUs;
            if (imuMode) {
                velFilt = (runTurnDir == TURN_RIGHT) ? -gzFilt : +gzFilt;
                posAvgPrev = avg;
            } else {
                float velRaw = (avg - posAvgPrev) / dt;
                posAvgPrev   = avg;
                velFilt      = 0.7f * velFilt + 0.3f * velRaw;
            }
        }

        if (fabsf(velFilt) < effStallVel && fabsf(posErr) < effStallEmax) {
            if (stallStart == 0) stallStart = millis();
            if (millis() - stallStart > effStallMs) {
                endPhase("STALL");
                break;
            }
        } else {
            stallStart = 0;
        }

        int throttle;
        float vCmdSigned = 0.0f;
        float xCmdSigned = (float)runTarget;
        if (runPhase == PH_FORWARD) {
            // Position-based trapezoidal velocity command:
            //   vAccel = sqrt(2·a·xDone)  → smooth ramp from rest
            //   vDecel = sqrt(2·d·xRem)   → vCmd → 0 as remaining → 0
            //   vAbsCmd = min(vCruise, vAccel, vDecel)
            // Closed loop on actual distance; PID closes lag on posErr. xDone
            // is bootstrapped to ≥ 30 ticks so vAccel isn't zero at phase
            // entry (otherwise the wheels never start).
            float dist     = fabsf((float)runTarget);
            float absAvg   = fabsf(avg);
            float xRem     = (dist > absAvg) ? (dist - absAvg) : 0.0f;
            float xDoneEff = absAvg < 30.0f ? 30.0f : absAvg;
            float vCruise  = fastRunMode ? fastRunCruiseTps : FWD_V_CRUISE_TPS;
            float vAccel   = sqrtf(2.0f * FWD_ACCEL_TPS2 * xDoneEff);
            float vDecel   = sqrtf(2.0f * FWD_DECEL_TPS2 * xRem);
            float vAbsCmd  = vCruise;
            if (vAccel < vAbsCmd) vAbsCmd = vAccel;
            if (vDecel < vAbsCmd) vAbsCmd = vDecel;
            float dirSign  = ((float)runTarget - avg >= 0) ? +1.0f : -1.0f;
            vCmdSigned     = dirSign * vAbsCmd;
            xCmdSigned     = (float)runTarget;
            float velErr   = vCmdSigned - velFilt;
            float uFF      = (vAbsCmd > 5.0f) ? dirSign * ((float)FWD_STICTION_FF
                                                        + FWD_KV_SLOPE * vAbsCmd) : 0.0f;
            float u        = uFF + POS_KP * posErr + POS_KD * velErr;
            // Residual stiction floor: trapezoid at v=0 but robot still
            // outside the settle band. PID alone gives sub-breakaway PWM.
            if (vAbsCmd <= 5.0f && fabsf(posErr) > (float)POS_HOLD_BAND
                && fabsf(u) < (float)FWD_STICTION_FF) {
                u = ((posErr >= 0) ? +1.0f : -1.0f) * (float)FWD_STICTION_FF;
            }
            // Dynamic cap: PWM ≤ FF(vCmd) + small PID headroom so commanded
            // velocity is actually enforced.
            float ffMag = (vAbsCmd > 5.0f) ? ((float)FWD_STICTION_FF + FWD_KV_SLOPE * vAbsCmd) : 0.0f;
            int   dynMax = (int)ffMag + 40;
            if (dynMax > POS_MAX_PWM) dynMax = POS_MAX_PWM;
            if (dynMax < (int)FWD_STICTION_FF + 20) dynMax = (int)FWD_STICTION_FF + 20;
            throttle = constrain((int)u, -dynMax, dynMax);
        } else {
            // PIVOT/SPOT: position PID with soft stiction floor.
            float u   = effKp * posErr - effKd * velFilt;
            int   mag = (int)fabsf(u);
            if (mag > effMaxPwm) mag = effMaxPwm;
            float errAbs = fabsf(posErr);
            if (errAbs > effFz) {
                float tBlend = constrain((errAbs - effFz) / effStkSoft, 0.0f, 1.0f);
                int softFloor = (int)(tBlend * (float)effStkPwm);
                if (mag < softFloor) mag = softFloor;
            }
            throttle = (u >= 0) ? mag : -mag;
        }

        // IR centering (forward phase only).
        float corr = 0.0f;
        if (runPhase == PH_FORWARD && USE_IR) {
            sampleIR();
            constexpr float IR_CONF_LO    = 10.0f;   // above noise floor
            constexpr float IR_CONF_HI    = 150.0f;  // full conf well below cal@center(~330-403)
            constexpr float IR_EDGE_DELTA = 60.0f;   // wall appear/vanish in 0-400 range

            if (irFirstSample) {
                irLSm = irVal[1]; irRSm = irVal[2];
                irLPrev = irLSm;  irRPrev = irRSm;
                irFirstSample = false;
            }
            irLSm = 0.7f * irLSm + 0.3f * irVal[1];
            irRSm = 0.7f * irRSm + 0.3f * irVal[2];
            float dL = irLSm - irLPrev, dR = irRSm - irRPrev;
            irLPrev = irLSm; irRPrev = irRSm;
            if (!fastRunMode) {
                if (dL < -IR_EDGE_DELTA) Serial.println("[EVENT] L wall opened");
                if (dR < -IR_EDGE_DELTA) Serial.println("[EVENT] R wall opened");
                if (dL >  IR_EDGE_DELTA) Serial.println("[EVENT] L wall appeared");
                if (dR >  IR_EDGE_DELTA) Serial.println("[EVENT] R wall appeared");
            }

            auto wallConf = [](float v) -> float {
                if (v < IR_CONF_LO) return 0.0f;
                if (v > IR_CONF_HI) return 1.0f;
                return (v - IR_CONF_LO) / (IR_CONF_HI - IR_CONF_LO);
            };
            float cL = wallConf(irLSm);
            float cR = wallConf(irRSm);
            float fErrL = cL * (irLSm - (float)calL);
            float fErrR = cR * (irRSm - (float)calR);
            float fErr  = fErrR - fErrL;
            corr = (cL > 0.05f || cR > 0.05f) ? pid.compute(fErr) : 0.0f;
        }

        int pwmL, pwmR;
        if (runPhase == PH_FORWARD) {
            // Hold against the commanded heading.
            int yawBias    = (USE_IMU && imuReady)
                              ? (int)(-YAW_HOLD_KP * (yawDeg - yawTargetDeg) - YAW_HOLD_KD * gzFilt) : 0;
            int encBalance = (int)((tL - tR) * BALANCE_KP);
            int bias = (int)corr + encBalance + yawBias;
            pwmL = constrain(throttle - bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            pwmR = constrain(throttle + bias, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            leftMotor.drive(pwmL); rightMotor.drive(pwmR);
        } else if (runPhase == PH_PIVOT) {
            // Pivot: one wheel drives, the other brakes. Clamp throttle ≥ 0
            // so an overshoot doesn't reverse the active wheel into a wobble.
            int pivotThrottle = (throttle > 0) ? throttle : 0;
            if (runTurnDir == TURN_RIGHT) {
                pwmL = pivotThrottle; pwmR = 0;
                if (pivotThrottle > 0) leftMotor.drive(pwmL);
                else                    leftMotor.brake();
                rightMotor.brake();
            } else {
                pwmL = 0; pwmR = pivotThrottle;
                leftMotor.brake();
                if (pivotThrottle > 0) rightMotor.drive(pwmR);
                else                    rightMotor.brake();
            }
        } else {
            // SPOT: both wheels opposite — robot rotates about its centre.
            int sign = (runTurnDir == TURN_RIGHT) ? +1 : -1;
            pwmL = constrain( sign * throttle, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            pwmR = constrain(-sign * throttle, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
            leftMotor.drive(pwmL); rightMotor.drive(pwmR);
        }

        static uint32_t lastOled = 0;
        if (millis() - lastOled > 150) {
            oledRun((long)avg, runTarget, tL, tR);
            lastOled = millis();
        }
        if (TELEMETRY && !fastRunMode) {
            static uint32_t lastTel = 0;
            if (millis() - lastTel > 80) {
                const char* phn = (runPhase == PH_FORWARD) ? "FWD"
                                : (runPhase == PH_PIVOT)   ? "PIV" : "SPOT";
                if (runPhase == PH_FORWARD) {
                    Serial.printf("t=%lu ph=FWD/TRAP tgt=%ld xCmd=%+.0f vCmd=%+.0f avg=%+.1f err=%+.1f v=%+.1f thr=%+d pwmL=%+d pwmR=%+d yaw=%+.2f tL=%ld tR=%ld\n",
                                  (unsigned long)millis(), runTarget,
                                  xCmdSigned, vCmdSigned, avg, posErr, velFilt,
                                  throttle, pwmL, pwmR, yawDeg, tL, tR);
                } else {
                    Serial.printf("t=%lu ph=%s%s tgt=%ld avg=%+.1f err=%+.1f v=%+.1f thr=%+d pwmL=%+d pwmR=%+d yaw=%+.2f tL=%ld tR=%ld\n",
                                  (unsigned long)millis(), phn, imuMode ? "/IMU" : "",
                                  runTarget, avg, posErr, velFilt, throttle, pwmL, pwmR, yawDeg, tL, tR);
                }
                lastTel = millis();
            }
        }
        break;
    }

    case GOAL:
    case CRASH: {
        uint32_t elapsed = millis() - rgbStateT0;
        if (state == GOAL) rgbBlinkGoal(elapsed);
        else               rgbBlinkCrash(elapsed);

        if (buttonEdge()) {
            rgbOff();
            exploreMode = false; fastRunMode = false; returnHomeMode = false;
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
        }
        break;
    }

    }
}
