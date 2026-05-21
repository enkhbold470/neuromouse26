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
MicromouseEncoderPCNT leftEnc   (PCNT_UNIT_0, ENC_L_A, ENC_L_B, /*inverted=*/true);
MicromouseEncoderPCNT rightEnc  (PCNT_UNIT_1, ENC_R_A, ENC_R_B, /*inverted=*/true);

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

static void phaseEnter() {
    phaseStartTL     = leftEnc.getTicks();
    phaseStartTR     = rTicks();
    phaseStartUs     = micros();
    yawDeg           = 0.0f;
    phaseStartYawDeg = 0.0f;
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
enum State { IDLE, ENC_TEST, IR_TEST, FAST_SPEED_EDIT, EXPLORE_THINK, RUN, GOAL, CRASH };
static State state = IDLE;

// ── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_1, INPUT_PULLUP);

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
                exploreMode = true; fastRunMode = false; returnHomeMode = false;
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
                exploreMode = false; fastRunMode = true; returnHomeMode = false;
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
            case M_NVS_CLR:
                nvsClearWalls();
                Serial.println("[NVS] walls cleared");
                oledTerminal("NVS", "cleared");
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
            sampleIR();
            oledIrTest();
            Serial.printf("[IR] LF=%d L=%d R=%d RF=%d frontMm=%.1f\n",
                          irVal[0], irVal[1], irVal[2], irVal[3],
                          IRCal::estimateFrontDistMM(irVal[0], irVal[3]));
            last = millis();
        }
        if (buttonEdge()) {
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            state = IDLE;
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
        // Zero heading reference at every cell arrival to bound gyro drift
        // accumulation to a single move.
        yawDeg       = 0.0f;
        yawTargetDeg = 0.0f;
        if (exploreMode) {
            senseAndStoreWalls();
        }
        maze.visited[robotRow][robotCol] = true;
        if (maze.isGoal(robotRow, robotCol)) {
            if (exploreMode && !returnHomeMode) {
                // First leg done — flip the goal to home (0,0) and keep
                // exploring back. Walls accumulated on the return leg fill
                // in anything missed on the way out.
                Serial.printf("--- FWD GOAL reached (%d,%d), returning home ---\n",
                              robotRow, robotCol);
                maze.setGoalSingle(START_ROW, START_COL);
                returnHomeMode = true;
                // Fall through into floodFill / planning with the new goal.
            } else {
                stopMotors();
                if (exploreMode) {
                    // Restore the forward goal in the maze so the saved
                    // walls are paired with the right target for fast run.
                    maze.setGoalSingle(GOAL_ROW, GOAL_COL);
                    if (nvsSaveWalls()) Serial.println("[NVS] walls saved (round trip done)");
                }
                Serial.printf("--- DONE at (%d,%d) ---\n", robotRow, robotCol);
                oledTerminal("DONE!", returnHomeMode ? "home" : "goal");
                rgbStateT0 = millis();
                state = GOAL;
                break;
            }
        }
        maze.floodFill();
        uint8_t bestDist;
        AbsDir bestDir = maze.bestDirectionBiased(robotRow, robotCol,
                                                  (AbsDir)robotHeading, bestDist);
        if (bestDist == FLOOD_INFINITY) {
            stopMotors();
            Serial.println("[CRASH] no reachable goal — boxed in");
            oledTerminal("CRASH", "boxed");
            rgbStateT0 = millis();
            state = CRASH;
            break;
        }
        Serial.printf("[PLAN] (%d,%d,%c) dist=%d → %c\n",
                      robotRow, robotCol, "NESW"[robotHeading], bestDist, "NESW"[bestDir]);
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
                Serial.printf("--- STEP END idx=%d/%d ph=ALIGN reason=%s LF=%d RF=%d tL=%ld tR=%ld ---\n",
                              scriptIdx + 1, scriptLen, reason, irVal[0], irVal[3], tL, tR);
                alignSettleStart = 0;
                if (scriptIdx + 1 >= scriptLen) {
                    robotRow = plannedRow; robotCol = plannedCol; robotHeading = plannedHeading;
                    runTurnDir = TURN_NONE;
                    Serial.printf("--- MOVE DONE pos=(%d,%d,%c) ---\n",
                                  robotRow, robotCol, "NESW"[robotHeading]);
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
            if (TELEMETRY) {
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
            Serial.printf("--- STEP END idx=%d/%d ph=%s reason=%s err=%+.2f tL=%ld tR=%ld yaw=%+.2f ---\n",
                          scriptIdx + 1, scriptLen, phN, reason, posErr, tL, tR, yawDeg);

            if (scriptIdx + 1 >= scriptLen) {
                robotRow = plannedRow; robotCol = plannedCol; robotHeading = plannedHeading;
                resetPidState();
                runTurnDir = TURN_NONE;
                Serial.printf("--- MOVE DONE pos=(%d,%d,%c) ---\n",
                              robotRow, robotCol, "NESW"[robotHeading]);
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
            constexpr float IR_CONF_LO    = 200.0f;
            constexpr float IR_CONF_HI    = 800.0f;
            constexpr float IR_EDGE_DELTA = 300.0f;

            if (irFirstSample) {
                irLSm = irVal[1]; irRSm = irVal[2];
                irLPrev = irLSm;  irRPrev = irRSm;
                irFirstSample = false;
            }
            irLSm = 0.7f * irLSm + 0.3f * irVal[1];
            irRSm = 0.7f * irRSm + 0.3f * irVal[2];
            float dL = irLSm - irLPrev, dR = irRSm - irRPrev;
            irLPrev = irLSm; irRPrev = irRSm;
            if (dL < -IR_EDGE_DELTA) Serial.println("[EVENT] L wall opened");
            if (dR < -IR_EDGE_DELTA) Serial.println("[EVENT] R wall opened");
            if (dL >  IR_EDGE_DELTA) Serial.println("[EVENT] L wall appeared");
            if (dR >  IR_EDGE_DELTA) Serial.println("[EVENT] R wall appeared");

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
        if (TELEMETRY) {
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
