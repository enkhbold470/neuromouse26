// =============================================================================
// main.cpp — Micromouse26 single-file firmware
//
// Core: wall-follow-encoder-count-cell (IR L45/R45 centering + encoder
//        straight-keeping PID + cell-boundary detection + 90° pivot turn).
// Wrapped: flood-fill maze solver (16×16, configurable goal).
//
// FSM:
//   IDLE → button → RUN (drive 1 cell, sense walls, flood, decide) → goal/dead
//   any state → button → IDLE
//
// No Serial, no BLE, no IMU.
// =============================================================================

#include <Arduino.h>
#include <FastLED.h>

// ── Pin Config ────────────────────────────────────────────────────────────────
#define MOTOR_L_IN1     15
#define MOTOR_L_IN2     16
#define MOTOR_R_IN3     18
#define MOTOR_R_IN4     17
#define DRV_SLEEP_PIN   41

#define ENC_L_A         21
#define ENC_L_B         14
#define ENC_R_A         38
#define ENC_R_B         39

#define RX_LF           4
#define RX_L45          6
#define RX_R45          2
#define RX_RF           1

#define EMIT_LF         13
#define EMIT_L45        45
#define EMIT_R45        12
#define EMIT_RF         11

#define BUTTON_1        42
#define BUZZER_PIN      40
#define BUZZER_LEDC_CH  4
#define BUZZER_FREQ     4000
#define BUZZER_RES      8
#define BUZZER_DUTY     25
#define WS2812_DATA     3

// ── Physics ───────────────────────────────────────────────────────────────────
#define WHEEL_DIAMETER  33.4f
#define TICKS_PER_REV   210.0f
#define MOTOR_PWM_FREQ  20000
#define MOTOR_PWM_RES   10

// ── IR thresholds (no runtime calibration) ────────────────────────────────────
#define L45_CENTER      865
#define R45_CENTER      477
#define L45_THRESH      433
#define R45_THRESH      238
#define LF_THRESH       400   // tune for front wall detection
#define RF_THRESH       400   // tune for front wall detection

// ── Drive tuning ──────────────────────────────────────────────────────────────
#define BASE_PWM        250

#define WALL_KP         500.0f
#define WALL_KI         50.0f
#define WALL_KD         300.0f
#define WALL_MAX_CORR   400
#define ERROR_TRIM      0.10f

#define ENC_KP          9.0f
#define ENC_KI          0.8f
#define ENC_KD          0.5f
#define ENC_MAX_CORR    120

// ── Cell math ─────────────────────────────────────────────────────────────────
//   circumference = π × 33.4 ≈ 104.93 mm
//   ticks/mm = 210 / 104.93 ≈ 2.001
//   1 cell = 180 mm → 360 ticks
#define TICKS_PER_CELL  360L
#define CELL_PAUSE_MS   40

// ── 90° pivot turn ────────────────────────────────────────────────────────────
//   ticks = wheelTrack × ticksPerRev / (4 × wheelDiameter)
//         = 74 × 210 / (4 × 33.4) ≈ 116
#define WHEEL_TRACK_MM  74.0f
#define TICKS_PER_90    (long)(WHEEL_TRACK_MM * TICKS_PER_REV / (4.0f * WHEEL_DIAMETER))
#define TURN_PWM        200

// ── Maze constants ────────────────────────────────────────────────────────────
#define MAZE_SIZE       16
#define MAZE_CELLS      (MAZE_SIZE * MAZE_SIZE)

#define WALL_NORTH      0x01
#define WALL_EAST       0x02
#define WALL_SOUTH      0x04
#define WALL_WEST       0x08

#define FLOOD_INFINITY  255

enum AbsDir : uint8_t { DIR_NORTH = 0, DIR_EAST = 1, DIR_SOUTH = 2, DIR_WEST = 3 };

static const uint8_t DIR_WALL[4]     = { WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST };
static const uint8_t DIR_OPPOSITE[4] = { DIR_SOUTH,  DIR_WEST,  DIR_NORTH,  DIR_EAST  };
static const int8_t  DIR_DC[4]       = {  0,  1,  0, -1 };
static const int8_t  DIR_DR[4]       = {  1,  0, -1,  0 };

// =============================================================================
// MicromouseMotor — DRV8833 wrapper
// =============================================================================
class MicromouseMotor {
    uint8_t pinIN1, pinIN2;
    uint8_t ch1, ch2;
    static const int MAX_PWM   = 1023;
    static const int MIN_POWER = 150;
public:
    MicromouseMotor(uint8_t in1, uint8_t in2, uint8_t c1, uint8_t c2)
        : pinIN1(in1), pinIN2(in2), ch1(c1), ch2(c2) {}

    void begin() {
        ledcSetup(ch1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
        ledcAttachPin(pinIN1, ch1);
        ledcSetup(ch2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
        ledcAttachPin(pinIN2, ch2);
        coast();
    }

    void drive(int speed) {
        speed = constrain(speed, -MAX_PWM, MAX_PWM);
        if (speed == 0) { coast(); return; }
        if (speed > 0) {
            int pwm = map(speed, 1, MAX_PWM, MIN_POWER, MAX_PWM);
            ledcWrite(ch1, pwm); ledcWrite(ch2, 0);
        } else {
            int pwm = map(-speed, 1, MAX_PWM, MIN_POWER, MAX_PWM);
            ledcWrite(ch1, 0);   ledcWrite(ch2, pwm);
        }
    }

    void brake() { ledcWrite(ch1, MAX_PWM); ledcWrite(ch2, MAX_PWM); }
    void coast() { ledcWrite(ch1, 0);       ledcWrite(ch2, 0);       }
};

// =============================================================================
// MicromouseEncoder — single-channel RISING quadrature reader
// =============================================================================
class MicromouseEncoder {
    uint8_t       pinA, pinB;
    volatile long count;
public:
    MicromouseEncoder(uint8_t a, uint8_t b) : pinA(a), pinB(b), count(0) {}

    void begin(void (*isr)()) {
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pinA), isr, RISING);
    }

    void IRAM_ATTR handleInterrupt() {
        digitalRead(pinB) ? count++ : count--;
    }

    long getTicks() {
        noInterrupts();
        long v = count;
        interrupts();
        return v;
    }

    void reset() {
        noInterrupts();
        count = 0;
        interrupts();
    }
};

// =============================================================================
// MicromouseMaze — 16×16 flood-fill solver
// =============================================================================
class MicromouseMaze {
public:
    uint8_t walls[MAZE_SIZE][MAZE_SIZE];
    uint8_t flood[MAZE_SIZE][MAZE_SIZE];
    bool    visited[MAZE_SIZE][MAZE_SIZE];

    uint8_t goalCount;
    uint8_t goalRow[4];
    uint8_t goalCol[4];

    MicromouseMaze() { reset(); }

    bool inBounds(int r, int c) const {
        return r >= 0 && r < MAZE_SIZE && c >= 0 && c < MAZE_SIZE;
    }

    void setWall(int r, int c, AbsDir d, bool present) {
        if (!inBounds(r, c)) return;
        uint8_t bit = DIR_WALL[d];
        if (present) walls[r][c] |= bit; else walls[r][c] &= ~bit;
        int nr = r + DIR_DR[d], nc = c + DIR_DC[d];
        if (inBounds(nr, nc)) {
            uint8_t opp = DIR_WALL[DIR_OPPOSITE[d]];
            if (present) walls[nr][nc] |= opp; else walls[nr][nc] &= ~opp;
        }
    }

    bool hasWall(int r, int c, AbsDir d) const {
        if (!inBounds(r, c)) return true;
        return (walls[r][c] & DIR_WALL[d]) != 0;
    }

    void reset() {
        for (int r = 0; r < MAZE_SIZE; r++)
            for (int c = 0; c < MAZE_SIZE; c++) {
                walls[r][c]   = 0;
                flood[r][c]   = FLOOD_INFINITY;
                visited[r][c] = false;
            }
        for (int c = 0; c < MAZE_SIZE; c++) {
            setWall(0,           c, DIR_SOUTH, true);
            setWall(MAZE_SIZE-1, c, DIR_NORTH, true);
        }
        for (int r = 0; r < MAZE_SIZE; r++) {
            setWall(r, 0,           DIR_WEST, true);
            setWall(r, MAZE_SIZE-1, DIR_EAST, true);
        }
        setGoalCentre4();
    }

    void setGoalCentre4() {
        goalCount  = 4;
        goalRow[0] = 7;  goalCol[0] = 7;
        goalRow[1] = 7;  goalCol[1] = 8;
        goalRow[2] = 8;  goalCol[2] = 7;
        goalRow[3] = 8;  goalCol[3] = 8;
    }

    void setGoalSingle(uint8_t r, uint8_t c) {
        goalCount = 1;  goalRow[0] = r;  goalCol[0] = c;
    }

    bool isGoal(uint8_t r, uint8_t c) const {
        for (int i = 0; i < goalCount; i++)
            if (goalRow[i] == r && goalCol[i] == c) return true;
        return false;
    }

    void floodFill() {
        for (int r = 0; r < MAZE_SIZE; r++)
            for (int c = 0; c < MAZE_SIZE; c++)
                flood[r][c] = FLOOD_INFINITY;

        static uint8_t qRow[MAZE_CELLS];
        static uint8_t qCol[MAZE_CELLS];
        uint16_t head = 0, tail = 0;

        for (int i = 0; i < goalCount; i++) {
            flood[goalRow[i]][goalCol[i]] = 0;
            qRow[tail] = goalRow[i];
            qCol[tail] = goalCol[i];
            tail++;
        }

        while (head != tail) {
            uint8_t r = qRow[head], c = qCol[head];
            head++;
            uint8_t nextDist = flood[r][c] + 1;
            for (int d = 0; d < 4; d++) {
                if (hasWall(r, c, (AbsDir)d)) continue;
                int nr = r + DIR_DR[d], nc = c + DIR_DC[d];
                if (!inBounds(nr, nc)) continue;
                if (flood[nr][nc] <= nextDist) continue;
                flood[nr][nc] = nextDist;
                qRow[tail % MAZE_CELLS] = nr;
                qCol[tail % MAZE_CELLS] = nc;
                tail++;
            }
        }
    }

    AbsDir bestDirectionBiased(uint8_t r, uint8_t c, AbsDir h, uint8_t &bestDist) const {
        bestDist = FLOOD_INFINITY;
        AbsDir best = h;
        int bestPref = 99;
        for (int d = 0; d < 4; d++) {
            if (hasWall(r, c, (AbsDir)d)) continue;
            int nr = r + DIR_DR[d], nc = c + DIR_DC[d];
            if (!inBounds(nr, nc)) continue;
            uint8_t dist = flood[nr][nc];
            int turn = ((int)d - (int)h + 4) % 4;
            int pref;
            if      (turn == 0) pref = 0;
            else if (turn == 3) pref = 1;
            else if (turn == 1) pref = 2;
            else                pref = 3;
            if (dist < bestDist || (dist == bestDist && pref < bestPref)) {
                bestDist = dist;
                best     = (AbsDir)d;
                bestPref = pref;
            }
        }
        return best;
    }
};

// =============================================================================
// Hardware
// =============================================================================
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3);
MicromouseEncoder encLeft   (ENC_L_A, ENC_L_B);
MicromouseEncoder encRight  (ENC_R_A, ENC_R_B);
MicromouseMaze    maze;

void IRAM_ATTR isrLeft()  { encLeft.handleInterrupt();  }
void IRAM_ATTR isrRight() { encRight.handleInterrupt(); }

static const uint8_t EMIT_PINS[4] = { EMIT_LF, EMIT_L45, EMIT_R45, EMIT_RF };
static const uint8_t RX_PINS[4]   = { RX_LF,   RX_L45,   RX_R45,   RX_RF  };

#define IR_LF  0
#define IR_L45 1
#define IR_R45 2
#define IR_RF  3

// ── LEDs ──────────────────────────────────────────────────────────────────────
#define NUM_LEDS   8
#define LED_BRIGHT 5
CRGB leds[NUM_LEDS];

// =============================================================================
// IR + PID
// =============================================================================
int irRead(int idx) {
    int amb = analogRead(RX_PINS[idx]);
    digitalWrite(EMIT_PINS[idx], HIGH);
    delayMicroseconds(50);
    int lit = analogRead(RX_PINS[idx]);
    digitalWrite(EMIT_PINS[idx], LOW);
    return max(0, lit - amb);
}

float normL45(int raw) { return constrain((float)raw / L45_CENTER, 0.0f, 2.0f); }
float normR45(int raw) { return constrain((float)raw / R45_CENTER, 0.0f, 2.0f); }

struct PID {
    float kp, ki, kd, maxOut;
    float integral  = 0;
    float prevError = 0;
    unsigned long prevUs = 0;

    PID(float p, float i, float d, float mx) : kp(p), ki(i), kd(d), maxOut(mx) {}

    float compute(float error) {
        unsigned long now = micros();
        float dt = (prevUs == 0) ? 0.001f
                                 : constrain((now - prevUs) / 1e6f, 0.0001f, 0.05f);
        prevUs = now;
        integral += error * dt;
        integral  = constrain(integral, -2.0f, 2.0f);
        float deriv = (error - prevError) / dt;
        prevError = error;
        return constrain(kp * error + ki * integral + kd * deriv, -maxOut, maxOut);
    }

    void reset() { integral = 0; prevError = 0; prevUs = 0; }
};

PID wallPid(WALL_KP, WALL_KI, WALL_KD, (float)WALL_MAX_CORR);
PID encPid (ENC_KP,  ENC_KI,  ENC_KD,  (float)ENC_MAX_CORR);

// =============================================================================
// Buzzer / Button
// =============================================================================
void beep(int ms) {
    ledcWrite(BUZZER_LEDC_CH, BUZZER_DUTY);
    delay(ms);
    ledcWrite(BUZZER_LEDC_CH, 0);
}
void beepDone() { beep(60); delay(80); beep(60); }
void beepGoal() { for (int i = 0; i < 3; i++) { beep(120); delay(100); } }
void beepError(){ beep(500); }

bool buttonEdge() {
    static bool last = HIGH;
    bool cur  = digitalRead(BUTTON_1);
    bool edge = (last == HIGH && cur == LOW);
    last = cur;
    return edge;
}

void stopMotors() {
    leftMotor.brake(); rightMotor.brake();
    delay(80);
    leftMotor.coast(); rightMotor.coast();
}

// =============================================================================
// 90° pivot turn (encoder-based)
//   dir =  1 → right (L fwd, R back)
//   dir = -1 → left  (L back, R fwd)
// =============================================================================
void turn90(int dir) {
    encLeft.reset();
    encRight.reset();
    while (true) {
        long ticksL = encLeft.getTicks();
        long ticksR = encRight.getTicks();
        long avg = (dir == 1)
                   ? ( ticksL + (-ticksR)) / 2
                   : ((-ticksL) + ticksR ) / 2;
        if (avg >= TICKS_PER_90) break;
        leftMotor.drive ( dir * TURN_PWM);
        rightMotor.drive(-dir * TURN_PWM);
    }
    stopMotors();
}

void turnAround() { turn90(1); delay(80); turn90(1); }

// =============================================================================
// Robot state
// =============================================================================
bool    running   = false;
uint8_t robotRow  = 0;
uint8_t robotCol  = 0;
AbsDir  heading   = DIR_NORTH;
long    cellBaseL = 0;
long    cellBaseR = 0;

void resetCellBase() {
    cellBaseL = encLeft.getTicks();
    cellBaseR = encRight.getTicks();
}

// =============================================================================
// senseWalls — read IRs at current cell, write into maze
// =============================================================================
void senseWalls() {
    int lf  = irRead(IR_LF);
    int l45 = irRead(IR_L45);
    int r45 = irRead(IR_R45);
    int rf  = irRead(IR_RF);

    bool wallFront = (lf > LF_THRESH) || (rf > RF_THRESH);
    bool wallLeft  = (l45 > L45_THRESH);
    bool wallRight = (r45 > R45_THRESH);

    AbsDir front = heading;
    AbsDir left  = (AbsDir)((heading + 3) % 4);
    AbsDir right = (AbsDir)((heading + 1) % 4);
    AbsDir back  = (AbsDir)((heading + 2) % 4);

    maze.setWall(robotRow, robotCol, front, wallFront);
    maze.setWall(robotRow, robotCol, left,  wallLeft);
    maze.setWall(robotRow, robotCol, right, wallRight);
    maze.setWall(robotRow, robotCol, back,  false);
}

void updatePosition(AbsDir d) {
    robotRow = (uint8_t)(robotRow + DIR_DR[d]);
    robotCol = (uint8_t)(robotCol + DIR_DC[d]);
}

// =============================================================================
// driveOneCell — IR centering + encoder PID, exit at TICKS_PER_CELL
// =============================================================================
void driveOneCell() {
    resetCellBase();
    wallPid.reset();
    encPid.reset();

    while (true) {
        // ── IR wall centering ────────────────────────────────────────────
        int   l45 = irRead(IR_L45);
        int   r45 = irRead(IR_R45);
        float nL  = normL45(l45);
        float nR  = normR45(r45);
        bool  wL  = l45 > L45_THRESH;
        bool  wR  = r45 > R45_THRESH;

        float wallErr = 0.0f;
        if      (wL && wR) wallErr =  nR - nL;
        else if (wR)       wallErr =  nR - 1.0f;
        else if (wL)       wallErr = -(nL - 1.0f);
        wallErr += ERROR_TRIM;

        float wallCorr = (wL || wR) ? wallPid.compute(wallErr) : 0.0f;

        // ── Encoder straight-keeping ──────────────────────────────────────
        long tL = encLeft.getTicks()  - cellBaseL;
        long tR = encRight.getTicks() - cellBaseR;
        float encErr  = (float)(tL - tR);
        float encCorr = encPid.compute(encErr);

        // ── Cell boundary ────────────────────────────────────────────────
        long avgTicks = (tL + tR) / 2;
        if (avgTicks >= TICKS_PER_CELL) break;

        // ── Motor output ─────────────────────────────────────────────────
        float totalCorr = wallCorr + encCorr;
        int pwmL = constrain(BASE_PWM - (int)totalCorr, -1023, 1023);
        int pwmR = constrain(BASE_PWM + (int)totalCorr, -1023, 1023);
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        // allow button press to abort
        if (buttonEdge()) { running = false; break; }
    }

    stopMotors();
}

// =============================================================================
// LEDs
// =============================================================================
void ledStatus(CRGB c) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    leds[7] = c;
    FastLED.show();
}

// =============================================================================
// setup
// =============================================================================
void setup() {
    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(DRV_SLEEP_PIN, OUTPUT);
    digitalWrite(DRV_SLEEP_PIN, HIGH);

    leftMotor.begin();
    rightMotor.begin();
    encLeft.begin(isrLeft);
    encRight.begin(isrRight);

    for (int i = 0; i < 4; i++) {
        pinMode(EMIT_PINS[i], OUTPUT);
        digitalWrite(EMIT_PINS[i], LOW);
        pinMode(RX_PINS[i], INPUT);
    }

    ledcSetup(BUZZER_LEDC_CH, BUZZER_FREQ, BUZZER_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH);
    ledcWrite(BUZZER_LEDC_CH, 0);

    FastLED.addLeds<WS2812B, WS2812_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHT);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();

    maze.reset();
    maze.setGoalCentre4();   // change to setGoalSingle(r,c) for small tests
    maze.floodFill();

    beepDone();
    ledStatus(CRGB(0, 200, 200));   // cyan = idle
}

// =============================================================================
// loop — IDLE / RUN
// =============================================================================
void loop() {
    if (buttonEdge()) {
        running = !running;
        if (!running) {
            stopMotors();
            ledStatus(CRGB::Red);
            beep(60);
        } else {
            wallPid.reset();
            encPid.reset();
            encLeft.reset();
            encRight.reset();
            robotRow = 0;
            robotCol = 0;
            heading  = DIR_NORTH;
            maze.reset();
            maze.setGoalCentre4();
            maze.floodFill();
            ledStatus(CRGB::Green);
            beep(80);
        }
        delay(800);
    }

    if (!running) {
        delay(20);
        return;
    }

    // ── At current cell: sense + decide ─────────────────────────────────────
    maze.visited[robotRow][robotCol] = true;
    senseWalls();

    if (maze.isGoal(robotRow, robotCol)) {
        stopMotors();
        ledStatus(CRGB::White);
        beepGoal();
        running = false;
        return;
    }

    maze.floodFill();

    uint8_t bestDist;
    AbsDir nextDir = maze.bestDirectionBiased(robotRow, robotCol, heading, bestDist);
    if (bestDist == FLOOD_INFINITY) {
        stopMotors();
        ledStatus(CRGB::Red);
        beepError();
        running = false;
        return;
    }

    // ── Rotate to face nextDir ──────────────────────────────────────────────
    int turnSteps = ((int)nextDir - (int)heading + 4) % 4;
    switch (turnSteps) {
        case 1: turn90( 1); break;
        case 2: turnAround(); break;
        case 3: turn90(-1); break;
        default: break;
    }
    heading = nextDir;

    // ── Drive 1 cell ────────────────────────────────────────────────────────
    driveOneCell();
    if (!running) return;
    updatePosition(nextDir);
    delay(CELL_PAUSE_MS);
}
