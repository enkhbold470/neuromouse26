// test/main-with-wifi.cpp — Micromouse26 main firmware + real-time WiFi telemetry.
//
// Forked from src/main.cpp. Same motion stack (cascaded VPID + gyro-trapezoid
// turns + flood-fill solver). Bolts on:
//   • WiFi AP "MM26-debug" / "micromouse26"  (192.168.4.1, mDNS "mm26.local")
//   • Live JSON snapshot at GET /state
//   • Static HUD at GET /  (vanilla HTML+JS+Canvas — no CDN deps)
//   • Event log ring buffer at GET /log
//
// HUD shows, refreshed at ~30 Hz:
//   - 6×3 maze with explored walls, visited cells, flood distances, robot
//     pose (triangle, color-coded by state)
//   - Velocity dial: target vs measured L/R mm/s
//   - PWM bars L/R with biases broken out (FF / speed-PI / straight-PI / lateral)
//   - IR bars LF/L/R/RF with WALL_*_THRESH markers + front distance estimate
//   - Yaw needle + integrated yaw value
//   - Battery voltage + vScale factor
//   - State / cell / heading panel + scrolling event log
//
// WiFi handling is pinned to Core 0; all motion runs on Core 1 (Arduino loop).
// Telemetry struct is protected by a portMUX spinlock — the inner control loop
// writes a snapshot once per VPID tick (200 Hz) and the WiFi task reads it on
// demand. No allocations on the hot path.
//
// platformio env: `pio run -e main-with-wifi -t upload`
//   then connect phone/laptop to AP and open http://mm26.local/ (or 192.168.4.1)

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "PinConfig.h"
#include "IRCalibration.h"
#include "MicromouseMotor.h"
#include "MicromouseEncoder.h"
#include "MicromouseMaze.h"

// ─────────────────────────────────────────────────────────────────────────────
// WiFi config
// Primary: STA mode, joins existing router. Falls back to softAP if STA fails
// within STA_TIMEOUT_MS so debug HUD still reachable in unfamiliar networks.
// ─────────────────────────────────────────────────────────────────────────────
static const char* STA_SSID = "NETGEAR38";
static const char* STA_PASS = "mightygiant145";
static const char* AP_SSID  = "MM26-debug";       // fallback only
static const char* AP_PASS  = "micromouse26";
static const char* MDNS_NAME = "mm26";
static constexpr unsigned long STA_TIMEOUT_MS = 12000;
static WebServer server(80);

// ─────────────────────────────────────────────────────────────────────────────
// Smoothness overrides (debug build only — don't touch PinConfig.h).
// Lower peak ω and accel reduce overshoot at the cost of ~150 ms per turn.
// Field log showed +102° fin / -12.6° err / 42° peak overshoot with stock
// 360 dps + 1800 dps² profile — pivot saturates feed-forward before PID
// can catch the ramp. New profile is conservative.
// ─────────────────────────────────────────────────────────────────────────────
// LIVE-TUNABLE via /cmd/set. Defaults below; supervisor + viewer can overwrite.
//
// Authoritative tuned-param block — promoted from test/velocity-pid-ble.cpp
// run 2026-05-17 ("rt" log at 8.0 V showed clean 300 mm/s tracking, ±15 mm/s
// wheel error, sE within ±4 ticks). Companion VPID constants already match
// PinConfig.h (KP=1.00 KI=1.50 EMA=0.50 STR=6.00 SM=500 KVL=2.961 KVR=2.749
// RS=1.0135). Only cell_target_mms differs (300 here vs 250 in PinConfig.h);
// initialized in tune_cell_target_mms below.
static float TURN_PEAK_OMEGA_DBG  = 240.0f;
static float TURN_ACCEL_DBG       = 1100.0f;
static float TURN_KP_DBG          = 14.0f;
static float TURN_KD_DBG          = 0.55f;
static float LATERAL_BIAS_EMA_A   = 0.22f;     // was 0.35 — stronger filter for wall-transition spikes
static constexpr unsigned long VPID_RUNMS_DBG = 4000;

// Live shadows of PinConfig turn/drive constants. Initialized to PinConfig
// defaults; doTurn / driveChain reference these so /cmd/set can adjust at run
// time without rebuilding the firmware.
static float         tune_turn_kff           = TURN_KFF_PWM_PER_DPS;
static int           tune_turn_pwm_sat       = TURN_PWM;
static int           tune_turn_min_hold_pwm  = TURN_MIN_HOLD_PWM;
static float         tune_turn_deadband_deg  = TURN_DEADBAND_DEG;
static unsigned long tune_turn_hold_ms       = TURN_HOLD_MS;
static unsigned long tune_turn_timeout_ms    = TURN_TIMEOUT_MS;
static unsigned long tune_turn_settle_ms     = TURN_SETTLE_MS;
// velocity-pid-ble.cpp ran 300 mm/s cleanly IN OPEN SPACE. With walls + IR
// lateral control loop active, 300 mm/s makes wall-transitions saturate the
// posErr (trace 2026-05-18 showed pe swings ±1300 in 25 ms at 300 mm/s,
// driving the robot into the right wall before the bias EMA could converge).
// 220 mm/s gives the lateral loop time to react. Bump back up once
// pos_kp_pwm + lateral_ema are confirmed stable.
static float         tune_cell_target_mms    = 220.0f;
static int           tune_wall_side_thresh   = WALL_SIDE_THRESH;
static int           tune_wall_front_thresh  = WALL_FRONT_THRESH;
static float         tune_cell_anchor_mm     = 50.0f;
static float         tune_crash_brake_mm     = 30.0f;
// Boosted from 0.04 → 0.10. At 0.04 a pe of -1000 (drifted hard right) only
// produced -40 PWM bias, ~15 % authority vs base PWM, not enough to recover.
// At 0.10 pe of -500 already produces -50 PWM, halving recovery distance.
static float         tune_pos_kp_pwm         = 0.10f;
// Boosted from 4.0 → 9.0. Same reasoning — yaw=-7° produced only -28 PWM
// (saw straight-drive yaw oscillating ±7.8° from chassis slip). At 9.0 the
// same yaw produces -63 PWM, restoring authority over slip-induced drift.
static float         tune_yaw_kp_pwm         = 9.0f;
// Soft zone around centre — ignore tiny IR mismatches inside ±DEADBAND that
// otherwise jitter the bias signal. Also cap |posErr| so wall transitions
// don't inject ±1300-unit spikes into the EMA.
static int           tune_pos_err_deadband   = 60;
static int           tune_pos_err_clamp      = 700;
static float         tune_right_enc_scale    = RIGHT_ENC_SCALE;

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry struct — written by control loop, read by WiFi task under spinlock.
// ─────────────────────────────────────────────────────────────────────────────
struct Tele {
    uint32_t t_ms;
    const char* state;            // "IDLE" | "RUN" | "TURN" | "GOAL" | "CRASH" | ...
    const char* phase;            // sub-phase: "drive" | "advance" | "turn" | "square" | "backup"
    uint8_t  row, col;
    uint8_t  heading;             // 0=N 1=E 2=S 3=W
    float    yaw_deg;
    float    vL_mmps, vR_mmps;    // EMA velocities
    float    tgt_mmps;
    int      pwmL, pwmR;
    int      bias_ff, bias_speed, bias_straight, bias_lateral;
    long     ticksL, ticksR;
    long     cell_ref_ticks_avg;  // avg progress into current cell
    long     cell_boundary;
    int      ir[4];               // LF, L, R, RF
    float    frontMM;
    int      pos_err;
    int      raw_pos_err;
    bool     ir_fusion_valid;
    bool     slip_detected;
    float    vbat;
    float    vScale;
    float    int_speed;
    long     straight_err;
    bool     wall_front, wall_left, wall_right;
    uint8_t  flood_here;
    bool     crash_flag;
    uint8_t  walls[MAZE_SIZE][MAZE_SIZE];
    bool     visited[MAZE_SIZE][MAZE_SIZE];
    // Turn-only fields
    float    turn_target;
    float    turn_angle_des;
    float    turn_omega_des;
    float    turn_err;
    // Diagnostics — high-value debug fields surfaced to HUD.
    float    gyro_bias_dps;     // last cal result; flag if |bias| > GYRO_BIAS_WARN_DPS
    float    gyro_bias_std;     // sample std-dev during cal (radians of trust)
    bool     gyro_bias_warn;    // true if cal didn't converge cleanly
    int      no_path_retries;   // current dead-end retry counter (RUN)
    // Last 6 turn outcomes — newest at idx 0.
    struct TurnRec { float target, final_yaw, err, peak_err; bool ok; };
    TurnRec  turn_hist[6];
    int      turn_hist_count;
};
static Tele tele;
static portMUX_TYPE teleMux = portMUX_INITIALIZER_UNLOCKED;

// Event log — ring buffer of short strings. Pushed from anywhere; served as JSON.
struct LogEntry { uint32_t t_ms; char msg[80]; };
static constexpr int LOG_CAP = 32;
static LogEntry logBuf[LOG_CAP];
static volatile int logHead = 0;          // next slot to write
static volatile int logCount = 0;
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

// Time-series trace — last ~5 s of motion at 40 Hz. Lets the user (or a copy-
// paste to an LLM) reason about the full motion profile across multiple cells.
// Pushed from vpidStep at 1/5 cadence (200 Hz / 5).
struct TraceSample {
    uint32_t t_ms;
    int16_t  vL, vR, tgt;         // mm/s (int16 saves bandwidth)
    int16_t  pwmL, pwmR;
    float    yaw;
    int16_t  posErr, bias;
    int16_t  ir[4];
    int32_t  ticksAvg;            // (curL-refL + curR-refR)/2
    uint8_t  phase_id;            // 0=drive 1=advance 2=turn 3=square 4=backup
};
static constexpr int TRACE_CAP = 200;
static TraceSample traceBuf[TRACE_CAP];
static volatile int traceHead = 0;
static volatile int traceCount = 0;
static volatile uint8_t traceDiv = 0;
static portMUX_TYPE traceMux = portMUX_INITIALIZER_UNLOCKED;

// Loop-rate counter — number of vpidStep calls in last 1 s. Lets the user see
// when the WiFi task is stealing cycles from the inner loop.
static volatile uint32_t loopTicks = 0;
static volatile uint32_t loopHzMeasured = 0;
static volatile uint32_t loopWindowStartMs = 0;

static void logEvent(const char* fmt, ...) {
    char tmp[80];
    va_list ap; va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    portENTER_CRITICAL(&logMux);
    logBuf[logHead].t_ms = millis();
    strncpy(logBuf[logHead].msg, tmp, sizeof(logBuf[logHead].msg) - 1);
    logBuf[logHead].msg[sizeof(logBuf[logHead].msg) - 1] = 0;
    logHead = (logHead + 1) % LOG_CAP;
    if (logCount < LOG_CAP) logCount++;
    portEXIT_CRITICAL(&logMux);
    Serial.printf("[LOG] %s\n", tmp);
}

// Inline phase/state setters — called from motion functions to mark sub-phase.
static inline void tele_set_state(const char* s)  { portENTER_CRITICAL(&teleMux); tele.state = s; portEXIT_CRITICAL(&teleMux); }
static inline void tele_set_phase(const char* p)  { portENTER_CRITICAL(&teleMux); tele.phase = p; portEXIT_CRITICAL(&teleMux); }

// Bias above this is implausible at rest — flag for HUD warning.
static constexpr float GYRO_BIAS_WARN_DPS = 5.0f;

static uint8_t phase_id_from(const char* p) {
    if (!p) return 0;
    if (!strcmp(p, "drive"))   return 0;
    if (!strcmp(p, "advance")) return 1;
    if (!strcmp(p, "turn"))    return 2;
    if (!strcmp(p, "square"))  return 3;
    if (!strcmp(p, "backup"))  return 4;
    return 9;
}

// Single-pusher trace. Caller already holds latest control-loop snapshot.
static void tracePush(int posErr, int bias, long ticksAvg) {
    if (++traceDiv < 5) return;     // 200 Hz / 5 = 40 Hz
    traceDiv = 0;
    portENTER_CRITICAL(&traceMux);
    TraceSample& s = traceBuf[traceHead];
    s.t_ms   = tele.t_ms;
    s.vL     = (int16_t)tele.vL_mmps;
    s.vR     = (int16_t)tele.vR_mmps;
    s.tgt    = (int16_t)tele.tgt_mmps;
    s.pwmL   = (int16_t)tele.pwmL;
    s.pwmR   = (int16_t)tele.pwmR;
    s.yaw    = tele.yaw_deg;
    s.posErr = (int16_t)posErr;
    s.bias   = (int16_t)bias;
    for (int i = 0; i < 4; i++) s.ir[i] = (int16_t)tele.ir[i];
    s.ticksAvg = (int32_t)ticksAvg;
    s.phase_id = phase_id_from(tele.phase);
    traceHead = (traceHead + 1) % TRACE_CAP;
    if (traceCount < TRACE_CAP) traceCount++;
    portEXIT_CRITICAL(&traceMux);
}

// Per-call counter for loop_hz measurement (1 s rolling window).
static inline void tickLoopRate() {
    loopTicks++;
    uint32_t now = millis();
    if (loopWindowStartMs == 0) loopWindowStartMs = now;
    if (now - loopWindowStartMs >= 1000) {
        loopHzMeasured = loopTicks;
        loopTicks = 0;
        loopWindowStartMs = now;
    }
}

static inline void tele_record_turn(float target, float final_yaw,
                                    float err, float peak_err, bool ok) {
    portENTER_CRITICAL(&teleMux);
    for (int i = 5; i > 0; i--) tele.turn_hist[i] = tele.turn_hist[i - 1];
    tele.turn_hist[0] = { target, final_yaw, err, peak_err, ok };
    if (tele.turn_hist_count < 6) tele.turn_hist_count++;
    portEXIT_CRITICAL(&teleMux);
}

// ─────────────────────────────────────────────────────────────────────────────
// Persistent maze (NVS)
// ─────────────────────────────────────────────────────────────────────────────
static Preferences prefs;
static constexpr const char* NVS_NS         = "mm26";
static constexpr const char* NVS_KEY_WALLS  = "walls";

static bool hasSavedMaze() {
    if (!prefs.begin(NVS_NS, true)) return false;
    bool found = prefs.isKey(NVS_KEY_WALLS);
    prefs.end();
    return found;
}
static bool saveMazeFlash(const MicromouseMaze& m) {
    if (!prefs.begin(NVS_NS, false)) return false;
    size_t n = prefs.putBytes(NVS_KEY_WALLS, m.walls, sizeof(m.walls));
    prefs.end();
    return n == sizeof(m.walls);
}
static bool loadMazeFlash(MicromouseMaze& m) {
    if (!prefs.begin(NVS_NS, true)) return false;
    bool ok = false;
    if (prefs.isKey(NVS_KEY_WALLS)) {
        size_t n = prefs.getBytes(NVS_KEY_WALLS, m.walls, sizeof(m.walls));
        ok = (n == sizeof(m.walls));
    }
    prefs.end();
    return ok;
}
static bool clearSavedMaze() {
    if (!prefs.begin(NVS_NS, false)) return false;
    bool ok = prefs.remove(NVS_KEY_WALLS);
    prefs.end();
    return ok;
}

// ─────────────────────────────────────────────────────────────────────────────
// OLED
// ─────────────────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ─────────────────────────────────────────────────────────────────────────────
// Maze geometry
// ─────────────────────────────────────────────────────────────────────────────
constexpr uint8_t MAZE_ROWS = 6;
constexpr uint8_t MAZE_COLS = 3;
constexpr uint8_t GOAL_ROW  = 5;
constexpr uint8_t GOAL_COL  = 2;

// ─────────────────────────────────────────────────────────────────────────────
// Hardware
// ─────────────────────────────────────────────────────────────────────────────
MicromouseMotor   leftMotor (MOTOR_L_IN1, MOTOR_L_IN2, 0, 1, MOTOR_L_INV);
MicromouseMotor   rightMotor(MOTOR_R_IN3, MOTOR_R_IN4, 2, 3, MOTOR_R_INV);
MicromouseEncoder leftEnc   (ENC_L_A, ENC_L_B);
MicromouseEncoder rightEnc  (ENC_R_A, ENC_R_B);
MicromouseMaze    maze;

static inline long rTicks() { return (long)(rightEnc.getTicks() * RIGHT_ENC_SCALE); }

// ─────────────────────────────────────────────────────────────────────────────
// IR
// ─────────────────────────────────────────────────────────────────────────────
struct IRPair { uint8_t emit, rx; };
static IRPair PAIRS[4] = {
    { EMIT_LF, RX_LF },
    { EMIT_L,  RX_L  },
    { EMIT_R,  RX_R  },
    { EMIT_RF, RX_RF },
};
static int irVal[4] = {0, 0, 0, 0};

static int readIR(const IRPair& p) {
    digitalWrite(p.emit, LOW);
    delayMicroseconds(80);
    int amb = analogRead(p.rx);
    digitalWrite(p.emit, HIGH);
    delayMicroseconds(80);
    int lit = analogRead(p.rx);
    digitalWrite(p.emit, LOW);
    int d = amb - lit;
    return d < 0 ? 0 : d;
}
static void sampleIR() { for (int i = 0; i < 4; i++) irVal[i] = readIR(PAIRS[i]); }

static int calLF = IR_CAL_LF;
static int calL  = IR_CAL_L;
static int calR  = IR_CAL_R;
static int calRF = IR_CAL_RF;

static inline bool wallFront() { return irVal[0] > WALL_FRONT_THRESH || irVal[3] > WALL_FRONT_THRESH; }
static inline bool wallLeft()  { return irVal[1] > WALL_SIDE_THRESH; }
static inline bool wallRight() { return irVal[2] > WALL_SIDE_THRESH; }

// ─────────────────────────────────────────────────────────────────────────────
// Sensor fusion — IR + gyro + encoders. No single sensor is trusted alone.
//
// Why: trace 2026-05-18 showed
//   • IR posErr swung ±1300 in 25 ms at wall transitions (sensor sees front
//     wall edge as a phantom side-wall reading) — bad if used raw.
//   • Gyro yaw oscillated ±7.8° during straight drive even though encoders
//     showed matched wheel speeds — physical slip, gyro-only would conclude
//     robot is rotating when it's actually sliding sideways.
//   • Encoder L=R doesn't mean robot is going straight if wheels slip.
//
// Fusion strategy:
//   • IR posErr: clamp ±tune_pos_err_clamp, deadband ±tune_pos_err_deadband,
//     EMA-smoothed (α=0.35), 15 ms blanking after wall on/off transitions.
//     Hard-zero after 500 ms with no walls (stale).
//   • Single-wall extrapolation 1.5x (not 2.0x — symmetric assumption is
//     weak when cell geometry skewed).
//   • Heading err = gyro yaw (primary). IR side-geometry not used because
//     readings are ambient-subtracted differential, not metric distance.
//   • Slip flag = (yaw_rate != encoder_rate) for sustained windows. Set
//     after 11 consecutive disagreement cycles (~55 ms); cleared by agreement.
//     Surfaced in telemetry; future use: trigger re-calibration.
// ─────────────────────────────────────────────────────────────────────────────
struct LateralFusion {
    bool     prev_wL, prev_wR;
    uint8_t  reject_cycles;
    float    ema_pe;
    long     prev_curL, prev_curR;
    float    prev_yaw;
    int      slip_strikes;
    uint16_t no_wall_cycles;
    void reset() {
        prev_wL = prev_wR = false; reject_cycles = 0; ema_pe = 0;
        prev_curL = prev_curR = 0; prev_yaw = 0;
        slip_strikes = 0; no_wall_cycles = 0;
    }
};

struct LateralFusionOut {
    int   fused_pos_err;
    float fused_yaw_err;
    int   raw_pe;
    bool  ir_valid;
    bool  slip_detected;
    bool  wall_transition;
};

// Pure function: no globals written. `tune_pos_err_*` are read-only here.
static LateralFusionOut lateralFuse(LateralFusion& f,
                                    bool wL, bool wR, int irL, int irR,
                                    int calL_, int calR_,
                                    float gyro_yaw_deg,
                                    long curL, long curR) {
    LateralFusionOut o = {};

    // Transition detection — IR is unreliable for ~3 cycles after a side
    // wall appears or disappears (sensor geometry sweeps through edge).
    bool transition = (wL != f.prev_wL) || (wR != f.prev_wR);
    if (transition) f.reject_cycles = 3;
    if (f.reject_cycles > 0) f.reject_cycles--;
    bool ir_ok = !transition && (f.reject_cycles == 0);
    o.wall_transition = transition;

    // Raw IR posErr. Single-wall extrapolation uses 1.5x (gentler than 2x to
    // limit overshoot when only one wall present and cell is wider than nominal).
    int raw_pe = 0;
    bool any_wall = wL || wR;
    if (wL && wR)       raw_pe = (irL - calL_) - (irR - calR_);
    else if (wL)        raw_pe = ((irL - calL_) * 3) / 2;
    else if (wR)        raw_pe = -((irR - calR_) * 3) / 2;
    o.raw_pe = raw_pe;

    // Clamp wild values (saturation noise, wall-corner artifacts).
    if (raw_pe >  tune_pos_err_clamp) raw_pe =  tune_pos_err_clamp;
    if (raw_pe < -tune_pos_err_clamp) raw_pe = -tune_pos_err_clamp;
    if (abs(raw_pe) < tune_pos_err_deadband) raw_pe = 0;

    // EMA update only when IR is trustworthy. After sustained no-wall, hard
    // reset so stale bias doesn't get applied when walls reappear.
    if (any_wall) {
        f.no_wall_cycles = 0;
        if (ir_ok) f.ema_pe += 0.35f * ((float)raw_pe - f.ema_pe);
    } else {
        if (f.no_wall_cycles < 65535) f.no_wall_cycles++;
        if (f.no_wall_cycles > 100) f.ema_pe = 0;     // ~500 ms
        else                        f.ema_pe *= 0.95f;
    }
    o.fused_pos_err = (int)f.ema_pe;
    o.ir_valid      = ir_ok && any_wall;

    // Rate-based slip detection — compares dYaw vs dEnc since last call.
    float dYaw = gyro_yaw_deg - f.prev_yaw;
    f.prev_yaw = gyro_yaw_deg;
    long  dL = curL - f.prev_curL;
    long  dR = curR - f.prev_curR;
    f.prev_curL = curL; f.prev_curR = curR;
    long  enc_diff = dL - dR;
    bool yaw_rotating = fabsf(dYaw)  > 0.05f;    // > 10 dps
    bool enc_rotating = labs(enc_diff) > 4;
    if (yaw_rotating != enc_rotating) {
        if (f.slip_strikes < 30) f.slip_strikes++;
    } else if (f.slip_strikes > 0) {
        f.slip_strikes -= 2;                      // recover faster than accrue
        if (f.slip_strikes < 0) f.slip_strikes = 0;
    }
    o.slip_detected = (f.slip_strikes > 11);

    // Heading err is gyro yaw — high bandwidth, low bias after cal.
    o.fused_yaw_err = gyro_yaw_deg;

    f.prev_wL = wL; f.prev_wR = wR;
    return o;
}

// ─────────────────────────────────────────────────────────────────────────────
// Robot pose
// ─────────────────────────────────────────────────────────────────────────────
uint8_t robotRow = 0;
uint8_t robotCol = 0;
AbsDir  robotHeading = DIR_NORTH;
static bool firstCellOfRun = false;

// Crash
static bool    crashFlag = false;

// HTTP-driven control commands (set by /cmd/* handlers, applied by loop()/motion).
struct CmdState {
    volatile bool start_explore  = false;
    volatile bool start_fast     = false;
    volatile bool stop_motion    = false;
    volatile bool reset_pose     = false;
    volatile bool clear_crash    = false;
    volatile bool clear_save     = false;
    volatile bool teleport       = false;
    uint8_t       tp_row = 0, tp_col = 0, tp_heading = 0;
    volatile bool walls_override = false;
    uint8_t       walls_buf[MAZE_ROWS][MAZE_COLS];
};
static CmdState cmd;

static inline bool cmdMotionAbort() {
    return cmd.stop_motion || cmd.reset_pose;
}
static int     crashIR[4] = {0,0,0,0};
static uint8_t crashRow = 0, crashCol = 0;
static AbsDir  crashHeading = DIR_NORTH;
static const char* crashReason = "";
static bool   crashDrawn = false;

// ─────────────────────────────────────────────────────────────────────────────
// MPU-6500
// ─────────────────────────────────────────────────────────────────────────────
#define MPU_ADDR          0x68
#define REG_WHO_AM_I      0x75
#define REG_PWR_MGMT_1    0x6B
#define REG_GYRO_CFG      0x1B
#define REG_ACCEL_CFG     0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define GYRO_FS_SEL       0x10
#define GYRO_SCALE        32.8f

struct ImuRaw { int16_t ax, ay, az, temp, gx, gy, gz; };
static float gyroBiasZ = 0.0f;
static float yaw = 0.0f;
static unsigned long lastImuUs = 0;

static bool mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}
static bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }
    return true;
}
static int16_t imu_to16(uint8_t hi, uint8_t lo) { return (int16_t)((hi << 8) | lo); }
static bool imuReadAll(ImuRaw& d) {
    uint8_t b[14];
    if (!mpuRead(REG_ACCEL_XOUT_H, b, 14)) return false;
    d.ax=imu_to16(b[0],b[1]); d.ay=imu_to16(b[2],b[3]); d.az=imu_to16(b[4],b[5]);
    d.temp=imu_to16(b[6],b[7]);
    d.gx=imu_to16(b[8],b[9]); d.gy=imu_to16(b[10],b[11]); d.gz=imu_to16(b[12],b[13]);
    return true;
}
static void updateYaw() {
    ImuRaw d;
    if (!imuReadAll(d)) return;
    unsigned long now = micros();
    float dt = (lastImuUs == 0) ? 0.001f : (now - lastImuUs) / 1e6f;
    if (dt > 0.05f) dt = 0.05f;
    lastImuUs = now;
    float gz = d.gz / GYRO_SCALE - gyroBiasZ;
    if (fabsf(gz) < 0.05f) gz = 0;
    yaw += gz * dt;
}

static float readVbat() {
    int raw = analogRead(BAT_V_SENSE);
    return (raw / 4095.0f) * 3.3f * BAT_VDIV_MULT;
}

// ─────────────────────────────────────────────────────────────────────────────
// Turn profile (trapezoid)
// ─────────────────────────────────────────────────────────────────────────────
struct TurnProfile {
    float accel, peak, target;
    float t_acc, t_cru, t_tot;
    float d_acc, d_cru;
    int   sign;
    void init(float targetDeg) {
        sign   = (targetDeg >= 0) ? 1 : -1;
        target = fabsf(targetDeg);
        accel  = TURN_ACCEL_DBG;
        peak   = TURN_PEAK_OMEGA_DBG;
        float d_full_acc = (peak * peak) / (2.0f * accel);
        if (2.0f * d_full_acc >= target) {
            peak  = sqrtf(accel * target);
            d_acc = target * 0.5f;
            d_cru = 0;
        } else {
            d_acc = d_full_acc;
            d_cru = target - 2.0f * d_full_acc;
        }
        t_acc = peak / accel;
        t_cru = (peak > 0) ? d_cru / peak : 0;
        t_tot = 2.0f * t_acc + t_cru;
    }
    void at(float t, float& a_des, float& w_des) const {
        float a, w;
        if (t < t_acc) {
            w = accel * t;
            a = 0.5f * accel * t * t;
        } else if (t < t_acc + t_cru) {
            w = peak;
            a = d_acc + peak * (t - t_acc);
        } else if (t < t_tot) {
            float tp = t - t_acc - t_cru;
            w = peak - accel * tp;
            a = d_acc + d_cru + peak * tp - 0.5f * accel * tp * tp;
        } else {
            w = 0;
            a = target;
        }
        a_des = sign * a;
        w_des = sign * w;
    }
};

void stopMotors()    { leftMotor.brake(); rightMotor.brake(); }
void encodersReset() { leftEnc.reset(); rightEnc.reset(); }

// Snapshot turn sub-state for the HUD on every iteration.
static inline void tele_push_turn(float target, float a_des, float w_des, float err) {
    portENTER_CRITICAL(&teleMux);
    tele.turn_target    = target;
    tele.turn_angle_des = a_des;
    tele.turn_omega_des = w_des;
    tele.turn_err       = err;
    tele.yaw_deg        = yaw;
    portEXIT_CRITICAL(&teleMux);
}

bool doTurn(float targetDeg) {
    tele_set_phase("turn");
    logEvent("turn start tgt=%+0.1f", targetDeg);
    yaw = 0;
    lastImuUs = micros();
    TurnProfile prof; prof.init(targetDeg);

    float vbat   = readVbat();
    float vScale = (vbat > 5.5f) ? (NOMINAL_VBAT / vbat) : 1.0f;

    unsigned long t0     = millis();
    unsigned long lastUs = micros();
    float prevErr  = 0;
    float peakErr  = 0;
    unsigned long inBandStart = 0;
    bool  converged = false;

    while (true) {
        if (cmdMotionAbort()) { stopMotors(); converged = false; break; }
        updateYaw();
        unsigned long nowMs = millis();
        unsigned long nowUs = micros();
        float t  = (nowMs - t0) / 1000.0f;
        float dt = (nowUs - lastUs) / 1.0e6f;
        if (dt < 1.0e-4f) continue;
        lastUs = nowUs;

        float angle_des, omega_des;
        prof.at(t, angle_des, omega_des);

        float err  = angle_des - yaw;
        float derr = (err - prevErr) / dt;
        prevErr    = err;

        float pwm = tune_turn_kff       * omega_des * vScale
                  + TURN_KP_DBG         * err
                  + TURN_KD_DBG         * derr;

        bool holding = (t >= prof.t_tot);
        if (holding && fabsf(pwm) > 1.0f && fabsf(pwm) < tune_turn_min_hold_pwm) {
            pwm = (pwm > 0) ? tune_turn_min_hold_pwm : -tune_turn_min_hold_pwm;
        }
        if (pwm >  tune_turn_pwm_sat) pwm =  tune_turn_pwm_sat;
        if (pwm < -tune_turn_pwm_sat) pwm = -tune_turn_pwm_sat;

        leftMotor.drive(-(int)pwm);
        rightMotor.drive( (int)pwm);

        tele_push_turn(targetDeg, angle_des, omega_des, err);

        if (fabsf(err) > peakErr) peakErr = fabsf(err);

        if (holding && fabsf(targetDeg - yaw) <= tune_turn_deadband_deg) {
            if (inBandStart == 0) inBandStart = nowMs;
            if (nowMs - inBandStart >= tune_turn_hold_ms) { converged = true; break; }
        } else {
            inBandStart = 0;
        }
        if (nowMs - t0 > tune_turn_timeout_ms) break;
    }
    stopMotors();
    unsigned long settleStart = millis();
    while (millis() - settleStart < tune_turn_settle_ms) { updateYaw(); }
    bool ok = converged && fabsf(targetDeg - yaw) <= 2.0f * tune_turn_deadband_deg;
    tele_record_turn(targetDeg, yaw, targetDeg - yaw, peakErr, ok);
    logEvent("turn done fin=%+0.2f err=%+0.2f peak=%0.2f %s",
             yaw, targetDeg - yaw, peakErr, ok ? "OK" : "FAIL");
    return ok;
}

static void quickGyroRecal(unsigned ms = 80) {
    stopMotors();
    unsigned long t0 = millis();
    float sum = 0;
    int   n   = 0;
    while (millis() - t0 < ms) {
        ImuRaw d;
        if (imuReadAll(d)) { sum += d.gz / GYRO_SCALE; n++; }
        delayMicroseconds(500);
    }
    if (n > 0) gyroBiasZ = sum / n;
    yaw = 0;
    lastImuUs = micros();
}

static void squareToFrontWall(unsigned long timeoutMs = 1200) {
    tele_set_phase("square");
    constexpr float SQUARE_KP        = 0.30f;
    constexpr int   SQUARE_MAX_PWM   = 120;
    constexpr int   SQUARE_MIN_PWM   = 70;
    constexpr int   SQUARE_TOL       = 40;
    constexpr unsigned long HOLD_MS  = 150;

    unsigned long t0 = millis();
    unsigned long inBand = 0;
    while (millis() - t0 < timeoutMs) {
        sampleIR();
        if (irVal[0] < WALL_FRONT_THRESH || irVal[3] < WALL_FRONT_THRESH) break;
        int diff = irVal[0] - irVal[3];
        if (abs(diff) <= SQUARE_TOL) {
            if (inBand == 0) inBand = millis();
            if (millis() - inBand >= HOLD_MS) { stopMotors(); break; }
            stopMotors();
            delay(10);
            continue;
        } else {
            inBand = 0;
        }
        int pwm = (int)constrain(SQUARE_KP * (float)diff,
                                 (float)-SQUARE_MAX_PWM, (float)SQUARE_MAX_PWM);
        if (pwm > 0 && pwm < SQUARE_MIN_PWM) pwm = SQUARE_MIN_PWM;
        if (pwm < 0 && pwm > -SQUARE_MIN_PWM) pwm = -SQUARE_MIN_PWM;
        constexpr float kV_avg = 0.5f * (KV_L + KV_R);
        leftMotor.drive( (int)( pwm * (kV_avg / KV_L)));
        rightMotor.drive((int)(-pwm * (kV_avg / KV_R)));
        delay(8);
    }
    stopMotors();
    yaw = 0;
    lastImuUs = micros();
}

bool turnRight() { quickGyroRecal();  return doTurn(-90); }
bool turnLeft()  { quickGyroRecal();  return doTurn(+90); }
bool turnAround() {
    quickGyroRecal();
    bool ok1 = doTurn(-90);
    float residual = -90.0f - yaw;
    delay(80);
    quickGyroRecal();
    bool ok2 = doTurn(-90.0f + residual);
    return ok1 && ok2;
}

// ─────────────────────────────────────────────────────────────────────────────
// Cascaded velocity PI — with telemetry pushed each cycle
// ─────────────────────────────────────────────────────────────────────────────
struct VpidState {
    float intSpeed;
    float velL_ema;
    float velR_ema;
    long  prevL;
    long  prevR;
    float vScale;
    void reset(float vbat) {
        intSpeed = 0;
        velL_ema = velR_ema = 0;
        prevL = prevR = 0;
        float vb = (vbat > 5.5f) ? vbat : NOMINAL_VBAT;
        vScale = NOMINAL_VBAT / vb;
    }
};

static int vpidFeedforward(float target) {
    if (target <= 0.0f) return 0;
    float kV_avg  = 0.5f * (KV_L + KV_R);
    float off_avg = 0.5f * (OFF_L + OFF_R);
    if (kV_avg <= 0.0f) return 0;
    float pwm = (target / kV_avg) + off_avg;
    return (int)constrain(pwm, 0.0f, (float)MOTOR_PWM_MAX);
}

// Same control math as src/main.cpp but writes intermediate terms to `tele`.
static void vpidStep(VpidState& s, float target, long curL, long curR,
                     int lateralBias, int& pwmL, int& pwmR) {
    float dt = VPID_LOOP_US / 1000000.0f;
    float instL = ((curL - s.prevL) * MM_PER_TICK) / dt;
    float instR = ((curR - s.prevR) * MM_PER_TICK) / dt;
    s.prevL = curL; s.prevR = curR;
    s.velL_ema += VPID_EMA_ALPHA * (instL - s.velL_ema);
    s.velR_ema += VPID_EMA_ALPHA * (instR - s.velR_ema);

    float velAvg   = 0.5f * (s.velL_ema + s.velR_ema);
    float errSpeed = target - velAvg;
    s.intSpeed = constrain(s.intSpeed + errSpeed * dt,
                           -VPID_INTEG_LIM, VPID_INTEG_LIM);
    int   pidSpeed = (int)(VPID_LOOP_KP * errSpeed + VPID_LOOP_KI * s.intSpeed);

    long  straightErr = curL - curR;
    int   pidStraight = (int)constrain(
        (float)(VPID_STRAIGHT_KP * straightErr),
        (float)-VPID_STRAIGHT_MAX, (float)VPID_STRAIGHT_MAX);

    int ffBase = (int)(vpidFeedforward(target) * s.vScale);

    int basePwm = ffBase + pidSpeed;
    pwmL = constrain(basePwm - pidStraight + L_PWM_BIAS + lateralBias,
                     0, MOTOR_PWM_MAX);
    pwmR = constrain(basePwm + pidStraight + R_PWM_BIAS - lateralBias,
                     0, MOTOR_PWM_MAX);

    // Telemetry — single critical section per cycle.
    portENTER_CRITICAL(&teleMux);
    tele.t_ms          = millis();
    tele.vL_mmps       = s.velL_ema;
    tele.vR_mmps       = s.velR_ema;
    tele.tgt_mmps      = target;
    tele.pwmL          = pwmL;
    tele.pwmR          = pwmR;
    tele.ticksL        = curL;
    tele.ticksR        = curR;
    tele.bias_ff       = ffBase;
    tele.bias_speed    = pidSpeed;
    tele.bias_straight = pidStraight;
    tele.bias_lateral  = lateralBias;
    tele.int_speed     = s.intSpeed;
    tele.straight_err  = straightErr;
    tele.vScale        = s.vScale;
    tele.row           = robotRow;
    tele.col           = robotCol;
    tele.heading       = (uint8_t)robotHeading;
    tele.yaw_deg       = yaw;
    for (int i = 0; i < 4; i++) tele.ir[i] = irVal[i];
    portEXIT_CRITICAL(&teleMux);
    tickLoopRate();
}

// ─────────────────────────────────────────────────────────────────────────────
// Wall sensing + look-ahead
// ─────────────────────────────────────────────────────────────────────────────
void senseWallsFrontOnly() {
    maze.setWall(robotRow, robotCol, robotHeading, wallFront());
}
void senseWalls() {
    AbsDir leftDir  = (AbsDir)(((int)robotHeading + 3) % 4);
    AbsDir rightDir = (AbsDir)(((int)robotHeading + 1) % 4);
    maze.setWall(robotRow, robotCol, robotHeading, wallFront());
    maze.setWall(robotRow, robotCol, leftDir,      wallLeft());
    maze.setWall(robotRow, robotCol, rightDir,     wallRight());
}

struct SideLook {
    int  Lhigh, Llow;
    int  Rhigh, Rlow;
    int  n;
    void reset() { Lhigh = 0; Llow = INT_MAX; Rhigh = 0; Rlow = INT_MAX; n = 0; }
    void update(int L, int R) {
        if (L > Lhigh) Lhigh = L;  if (L < Llow) Llow = L;
        if (R > Rhigh) Rhigh = R;  if (R < Rlow) Rlow = R;
        n++;
    }
};
static SideLook sideLook = { 0, INT_MAX, 0, INT_MAX, 0 };

static void applyLookaheadSides() {
    AbsDir leftDir  = (AbsDir)(((int)robotHeading + 3) % 4);
    AbsDir rightDir = (AbsDir)(((int)robotHeading + 1) % 4);
    constexpr int MIN_LOOKAHEAD_SAMPLES = 3;
    if (sideLook.n >= MIN_LOOKAHEAD_SAMPLES) {
        bool hasL = (sideLook.Lhigh > WALL_SIDE_THRESH) &&
                    (sideLook.Llow  > WALL_SIDE_THRESH / 2);
        bool hasR = (sideLook.Rhigh > WALL_SIDE_THRESH) &&
                    (sideLook.Rlow  > WALL_SIDE_THRESH / 2);
        maze.setWall(robotRow, robotCol, leftDir,  hasL);
        maze.setWall(robotRow, robotCol, rightDir, hasR);
    }
    sideLook.reset();
}

void ensureFrontClearance() {
    int safeLF = min((int)(calLF * TURN_CLEAR_FRAC), 4090);
    int safeRF = min((int)(calRF * TURN_CLEAR_FRAC), 4090);
    sampleIR();
    if (irVal[0] < safeLF && irVal[3] < safeRF) return;
    tele_set_phase("backup");
    logEvent("backup before turn LF=%d RF=%d", irVal[0], irVal[3]);
    constexpr long BACKUP_LIMIT_TICKS = 120;
    long startL = leftEnc.getTicks();
    long startR = rTicks();
    leftMotor.drive(-DRIVE_PWM_MIN);
    rightMotor.drive(-DRIVE_PWM_MIN);
    unsigned long t0 = millis();
    while (millis() - t0 < 900) {
        sampleIR();
        long dL = abs(leftEnc.getTicks() - startL);
        long dR = abs(rTicks() - startR);
        if ((dL + dR) / 2 >= BACKUP_LIMIT_TICKS) break;
        if (irVal[0] < safeLF - 150 && irVal[3] < safeRF - 150) break;
    }
    stopMotors();
    delay(60);
}

bool rotateToHeading(AbsDir target) {
    int diff = ((int)target - (int)robotHeading + 4) % 4;
    if (diff == 0) return true;
    ensureFrontClearance();
    bool ok = true;
    if      (diff == 1) { ok = turnRight();  robotHeading = (AbsDir)(((int)robotHeading + 1) % 4); }
    else if (diff == 3) { ok = turnLeft();   robotHeading = (AbsDir)(((int)robotHeading + 3) % 4); }
    else if (diff == 2) { ok = turnAround(); robotHeading = (AbsDir)(((int)robotHeading + 2) % 4); }
    return ok;
}

constexpr long CENTER_ADVANCE_TICKS = TICKS_PER_CELL / 2;

void advanceToCellCenter() {
    tele_set_phase("advance");
    leftEnc.reset(); rightEnc.reset();
    long refL = leftEnc.getTicks(), refR = rTicks();
    yaw = 0; lastImuUs = micros();

    VpidState st; st.reset(readVbat());
    unsigned long nextUs = micros();
    unsigned long t0     = millis();

    constexpr int   POS_BIAS_MAX   = 150;
    constexpr int   YAW_BIAS_MAX   = 200;
    constexpr int   LATERAL_MAX    = 260;
    constexpr float CRASH_BRAKE_MM = 30.0f;

    LateralFusion fuse; fuse.reset();
    float biasEma = 0.0f;
    while (true) {
        if (cmdMotionAbort()) { stopMotors(); return; }
        while ((long)(micros() - nextUs) < 0) {}
        nextUs += VPID_LOOP_US;
        updateYaw();
        sampleIR();

        long curL = leftEnc.getTicks();
        long curR = rTicks();
        long avg  = ((curL - refL) + (curR - refR)) / 2;
        if (avg >= CENTER_ADVANCE_TICKS) break;
        if (millis() - t0 > 1500) break;

        if (wallFront() && IRCal::estimateFrontDistMM(irVal[0], irVal[3]) <= CRASH_BRAKE_MM) break;

        bool wL = irVal[1] > WALL_SIDE_THRESH;
        bool wR = irVal[2] > WALL_SIDE_THRESH;

        LateralFusionOut fout = lateralFuse(fuse, wL, wR, irVal[1], irVal[2],
                                            calL, calR, yaw, curL, curR);
        int posErr = fout.fused_pos_err;
        int posBias = (int)constrain(tune_pos_kp_pwm * (float)posErr,
                                     (float)-POS_BIAS_MAX, (float)POS_BIAS_MAX);
        int yawBias = (int)constrain(tune_yaw_kp_pwm * fout.fused_yaw_err,
                                     (float)-YAW_BIAS_MAX, (float)YAW_BIAS_MAX);
        int rawBias = constrain(posBias + yawBias, -LATERAL_MAX, LATERAL_MAX);
        biasEma += LATERAL_BIAS_EMA_A * ((float)rawBias - biasEma);
        int bias = (int)biasEma;

        int pwmL, pwmR;
        vpidStep(st, tune_cell_target_mms, curL, curR, bias, pwmL, pwmR);
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        portENTER_CRITICAL(&teleMux);
        tele.pos_err            = posErr;
        tele.raw_pos_err        = fout.raw_pe;
        tele.ir_fusion_valid    = fout.ir_valid;
        tele.slip_detected      = fout.slip_detected;
        tele.cell_ref_ticks_avg = avg;
        tele.cell_boundary      = CENTER_ADVANCE_TICKS;
        tele.frontMM            = wallFront() ? IRCal::estimateFrontDistMM(irVal[0], irVal[3]) : 999.0f;
        tele.wall_front         = wallFront();
        tele.wall_left          = wL;
        tele.wall_right         = wR;
        tele.vbat               = readVbat();
        portEXIT_CRITICAL(&teleMux);
        tracePush(posErr, bias, avg);
    }
    stopMotors();
}

void driveChain() {
    tele_set_phase("drive");
    leftEnc.reset(); rightEnc.reset();
    long refL = leftEnc.getTicks(), refR = rTicks();
    yaw = 0; lastImuUs = micros();
    sideLook.reset();

    VpidState st; st.reset(readVbat());
    unsigned long nextUs  = micros();
    constexpr long FIRST_CELL_EXTRA = (long)(40.0f / MM_PER_TICK);
    long          cellBoundary = firstCellOfRun
        ? TICKS_PER_CELL + FIRST_CELL_EXTRA
        : TICKS_PER_CELL;
    const long LOOKAHEAD_START_TICKS = TICKS_PER_CELL / 5;
    unsigned long startMs = millis();

    constexpr int   POS_BIAS_MAX = 150;
    constexpr int   YAW_BIAS_MAX = 200;
    constexpr int   LATERAL_MAX  = 260;

    constexpr float CRASH_BRAKE_MM  = 30.0f;
    constexpr float CELL_ANCHOR_MM  = 50.0f;

    LateralFusion fuse; fuse.reset();
    float biasEma = 0.0f;
    while (true) {
        if (cmdMotionAbort()) { stopMotors(); return; }
        while ((long)(micros() - nextUs) < 0) {}
        nextUs += VPID_LOOP_US;
        updateYaw();

        long curL = leftEnc.getTicks();
        long curR = rTicks();
        long avg  = ((curL - refL) + (curR - refR)) / 2;

        sampleIR();

        bool  frontWall = wallFront();
        float frontMM   = frontWall
            ? IRCal::estimateFrontDistMM(irVal[0], irVal[3])
            : 999.0f;

        constexpr float SIDE_POISON_MM = 60.0f;
        if (avg > LOOKAHEAD_START_TICKS && frontMM > SIDE_POISON_MM) {
            sideLook.update(irVal[1], irVal[2]);
        }

        bool crashBrake = frontWall && frontMM <= CRASH_BRAKE_MM;
        bool irAnchor   = frontWall && frontMM <= CELL_ANCHOR_MM;
        bool encAnchor  = (avg >= cellBoundary);

        if (crashBrake || irAnchor || encAnchor) {
            stopMotors();
            if (CELL_STOP_DELAY_MS > 0) delay(CELL_STOP_DELAY_MS);
            robotRow += DIR_DR[robotHeading];
            robotCol += DIR_DC[robotHeading];
            maze.visited[robotRow][robotCol] = true;
            AbsDir back = (AbsDir)(((int)robotHeading + 2) % 4);
            maze.setWall(robotRow, robotCol, back, false);
            sampleIR();
            if (wallFront()) {
                float fdMM = IRCal::estimateFrontDistMM(irVal[0], irVal[3]);
                if (fdMM > 35.0f && fdMM < 80.0f) {
                    squareToFrontWall();
                    sampleIR();
                }
            }
            senseWallsFrontOnly();
            applyLookaheadSides();
            firstCellOfRun = false;
            maze.floodFill();

            logEvent("cell r%u c%u (%s)", robotRow, robotCol,
                     crashBrake ? "crash" : irAnchor ? "ir" : "enc");

            if (maze.isGoal(robotRow, robotCol)) return;
            uint8_t bd;
            AbsDir best = maze.bestDirectionBiased(robotRow, robotCol, robotHeading, bd);
            if (bd == FLOOD_INFINITY || best != robotHeading) return;
            if (crashBrake) return;

            float seed = 0;
            int   align = 0;
            bool  wLn = wallLeft();
            bool  wRn = wallRight();
            if (wLn && wRn) align = (irVal[1] - calL) - (irVal[2] - calR);
            else if (wLn)   align =  (irVal[1] - calL);
            else if (wRn)   align = -(irVal[2] - calR);
            seed = constrain(0.005f * (float)align, -5.0f, 5.0f);

            refL = leftEnc.getTicks();
            refR = rTicks();
            cellBoundary = TICKS_PER_CELL;
            yaw          = seed;
            lastImuUs    = micros();
            st.reset(readVbat());
            st.prevL = curL; st.prevR = curR;
            biasEma      = 0.0f;
            fuse.reset();
            tele_set_phase("drive");
            continue;
        }

        bool wL = irVal[1] > WALL_SIDE_THRESH;
        bool wR = irVal[2] > WALL_SIDE_THRESH;

        LateralFusionOut fout = lateralFuse(fuse, wL, wR, irVal[1], irVal[2],
                                            calL, calR, yaw, curL, curR);
        int posErr = fout.fused_pos_err;
        int posBias = (int)constrain(tune_pos_kp_pwm * (float)posErr,
                                     (float)-POS_BIAS_MAX, (float)POS_BIAS_MAX);
        int yawBias = (int)constrain(tune_yaw_kp_pwm * fout.fused_yaw_err,
                                     (float)-YAW_BIAS_MAX, (float)YAW_BIAS_MAX);
        int rawBias = constrain(posBias + yawBias, -LATERAL_MAX, LATERAL_MAX);
        biasEma += LATERAL_BIAS_EMA_A * ((float)rawBias - biasEma);
        int lateralBias = (int)biasEma;
        if (fout.slip_detected) {
            static uint32_t lastSlipLog = 0;
            if (millis() - lastSlipLog > 500) {
                logEvent("SLIP detected (yaw vs enc disagree)");
                lastSlipLog = millis();
            }
        }
        if (fout.wall_transition) {
            static uint32_t lastTransLog = 0;
            if (millis() - lastTransLog > 200) {
                logEvent("wall transition wL=%d wR=%d", wL, wR);
                lastTransLog = millis();
            }
        }

        int pwmL, pwmR;
        vpidStep(st, tune_cell_target_mms, curL, curR, lateralBias, pwmL, pwmR);
        leftMotor.drive(pwmL);
        rightMotor.drive(pwmR);

        portENTER_CRITICAL(&teleMux);
        tele.pos_err            = posErr;
        tele.raw_pos_err        = fout.raw_pe;
        tele.ir_fusion_valid    = fout.ir_valid;
        tele.slip_detected      = fout.slip_detected;
        tele.cell_ref_ticks_avg = avg;
        tele.cell_boundary      = cellBoundary;
        tele.frontMM            = frontMM;
        tele.wall_front         = frontWall;
        tele.wall_left          = wL;
        tele.wall_right         = wR;
        tele.vbat               = readVbat();
        portEXIT_CRITICAL(&teleMux);
        tracePush(posErr, lateralBias, avg);

        if (millis() - startMs > (unsigned long)(TIMEOUT_MS * 10)) {
            stopMotors();
            return;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// State + menu (mirror src/main.cpp)
// ─────────────────────────────────────────────────────────────────────────────
enum State { IDLE, TEST_MOTOR, TEST_ENC, TEST_IR, RUN, GOAL, CRASH };
State robotState = IDLE;

enum MenuItem {
    M_TEST_MOTOR = 0, M_TEST_ENC, M_TEST_IR,
    M_START, M_FAST, M_CLEAR_SAVE,
    M_COUNT
};
static const char* MENU_LABELS[M_COUNT] = {
    "Test Motor", "Test Encoder", "Test IR",
    "START", "FAST (load)", "Clear Save"
};
static int  menuSel    = M_START;
static long menuEncRef = 0;
constexpr long ENC_PER_MENU_STEP = 20;

bool buttonEdge() {
    static unsigned long pressStart = 0;
    static bool armed = true;
    bool low = (digitalRead(BUTTON_1) == LOW);
    unsigned long now = millis();
    if (!low) { pressStart = 0; armed = true; return false; }
    if (pressStart == 0) pressStart = now;
    if (armed && (now - pressStart >= BUTTON_HOLD_MS)) {
        armed = false;
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// OLED screens (carried over from main.cpp, condensed)
// ─────────────────────────────────────────────────────────────────────────────
static void drawBatteryIcon(int x, int y) {
    float vb  = readVbat();
    float pct = constrain((vb - 6.0f) / (8.4f - 6.0f), 0.0f, 1.0f);
    constexpr int W = 14, H = 6;
    oled.drawFrame(x, y, W, H);
    oled.drawBox(x + W, y + 2, 2, H - 4);
    int fill = (int)(pct * (W - 2) + 0.5f);
    if (fill > 0) oled.drawBox(x + 1, y + 1, fill, H - 2);
}
static void drawBatteryHeader(int yBaseline) {
    char vbuf[8]; snprintf(vbuf, sizeof(vbuf), "%.1fV", readVbat());
    int vw = oled.getStrWidth(vbuf);
    int iconX = 128 - 16, iconY = yBaseline - 7;
    drawBatteryIcon(iconX, iconY);
    oled.drawStr(iconX - vw - 2, yBaseline, vbuf);
}
void oledMenu() {
    const int VIS = 5;
    int top = menuSel - VIS / 2;
    if (top < 0) top = 0;
    if (top > M_COUNT - VIS) top = M_COUNT - VIS;
    if (top < 0) top = 0;
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "MM26+WiFi");
    drawBatteryHeader(8);
    oled.drawHLine(0, 10, 128);
    const int LH = 10;
    for (int i = 0; i < VIS; i++) {
        int idx = top + i; if (idx >= M_COUNT) break;
        int y = 12 + i * LH;
        if (idx == menuSel) {
            oled.drawBox(0, y, 128, LH);
            oled.setDrawColor(0);
            oled.drawStr(3, y + 8, MENU_LABELS[idx]);
            oled.setDrawColor(1);
        } else {
            oled.drawStr(3, y + 8, MENU_LABELS[idx]);
        }
    }
    oled.sendBuffer();
}
void oledGyroCal(int prog, int total) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf); oled.drawStr(0, 10, "CAL GYRO");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_8x13B_tf); oled.drawStr(0, 32, "STILL");
    char buf[24]; snprintf(buf, sizeof(buf), "%d/%d", prog, total);
    oled.drawStr(0, 50, buf);
    oled.sendBuffer();
}
// Two-stage cal:
//   1) Warm-up: drain ~400 ms of samples without summing (MPU-6500 needs
//      ~50 ms PWR_MGMT_1 stabilize + filter settle).
//   2) Sample N readings, compute mean + std-dev, reject samples > 3·σ from
//      mean (motion/jitter), retake mean from survivors.
//   3) If |mean| > GYRO_BIAS_WARN_DPS, retry up to RETRIES; otherwise accept
//      and log warning so HUD shows a red badge.
// Previous version blindly averaged the first 300 samples — explains the
// 104 dps bias observed in the field log (sensor not stable yet).
void calibrateGyro() {
    constexpr int N        = 400;
    constexpr int WARMUP_MS = 400;
    constexpr int RETRIES   = 3;

    oledGyroCal(0, N);
    unsigned long warm = millis();
    while (millis() - warm < WARMUP_MS) { ImuRaw d; imuReadAll(d); delay(2); }

    float finalBias = 0;
    float finalStd  = 0;
    bool  warn = true;

    for (int attempt = 0; attempt < RETRIES; attempt++) {
        static float samples[N];
        int good = 0;
        for (int i = 0; i < N; i++) {
            ImuRaw d;
            if (imuReadAll(d)) {
                samples[good++] = d.gz / GYRO_SCALE;
            }
            if ((i & 0x3F) == 0) oledGyroCal(i, N);
            delay(2);
        }
        if (good == 0) { finalBias = 0; break; }

        float sum = 0;
        for (int i = 0; i < good; i++) sum += samples[i];
        float mean = sum / good;
        float varSum = 0;
        for (int i = 0; i < good; i++) {
            float d = samples[i] - mean;
            varSum += d * d;
        }
        float std = sqrtf(varSum / good);

        // Reject > 3·σ outliers, recompute.
        float sum2 = 0; int kept = 0;
        for (int i = 0; i < good; i++) {
            if (fabsf(samples[i] - mean) <= 3.0f * std) { sum2 += samples[i]; kept++; }
        }
        float refined = (kept > 0) ? sum2 / kept : mean;
        finalBias = refined;
        finalStd  = std;

        logEvent("cal[%d] N=%d kept=%d mean=%.3f std=%.3f", attempt, good, kept, refined, std);

        if (fabsf(refined) <= GYRO_BIAS_WARN_DPS) { warn = false; break; }
        // Bad cal — give the chassis another 300 ms to settle and retry.
        delay(300);
    }

    gyroBiasZ = finalBias;
    yaw = 0;
    lastImuUs = micros();

    portENTER_CRITICAL(&teleMux);
    tele.gyro_bias_dps  = finalBias;
    tele.gyro_bias_std  = finalStd;
    tele.gyro_bias_warn = warn;
    portEXIT_CRITICAL(&teleMux);

    logEvent("gyro bias %.3f dps std %.3f %s",
             finalBias, finalStd, warn ? "WARN-HOLD-STILL" : "OK");
}
void oledEncoderTest() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf); oled.drawStr(0, 10, "Encoder Test");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "L %ld", (long)leftEnc.getTicks()); oled.drawStr(0, 32, buf);
    snprintf(buf, sizeof(buf), "R %ld", (long)rightEnc.getTicks()); oled.drawStr(0, 50, buf);
    oled.setFont(u8g2_font_5x7_tf); oled.drawStr(0, 62, "btn = back");
    oled.sendBuffer();
}
void oledMotorMsg(const char* line1, const char* line2) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf); oled.drawStr(0, 10, "Motor Test");
    oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_8x13B_tf);
    oled.drawStr(0, 32, line1);
    if (line2) oled.drawStr(0, 50, line2);
    oled.sendBuffer();
}
void oledBars() {
    static const uint8_t order[4] = { 1, 0, 3, 2 };
    static const char*   lbl  [4] = { "L", "LF", "RF", "R" };
    const int H = 52, Y0 = 62, W = 26, GAP = 6, X0 = 4;
    oled.clearBuffer();
    oled.setFont(u8g2_font_5x7_tf);
    for (int i = 0; i < 4; i++) {
        int v = irVal[order[i]];
        if (v < 0) v = 0; if (v > 4095) v = 4095;
        int h = (v * H) / 4095;
        int x = X0 + i * (W + GAP);
        oled.drawFrame(x, Y0 - H, W, H);
        if (h > 0) oled.drawBox(x, Y0 - h, W, h);
        oled.drawStr(x + (W - (int)oled.getStrWidth(lbl[i])) / 2, Y0 - H - 2, lbl[i]);
    }
    oled.sendBuffer();
}
void oledRunStatus(const char* msg) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    oled.drawStr(0, 8, "RUN"); drawBatteryHeader(8);
    oled.setFont(u8g2_font_6x12_tf); oled.drawHLine(0, 12, 128);
    char buf[24];
    snprintf(buf, sizeof(buf), "r%u c%u", robotRow, robotCol); oled.drawStr(0, 28, buf);
    snprintf(buf, sizeof(buf), "h%d d%u", (int)robotHeading, maze.flood[robotRow][robotCol]); oled.drawStr(0, 44, buf);
    if (msg) oled.drawStr(0, 62, msg);
    oled.sendBuffer();
}
void oledCrash() {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x10_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "CRASH %s", crashReason); oled.drawStr(0, 8, buf);
    oled.drawHLine(0, 10, 128);
    snprintf(buf, sizeof(buf), "r=%u c=%u h=%d", crashRow, crashCol, (int)crashHeading); oled.drawStr(0, 20, buf);
    snprintf(buf, sizeof(buf), "LF%4d L%4d", crashIR[0], crashIR[1]); oled.drawStr(0, 30, buf);
    snprintf(buf, sizeof(buf), "RF%4d R%4d", crashIR[3], crashIR[2]); oled.drawStr(0, 40, buf);
    oled.setFont(u8g2_font_5x7_tf); oled.drawStr(0, 63, "btn=back");
    oled.sendBuffer();
}
void oledCountdown(int n) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_6x12_tf); oled.drawStr(0, 10, "START"); oled.drawHLine(0, 12, 128);
    oled.setFont(u8g2_font_logisoso42_tn);
    char buf[4]; snprintf(buf, sizeof(buf), "%d", n);
    int w = oled.getStrWidth(buf);
    oled.drawStr((128 - w) / 2, 60, buf);
    oled.sendBuffer();
}
void setupMaze() {
    maze.reset();
    for (int c = 0; c < MAZE_COLS; c++) maze.setWall(MAZE_ROWS - 1, c, DIR_NORTH, true);
    for (int r = 0; r < MAZE_ROWS; r++) maze.setWall(r, MAZE_COLS - 1, DIR_EAST,  true);
    maze.setGoalSingle(GOAL_ROW, GOAL_COL);
    maze.floodFill();
}
bool setupMazeFast() {
    if (!loadMazeFlash(maze)) { setupMaze(); return false; }
    for (int r = 0; r < MAZE_SIZE; r++)
        for (int c = 0; c < MAZE_SIZE; c++) {
            maze.flood[r][c]   = FLOOD_INFINITY;
            maze.visited[r][c] = false;
        }
    maze.setGoalSingle(GOAL_ROW, GOAL_COL);
    maze.floodFill();
    return true;
}
void oledShortMsg(const char* line) {
    oled.clearBuffer();
    oled.setFont(u8g2_font_8x13B_tf);
    int w = oled.getStrWidth(line);
    oled.drawStr((128 - w) / 2, 38, line);
    oled.sendBuffer();
}
void runMotorTest() {
    const int PWM = DRIVE_PWM, DUR = 350;
    oledMotorMsg("L fwd", nullptr);  leftMotor.drive( PWM);  delay(DUR); leftMotor.coast();  delay(200);
    oledMotorMsg("L rev", nullptr);  leftMotor.drive(-PWM);  delay(DUR); leftMotor.coast();  delay(300);
    oledMotorMsg("R fwd", nullptr);  rightMotor.drive( PWM); delay(DUR); rightMotor.coast(); delay(200);
    oledMotorMsg("R rev", nullptr);  rightMotor.drive(-PWM); delay(DUR); rightMotor.coast(); delay(200);
    stopMotors();
    oledMotorMsg("done", "btn = back");
}

// ─────────────────────────────────────────────────────────────────────────────
// HTML HUD (served at /). Plain JS, polls /state every 33 ms.
// Kept in PROGMEM to save RAM.
// ─────────────────────────────────────────────────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"/>
<title>MM26 HUD</title>
<style>
  :root{--bg:#0b0e14;--panel:#141821;--accent:#7cf;--ok:#5d5;--warn:#fa3;--bad:#f55;--dim:#888;--fg:#eee;}
  *{box-sizing:border-box;font-family:ui-monospace,Menlo,Consolas,monospace;}
  body{margin:0;background:var(--bg);color:var(--fg);font-size:13px;}
  #wrap{display:grid;grid-template-columns:420px 1fr 320px;gap:8px;padding:8px;height:100vh;}
  .panel{background:var(--panel);border-radius:6px;padding:8px;overflow:auto;}
  h3{margin:0 0 6px 0;font-size:12px;color:var(--accent);text-transform:uppercase;letter-spacing:.08em;}
  canvas{background:#000;border-radius:4px;display:block;width:100%;}
  .row{display:flex;gap:6px;align-items:center;justify-content:space-between;padding:1px 0;}
  .row span:first-child{color:var(--dim);}
  .row .val{font-variant-numeric:tabular-nums;}
  .bar{position:relative;height:14px;background:#222;border-radius:3px;overflow:hidden;}
  .bar > div{position:absolute;top:0;bottom:0;left:50%;background:var(--accent);}
  .bar.signed > div{transform-origin:left;}
  .bar.unsigned > div{left:0;}
  .label{display:flex;justify-content:space-between;font-size:11px;color:var(--dim);}
  .grid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;}
  .pill{display:inline-block;padding:2px 6px;border-radius:99px;background:#222;color:var(--fg);font-size:11px;}
  .pill.ok{background:#143;color:#9f9;}
  .pill.warn{background:#421;color:#fda;}
  .pill.bad{background:#411;color:#fbb;}
  #log{font-size:11px;line-height:1.4;height:calc(100% - 30px);overflow:auto;}
  #log div{padding:1px 0;border-bottom:1px solid #1a1f2a;}
  .t{color:var(--dim);}
  hr{border:0;border-top:1px solid #222;margin:6px 0;}
</style></head><body>
<div id="wrap">
  <div class="panel">
    <h3>Maze (6×3 active region)</h3>
    <canvas id="maze" width="400" height="600"></canvas>
  </div>
  <div class="panel">
    <h3>State <span id="st" class="pill">…</span><span id="ph" class="pill">…</span><span id="gy" class="pill" style="display:none">gyro</span><span id="np" class="pill" style="display:none">retry</span></h3>
    <div class="grid">
      <div>
        <div class="label"><span>vL (mm/s)</span><span id="vL">0</span></div>
        <div class="bar signed"><div id="vLbar"></div></div>
        <div class="label"><span>vR (mm/s)</span><span id="vR">0</span></div>
        <div class="bar signed"><div id="vRbar"></div></div>
        <div class="label"><span>target</span><span id="vTgt">0</span></div>
        <hr>
        <div class="label"><span>pwmL (0-1023)</span><span id="pL">0</span></div>
        <div class="bar unsigned"><div id="pLbar"></div></div>
        <div class="label"><span>pwmR</span><span id="pR">0</span></div>
        <div class="bar unsigned"><div id="pRbar"></div></div>
      </div>
      <div>
        <div class="row"><span>FF</span><span class="val" id="bFF">0</span></div>
        <div class="row"><span>speed-PI</span><span class="val" id="bSp">0</span></div>
        <div class="row"><span>straight-PI</span><span class="val" id="bSt">0</span></div>
        <div class="row"><span>lateral</span><span class="val" id="bLat">0</span></div>
        <div class="row"><span>int(speed)</span><span class="val" id="iSp">0</span></div>
        <div class="row"><span>straight err</span><span class="val" id="sErr">0</span></div>
        <hr>
        <div class="row"><span>yaw (°)</span><span class="val" id="yaw">0</span></div>
        <div class="row"><span>vBat / vScale</span><span class="val" id="vbat">0</span></div>
        <div class="row"><span>ticks L/R</span><span class="val" id="tk">0</span></div>
        <div class="row"><span>cell prog</span><span class="val" id="cp">0</span></div>
        <div class="row"><span>front (mm)</span><span class="val" id="fmm">0</span></div>
        <div class="row"><span>pos err</span><span class="val" id="pE">0</span></div>
        <div class="row"><span>loop Hz</span><span class="val" id="lhz">0</span></div>
        <div class="row"><span>trace n</span><span class="val" id="trn">0</span></div>
        <div class="row"><span>walls F/L/R</span><span class="val" id="wlr">0/0/0</span></div>
      </div>
    </div>
    <hr>
    <h3>IR (LF L R RF)</h3>
    <canvas id="ir" width="600" height="120"></canvas>
    <hr>
    <h3>Yaw + Turn</h3>
    <canvas id="yawc" width="600" height="120"></canvas>
    <hr>
    <h3>Recent Turns</h3>
    <table id="turns" style="width:100%;border-collapse:collapse;font-size:11px;">
      <thead><tr style="color:#888;"><th align="left">tgt°</th><th align="left">fin°</th><th align="left">err°</th><th align="left">peak°</th><th align="left">ok</th></tr></thead>
      <tbody></tbody>
    </table>
    <hr>
    <h3>Gyro Cal</h3>
    <div class="row"><span>bias (dps)</span><span class="val" id="gb">–</span></div>
    <div class="row"><span>sample std</span><span class="val" id="gs">–</span></div>
  </div>
  <div class="panel">
    <h3>Control <span id="cmdMsg" style="font-size:10px;color:#9cf;float:right;font-weight:400;text-transform:none;letter-spacing:0;"></span></h3>
    <div style="display:flex;flex-wrap:wrap;gap:4px;margin-bottom:6px;">
      <button class="cbtn ok"   onclick="cmd('start')">START</button>
      <button class="cbtn bad"  onclick="cmd('stop')">STOP</button>
      <button class="cbtn"      onclick="cmd('reset')">reset</button>
      <button class="cbtn"      onclick="cmd('clear_crash')">clear crash</button>
      <button class="cbtn"      onclick="cmd('fast')">FAST</button>
      <button class="cbtn"      onclick="cmd('square')">square front</button>
      <button class="cbtn"      onclick="cmd('clear_save')">clear save</button>
    </div>
    <h3>Tests <span id="testMsg" style="font-size:10px;color:#9cf;float:right;font-weight:400;text-transform:none;letter-spacing:0;"></span></h3>
    <table style="font-size:11px;border-collapse:collapse;width:100%;">
      <tr><td class="k" style="color:#9cf">motor</td>
          <td>L<input id="t_mL" value="200" size="4"> R<input id="t_mR" value="200" size="4"> ms<input id="t_mMs" value="500" size="4"></td>
          <td><button class="cbtn" onclick="testMotor()">run</button></td></tr>
      <tr><td class="k" style="color:#9cf">turn</td>
          <td>deg<input id="t_tDeg" value="90" size="6"></td>
          <td><button class="cbtn" onclick="testTurn()">run</button>
              <button class="cbtn" onclick="testTurn(-90)" title="−90">L</button>
              <button class="cbtn" onclick="testTurn(90)" title="+90">R</button>
              <button class="cbtn" onclick="testTurn(180)">180</button></td></tr>
      <tr><td class="k" style="color:#9cf">drive</td>
          <td>cells<input id="t_dN" value="1" size="4"></td>
          <td><button class="cbtn" onclick="testDrive()">run</button></td></tr>
      <tr><td class="k" style="color:#9cf">velocity</td>
          <td>mm/s<input id="t_vM" value="200" size="5"> ms<input id="t_vMs" value="1500" size="5"></td>
          <td><button class="cbtn" onclick="testVelocity()">run</button></td></tr>
      <tr><td class="k" style="color:#9cf">IR cal</td>
          <td colspan="2">place robot in dead-end →
            <button class="cbtn" onclick="cmd('test/ir_cal')">capture 32×</button></td></tr>
      <tr><td class="k" style="color:#9cf">gyro</td>
          <td colspan="2">standstill →
            <button class="cbtn" onclick="cmd('test/gyro_recal')">recal bias</button></td></tr>
      <tr><td class="k" style="color:#9cf">buzzer</td>
          <td>f<input id="t_bF" value="2000" size="5"> ms<input id="t_bMs" value="200" size="4"></td>
          <td><button class="cbtn" onclick="testBuzzer()">beep</button></td></tr>
    </table>
    <hr>
    <h3>Live Tune <button onclick="loadTune()" style="float:right;background:#234;color:#fff;border:0;padding:2px 6px;border-radius:3px;cursor:pointer;font-size:10px;">refresh</button></h3>
    <table id="tuneTbl" style="width:100%;font-size:11px;border-collapse:collapse;"></table>
    <hr>
    <h3>Debug Bundle <button id="dump" style="float:right;background:#234;color:#fff;border:0;padding:3px 8px;border-radius:3px;cursor:pointer;font-size:11px;">Copy for Claude</button></h3>
    <textarea id="dumptext" style="display:none;width:100%;height:160px;background:#000;color:#eee;border:1px solid #234;font-size:10px;"></textarea>
    <h3>Event Log</h3>
    <div id="log"></div>
  </div>
</div>
<style>
  .cbtn{background:#23272f;color:#eee;border:1px solid #333;padding:4px 9px;border-radius:3px;cursor:pointer;font-size:11px;font-family:inherit;}
  .cbtn:hover{background:#2c333c;}
  .cbtn.ok{background:#1f4a2b;border-color:#296;color:#cfc;}
  .cbtn.bad{background:#4a1f1f;border-color:#a44;color:#fcc;}
  #tuneTbl td{padding:1px 3px;}
  #tuneTbl input{background:#0e1118;color:#eee;border:1px solid #333;padding:1px 4px;width:64px;font-family:inherit;font-size:11px;}
  #tuneTbl .k{color:#9cf;}
  #tuneTbl .live{color:#888;text-align:right;font-variant-numeric:tabular-nums;}
</style>
<script>
const $ = id => document.getElementById(id);
const sbar = (el, v, max) => { const f = Math.max(-1, Math.min(1, v / max)); el.style.width = (Math.abs(f) * 50) + '%'; el.style.left = (f >= 0 ? 50 : (50 + f * 50)) + '%'; el.style.background = f >= 0 ? 'var(--accent)' : 'var(--warn)'; };
const ubar = (el, v, max) => { el.style.width = Math.max(0, Math.min(1, v / max)) * 100 + '%'; el.style.left = '0'; };

const mazeCanvas = $('maze'), mctx = mazeCanvas.getContext('2d');
const irCanvas = $('ir'), ictx = irCanvas.getContext('2d');
const yawCanvas = $('yawc'), yctx = yawCanvas.getContext('2d');

const ROWS = 6, COLS = 3;
const CELL = 80, MARGIN = 20;
function drawMaze(s){
  const w = COLS*CELL + 2*MARGIN, h = ROWS*CELL + 2*MARGIN;
  mazeCanvas.width = w; mazeCanvas.height = h;
  mctx.fillStyle = '#000'; mctx.fillRect(0,0,w,h);
  // walls: bits N=1 E=2 S=4 W=8
  for (let r = 0; r < ROWS; r++){
    for (let c = 0; c < COLS; c++){
      const x = MARGIN + c*CELL;
      const y = MARGIN + (ROWS-1-r)*CELL;
      // visited shade
      if (s.visited[r][c]) { mctx.fillStyle = '#11202a'; mctx.fillRect(x,y,CELL,CELL); }
      // flood number
      mctx.fillStyle = '#5b6'; mctx.font = '12px monospace';
      mctx.fillText(s.flood[r][c] === 255 ? '·' : s.flood[r][c], x+4, y+14);
      // goal cell highlight
      if (r === s.goal[0] && c === s.goal[1]){
        mctx.strokeStyle = '#ff5'; mctx.lineWidth = 1;
        mctx.strokeRect(x+2, y+2, CELL-4, CELL-4);
      }
      const w8 = s.walls[r][c];
      mctx.strokeStyle = '#fc7'; mctx.lineWidth = 3;
      if (w8 & 1) { mctx.beginPath(); mctx.moveTo(x, y); mctx.lineTo(x+CELL, y); mctx.stroke(); }
      if (w8 & 2) { mctx.beginPath(); mctx.moveTo(x+CELL, y); mctx.lineTo(x+CELL, y+CELL); mctx.stroke(); }
      if (w8 & 4) { mctx.beginPath(); mctx.moveTo(x, y+CELL); mctx.lineTo(x+CELL, y+CELL); mctx.stroke(); }
      if (w8 & 8) { mctx.beginPath(); mctx.moveTo(x, y); mctx.lineTo(x, y+CELL); mctx.stroke(); }
    }
  }
  // robot
  const rx = MARGIN + s.col*CELL + CELL/2;
  const ry = MARGIN + (ROWS-1-s.row)*CELL + CELL/2;
  const heading = s.heading; // 0=N 1=E 2=S 3=W
  const ang = [-Math.PI/2, 0, Math.PI/2, Math.PI][heading];
  mctx.save();
  mctx.translate(rx, ry); mctx.rotate(ang);
  mctx.fillStyle = s.crash ? '#f55' : (s.state === 'RUN' ? '#5cf' : '#5d5');
  mctx.beginPath();
  mctx.moveTo(20, 0); mctx.lineTo(-15, 12); mctx.lineTo(-15, -12); mctx.closePath();
  mctx.fill();
  // yaw overlay (cyan line)
  mctx.strokeStyle = '#fa3'; mctx.lineWidth = 2;
  mctx.beginPath(); mctx.moveTo(0, 0);
  const yang = s.yaw * Math.PI / 180;
  mctx.lineTo(Math.cos(yang) * 28, Math.sin(yang) * 28);
  mctx.stroke();
  mctx.restore();
}

function drawIR(s){
  const w = irCanvas.clientWidth, h = 120;
  irCanvas.width = w; irCanvas.height = h;
  ictx.fillStyle = '#000'; ictx.fillRect(0, 0, w, h);
  // Physical layout, left-to-right: L, LF, RF, R.
  // Backing array irVal[] = LF(0), L(1), R(2), RF(3) — so we re-map.
  // isFront[] selects which threshold line to overlay on each bar.
  const labels  = ['L', 'LF', 'RF', 'R'];
  const order   = [1, 0, 3, 2];          // ir-array index per visual slot
  const isFront = [false, true, true, false];
  const bw = (w - 40) / 4;
  const ySide  = h - 10 - (s.thS / 4095) * (h - 20);
  const yFront = h - 10 - (s.thF / 4095) * (h - 20);
  for (let i = 0; i < 4; i++){
    const v  = s.ir[order[i]];
    const x  = 20 + i*bw;
    const bh = Math.min(1, v/4095) * (h - 20);
    const thY = isFront[i] ? yFront : ySide;
    const thV = isFront[i] ? s.thF  : s.thS;
    const overTh = v > thV;
    ictx.fillStyle = overTh ? '#fa3' : '#7cf';
    ictx.fillRect(x, h - 10 - bh, bw - 10, bh);
    ictx.fillStyle = '#fff'; ictx.font = '12px monospace';
    ictx.fillText(labels[i] + ' ' + v, x, 12);
    // per-bar threshold dashed line
    ictx.strokeStyle = isFront[i] ? '#fa3' : '#f55';
    ictx.setLineDash([4, 2]); ictx.lineWidth = 1;
    ictx.beginPath();
    ictx.moveTo(x, thY); ictx.lineTo(x + bw - 10, thY); ictx.stroke();
    ictx.setLineDash([]);
  }
}

function drawYaw(s){
  const w = yawCanvas.clientWidth, h = 120;
  yawCanvas.width = w; yawCanvas.height = h;
  yctx.fillStyle = '#000'; yctx.fillRect(0, 0, w, h);
  const cx = w/2, cy = h/2;
  // dial
  yctx.strokeStyle = '#234'; yctx.lineWidth = 1;
  yctx.beginPath(); yctx.arc(cx, cy, 50, 0, Math.PI*2); yctx.stroke();
  for (let a = 0; a < 360; a += 30){
    const r = a*Math.PI/180;
    yctx.beginPath();
    yctx.moveTo(cx + Math.cos(r)*46, cy + Math.sin(r)*46);
    yctx.lineTo(cx + Math.cos(r)*50, cy + Math.sin(r)*50);
    yctx.stroke();
  }
  // yaw needle
  const ang = s.yaw * Math.PI / 180;
  yctx.strokeStyle = '#5cf'; yctx.lineWidth = 3;
  yctx.beginPath(); yctx.moveTo(cx, cy); yctx.lineTo(cx + Math.cos(ang)*48, cy + Math.sin(ang)*48); yctx.stroke();
  // turn-desired needle
  if (s.phase === 'turn'){
    const a2 = s.turn_des * Math.PI / 180;
    yctx.strokeStyle = '#fa3'; yctx.lineWidth = 2;
    yctx.beginPath(); yctx.moveTo(cx, cy); yctx.lineTo(cx + Math.cos(a2)*48, cy + Math.sin(a2)*48); yctx.stroke();
  }
  yctx.fillStyle = '#fff'; yctx.font = '14px monospace';
  yctx.fillText('yaw ' + s.yaw.toFixed(1) + '°', 10, 18);
  if (s.phase === 'turn'){
    yctx.fillStyle = '#fa3';
    yctx.fillText('tgt ' + s.turn_target.toFixed(0) + '° des ' + s.turn_des.toFixed(1) + '°', 10, 34);
    yctx.fillStyle = '#f55';
    yctx.fillText('err ' + s.turn_err.toFixed(2) + '°', 10, 50);
  }
}

// ── Control + Tune ───────────────────────────────────────────────────────
async function cmd(path, params){
  const qs = params ? '?' + new URLSearchParams(params).toString() : '';
  const el = $('cmdMsg');
  el.textContent = path + '…';
  try{
    const r = await fetch('/cmd/' + path + qs);
    const t = await r.text();
    el.textContent = path + ' → ' + r.status + ' ' + t;
    el.style.color = r.ok ? '#9f9' : '#fbb';
  }catch(e){ el.textContent = path + ' err: ' + e.message; el.style.color = '#fbb'; }
}
const TUNE_KEYS = [
  ['turn_kp','Turn P (yaw err gain)'],
  ['turn_kd','Turn D (damping — raise to kill overshoot)'],
  ['turn_peak','Turn peak ω deg/s'],
  ['turn_accel','Turn accel deg/s²'],
  ['turn_kff','Turn KFF (PWM per deg/s)'],
  ['turn_pwm_sat','Turn PWM cap'],
  ['turn_min_hold_pwm','Min hold PWM (stiction)'],
  ['turn_deadband_deg','Turn deadband °'],
  ['turn_hold_ms','Turn hold ms (longer settles better)'],
  ['turn_timeout_ms','Turn timeout ms'],
  ['turn_settle_ms','Turn settle ms'],
  ['lateral_ema','Lateral bias EMA α'],
  ['cell_target_mms','Cell target mm/s'],
  ['wall_side_thresh','Side wall IR threshold'],
  ['wall_front_thresh','Front wall IR threshold'],
  ['cell_anchor_mm','Cell anchor stop mm'],
  ['crash_brake_mm','Crash brake mm'],
  ['pos_kp_pwm','Lateral pos KP'],
  ['yaw_kp_pwm','Yaw KP'],
  ['right_enc_scale','R encoder scale'],
];
let tuneBuilt = false;
function renderTune(tune){
  const tbl = $('tuneTbl');
  if (!tune){ if (!tuneBuilt) tbl.innerHTML = '<tr><td style="color:#888">no tune in /state</td></tr>'; return; }
  if (!tuneBuilt){
    let html = '';
    for (const [k,desc] of TUNE_KEYS){
      const v = tune[k];
      html += '<tr title="'+desc+'">' +
              '<td class="k">'+k+'</td>' +
              '<td><input id="t_'+k+'" value="'+(v!=null?v:'')+'"></td>' +
              '<td><button class="cbtn" style="padding:1px 6px;font-size:10px" onclick="applyTune(\''+k+'\')">set</button></td>' +
              '<td class="live" id="L_'+k+'">'+(v!=null?v:'—')+'</td>' +
              '</tr>';
    }
    tbl.innerHTML = html;
    tuneBuilt = true;
  } else {
    for (const [k] of TUNE_KEYS){
      const el = $('L_'+k);
      if (el) el.textContent = tune[k] != null ? tune[k] : '—';
    }
  }
}
async function applyTune(k){
  const v = $('t_'+k).value;
  await cmd('set', {k, v});
}

// ── Test runners ─────────────────────────────────────────────────────────
async function runTest(path, params){
  const el = $('testMsg');
  el.textContent = path + '…'; el.style.color = '#9cf';
  try{
    const qs = params ? '?' + new URLSearchParams(params).toString() : '';
    const r = await fetch('/cmd/' + path + qs);
    const t = await r.text();
    el.textContent = path + ' → ' + t;
    el.style.color = r.ok ? '#9f9' : '#fbb';
  }catch(e){ el.textContent = path + ' err: ' + e.message; el.style.color = '#fbb'; }
}
function testMotor(){ runTest('test/motor', {l:$('t_mL').value, r:$('t_mR').value, ms:$('t_mMs').value}); }
function testTurn(deg){ const d = (deg!=null)?deg:$('t_tDeg').value; runTest('test/turn', {deg:d}); }
function testDrive(){ runTest('test/drive', {cells:$('t_dN').value}); }
function testVelocity(){ runTest('test/velocity', {mms:$('t_vM').value, ms:$('t_vMs').value}); }
function testBuzzer(){ runTest('test/buzzer', {freq:$('t_bF').value, ms:$('t_bMs').value}); }
async function loadTune(){
  try{
    const s = await (await fetch('/state')).json();
    tuneBuilt = false;
    renderTune(s.tune);
  }catch(e){}
}
window.addEventListener('load', loadTune);

let lastLogT = 0;
async function tick(){
  try{
    const r = await fetch('/state');
    if (r.ok){
      const s = await r.json();
      $('st').textContent = s.state; $('st').className = 'pill ' + (s.crash ? 'bad' : s.state === 'RUN' ? 'ok' : '');
      $('ph').textContent = s.phase || '–';
      $('vL').textContent = s.vL.toFixed(0); sbar($('vLbar'), s.vL, 400);
      $('vR').textContent = s.vR.toFixed(0); sbar($('vRbar'), s.vR, 400);
      $('vTgt').textContent = s.tgt.toFixed(0);
      $('pL').textContent = s.pwmL; ubar($('pLbar'), s.pwmL, 1023);
      $('pR').textContent = s.pwmR; ubar($('pRbar'), s.pwmR, 1023);
      $('bFF').textContent = s.bias_ff;
      $('bSp').textContent = s.bias_speed;
      $('bSt').textContent = s.bias_straight;
      $('bLat').textContent = s.bias_lateral;
      $('iSp').textContent = s.int_speed.toFixed(1);
      $('sErr').textContent = s.straight_err;
      $('yaw').textContent = s.yaw.toFixed(2);
      $('vbat').textContent = s.vbat.toFixed(2) + ' / ' + s.vScale.toFixed(2);
      $('tk').textContent = s.ticksL + ' / ' + s.ticksR;
      $('cp').textContent = s.cell_prog + ' / ' + s.cell_bound;
      $('fmm').textContent = s.frontMM.toFixed(1);
      $('pE').textContent = s.pos_err;
      $('lhz').textContent = s.loop_hz;
      $('trn').textContent = s.trace_n;
      $('wlr').textContent = (s.wall_front?'F':'-') + '/' + (s.wall_left?'L':'-') + '/' + (s.wall_right?'R':'-');
      $('gb').textContent = s.gyro_bias.toFixed(3);
      $('gs').textContent = s.gyro_std.toFixed(3);
      const gy = $('gy');
      if (s.gyro_warn) { gy.style.display = ''; gy.className = 'pill bad'; gy.textContent = 'gyro ' + s.gyro_bias.toFixed(1) + 'dps'; }
      else gy.style.display = 'none';
      const np = $('np');
      if (s.no_path > 0) { np.style.display = ''; np.className = 'pill warn'; np.textContent = 'no-path retry ' + s.no_path; }
      else np.style.display = 'none';
      const tb = document.querySelector('#turns tbody');
      tb.innerHTML = '';
      for (const r of (s.turn_hist || [])) {
        const tr = document.createElement('tr');
        const cls = r.ok ? '#9f9' : '#fbb';
        tr.innerHTML = '<td>' + r.tgt.toFixed(1) + '</td><td>' + r.fin.toFixed(2) + '</td><td style="color:' + cls + '">' + r.err.toFixed(2) + '</td><td>' + r.peak.toFixed(1) + '</td><td style="color:' + cls + '">' + (r.ok ? 'OK' : 'FAIL') + '</td>';
        tb.appendChild(tr);
      }
      drawMaze(s); drawIR(s); drawYaw(s);
      renderTune(s.tune);
    }
  }catch(e){}
  try{
    const r2 = await fetch('/log?since=' + lastLogT);
    if (r2.ok){
      const arr = await r2.json();
      const box = $('log');
      for (const e of arr){
        const d = document.createElement('div');
        d.innerHTML = '<span class="t">' + (e.t/1000).toFixed(2) + 's</span> ' + e.m;
        box.insertBefore(d, box.firstChild);
        lastLogT = Math.max(lastLogT, e.t);
      }
      while (box.children.length > 80) box.removeChild(box.lastChild);
    }
  }catch(e){}
  setTimeout(tick, 33);
}
tick();

// Copy-Debug bundle — assembles /state + /trace + /log into a single JSON
// blob with metadata for pasting into an LLM. Falls back to a textarea if
// clipboard write is blocked (insecure-context, iOS, etc.).
$('dump').onclick = async () => {
  const btn = $('dump');
  btn.textContent = '...';
  try {
    const [s, t, l] = await Promise.all([
      fetch('/state').then(r => r.json()),
      fetch('/trace').then(r => r.json()),
      fetch('/log?since=0').then(r => r.json()),
    ]);
    const bundle = {
      generated_at: new Date().toISOString(),
      uptime_ms: s.t,
      summary: {
        state: s.state, phase: s.phase, cell: [s.row, s.col], heading: s.heading,
        goal: s.goal, yaw: s.yaw, gyro_bias_dps: s.gyro_bias,
        loop_hz: s.loop_hz, vbat: s.vbat,
        last_turn: s.turn_hist && s.turn_hist[0] ? s.turn_hist[0] : null,
        wall: { front: !!s.wall_front, left: !!s.wall_left, right: !!s.wall_right },
        ir_physical_order: s.ir_layout,
        ir_physical_values: s.ir_phys,
        thresh_side: s.thS, thresh_front: s.thF,
      },
      const: s.const,
      state: s,
      trace_40hz: t,
      events: l,
    };
    const txt = JSON.stringify(bundle, null, 2);
    let copied = false;
    if (navigator.clipboard && window.isSecureContext) {
      try { await navigator.clipboard.writeText(txt); copied = true; } catch(e){}
    }
    const ta = $('dumptext');
    ta.value = txt; ta.style.display = '';
    if (!copied) { ta.focus(); ta.select(); }
    btn.textContent = copied ? 'Copied ✓' : 'Select+Copy';
  } catch(e) {
    btn.textContent = 'ERR';
  }
  setTimeout(() => { btn.textContent = 'Copy for Claude'; }, 2000);
};
</script>
</body></html>)HTML";

// ─────────────────────────────────────────────────────────────────────────────
// HTTP handlers
// ─────────────────────────────────────────────────────────────────────────────
// ------- Control command handlers (POST/GET, URL-arg encoded) -------
static void handleCmdStart() {
    cmd.start_explore = true;
    logEvent("cmd: start (explore)");
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"start\"}");
}
static void handleCmdFast() {
    cmd.start_fast = true;
    logEvent("cmd: start (fast)");
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"fast\"}");
}
static void handleCmdStop() {
    cmd.stop_motion = true;
    logEvent("cmd: stop");
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"stop\"}");
}
static void handleCmdReset() {
    cmd.reset_pose = true;
    logEvent("cmd: reset");
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"reset\"}");
}
static void handleCmdClearCrash() {
    cmd.clear_crash = true;
    logEvent("cmd: clear_crash");
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"clear_crash\"}");
}
static void handleCmdClearSave() {
    cmd.clear_save = true;
    logEvent("cmd: clear_save");
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"clear_save\"}");
}
static void handleCmdTeleport() {
    // /cmd/teleport?row=R&col=C&heading=H   (heading 0=N,1=E,2=S,3=W)
    if (!server.hasArg("row") || !server.hasArg("col") || !server.hasArg("heading")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"need row,col,heading\"}");
        return;
    }
    int r = server.arg("row").toInt();
    int c = server.arg("col").toInt();
    int h = server.arg("heading").toInt();
    if (r < 0 || r >= MAZE_ROWS || c < 0 || c >= MAZE_COLS || h < 0 || h > 3) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"out of range\"}");
        return;
    }
    cmd.tp_row = (uint8_t)r; cmd.tp_col = (uint8_t)c; cmd.tp_heading = (uint8_t)h;
    cmd.teleport = true;
    logEvent("cmd: teleport r%d c%d h%d", r, c, h);
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"teleport\"}");
}
// /cmd/set?k=<key>&v=<value> — live tune mutable params. Returns echo of new
// value. Unknown keys -> 400.
static void handleCmdSet() {
    if (!server.hasArg("k") || !server.hasArg("v")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"need k,v\"}");
        return;
    }
    String k = server.arg("k");
    String v = server.arg("v");
    float  fv = v.toFloat();
    long   iv = v.toInt();
    bool ok = true;
    if      (k == "turn_kp")             TURN_KP_DBG = fv;
    else if (k == "turn_kd")             TURN_KD_DBG = fv;
    else if (k == "turn_peak")           TURN_PEAK_OMEGA_DBG = fv;
    else if (k == "turn_accel")          TURN_ACCEL_DBG = fv;
    else if (k == "turn_kff")            tune_turn_kff = fv;
    else if (k == "turn_pwm_sat")        tune_turn_pwm_sat = (int)iv;
    else if (k == "turn_min_hold_pwm")   tune_turn_min_hold_pwm = (int)iv;
    else if (k == "turn_deadband_deg")   tune_turn_deadband_deg = fv;
    else if (k == "turn_hold_ms")        tune_turn_hold_ms = (unsigned long)iv;
    else if (k == "turn_timeout_ms")     tune_turn_timeout_ms = (unsigned long)iv;
    else if (k == "turn_settle_ms")      tune_turn_settle_ms = (unsigned long)iv;
    else if (k == "lateral_ema")         LATERAL_BIAS_EMA_A = fv;
    else if (k == "cell_target_mms")     tune_cell_target_mms = fv;
    else if (k == "wall_side_thresh")    tune_wall_side_thresh = (int)iv;
    else if (k == "wall_front_thresh")   tune_wall_front_thresh = (int)iv;
    else if (k == "cell_anchor_mm")      tune_cell_anchor_mm = fv;
    else if (k == "crash_brake_mm")      tune_crash_brake_mm = fv;
    else if (k == "pos_kp_pwm")          tune_pos_kp_pwm = fv;
    else if (k == "yaw_kp_pwm")          tune_yaw_kp_pwm = fv;
    else if (k == "right_enc_scale")     tune_right_enc_scale = fv;
    else ok = false;
    if (!ok) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"unknown key\"}");
        return;
    }
    logEvent("set %s=%s", k.c_str(), v.c_str());
    String out = "{\"ok\":true,\"k\":\"" + k + "\",\"v\":\"" + v + "\"}";
    server.send(200, "application/json", out);
}

// /cmd/square — run squareToFrontWall once (must be IDLE).
static void handleCmdSquare() {
    if (robotState != IDLE) {
        server.send(409, "application/json", "{\"ok\":false,\"err\":\"not IDLE\"}");
        return;
    }
    squareToFrontWall();
    logEvent("cmd: square");
    server.send(200, "application/json", "{\"ok\":true}");
}

// ────── Diagnostic tests (consolidate test/*.cpp into one firmware) ──────
// All require IDLE. cmdMotionAbort() (e.g. /cmd/stop) interrupts mid-test.

// /cmd/test/motor?l=PWM&r=PWM&ms=DURATION  signed PWM -1023..1023
static void handleTestMotor() {
    if (robotState != IDLE) { server.send(409, "application/json", "{\"ok\":false,\"err\":\"not IDLE\"}"); return; }
    int l = server.hasArg("l") ? server.arg("l").toInt() : 0;
    int r = server.hasArg("r") ? server.arg("r").toInt() : 0;
    int ms = server.hasArg("ms") ? server.arg("ms").toInt() : 500;
    l = constrain(l, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    r = constrain(r, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    ms = constrain(ms, 50, 5000);
    logEvent("test motor L=%d R=%d %dms", l, r, ms);
    tele_set_state("TEST"); tele_set_phase("motor");
    leftEnc.reset(); rightEnc.reset();
    leftMotor.drive(l); rightMotor.drive(r);
    unsigned long t0 = millis();
    while (millis() - t0 < (unsigned long)ms) {
        if (cmdMotionAbort()) break;
        delay(5);
    }
    stopMotors();
    long lt = leftEnc.getTicks(), rt = rTicks();
    tele_set_state("IDLE"); tele_set_phase("menu");
    String out = "{\"ok\":true,\"l\":" + String(l) + ",\"r\":" + String(r) +
                 ",\"ms\":" + String(ms) + ",\"ticksL\":" + String(lt) +
                 ",\"ticksR\":" + String(rt) + "}";
    server.send(200, "application/json", out);
}

// /cmd/test/turn?deg=DEG    single doTurn(), no robotHeading update.
static void handleTestTurn() {
    if (robotState != IDLE) { server.send(409, "application/json", "{\"ok\":false,\"err\":\"not IDLE\"}"); return; }
    float deg = server.hasArg("deg") ? server.arg("deg").toFloat() : 90.0f;
    deg = constrain(deg, -360.0f, 360.0f);
    logEvent("test turn %+0.1f°", deg);
    tele_set_state("TEST"); tele_set_phase("turn");
    quickGyroRecal();
    bool ok = doTurn(deg);
    tele_set_state("IDLE"); tele_set_phase("menu");
    String out = "{\"ok\":" + String(ok ? "true" : "false") + ",\"deg\":" + String(deg, 1) +
                 ",\"yaw_final\":" + String(yaw, 2) + "}";
    server.send(200, "application/json", out);
}

// /cmd/test/drive?cells=N    chain through N cells without changing direction.
static void handleTestDrive() {
    if (robotState != IDLE) { server.send(409, "application/json", "{\"ok\":false,\"err\":\"not IDLE\"}"); return; }
    int n = server.hasArg("cells") ? server.arg("cells").toInt() : 1;
    n = constrain(n, 1, 16);
    logEvent("test drive %d cell(s)", n);
    tele_set_state("TEST"); tele_set_phase("drive");
    firstCellOfRun = true;
    for (int i = 0; i < n; i++) {
        if (cmdMotionAbort()) break;
        driveChain();
        if (crashFlag) break;
    }
    stopMotors();
    tele_set_state("IDLE"); tele_set_phase("menu");
    String out = "{\"ok\":true,\"cells\":" + String(n) +
                 ",\"crash\":" + (crashFlag ? "true" : "false") +
                 ",\"row\":" + String(robotRow) + ",\"col\":" + String(robotCol) + "}";
    server.send(200, "application/json", out);
}

// /cmd/test/velocity?mms=TARGET&ms=DURATION   run vpidStep at given mm/s.
static void handleTestVelocity() {
    if (robotState != IDLE) { server.send(409, "application/json", "{\"ok\":false,\"err\":\"not IDLE\"}"); return; }
    float target = server.hasArg("mms") ? server.arg("mms").toFloat() : 200.0f;
    int ms = server.hasArg("ms") ? server.arg("ms").toInt() : 1500;
    target = constrain(target, -500.0f, 500.0f);
    ms = constrain(ms, 100, 5000);
    logEvent("test velocity %0.1f mm/s for %d ms", target, ms);
    tele_set_state("TEST"); tele_set_phase("velocity");
    leftEnc.reset(); rightEnc.reset();
    VpidState st; st.reset(readVbat());
    unsigned long t0 = millis();
    unsigned long nextUs = micros();
    int pwmL = 0, pwmR = 0;
    while (millis() - t0 < (unsigned long)ms) {
        if (cmdMotionAbort()) break;
        while ((long)(micros() - nextUs) < 0) {}
        nextUs += VPID_LOOP_US;
        vpidStep(st, target, leftEnc.getTicks(), rTicks(), 0, pwmL, pwmR);
        leftMotor.drive(pwmL); rightMotor.drive(pwmR);
    }
    stopMotors();
    tele_set_state("IDLE"); tele_set_phase("menu");
    String out = "{\"ok\":true,\"mms\":" + String(target, 1) +
                 ",\"ms\":" + String(ms) +
                 ",\"vL\":" + String(st.velL_ema, 1) +
                 ",\"vR\":" + String(st.velR_ema, 1) +
                 ",\"int\":" + String(st.intSpeed, 3) + "}";
    server.send(200, "application/json", out);
}

// /cmd/test/ir_cal    32-sample mean (robot in dead-end). Writes RAM cal vars.
static void handleTestIrCal() {
    if (robotState != IDLE) { server.send(409, "application/json", "{\"ok\":false,\"err\":\"not IDLE\"}"); return; }
    logEvent("test ir_cal (dead-end placement)");
    tele_set_state("TEST"); tele_set_phase("ir_cal");
    long sLF = 0, sL = 0, sR = 0, sRF = 0;
    const int N = 32;
    for (int i = 0; i < N; i++) {
        if (cmdMotionAbort()) break;
        sampleIR();
        sLF += irVal[0]; sL += irVal[1]; sR += irVal[2]; sRF += irVal[3];
        delay(20);
    }
    calLF = sLF / N; calL = sL / N; calR = sR / N; calRF = sRF / N;
    tele_set_state("IDLE"); tele_set_phase("menu");
    String out = "{\"ok\":true,\"calLF\":" + String(calLF) + ",\"calL\":" + String(calL) +
                 ",\"calR\":" + String(calR) + ",\"calRF\":" + String(calRF) + "}";
    server.send(200, "application/json", out);
}

// /cmd/test/buzzer?freq=F&ms=D    simple audible test.
static void handleTestBuzzer() {
    int f = server.hasArg("freq") ? server.arg("freq").toInt() : 2000;
    int ms = server.hasArg("ms") ? server.arg("ms").toInt() : 200;
    f = constrain(f, 100, 8000);
    ms = constrain(ms, 20, 2000);
    tone(BUZZER_PIN, f, ms);
    server.send(200, "application/json", "{\"ok\":true}");
}

// /cmd/test/gyro_recal    re-zero gyro bias from standstill samples.
static void handleTestGyroRecal() {
    if (robotState != IDLE) { server.send(409, "application/json", "{\"ok\":false,\"err\":\"not IDLE\"}"); return; }
    tele_set_state("TEST"); tele_set_phase("gyro");
    quickGyroRecal();
    tele_set_state("IDLE"); tele_set_phase("menu");
    String out = "{\"ok\":true,\"gyroBiasDps\":" + String(gyroBiasZ * GYRO_SCALE, 4) + "}";
    server.send(200, "application/json", out);
}

static void handleCmdWalls() {
    // /cmd/walls?w=<hex>   hex string of MAZE_ROWS*MAZE_COLS bytes, row-major
    if (!server.hasArg("w")) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"need w=hex\"}");
        return;
    }
    String hex = server.arg("w");
    const int need = MAZE_ROWS * MAZE_COLS;
    if ((int)hex.length() != need * 2) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"hex length wrong\"}");
        return;
    }
    auto hx = [](char ch) -> int {
        if (ch >= '0' && ch <= '9') return ch - '0';
        if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
        if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
        return -1;
    };
    for (int i = 0; i < need; i++) {
        int hi = hx(hex[2 * i]), lo = hx(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) {
            server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad hex\"}");
            return;
        }
        cmd.walls_buf[i / MAZE_COLS][i % MAZE_COLS] = (uint8_t)((hi << 4) | lo);
    }
    cmd.walls_override = true;
    logEvent("cmd: walls override");
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"walls\"}");
}

static void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

static void handleState() {
    // Copy under lock.
    Tele t;
    portENTER_CRITICAL(&teleMux);
    t = tele;
    portEXIT_CRITICAL(&teleMux);

    // Compose JSON. Pre-size buffer for the 6×3 active region; the full 16×16
    // grids are too large to ship every frame, so we serialize only the maze
    // region the robot can actually be in.
    String out;
    out.reserve(3200);
    out += '{';
    out += "\"t\":";        out += t.t_ms;
    out += ",\"state\":\""; out += t.state ? t.state : "?"; out += "\"";
    out += ",\"phase\":\""; out += t.phase ? t.phase : "?"; out += "\"";
    out += ",\"row\":";     out += t.row;
    out += ",\"col\":";     out += t.col;
    out += ",\"heading\":"; out += t.heading;
    out += ",\"goal\":[";   out += GOAL_ROW; out += ','; out += GOAL_COL; out += ']';
    out += ",\"yaw\":";     out += String(t.yaw_deg, 2);
    out += ",\"vL\":";      out += String(t.vL_mmps, 1);
    out += ",\"vR\":";      out += String(t.vR_mmps, 1);
    out += ",\"tgt\":";     out += String(t.tgt_mmps, 1);
    out += ",\"pwmL\":";    out += t.pwmL;
    out += ",\"pwmR\":";    out += t.pwmR;
    out += ",\"bias_ff\":"; out += t.bias_ff;
    out += ",\"bias_speed\":";    out += t.bias_speed;
    out += ",\"bias_straight\":"; out += t.bias_straight;
    out += ",\"bias_lateral\":";  out += t.bias_lateral;
    out += ",\"int_speed\":";     out += String(t.int_speed, 2);
    out += ",\"straight_err\":";  out += t.straight_err;
    out += ",\"ticksL\":";        out += t.ticksL;
    out += ",\"ticksR\":";        out += t.ticksR;
    out += ",\"cell_prog\":";     out += t.cell_ref_ticks_avg;
    out += ",\"cell_bound\":";    out += t.cell_boundary;
    out += ",\"frontMM\":";       out += String(t.frontMM, 1);
    out += ",\"pos_err\":";       out += t.pos_err;
    out += ",\"raw_pos_err\":";   out += t.raw_pos_err;
    out += ",\"ir_fusion_valid\":"; out += t.ir_fusion_valid ? 1 : 0;
    out += ",\"slip_detected\":"; out += t.slip_detected ? 1 : 0;
    out += ",\"vbat\":";          out += String(t.vbat, 2);
    out += ",\"vScale\":";        out += String(t.vScale, 3);
    out += ",\"wall_front\":";    out += t.wall_front ? 1 : 0;
    out += ",\"wall_left\":";     out += t.wall_left  ? 1 : 0;
    out += ",\"wall_right\":";    out += t.wall_right ? 1 : 0;
    out += ",\"crash\":";         out += t.crash_flag ? 1 : 0;
    out += ",\"thS\":";           out += WALL_SIDE_THRESH;
    out += ",\"thF\":";           out += WALL_FRONT_THRESH;
    out += ",\"ir\":[";
    for (int i = 0; i < 4; i++) { if (i) out += ','; out += t.ir[i]; }
    out += "]";
    out += ",\"turn_target\":";   out += String(t.turn_target, 1);
    out += ",\"turn_des\":";      out += String(t.turn_angle_des, 2);
    out += ",\"turn_err\":";      out += String(t.turn_err, 2);
    out += ",\"gyro_bias\":";     out += String(t.gyro_bias_dps, 3);
    out += ",\"gyro_std\":";      out += String(t.gyro_bias_std, 3);
    out += ",\"gyro_warn\":";     out += t.gyro_bias_warn ? 1 : 0;
    out += ",\"no_path\":";       out += t.no_path_retries;
    out += ",\"loop_hz\":";       out += (uint32_t)loopHzMeasured;
    out += ",\"trace_n\":";       out += traceCount;
    out += ",\"ir_layout\":[\"L\",\"LF\",\"RF\",\"R\"]";
    out += ",\"ir_phys\":[";      // re-ordered to physical L,LF,RF,R
    out += t.ir[1]; out += ',';
    out += t.ir[0]; out += ',';
    out += t.ir[3]; out += ',';
    out += t.ir[2]; out += ']';
    out += ",\"cal\":["; out += IR_CAL_L; out += ','; out += IR_CAL_LF; out += ','; out += IR_CAL_RF; out += ','; out += IR_CAL_R; out += ']';
    // Live in-use values. cell_target_mms reads the tunable shadow so the HUD
    // and Copy-Debug bundle show what's actually running (default promoted to
    // 300 mm/s from PinConfig's 250 per velocity-pid-ble run 2026-05-17).
    out += ",\"const\":{";
    out +=   "\"cell_target_mms\":"; out += String(tune_cell_target_mms, 1);
    out += ",\"cell_target_mms_pinconfig\":"; out += CELL_TARGET_MMS;
    out += ",\"ticks_per_cell\":";   out += (long)TICKS_PER_CELL;
    out += ",\"vpid_loop_us\":";     out += (uint32_t)VPID_LOOP_US;
    out += ",\"vpid_kp\":";          out += String(VPID_LOOP_KP, 3);
    out += ",\"vpid_ki\":";          out += String(VPID_LOOP_KI, 3);
    out += ",\"vpid_integ_lim\":";   out += String(VPID_INTEG_LIM, 1);
    out += ",\"vpid_ema_alpha\":";   out += String(VPID_EMA_ALPHA, 3);
    out += ",\"vpid_straight_kp\":"; out += String(VPID_STRAIGHT_KP, 3);
    out += ",\"vpid_straight_max\":";out += VPID_STRAIGHT_MAX;
    out += ",\"l_pwm_bias\":";       out += L_PWM_BIAS;
    out += ",\"r_pwm_bias\":";       out += R_PWM_BIAS;
    out += ",\"kv_L\":";             out += String(KV_L, 3);
    out += ",\"kv_R\":";             out += String(KV_R, 3);
    out += ",\"off_L\":";            out += String(OFF_L, 3);
    out += ",\"off_R\":";            out += String(OFF_R, 3);
    out += ",\"r_enc_scale\":";      out += String(RIGHT_ENC_SCALE, 4);
    out += ",\"vpid_runms\":";       out += (uint32_t)VPID_RUNMS_DBG;
    out += ",\"turn_peak\":";        out += String(TURN_PEAK_OMEGA_DBG, 1);
    out += ",\"turn_accel\":";       out += String(TURN_ACCEL_DBG, 1);
    out += ",\"turn_kp\":";          out += String(TURN_KP_DBG, 2);
    out += ",\"turn_kd\":";          out += String(TURN_KD_DBG, 2);
    out += '}';
    // Live tune mirror (read-back so the viewer can show current values).
    out += ",\"tune\":{";
    out +=   "\"turn_kp\":";         out += String(TURN_KP_DBG, 3);
    out += ",\"turn_kd\":";          out += String(TURN_KD_DBG, 3);
    out += ",\"turn_peak\":";        out += String(TURN_PEAK_OMEGA_DBG, 1);
    out += ",\"turn_accel\":";       out += String(TURN_ACCEL_DBG, 1);
    out += ",\"turn_kff\":";         out += String(tune_turn_kff, 3);
    out += ",\"turn_pwm_sat\":";     out += tune_turn_pwm_sat;
    out += ",\"turn_min_hold_pwm\":";out += tune_turn_min_hold_pwm;
    out += ",\"turn_deadband_deg\":";out += String(tune_turn_deadband_deg, 2);
    out += ",\"turn_hold_ms\":";     out += (uint32_t)tune_turn_hold_ms;
    out += ",\"turn_timeout_ms\":";  out += (uint32_t)tune_turn_timeout_ms;
    out += ",\"turn_settle_ms\":";   out += (uint32_t)tune_turn_settle_ms;
    out += ",\"lateral_ema\":";      out += String(LATERAL_BIAS_EMA_A, 3);
    out += ",\"cell_target_mms\":";  out += String(tune_cell_target_mms, 1);
    out += ",\"wall_side_thresh\":"; out += tune_wall_side_thresh;
    out += ",\"wall_front_thresh\":";out += tune_wall_front_thresh;
    out += ",\"cell_anchor_mm\":";   out += String(tune_cell_anchor_mm, 1);
    out += ",\"crash_brake_mm\":";   out += String(tune_crash_brake_mm, 1);
    out += ",\"pos_kp_pwm\":";       out += String(tune_pos_kp_pwm, 4);
    out += ",\"yaw_kp_pwm\":";       out += String(tune_yaw_kp_pwm, 2);
    out += ",\"right_enc_scale\":";  out += String(tune_right_enc_scale, 4);
    out += '}';
    out += ",\"turn_hist\":[";
    for (int i = 0; i < t.turn_hist_count; i++) {
        if (i) out += ',';
        out += "{\"tgt\":"; out += String(t.turn_hist[i].target, 1);
        out += ",\"fin\":"; out += String(t.turn_hist[i].final_yaw, 2);
        out += ",\"err\":"; out += String(t.turn_hist[i].err, 2);
        out += ",\"peak\":"; out += String(t.turn_hist[i].peak_err, 2);
        out += ",\"ok\":"; out += t.turn_hist[i].ok ? 1 : 0;
        out += '}';
    }
    out += "]";

    // Walls + visited + flood for the active region only.
    out += ",\"walls\":[";
    for (int r = 0; r < MAZE_ROWS; r++) {
        if (r) out += ',';
        out += '[';
        for (int c = 0; c < MAZE_COLS; c++) {
            if (c) out += ',';
            out += t.walls[r][c];
        }
        out += ']';
    }
    out += "]";
    out += ",\"visited\":[";
    for (int r = 0; r < MAZE_ROWS; r++) {
        if (r) out += ',';
        out += '[';
        for (int c = 0; c < MAZE_COLS; c++) {
            if (c) out += ',';
            out += t.visited[r][c] ? 1 : 0;
        }
        out += ']';
    }
    out += "]";
    out += ",\"flood\":[";
    for (int r = 0; r < MAZE_ROWS; r++) {
        if (r) out += ',';
        out += '[';
        for (int c = 0; c < MAZE_COLS; c++) {
            if (c) out += ',';
            out += maze.flood[r][c];
        }
        out += ']';
    }
    out += "]";
    out += '}';
    server.send(200, "application/json", out);
}

static void handleTrace() {
    // Stream the ring buffer in insertion order (oldest -> newest). Each row
    // is a fixed-key JSON object so client-side parsing is trivial. Format
    // chosen for legibility when pasted into Claude — keys are short but
    // self-describing.
    String out;
    out.reserve(16384);
    out += "{\"rate_hz\":40,\"n\":";
    portENTER_CRITICAL(&traceMux);
    int n = traceCount;
    int start = (traceCount < TRACE_CAP) ? 0 : traceHead;
    out += n; out += ",\"samples\":[";
    for (int i = 0; i < n; i++) {
        int idx = (start + i) % TRACE_CAP;
        const TraceSample& s = traceBuf[idx];
        if (i) out += ',';
        out += "{\"t\":";    out += s.t_ms;
        out += ",\"ph\":";   out += s.phase_id;
        out += ",\"vL\":";   out += s.vL;
        out += ",\"vR\":";   out += s.vR;
        out += ",\"tgt\":";  out += s.tgt;
        out += ",\"pL\":";   out += s.pwmL;
        out += ",\"pR\":";   out += s.pwmR;
        out += ",\"yaw\":";  out += String(s.yaw, 2);
        out += ",\"pe\":";   out += s.posErr;
        out += ",\"bi\":";   out += s.bias;
        out += ",\"ir\":[";  out += s.ir[0]; out += ',';
                             out += s.ir[1]; out += ',';
                             out += s.ir[2]; out += ',';
                             out += s.ir[3]; out += ']';
        out += ",\"tk\":";   out += s.ticksAvg;
        out += '}';
    }
    portEXIT_CRITICAL(&traceMux);
    out += "]}";
    server.send(200, "application/json", out);
}

static void handleLog() {
    uint32_t since = 0;
    if (server.hasArg("since")) since = (uint32_t)server.arg("since").toInt();
    String out; out.reserve(2048);
    out += '[';
    bool first = true;
    portENTER_CRITICAL(&logMux);
    // Emit in insertion order (oldest -> newest).
    int start = (logCount < LOG_CAP) ? 0 : logHead;
    for (int i = 0; i < logCount; i++) {
        int idx = (start + i) % LOG_CAP;
        if (logBuf[idx].t_ms <= since) continue;
        if (!first) out += ',';
        first = false;
        out += "{\"t\":"; out += logBuf[idx].t_ms;
        out += ",\"m\":\"";
        for (const char* p = logBuf[idx].msg; *p; p++) {
            if (*p == '"' || *p == '\\') out += '\\';
            if (*p >= 32) out += *p;
        }
        out += "\"}";
    }
    portEXIT_CRITICAL(&logMux);
    out += ']';
    server.send(200, "application/json", out);
}

// Refresh maze walls/visited snapshot in `tele`. Called from RUN-state hooks
// because copying the 16×16 grids inside vpidStep would be too expensive.
static void copyMazeIntoTele() {
    portENTER_CRITICAL(&teleMux);
    memcpy(tele.walls,   maze.walls,   sizeof(maze.walls));
    memcpy(tele.visited, maze.visited, sizeof(maze.visited));
    tele.flood_here = maze.flood[robotRow][robotCol];
    tele.crash_flag = crashFlag;
    portEXIT_CRITICAL(&teleMux);
}

// ─────────────────────────────────────────────────────────────────────────────
// WiFi task (Core 0)
// ─────────────────────────────────────────────────────────────────────────────
static void wifiTask(void*) {
    // STA first — join the router so the HUD is reachable from any device on
    // the same LAN (no need to switch wifi networks on the laptop). If the
    // router is out of range or the password is wrong, fall back to softAP.
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                 // keep latency low for HUD polls
    WiFi.begin(STA_SSID, STA_PASS);
    Serial.printf("[WIFI] STA connecting to \"%s\"...\n", STA_SSID);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < STA_TIMEOUT_MS) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    IPAddress ip;
    bool sta_ok = (WiFi.status() == WL_CONNECTED);
    if (sta_ok) {
        ip = WiFi.localIP();
        Serial.printf("[WIFI] STA up at %s (rssi %d dBm)\n",
                      ip.toString().c_str(), WiFi.RSSI());
        logEvent("wifi STA %s rssi %d", ip.toString().c_str(), WiFi.RSSI());
    } else {
        Serial.println("[WIFI] STA timeout; falling back to softAP");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASS);
        ip = WiFi.softAPIP();
        Serial.printf("[WIFI] AP \"%s\" up at %s\n", AP_SSID, ip.toString().c_str());
        logEvent("wifi AP fallback %s", ip.toString().c_str());
    }
    if (MDNS.begin(MDNS_NAME)) {
        Serial.printf("[WIFI] mDNS: http://%s.local/\n", MDNS_NAME);
    }
    server.on("/",        HTTP_GET, handleRoot);
    server.on("/state",   HTTP_GET, handleState);
    server.on("/log",     HTTP_GET, handleLog);
    server.on("/trace",   HTTP_GET, handleTrace);
    // Control endpoints (accept GET or POST — easier for curl/JS).
    server.on("/cmd/start",        handleCmdStart);
    server.on("/cmd/fast",         handleCmdFast);
    server.on("/cmd/stop",         handleCmdStop);
    server.on("/cmd/reset",        handleCmdReset);
    server.on("/cmd/clear_crash",  handleCmdClearCrash);
    server.on("/cmd/clear_save",   handleCmdClearSave);
    server.on("/cmd/teleport",     handleCmdTeleport);
    server.on("/cmd/walls",        handleCmdWalls);
    server.on("/cmd/set",          handleCmdSet);
    server.on("/cmd/square",       handleCmdSquare);
    server.on("/cmd/test/motor",   handleTestMotor);
    server.on("/cmd/test/turn",    handleTestTurn);
    server.on("/cmd/test/drive",   handleTestDrive);
    server.on("/cmd/test/velocity",handleTestVelocity);
    server.on("/cmd/test/ir_cal",  handleTestIrCal);
    server.on("/cmd/test/buzzer",  handleTestBuzzer);
    server.on("/cmd/test/gyro_recal", handleTestGyroRecal);
    server.onNotFound([](){ server.send(404, "text/plain", "nope"); });
    server.begin();

    for (;;) {
        server.handleClient();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// setup() / loop()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_1, INPUT_PULLUP);

    leftMotor.begin();
    rightMotor.begin();
    leftEnc.begin();
    rightEnc.begin();

    for (auto& p : PAIRS) {
        pinMode(p.emit, OUTPUT);
        digitalWrite(p.emit, LOW);
        pinMode(p.rx, INPUT);
    }
    analogReadResolution(12);

    Wire.begin(OLED_SDA, OLED_SCL, 400000);
    oled.setI2CAddress(OLED_ADDR << 1);
    oled.begin();

    uint8_t who = 0;
    mpuRead(REG_WHO_AM_I, &who, 1);
    mpuWrite(REG_PWR_MGMT_1, 0x00); delay(50);
    mpuWrite(REG_GYRO_CFG,   GYRO_FS_SEL);
    mpuWrite(REG_ACCEL_CFG,  0x00);
    lastImuUs = micros();
    Serial.printf("[INIT] MPU WHO=0x%02X\n", who);

    // Init telemetry struct.
    portENTER_CRITICAL(&teleMux);
    memset(&tele, 0, sizeof(tele));
    tele.state = "BOOT";
    tele.phase = "init";
    portEXIT_CRITICAL(&teleMux);

    calibrateGyro();
    setupMaze();
    copyMazeIntoTele();
    menuEncRef = rightEnc.getTicks();
    oledMenu();
    Serial.println("[INIT] ready");

    // WiFi/HTTP task pinned to Core 0. Stack 8 KB, priority 1 (above idle).
    xTaskCreatePinnedToCore(wifiTask, "wifi", 8192, NULL, 1, NULL, 0);

    tele_set_state("IDLE");
    tele_set_phase("menu");
}

static void applyWallsOverride() {
    if (!cmd.walls_override) return;
    cmd.walls_override = false;
    for (int r = 0; r < MAZE_ROWS; r++) {
        for (int c = 0; c < MAZE_COLS; c++) {
            uint8_t m = cmd.walls_buf[r][c];
            if (m & WALL_NORTH) maze.setWall(r, c, DIR_NORTH, true);
            if (m & WALL_EAST)  maze.setWall(r, c, DIR_EAST,  true);
            if (m & WALL_SOUTH) maze.setWall(r, c, DIR_SOUTH, true);
            if (m & WALL_WEST)  maze.setWall(r, c, DIR_WEST,  true);
        }
    }
    maze.floodFill();
    copyMazeIntoTele();
    logEvent("walls applied (cam)");
}

static void applyCmds() {
    // Teleport: trust camera ground truth, jump robot pose.
    if (cmd.teleport) {
        cmd.teleport = false;
        robotRow = cmd.tp_row; robotCol = cmd.tp_col;
        robotHeading = (AbsDir)cmd.tp_heading;
        copyMazeIntoTele();
        logEvent("teleport r%u c%u h%u", robotRow, robotCol, (uint8_t)robotHeading);
    }
    // Clear-crash works in any state; reset to IDLE.
    if (cmd.clear_crash) {
        cmd.clear_crash = false;
        crashFlag = false;
        crashDrawn = false;
        if (robotState == CRASH) {
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            robotState = IDLE; tele_set_state("IDLE");
        }
        logEvent("crash cleared");
    }
    if (cmd.clear_save) {
        cmd.clear_save = false;
        bool ok = clearSavedMaze();
        logEvent(ok ? "save cleared" : "save already empty");
    }
    // Stop motion: motion loops poll cmdMotionAbort() and return; here we
    // finish the transition to IDLE.
    if (cmd.stop_motion) {
        cmd.stop_motion = false;
        stopMotors();
        if (robotState == RUN) {
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            robotState = IDLE; tele_set_state("IDLE");
            logEvent("stopped");
        }
    }
    // Reset: zero pose, clear walls, redo setup. Same abort semantics.
    if (cmd.reset_pose) {
        cmd.reset_pose = false;
        stopMotors();
        robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
        firstCellOfRun = true;
        crashFlag = false; crashDrawn = false;
        setupMaze();
        copyMazeIntoTele();
        menuEncRef = rightEnc.getTicks();
        oledMenu();
        robotState = IDLE; tele_set_state("IDLE");
        logEvent("reset");
    }
    // Start explore: only from IDLE; otherwise stop first.
    if (cmd.start_explore && robotState == IDLE) {
        cmd.start_explore = false;
        robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
        firstCellOfRun = true;
        setupMaze();
        copyMazeIntoTele();
        oledRunStatus("go");
        logEvent("RUN start (explore via /cmd)");
        robotState = RUN; tele_set_state("RUN");
    }
    if (cmd.start_fast && robotState == IDLE) {
        cmd.start_fast = false;
        bool loaded = setupMazeFast();
        if (!loaded) { logEvent("FAST: no save"); return; }
        robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
        firstCellOfRun = true;
        copyMazeIntoTele();
        oledRunStatus("FAST");
        logEvent("RUN start (FAST via /cmd)");
        robotState = RUN; tele_set_state("RUN");
    }
    // Apply walls AFTER start so setupMaze() doesn't wipe them. Supervisor
    // sequence: POST /cmd/start then POST /cmd/walls within the same tick.
    applyWallsOverride();
}

void loop() {
    applyCmds();
    // Continuous IR sampling at loop rate — so /state has fresh readings even
    // when robot is IDLE or stopped after a crash. Cheap (4 ADC reads, ~80 µs).
    sampleIR();
    portENTER_CRITICAL(&teleMux);
    for (int i = 0; i < 4; i++) tele.ir[i] = irVal[i];
    tele.wall_front = wallFront();
    tele.wall_left  = wallLeft();
    tele.wall_right = wallRight();
    portEXIT_CRITICAL(&teleMux);

    switch (robotState) {

        case IDLE: {
            long delta = rightEnc.getTicks() - menuEncRef;
            if (delta >= ENC_PER_MENU_STEP) {
                menuSel = (menuSel + 1) % M_COUNT;
                menuEncRef += ENC_PER_MENU_STEP;
                oledMenu();
            } else if (delta <= -ENC_PER_MENU_STEP) {
                menuSel = (menuSel - 1 + M_COUNT) % M_COUNT;
                menuEncRef -= ENC_PER_MENU_STEP;
                oledMenu();
            }
            if (buttonEdge()) {
                switch (menuSel) {
                    case M_TEST_MOTOR: robotState = TEST_MOTOR; tele_set_state("TEST_MOTOR"); break;
                    case M_TEST_ENC:   leftEnc.reset(); rightEnc.reset();
                                       oledEncoderTest();
                                       robotState = TEST_ENC; tele_set_state("TEST_ENC"); break;
                    case M_TEST_IR:    sampleIR(); oledBars();
                                       robotState = TEST_IR; tele_set_state("TEST_IR"); break;
                    case M_START:      for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(333); }
                                       robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
                                       firstCellOfRun = true;
                                       setupMaze();
                                       copyMazeIntoTele();
                                       oledRunStatus("go");
                                       logEvent("RUN start (explore)");
                                       robotState = RUN; tele_set_state("RUN"); break;
                    case M_FAST: {
                                       bool loaded = setupMazeFast();
                                       oledShortMsg(loaded ? "LOADED" : "NO SAVE");
                                       delay(700);
                                       if (!loaded) { oledMenu(); break; }
                                       for (int n = 3; n >= 1; n--) { oledCountdown(n); delay(333); }
                                       robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
                                       firstCellOfRun = true;
                                       copyMazeIntoTele();
                                       oledRunStatus("FAST");
                                       logEvent("RUN start (FAST)");
                                       robotState = RUN; tele_set_state("RUN"); break;
                                  }
                    case M_CLEAR_SAVE: {
                                       bool ok = clearSavedMaze();
                                       oledShortMsg(ok ? "CLEARED" : "EMPTY");
                                       delay(700); oledMenu();
                                       logEvent(ok ? "NVS cleared" : "NVS already empty");
                                       break;
                                  }
                }
            }
            break;
        }

        case TEST_MOTOR: {
            runMotorTest();
            while (!buttonEdge()) { delay(20); }
            menuEncRef = rightEnc.getTicks();
            oledMenu();
            robotState = IDLE; tele_set_state("IDLE");
            break;
        }

        case TEST_ENC: {
            static uint32_t last = 0;
            if (millis() - last > 150) { oledEncoderTest(); last = millis(); }
            if (buttonEdge()) {
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE; tele_set_state("IDLE");
            }
            break;
        }

        case TEST_IR: {
            static uint32_t last = 0;
            if (millis() - last > 100) {
                sampleIR(); oledBars();
                portENTER_CRITICAL(&teleMux);
                for (int i = 0; i < 4; i++) tele.ir[i] = irVal[i];
                portEXIT_CRITICAL(&teleMux);
                last = millis();
            }
            if (buttonEdge()) {
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE; tele_set_state("IDLE");
            }
            break;
        }

        case RUN: {
            if (maze.isGoal(robotRow, robotCol)) {
                robotState = GOAL; tele_set_state("GOAL");
                logEvent("GOAL reached r%u c%u", robotRow, robotCol);
                break;
            }
            maze.visited[robotRow][robotCol] = true;

            sampleIR();
            senseWallsFrontOnly();
            maze.floodFill();
            copyMazeIntoTele();

            uint8_t bestDist;
            AbsDir best = maze.bestDirectionBiased(robotRow, robotCol, robotHeading, bestDist);
            if (bestDist == FLOOD_INFINITY) {
                static int noPathRetries = 0;
                if (noPathRetries < 2) {
                    noPathRetries++;
                    portENTER_CRITICAL(&teleMux);
                    tele.no_path_retries = noPathRetries;
                    portEXIT_CRITICAL(&teleMux);
                    stopMotors();
                    logEvent("no path retry %d", noPathRetries);
                    turnAround();
                    robotHeading = (AbsDir)(((int)robotHeading + 2) % 4);
                    break;
                }
                noPathRetries = 0;
                portENTER_CRITICAL(&teleMux);
                tele.no_path_retries = 0;
                portEXIT_CRITICAL(&teleMux);
                stopMotors();
                crashFlag = true;
                crashRow = robotRow; crashCol = robotCol;
                crashHeading = robotHeading;
                for (int i = 0; i < 4; i++) crashIR[i] = irVal[i];
                crashReason = "no path x3";
                crashDrawn = false;
                logEvent("CRASH no path");
                robotState = CRASH; tele_set_state("CRASH");
                break;
            }

            if (best != robotHeading) {
                advanceToCellCenter();
            }
            bool turnOk = rotateToHeading(best);
            if (!turnOk) {
                stopMotors();
                crashFlag = true;
                crashRow = robotRow; crashCol = robotCol;
                crashHeading = robotHeading;
                for (int i = 0; i < 4; i++) crashIR[i] = irVal[i];
                crashReason = "turn fail";
                crashDrawn = false;
                logEvent("CRASH turn fail");
                robotState = CRASH; tele_set_state("CRASH");
                break;
            }
            driveChain();
            copyMazeIntoTele();
            if (crashFlag) { crashDrawn = false; robotState = CRASH; tele_set_state("CRASH"); }
            break;
        }

        case GOAL: {
            stopMotors();
            static bool drawn = false;
            if (!drawn) { oledRunStatus("GOAL btn=save"); drawn = true; }
            if (buttonEdge()) {
                drawn = false;
                bool ok = saveMazeFlash(maze);
                oledShortMsg(ok ? "MAZE SAVED" : "SAVE FAIL");
                logEvent(ok ? "maze saved to NVS" : "NVS save FAILED");
                delay(900);
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE; tele_set_state("IDLE");
            }
            break;
        }

        case CRASH: {
            if (!crashDrawn) { oledCrash(); crashDrawn = true; }
            if (buttonEdge()) {
                crashFlag = false;
                crashDrawn = false;
                menuEncRef = rightEnc.getTicks();
                oledMenu();
                robotState = IDLE; tele_set_state("IDLE");
            }
            break;
        }
    }
}
