#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "PinConfig.h"
#include "MicromouseMaze.h"
#include "Mm26WifiReplay.h"

#ifndef WIFI_SSID
#define WIFI_SSID "NETGEAR38"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "mightygiant145"
#endif

constexpr float MM26_ROBOT_LEN_MM    = 100.0f;
constexpr float MM26_ROBOT_WID_MM    = 85.0f;
constexpr float MM26_AXLE_REAR_MM    = 40.0f;
constexpr float MM26_START_OFFSET_MM = 45.0f;

struct Mm26WifiCfg {
    long  ticksPerCell       = 1405;
    long  startOffsetTicks   = 351;
    float approachCm         = 14.0f;
    float postPivotCm        = 11.0f;
    float pivotTrimCm        = 4.0f;
    float cellCm             = 18.0f;
    float backupOffsetMm     = -12.0f;
    float yawKp              = 6.0f;
    float fwdKp              = 0.8f;
    bool  useImu             = true;
};

struct Mm26WifiDebug {
    bool  valid       = false;
    long  encL        = 0;
    long  encR        = 0;
    long  tgtTicks    = 0;
    float avgTicks    = 0;
    float posErr      = 0;
    float prog        = 0;
    float yaw         = 0;
    float yawTgt      = 0;
    float gz          = 0;
    int   irLF        = 0;
    int   irL         = 0;
    int   irR         = 0;
    int   irRF        = 0;
    float vbat        = 0;
    int   batPct      = 0;
    int   pwmL        = 0;
    int   pwmR        = 0;
    uint8_t planRow   = 0;
    uint8_t planCol   = 0;
    uint8_t planHd    = 0;
    uint8_t bestDist  = 255;
    char    bestDir   = '?';
    bool    wallF     = false;
    bool    wallL     = false;
    bool    wallR     = false;
    char    phase[12] = "—";
};

struct Mm26WifiLiveIn {
    const MicromouseMaze* maze     = nullptr;
    uint8_t               mazeRows = 6;
    uint8_t               mazeCols = 3;
    uint8_t               goalRow  = 0;
    uint8_t               goalCol  = 2;

    uint8_t robotRow = 0, robotCol = 0, robotHeading = 0;
    int     state      = 0;
    bool    explore    = false;
    bool    fast       = false;

    bool    inRun          = false;
    uint8_t runPhase       = 0;
    uint8_t turnDir        = 0;
    long    runTargetTicks = 0;
    float   phaseProgress  = 0;
    int     scriptIdx      = 0;
    int     scriptLen      = 0;

    Mm26WifiCfg   cfg   = {};
    Mm26WifiDebug dbg   = {};
};

struct Mm26WifiScriptStep {
    uint8_t phase;
    uint8_t turnDir;
    long    ticks;
    float   distCm;
    float   distMm;
};

// ── Live pose ───────────────────────────────────────────────────────────────
static WebServer _mm26Ws(80);
static bool      _mm26WifiOk = false;
static Mm26WifiLiveIn _mm26Snap = {};
static Mm26WifiCfg    _mm26Cfg  = {};

static Mm26WifiScriptStep _mm26Script[8];
static int                _mm26ScriptLen = 0;

static float _mm26PoseX = 0, _mm26PoseY = 0;
static int   _mm26PoseHd = 0;
static float _mm26PoseHdDeg = 90.0f;
static float _mm26DisplayX = 0, _mm26DisplayY = 0, _mm26DisplayHdDeg = 90.0f;
static float _mm26SegStartX = 0, _mm26SegStartY = 0;
static int   _mm26SegStartHd = 0;
static int   _mm26LiveScriptIdx = -1;

struct Mm26TrailPt { float x, y; };
static Mm26TrailPt _mm26Trail[600];
static int         _mm26TrailLen = 0;

// ── Replay (offline fast-run sim) ───────────────────────────────────────────
static Mm26ReplayPlanCtx   _mm26ReplayCtx;
static Mm26ReplayStep      _mm26ReplayPath[MM26_REPLAY_MAX_PATH];
static int                 _mm26ReplayLen = 0;
static bool                _mm26ReplayReady = false;
static char                _mm26ReplayErr[64] = "";

static float _mm26ReplayX = 0, _mm26ReplayY = 0, _mm26ReplayHdDeg = 90.0f;
static float _mm26ReplaySegX = 0, _mm26ReplaySegY = 0;
static int   _mm26ReplaySegHd = 0;
static int   _mm26ReplaySegIdx = 0;
static float _mm26ReplaySegProg = 0;
static bool  _mm26ReplayPlaying = false;
static bool  _mm26ReplayShowGhost = false;
static uint32_t _mm26ReplayLastMs = 0;
static float _mm26ReplaySpeedMmS = 320.0f;

static Preferences _mm26Prefs;
static const char* _mm26NvsNs = "mm26";
static const char* _mm26NvsWalls = "walls";

static void _mm26TrailPush(float x, float y) {
    if (_mm26TrailLen < (int)(sizeof(_mm26Trail) / sizeof(_mm26Trail[0])))
        _mm26Trail[_mm26TrailLen++] = { x, y };
}

static void _mm26HeadingUnit(int h, float& ux, float& uy) {
    switch (h) {
        case DIR_NORTH: ux = 0;  uy = 1;  break;
        case DIR_EAST:  ux = 1;  uy = 0;  break;
        case DIR_SOUTH: ux = 0;  uy = -1; break;
        default:        ux = -1; uy = 0;  break;
    }
}

static float _mm26HeadingDeg(int h) {
    switch (h) {
        case DIR_NORTH: return 90.0f;
        case DIR_EAST:  return 0.0f;
        case DIR_SOUTH: return 270.0f;
        default:        return 180.0f;
    }
}

static float _mm26TicksPerMm() { return (float)_mm26Cfg.ticksPerCell / CELL_MM; }

static void _mm26PoseResetStart() {
    _mm26PoseX     = (0 + 0.5f) * CELL_MM;
    _mm26PoseY     = (0 + 0.5f) * CELL_MM - MM26_START_OFFSET_MM;
    _mm26PoseHd    = DIR_NORTH;
    _mm26PoseHdDeg = _mm26HeadingDeg(_mm26PoseHd);
    _mm26TrailLen  = 0;
    _mm26TrailPush(_mm26PoseX, _mm26PoseY);
    _mm26SegStartX = _mm26PoseX;
    _mm26SegStartY = _mm26PoseY;
    _mm26SegStartHd = _mm26PoseHd;
    _mm26LiveScriptIdx = -1;
}

static void _mm26ApplyTurn(int turnDir, int deg) {
    int steps = deg / 90;
    for (int i = 0; i < steps; i++) {
        if (turnDir == 1) _mm26PoseHd = (_mm26PoseHd + 1) % 4;
        else              _mm26PoseHd = (_mm26PoseHd + 3) % 4;
    }
    _mm26PoseHdDeg = _mm26HeadingDeg(_mm26PoseHd);
}

static void _mm26CommitScriptStep(const Mm26WifiScriptStep& s) {
    if (s.phase == 0) {
        float ux, uy;
        _mm26HeadingUnit(_mm26PoseHd, ux, uy);
        float d = (float)labs(s.ticks) / _mm26TicksPerMm();
        _mm26PoseX += ux * d;
        _mm26PoseY += uy * d;
    } else if (s.phase == 1) {
        _mm26ApplyTurn(s.turnDir, 90);
    } else if (s.phase == 2) {
        _mm26ApplyTurn(s.turnDir, 180);
    }
    _mm26PoseHdDeg = _mm26HeadingDeg(_mm26PoseHd);
    _mm26TrailPush(_mm26PoseX, _mm26PoseY);
}

static void _mm26ReplayPoseReset() {
    _mm26ReplayX = (0 + 0.5f) * CELL_MM;
    _mm26ReplayY = (0 + 0.5f) * CELL_MM - MM26_START_OFFSET_MM;
    _mm26ReplaySegHd = DIR_NORTH;
    _mm26ReplayHdDeg = _mm26HeadingDeg(_mm26ReplaySegHd);
    _mm26ReplaySegIdx = 0;
    _mm26ReplaySegProg = 0;
    _mm26ReplaySegX = _mm26ReplayX;
    _mm26ReplaySegY = _mm26ReplayY;
}

static void _mm26ReplayCommitStep(const Mm26ReplayStep& s) {
    if (s.phase == 0) {
        float ux, uy;
        _mm26HeadingUnit(_mm26ReplaySegHd, ux, uy);
        _mm26ReplayX += ux * s.distMm;
        _mm26ReplayY += uy * s.distMm;
    } else if (s.phase == 1) {
        if (s.turnDir == 1) _mm26ReplaySegHd = (_mm26ReplaySegHd + 1) % 4;
        else                _mm26ReplaySegHd = (_mm26ReplaySegHd + 3) % 4;
    } else if (s.phase == 2) {
        if (s.turnDir == 1) _mm26ReplaySegHd = (_mm26ReplaySegHd + 2) % 4;
        else                _mm26ReplaySegHd = (_mm26ReplaySegHd + 2) % 4;
    }
    _mm26ReplayHdDeg = _mm26HeadingDeg(_mm26ReplaySegHd);
}

static void _mm26ReplayInterp() {
    if (!_mm26ReplayReady || _mm26ReplaySegIdx >= _mm26ReplayLen) return;
    const Mm26ReplayStep& s = _mm26ReplayPath[_mm26ReplaySegIdx];
    float u = constrain(_mm26ReplaySegProg, 0.0f, 1.0f);
    float x = _mm26ReplaySegX, y = _mm26ReplaySegY;
    float hd = _mm26HeadingDeg(_mm26ReplaySegHd);
    if (s.phase == 0) {
        float ux, uy;
        _mm26HeadingUnit(_mm26ReplaySegHd, ux, uy);
        x += ux * s.distMm * u;
        y += uy * s.distMm * u;
    } else if (s.phase == 1) {
        hd += (s.turnDir == 1) ? -90.0f * u : 90.0f * u;
    } else if (s.phase == 2) {
        hd += (s.turnDir == 1) ? -180.0f * u : 180.0f * u;
    }
    _mm26ReplayX = x;
    _mm26ReplayY = y;
    _mm26ReplayHdDeg = hd;
}

static float _mm26ReplaySegDur(const Mm26ReplayStep& s) {
    if (s.phase == 0) return s.distMm / _mm26ReplaySpeedMmS;
    if (s.phase == 1) return 0.45f;
    return 0.75f;
}

static void _mm26ReplayAdvance(float dt) {
    if (!_mm26ReplayReady || _mm26ReplayLen == 0) return;
    while (dt > 0.0001f && _mm26ReplaySegIdx < _mm26ReplayLen) {
        const Mm26ReplayStep& s = _mm26ReplayPath[_mm26ReplaySegIdx];
        float dur = _mm26ReplaySegDur(s);
        float remain = (1.0f - _mm26ReplaySegProg) * dur;
        if (dt >= remain) {
            dt -= remain;
            _mm26ReplayCommitStep(s);
            _mm26ReplayX = _mm26ReplaySegX = _mm26ReplayX;
            _mm26ReplayY = _mm26ReplaySegY = _mm26ReplayY;
            _mm26ReplaySegIdx++;
            _mm26ReplaySegProg = 0;
            if (_mm26ReplaySegIdx >= _mm26ReplayLen) {
                _mm26ReplayPlaying = false;
                break;
            }
        } else {
            _mm26ReplaySegProg += dt / dur;
            dt = 0;
        }
    }
    _mm26ReplayInterp();
}

static void _mm26UpdateLivePose(const Mm26WifiLiveIn& in) {
    if (!in.inRun || in.scriptLen <= 0) {
        _mm26DisplayX = _mm26PoseX;
        _mm26DisplayY = _mm26PoseY;
        _mm26DisplayHdDeg = _mm26PoseHdDeg;
        return;
    }
    int idx = constrain(in.scriptIdx, 0, in.scriptLen - 1);
    if (idx != _mm26LiveScriptIdx) {
        for (int i = _mm26LiveScriptIdx + 1; i < idx && i < _mm26ScriptLen; i++)
            _mm26CommitScriptStep(_mm26Script[i]);
        _mm26SegStartX = _mm26PoseX;
        _mm26SegStartY = _mm26PoseY;
        _mm26SegStartHd = _mm26PoseHd;
        _mm26LiveScriptIdx = idx;
    }
    if (idx < _mm26ScriptLen) {
        const Mm26WifiScriptStep& s = _mm26Script[idx];
        float u = (in.phaseProgress >= 0) ? constrain(in.phaseProgress, 0.0f, 1.0f) : 0;
        float x = _mm26SegStartX, y = _mm26SegStartY;
        float hd = _mm26HeadingDeg(_mm26SegStartHd);
        if (s.phase == 0) {
            float ux, uy;
            _mm26HeadingUnit(_mm26SegStartHd, ux, uy);
            float d = (float)labs(s.ticks) / _mm26TicksPerMm();
            x += ux * d * u;
            y += uy * d * u;
        } else if (s.phase == 1) {
            hd += (s.turnDir == 1) ? -90.0f * u : 90.0f * u;
        } else if (s.phase == 2) {
            hd += (s.turnDir == 1) ? -180.0f * u : 180.0f * u;
        }
        _mm26DisplayX = x;
        _mm26DisplayY = y;
        _mm26DisplayHdDeg = hd;
    }
}

inline void mm26WifiSetCfg(const Mm26WifiCfg& c) { _mm26Cfg = c; }

inline void mm26WifiExportScript(uint8_t phases[], uint8_t dirs[], long targets[], int len) {
    _mm26ScriptLen = (len > 8) ? 8 : len;
    float tpm = _mm26TicksPerMm();
    for (int i = 0; i < _mm26ScriptLen; i++) {
        _mm26Script[i].phase   = phases[i];
        _mm26Script[i].turnDir = dirs[i];
        _mm26Script[i].ticks   = targets[i];
        if (phases[i] == 0) {
            _mm26Script[i].distMm = (float)labs(targets[i]) / tpm;
            _mm26Script[i].distCm = _mm26Script[i].distMm / 10.0f;
        } else {
            _mm26Script[i].distCm = (phases[i] == 1) ? 90.0f : 180.0f;
            _mm26Script[i].distMm = _mm26Script[i].distCm * 10.0f;
        }
    }
}

inline void mm26WifiOnRunStart() { _mm26PoseResetStart(); }

inline void mm26WifiOnMoveDone() {
    for (int i = _mm26LiveScriptIdx + 1; i < _mm26ScriptLen; i++)
        _mm26CommitScriptStep(_mm26Script[i]);
    _mm26LiveScriptIdx = -1;
}

static bool _mm26ReplayLoadNvs() {
    _mm26ReplayCtx.setupMaze();
    if (!_mm26Prefs.begin(_mm26NvsNs, true)) {
        snprintf(_mm26ReplayErr, sizeof(_mm26ReplayErr), "NVS open fail");
        return false;
    }
    if (!_mm26Prefs.isKey(_mm26NvsWalls)) {
        _mm26Prefs.end();
        snprintf(_mm26ReplayErr, sizeof(_mm26ReplayErr), "no walls — explore first");
        return false;
    }
    _mm26Prefs.getBytes(_mm26NvsWalls, _mm26ReplayCtx.maze.walls, sizeof(_mm26ReplayCtx.maze.walls));
    _mm26Prefs.end();
  _mm26ReplayErr[0] = '\0';
    return true;
}

static bool _mm26ReplayPlan() {
    _mm26ReplayCtx.ticksPerCell = _mm26Cfg.ticksPerCell;
    _mm26ReplayCtx.startOffsetTicks = _mm26Cfg.startOffsetTicks;
    _mm26ReplayLen = _mm26ReplayCtx.planFull(_mm26ReplayPath, MM26_REPLAY_MAX_PATH,
                                             _mm26ReplayErr, sizeof(_mm26ReplayErr));
    _mm26ReplayReady = (_mm26ReplayLen > 0);
    if (_mm26ReplayReady) {
        _mm26ReplayShowGhost = true;
        _mm26ReplayPoseReset();
        Serial.printf("[WiFi] replay plan %d segs\n", _mm26ReplayLen);
    }
    return _mm26ReplayReady;
}

static String _mm26AppendScriptJson(String& j, const char* key,
                                    Mm26WifiScriptStep* steps, int n, int activeIdx) {
    j += ",\"" + String(key) + "\":[";
    for (int i = 0; i < n; i++) {
        if (i) j += ',';
        const char* ph = (steps[i].phase == 0) ? "FWD"
                       : (steps[i].phase == 1) ? "PIVOT" : "SPOT";
        j += "{\"i\":" + String(i + 1);
        j += ",\"ph\":\"" + String(ph) + "\"";
        j += ",\"ticks\":" + String(steps[i].ticks);
        j += ",\"cm\":" + String(steps[i].distCm, 2);
        j += ",\"mm\":" + String(steps[i].distMm, 1);
        j += ",\"dir\":\"";
        j += (steps[i].turnDir == 1) ? "R" : (steps[i].turnDir == 2 ? "L" : "");
        j += "\",\"active\":" + String((activeIdx == i) ? "true" : "false") + "}";
    }
    j += "]";
    return j;
}

static String _mm26BuildLiveJson() {
    const Mm26WifiLiveIn& in = _mm26Snap;
    _mm26UpdateLivePose(in);

    const char* stName = "IDLE";
    switch (in.state) {
        case 4: stName = "THINK"; break;
        case 5: stName = "RUN"; break;
        case 6: stName = "GOAL"; break;
        case 7: stName = "CRASH"; break;
        default: break;
    }

    String j = "{";
    j += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    j += "\"mode\":\"" + String(in.fast ? "FAST" : (in.explore ? "EXPLORE" : "IDLE")) + "\",";
    j += "\"state\":\"" + String(stName) + "\",";
    j += "\"rows\":" + String(in.mazeRows) + ",\"cols\":" + String(in.mazeCols);
    j += ",\"cellMm\":" + String(CELL_MM, 1);
    j += ",\"lenMm\":" + String(MM26_ROBOT_LEN_MM);
    j += ",\"widMm\":" + String(MM26_ROBOT_WID_MM);
    j += ",\"rearMm\":" + String(MM26_AXLE_REAR_MM);
    j += ",\"goal\":[" + String(in.goalRow) + "," + String(in.goalCol) + "],";

    j += "\"cfg\":{";
    j += "\"ticksPerCell\":" + String(_mm26Cfg.ticksPerCell);
    j += ",\"startOffsetTicks\":" + String(_mm26Cfg.startOffsetTicks);
    j += ",\"startOffsetCm\":" + String(_mm26Cfg.startOffsetTicks / _mm26TicksPerMm() / 10.0f, 2);
    j += ",\"approachCm\":" + String(_mm26Cfg.approachCm, 1);
    j += ",\"postPivotCm\":" + String(_mm26Cfg.postPivotCm, 1);
    j += ",\"pivotTrimCm\":" + String(_mm26Cfg.pivotTrimCm, 1);
    j += ",\"cellCm\":" + String(_mm26Cfg.cellCm, 1);
    j += ",\"ticksPerMm\":" + String(_mm26TicksPerMm(), 3);
    j += ",\"yawKp\":" + String(_mm26Cfg.yawKp, 2);
    j += ",\"fwdKp\":" + String(_mm26Cfg.fwdKp, 2);
    j += ",\"useImu\":" + String(_mm26Cfg.useImu ? "true" : "false");
    j += "},";

    j += "\"dbg\":{";
    if (in.dbg.valid) {
        j += "\"encL\":" + String(in.dbg.encL) + ",\"encR\":" + String(in.dbg.encR);
        j += ",\"tgt\":" + String(in.dbg.tgtTicks);
        j += ",\"avg\":" + String(in.dbg.avgTicks, 1);
        j += ",\"err\":" + String(in.dbg.posErr, 1);
        j += ",\"prog\":" + String(in.dbg.prog, 3);
        j += ",\"yaw\":" + String(in.dbg.yaw, 2);
        j += ",\"yawTgt\":" + String(in.dbg.yawTgt, 2);
        j += ",\"gz\":" + String(in.dbg.gz, 2);
        j += ",\"ir\":[" + String(in.dbg.irLF) + "," + String(in.dbg.irL) + ",";
        j += String(in.dbg.irR) + "," + String(in.dbg.irRF) + "]";
        j += ",\"vbat\":" + String(in.dbg.vbat, 2);
        j += ",\"bat\":" + String(in.dbg.batPct);
        j += ",\"pwmL\":" + String(in.dbg.pwmL) + ",\"pwmR\":" + String(in.dbg.pwmR);
        j += ",\"plan\":[" + String(in.dbg.planRow) + "," + String(in.dbg.planCol);
        j += ",\"" + String(in.dbg.bestDir) + "\"]";
        j += ",\"bestDist\":" + String(in.dbg.bestDist);
        j += ",\"walls\":[" + String(in.dbg.wallF ? 1 : 0) + ",";
        j += String(in.dbg.wallL ? 1 : 0) + "," + String(in.dbg.wallR ? 1 : 0) + "]";
        j += ",\"phase\":\"" + String(in.dbg.phase) + "\"";
    }
    j += "},";

    auto poseObj = [](float x, float y, float hd, const Mm26WifiLiveIn& pin, bool live) -> String {
        String s = "{\"xMm\":" + String(x, 1) + ",\"yMm\":" + String(y, 1);
        s += ",\"xCm\":" + String(x / 10.0f, 2) + ",\"yCm\":" + String(y / 10.0f, 2);
        s += ",\"hdDeg\":" + String(hd, 1);
        s += ",\"heading\":" + String(pin.robotHeading);
        s += ",\"row\":" + String(pin.robotRow) + ",\"col\":" + String(pin.robotCol);
        s += ",\"live\":" + String(live ? "true" : "false") + "}";
        return s;
    };
    j += "\"pose\":" + poseObj(_mm26DisplayX, _mm26DisplayY, _mm26DisplayHdDeg, in, true);
    if (_mm26ReplayReady)
        j += ",\"replayPose\":" + poseObj(_mm26ReplayX, _mm26ReplayY, _mm26ReplayHdDeg, in, false);
    j += ",";

    j += "\"scriptIdx\":" + String(in.inRun ? in.scriptIdx : -1);
    _mm26AppendScriptJson(j, "liveScript", _mm26Script, _mm26ScriptLen,
                          in.inRun ? in.scriptIdx : -1);

    j += ",\"replay\":{";
    j += "\"ready\":" + String(_mm26ReplayReady ? "true" : "false");
    j += ",\"playing\":" + String(_mm26ReplayPlaying ? "true" : "false");
    j += ",\"seg\":" + String(_mm26ReplaySegIdx);
    j += ",\"prog\":" + String(_mm26ReplaySegProg, 3);
    j += ",\"len\":" + String(_mm26ReplayLen);
    j += ",\"err\":\"" + String(_mm26ReplayErr) + "\"";
    float totalCm = 0;
    j += ",\"path\":[";
    for (int i = 0; i < _mm26ReplayLen; i++) {
        if (i) j += ',';
        const Mm26ReplayStep& s = _mm26ReplayPath[i];
        const char* ph = (s.phase == 0) ? "FWD" : (s.phase == 1) ? "PIVOT" : "SPOT";
        j += "{\"i\":" + String(i + 1) + ",\"ph\":\"" + String(ph) + "\"";
        j += ",\"ticks\":" + String(s.ticks) + ",\"cm\":" + String(s.distCm, 2);
        j += ",\"mm\":" + String(s.distMm, 1) + ",\"dir\":\"";
        j += (s.turnDir == 1) ? "R" : (s.turnDir == 2 ? "L" : "");
        j += "\",\"active\":" + String((i == _mm26ReplaySegIdx) ? "true" : "false") + "}";
        if (s.phase == 0) totalCm += s.distCm;
    }
    j += "],\"totalFwdCm\":" + String(totalCm, 1) + "},";

    j += "\"walls\":[";
    if (in.maze) {
        for (int r = 0; r < in.mazeRows; r++)
            for (int c = 0; c < in.mazeCols; c++) {
                if (r || c) j += ',';
                j += String(in.maze->walls[r][c]);
            }
    }
    j += "],\"replayWalls\":[";
    for (int r = 0; r < MM26_REPLAY_ROWS; r++)
        for (int c = 0; c < MM26_REPLAY_COLS; c++) {
            if (r || c) j += ',';
            j += String(_mm26ReplayCtx.maze.walls[r][c]);
        }
    j += "],\"visited\":[";
    if (in.maze) {
        for (int r = 0; r < in.mazeRows; r++)
            for (int c = 0; c < in.mazeCols; c++) {
                if (r || c) j += ',';
                j += in.maze->visited[r][c] ? "1" : "0";
            }
    }
    j += "],\"flood\":[";
    if (in.maze) {
        for (int r = 0; r < in.mazeRows; r++)
            for (int c = 0; c < in.mazeCols; c++) {
                if (r || c) j += ',';
                j += String(in.maze->flood[r][c]);
            }
    }
    j += "],\"trail\":[";
    for (int i = 0; i < _mm26TrailLen; i++) {
        if (i) j += ',';
        j += "{\"x\":" + String(_mm26Trail[i].x, 1) + ",\"y\":" + String(_mm26Trail[i].y, 1) + "}";
    }
    j += "]}";
    return j;
}

// HTML in separate include to keep compile manageable
#include "Mm26WifiLiveHtml.h"

inline void mm26WifiBegin() {
    Mm26WifiCfg c = {};
    c.ticksPerCell = 1405;
    c.startOffsetTicks = 351;
    c.approachCm = 14.0f;
    c.postPivotCm = 11.0f;
    c.pivotTrimCm = 4.0f;
    c.cellCm = CELL_MM / 10.0f;
    mm26WifiSetCfg(c);
    _mm26PoseResetStart();
    _mm26ReplayCtx.setupMaze();

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] joining %s", WIFI_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) {
        delay(300);
        Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
        _mm26WifiOk = true;
        Serial.printf("\n[WiFi] dashboard http://%s/\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WiFi] failed");
    }

    _mm26Ws.on("/", []() { _mm26Ws.send_P(200, "text/html", MM26_WIFI_PAGE); });
    _mm26Ws.on("/api/live", HTTP_GET, []() {
        _mm26Ws.sendHeader("Access-Control-Allow-Origin", "*");
        _mm26Ws.send(200, "application/json", _mm26BuildLiveJson());
    });
    _mm26Ws.on("/api/replay/load", HTTP_POST, []() {
        bool ok = _mm26ReplayLoadNvs();
        if (ok) _mm26ReplayCtx.maze.floodFill();
        _mm26Ws.sendHeader("Access-Control-Allow-Origin", "*");
        _mm26Ws.send(200, "application/json",
                       String("{\"ok\":") + (ok ? "true" : "false")
                       + ",\"err\":\"" + String(_mm26ReplayErr) + "\"}");
    });
    _mm26Ws.on("/api/replay/plan", HTTP_POST, []() {
        if (!_mm26ReplayLoadNvs()) {
            _mm26Ws.send(200, "application/json",
                         String("{\"ok\":false,\"err\":\"") + String(_mm26ReplayErr) + "\"}");
            return;
        }
        bool ok = _mm26ReplayPlan();
        _mm26Ws.send(200, "application/json",
                       String("{\"ok\":") + (ok ? "true" : "false")
                       + ",\"segs\":" + String(_mm26ReplayLen)
                       + ",\"err\":\"" + String(_mm26ReplayErr) + "\"}");
    });
    _mm26Ws.on("/api/replay/play", HTTP_POST, []() {
        if (_mm26ReplayReady) {
            _mm26ReplayPlaying = true;
            _mm26ReplayLastMs = millis();
        }
        _mm26Ws.send(200, "application/json", "{\"ok\":true}");
    });
    _mm26Ws.on("/api/replay/pause", HTTP_POST, []() {
        _mm26ReplayPlaying = false;
        _mm26Ws.send(200, "application/json", "{\"ok\":true}");
    });
    _mm26Ws.on("/api/replay/reset", HTTP_POST, []() {
        _mm26ReplayPlaying = false;
        _mm26ReplayPoseReset();
        _mm26Ws.send(200, "application/json", "{\"ok\":true}");
    });
    _mm26Ws.on("/api/replay/step", HTTP_POST, []() {
        if (_mm26ReplayReady && _mm26ReplaySegIdx < _mm26ReplayLen) {
            _mm26ReplayCommitStep(_mm26ReplayPath[_mm26ReplaySegIdx]);
            _mm26ReplaySegX = _mm26ReplayX;
            _mm26ReplaySegY = _mm26ReplayY;
            _mm26ReplaySegIdx++;
            _mm26ReplaySegProg = 0;
            _mm26ReplayInterp();
        }
        _mm26Ws.send(200, "application/json", "{\"ok\":true}");
    });
    _mm26Ws.begin();
}

inline void mm26WifiPoll(const Mm26WifiLiveIn& in) {
    _mm26Snap = in;
    if (_mm26ReplayPlaying) {
        uint32_t now = millis();
        float dt = (_mm26ReplayLastMs == 0) ? 0.0f : (now - _mm26ReplayLastMs) / 1000.0f;
        _mm26ReplayLastMs = now;
        if (dt > 0.05f) dt = 0.05f;
        _mm26ReplayAdvance(dt);
    } else {
        _mm26ReplayLastMs = millis();
    }
}

inline void mm26WifiHandle() {
    if (_mm26WifiOk) _mm26Ws.handleClient();
}
