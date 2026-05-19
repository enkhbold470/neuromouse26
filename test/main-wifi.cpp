// test/main-wifi.cpp — Fast-run path planner + WiFi pose simulation (ESP32-S3)
//
// Mirrors src/main.cpp fast-run geometry exactly:
//   • flood-fill + buildMoveScript(fastRunMode=true) + chain extension
//   • ticksPerCell=1405, cell=180 mm, startOffset=4.5 cm, pivot approach 14 cm,
//     post-pivot 11 cm, backup fudge only on explore 180° (not used in fast run)
//
// Web UI: maze map, planned path (cm per segment), animated robot footprint
// (100×85 mm, axle 40 mm from rear / 60 mm from nose).

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "PinConfig.h"
#include "MicromouseMaze.h"

#ifndef WIFI_SSID
#define WIFI_SSID "NETGEAR38"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "mightygiant145"
#endif

constexpr uint8_t MAZE_ROWS = 6;
constexpr uint8_t MAZE_COLS = 3;
constexpr uint8_t START_ROW = 0;
constexpr uint8_t START_COL = 0;
constexpr uint8_t GOAL_ROW  = 0;
constexpr uint8_t GOAL_COL  = 2;

constexpr float START_OFFSET_MM   = 45.0f;   // rear-wall start pose
constexpr float ROBOT_LEN_MM      = 100.0f;
constexpr float ROBOT_WID_MM      = 85.0f;
constexpr float AXLE_TO_REAR_MM   = 40.0f;
constexpr float AXLE_TO_NOSE_MM   = 60.0f;

// ── Planning tuning (must match src/main.cpp T) ─────────────────────────────
struct PlanTuning {
    long  ticksPerCell       = 1405;
    long  startOffsetTicks   = 351;
    float backupOffsetMm     = -12.0f;
    long  stopBias           = 0;
    bool  useImu             = true;
};
static PlanTuning PT;

static float ticksPerMm() { return (float)PT.ticksPerCell / CELL_MM; }
static float ticksToMm(long t) { return (float)t / ticksPerMm(); }
static float ticksToCm(long t) { return ticksToMm(t) / 10.0f; }

enum TurnDir   { TURN_NONE, TURN_RIGHT, TURN_LEFT };
enum RunPhase  { PH_FORWARD, PH_PIVOT, PH_SPOT, PH_FWD_TO_WALL, PH_REVERSE_TO_BACK };

struct PhaseStep {
    RunPhase phase;
    long     target;
    TurnDir  dir;
};

constexpr int MAX_SCRIPT = 8;
constexpr int MAX_PATH   = 192;

struct PathSeg {
    RunPhase phase;
    TurnDir  dir;
    float    distCm;
    long     ticks;
};

static PhaseStep script[MAX_SCRIPT];
static int       scriptLen = 0;
static PathSeg   path[MAX_PATH];
static int       pathLen = 0;

static MicromouseMaze maze;
static Preferences    prefs;
static const char*    NVS_NS    = "mm26";
static const char*    NVS_WALLS = "walls";

static uint8_t robotRow = START_ROW, robotCol = START_COL, robotHeading = DIR_NORTH;
static uint8_t plannedRow, plannedCol, plannedHeading;
static long    pendingOffsetTicks = 0;
static bool    fastRunMode = true;
static bool    wallsLoaded = false;
static char    planErr[64] = "";

static WebServer server(80);

// ── Pose simulation ─────────────────────────────────────────────────────────
struct SimPose {
    float xMm = 0, yMm = 0;
    int   heading = 0;   // AbsDir 0..3
    float headingDeg = 90.0f; // canvas: 0=E, 90=N (y-up)
};

static SimPose sim = {};
static SimPose segStartPose = {};
struct TrailPt { float x, y; };
static TrailPt trail[512];
static int     trailLen = 0;

static int   simSegIdx = 0;
static float simSegProg = 0;      // 0..1 within segment
static bool  simPlaying = false;
static float simSpeedMmS = 350.0f;
static uint32_t simLastMs = 0;
static bool  planReady = false;

static void trailPush(float x, float y) {
    if (trailLen < (int)(sizeof(trail) / sizeof(trail[0]))) {
        trail[trailLen++] = { x, y };
    }
}

static void headingUnit(int h, float& ux, float& uy) {
    switch (h) {
        case DIR_NORTH: ux = 0;  uy = 1;  break;
        case DIR_EAST:  ux = 1;  uy = 0;  break;
        case DIR_SOUTH: ux = 0;  uy = -1; break;
        default:        ux = -1; uy = 0;  break;
    }
}

static float headingToDeg(int h) {
    switch (h) {
        case DIR_NORTH: return 90.0f;
        case DIR_EAST:  return 0.0f;
        case DIR_SOUTH: return 270.0f;
        default:        return 180.0f;
    }
}

static void simResetPose() {
    sim.xMm = (START_COL + 0.5f) * CELL_MM;
    sim.yMm = (START_ROW + 0.5f) * CELL_MM - START_OFFSET_MM;
    sim.heading = DIR_NORTH;
    sim.headingDeg = headingToDeg(sim.heading);
    trailLen = 0;
    trailPush(sim.xMm, sim.yMm);
    simSegIdx = 0;
    simSegProg = 0;
}

static void applyTurn(TurnDir d, int deg) {
    int steps = deg / 90;
    for (int i = 0; i < steps; i++) {
        if (d == TURN_RIGHT)
            sim.heading = (sim.heading + 1) % 4;
        else
            sim.heading = (sim.heading + 3) % 4;
    }
    sim.headingDeg = headingToDeg(sim.heading);
}

static SimPose interpSeg(const PathSeg& seg, float u) {
    SimPose a = segStartPose;
    if (seg.phase == PH_FORWARD) {
        float ux, uy;
        headingUnit(a.heading, ux, uy);
        float dist = ticksToMm(seg.ticks);
        a.xMm += ux * dist * u;
        a.yMm += uy * dist * u;
    } else if (seg.phase == PH_PIVOT) {
        float deg = seg.dir == TURN_RIGHT ? -90.0f * u : 90.0f * u;
        a.headingDeg = headingToDeg(a.heading) + deg;
    } else if (seg.phase == PH_SPOT) {
        float deg = seg.dir == TURN_RIGHT ? -180.0f * u : 180.0f * u;
        a.headingDeg = headingToDeg(a.heading) + deg;
    }
    return a;
}

static void simCommitSeg(const PathSeg& seg) {
    if (seg.phase == PH_FORWARD) {
        float ux, uy;
        headingUnit(sim.heading, ux, uy);
        float dist = ticksToMm(seg.ticks);
        sim.xMm += ux * dist;
        sim.yMm += uy * dist;
    } else if (seg.phase == PH_PIVOT) {
        applyTurn(seg.dir, 90);
    } else if (seg.phase == PH_SPOT) {
        applyTurn(seg.dir, 180);
    }
    sim.headingDeg = headingToDeg(sim.heading);
    trailPush(sim.xMm, sim.yMm);
}

// ── Maze / NVS ───────────────────────────────────────────────────────────────
static void setupMaze() {
    maze.reset();
    for (int c = 0; c < MAZE_COLS; c++)
        maze.setWall(MAZE_ROWS - 1, c, DIR_NORTH, true);
    for (int r = 0; r < MAZE_ROWS; r++)
        maze.setWall(r, MAZE_COLS - 1, DIR_EAST, true);
    maze.setGoalSingle(GOAL_ROW, GOAL_COL);
}

static bool nvsLoadWalls() {
    if (!prefs.begin(NVS_NS, true)) return false;
    bool ok = prefs.isKey(NVS_WALLS);
    if (ok) prefs.getBytes(NVS_WALLS, maze.walls, sizeof(maze.walls));
    prefs.end();
    return ok;
}

// ── Script builder (mirror src/main.cpp buildMoveScript) ─────────────────────
static void scriptReset() { scriptLen = 0; }
static void scriptPush(RunPhase ph, long target, TurnDir d = TURN_NONE) {
    if (scriptLen < MAX_SCRIPT) script[scriptLen++] = { ph, target, d };
}
static void scriptPushFwd(long ticks) {
    scriptPush(PH_FORWARD, ticks + PT.stopBias, TURN_NONE);
}
static void scriptPushSpot(TurnDir d, float deg) {
    long target = PT.useImu ? (long)(deg + 0.5f) : 906;
    scriptPush(PH_SPOT, target, d);
}
static void scriptPushPivot(TurnDir d, float deg) {
    long target = PT.useImu ? (long)(deg + 0.5f) : 900;
    scriptPush(PH_PIVOT, target, d);
}

static void appendScriptToPath() {
    for (int i = 0; i < scriptLen; i++) {
        if (pathLen >= MAX_PATH) return;
        PathSeg s = {};
        s.phase = script[i].phase;
        s.dir   = script[i].dir;
        s.ticks = script[i].target;
        if (s.phase == PH_FORWARD)
            s.distCm = ticksToCm(labs(s.ticks));
        else
            s.distCm = (s.phase == PH_PIVOT) ? 90.0f : 180.0f;
        path[pathLen++] = s;
    }
}

static void buildMoveScript(AbsDir bestDir) {
    int diff = ((int)bestDir - (int)robotHeading + 4) % 4;
    scriptReset();
    bool startPivot = false;
    if (diff == 1) {
        scriptPushPivot(TURN_RIGHT, 90.0f);
        startPivot = true;
    } else if (diff == 3) {
        scriptPushPivot(TURN_LEFT, 90.0f);
        startPivot = true;
    } else if (diff == 2) {
        scriptPushSpot(TURN_RIGHT, 180.0f);
    }
    long fwd = PT.ticksPerCell + pendingOffsetTicks;
    if (startPivot) {
        long postPivotToCenter = (long)(110.0f * ticksPerMm() + 0.5f);
        fwd = postPivotToCenter + pendingOffsetTicks;
    }
    pendingOffsetTicks = 0;
    scriptPushFwd(fwd);

    plannedHeading = bestDir;
    plannedRow     = robotRow + DIR_DR[bestDir];
    plannedCol     = robotCol + DIR_DC[bestDir];

    if (fastRunMode && scriptLen > 0 && script[scriptLen - 1].phase == PH_FORWARD) {
        int rr = plannedRow, cc = plannedCol;
        AbsDir hh = (AbsDir)plannedHeading;
        long singleApproach    = (long)(140.0f * ticksPerMm() + 0.5f);
        long postPivotToCenter = (long)(110.0f * ticksPerMm() + 0.5f);
        long pivotFwdOffset    = PT.ticksPerCell - singleApproach;
        long trimTol           = (long)(15.0f * ticksPerMm() + 0.5f);

        if (rr != GOAL_ROW || cc != GOAL_COL) {
            uint8_t dPlanned;
            AbsDir nextAtPlanned = maze.bestDirectionBiased(rr, cc, hh, dPlanned);
            int planTurnDiff = ((int)nextAtPlanned - (int)hh + 4) % 4;
            bool pivotAtPlanned = (planTurnDiff == 1 || planTurnDiff == 3)
                               && dPlanned != FLOOD_INFINITY
                               && !maze.hasWall(rr, cc, nextAtPlanned)
                               && scriptLen + 2 <= MAX_SCRIPT;
            if (pivotAtPlanned) {
                int diagR = rr + DIR_DR[nextAtPlanned];
                int diagC = cc + DIR_DC[nextAtPlanned];
                bool diagOk = (diagR >= 0 && diagR < MAZE_ROWS
                            && diagC >= 0 && diagC < MAZE_COLS);
                if (diagOk) {
                    long& lastFwd = script[scriptLen - 1].target;
                    if (lastFwd <= postPivotToCenter + trimTol)
                        lastFwd = postPivotToCenter;
                    else
                        lastFwd -= pivotFwdOffset;
                    TurnDir td = (planTurnDiff == 1) ? TURN_RIGHT : TURN_LEFT;
                    scriptPushPivot(td, 90.0f);
                    scriptPushFwd(postPivotToCenter);
                    hh = nextAtPlanned;
                    rr = diagR;
                    cc = diagC;
                }
            }
        }

        int safetyCap = MAZE_ROWS * MAZE_COLS;
        while (safetyCap-- > 0) {
            if (rr == GOAL_ROW && cc == GOAL_COL) break;
            if (maze.hasWall(rr, cc, hh)) break;
            int nr = rr + DIR_DR[hh];
            int nc = cc + DIR_DC[hh];
            if (nr < 0 || nr >= MAZE_ROWS || nc < 0 || nc >= MAZE_COLS) break;
            uint8_t d;
            AbsDir next = maze.bestDirectionBiased(nr, nc, hh, d);
            if (d == FLOOD_INFINITY) break;

            if (next == hh) {
                script[scriptLen - 1].target += PT.ticksPerCell;
                rr = nr; cc = nc;
                continue;
            }
            int turnDiff = ((int)next - (int)hh + 4) % 4;
            if (turnDiff != 1 && turnDiff != 3) break;
            if (scriptLen + 2 > MAX_SCRIPT) break;
            int diagR = nr + DIR_DR[next];
            int diagC = nc + DIR_DC[next];
            if (diagR < 0 || diagR >= MAZE_ROWS || diagC < 0 || diagC >= MAZE_COLS) break;
            if (maze.hasWall(nr, nc, next)) break;

            script[scriptLen - 1].target += singleApproach;
            TurnDir td = (turnDiff == 1) ? TURN_RIGHT : TURN_LEFT;
            scriptPushPivot(td, 90.0f);
            scriptPushFwd(postPivotToCenter);
            hh = next;
            rr = diagR;
            cc = diagC;
        }
        plannedRow = rr;
        plannedCol = cc;
        plannedHeading = hh;
    }
}

static bool planFullFastRun() {
    pathLen = 0;
    planErr[0] = '\0';
    if (!wallsLoaded) {
        snprintf(planErr, sizeof(planErr), "no NVS walls");
        return false;
    }
    setupMaze();
    prefs.begin(NVS_NS, true);
    if (prefs.isKey(NVS_WALLS))
        prefs.getBytes(NVS_WALLS, maze.walls, sizeof(maze.walls));
    prefs.end();

    robotRow = START_ROW;
    robotCol = START_COL;
    robotHeading = DIR_NORTH;
    pendingOffsetTicks = PT.startOffsetTicks;
    fastRunMode = true;

    maze.floodFill();
    simResetPose();

    int moves = 0;
    while (!maze.isGoal(robotRow, robotCol) && moves < 80) {
        uint8_t bestDist;
        AbsDir bestDir = maze.bestDirectionBiased(robotRow, robotCol,
                                                  (AbsDir)robotHeading, bestDist);
        if (bestDist == FLOOD_INFINITY) {
            snprintf(planErr, sizeof(planErr), "no path at (%d,%d)",
                     robotRow, robotCol);
            return false;
        }
        buildMoveScript(bestDir);
        appendScriptToPath();
        robotRow = plannedRow;
        robotCol = plannedCol;
        robotHeading = plannedHeading;
        maze.visited[robotRow][robotCol] = true;
        moves++;
    }
    if (!maze.isGoal(robotRow, robotCol)) {
        snprintf(planErr, sizeof(planErr), "goal not reached");
        return false;
    }
    planReady = true;
    simSegIdx = 0;
    simSegProg = 0;
    simPlaying = false;
    simResetPose();
    Serial.printf("[PLAN] %d segments, goal (%d,%d)\n", pathLen, GOAL_ROW, GOAL_COL);
    return true;
}

// ── JSON helpers ──────────────────────────────────────────────────────────────
static const char* phaseName(RunPhase p) {
    switch (p) {
        case PH_FORWARD: return "FWD";
        case PH_PIVOT:   return "PIVOT";
        case PH_SPOT:    return "SPOT";
        default:         return "?";
    }
}

static String jsonEscape(const char* s) {
    String o = "\"";
    for (const char* p = s; *p; p++) {
        if (*p == '"' || *p == '\\') o += '\\';
        o += *p;
    }
    o += '"';
    return o;
}

static void sendJson(const String& body) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", body);
}

// ── Web UI (PROGMEM) ────────────────────────────────────────────────────────
static const char PAGE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>mm26 Fast Run Sim</title>
<style>
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;background:#0f1117;color:#e6edf3;margin:0;padding:12px}
h1{font-size:1.2rem;margin:0 0 8px}
.wrap{display:grid;grid-template-columns:1fr 320px;gap:12px}
@media(max-width:900px){.wrap{grid-template-columns:1fr}}
canvas{background:#1a1f2e;border:1px solid #30363d;border-radius:8px;width:100%;max-height:70vh}
.panel{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px;font-size:13px}
button{margin:4px 4px 4px 0;padding:6px 12px;background:#238636;color:#fff;border:0;border-radius:6px;cursor:pointer}
button.sec{background:#21262d;border:1px solid #30363d;color:#e6edf3}
#segList{max-height:220px;overflow:auto;font-family:monospace;font-size:11px}
.seg{padding:3px 0;border-bottom:1px solid #21262d}
.seg.active{color:#58a6ff}
.stat{color:#8b949e}
.ok{color:#3fb950}.err{color:#f85149}
</style></head><body>
<h1>mm26 — Fast Run path sim</h1>
<p class="stat">Robot 10×8.5 cm · axle 4 cm from rear · cell 18 cm · matches production fast-run ticks</p>
<div class="wrap">
<canvas id="cv" width="720" height="520"></canvas>
<div class="panel">
<div id="wifi" class="stat">connecting…</div>
<button onclick="loadNvs()">Load NVS walls</button>
<button onclick="doPlan()">Plan fast run</button>
<button onclick="simCtl('play')">Play</button>
<button class="sec" onclick="simCtl('pause')">Pause</button>
<button class="sec" onclick="simCtl('reset')">Reset pose</button>
<button class="sec" onclick="simCtl('step')">Step</button>
<p id="status" class="stat">idle</p>
<p><b>Planned segments</b></p>
<div id="segList"></div>
</div>
<script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
let maze={},path=[],trail=[],sim={},geom={};
async function j(u,opt){const r=await fetch(u,opt);return r.json();}
async function refresh(){
  const d=await j('/api/state');
  maze=d.maze; path=d.path; trail=d.trail; sim=d.sim; geom=d.geom;
  document.getElementById('wifi').textContent='http://'+location.host+' · '+d.ip;
  const st=document.getElementById('status');
  st.textContent=d.planReady?`seg ${d.sim.seg+1}/${path.length} · ${sim.xCm.toFixed(1)},${sim.yCm.toFixed(1)} cm hd ${['N','E','S','W'][sim.heading]}`:(d.planErr||'no plan');
  st.className=d.planReady?'ok':'err';
  const sl=document.getElementById('segList');
  sl.innerHTML=path.map((s,i)=>`<div class="seg ${i===d.sim.seg?'active':''}">${i+1}. ${s.phase} ${s.distCm.toFixed(1)} cm ${s.dir||''}</div>`).join('');
  draw();
}
function draw(){
  const rows=maze.rows||6,cols=maze.cols||3,cell=geom.cellMm||180;
  const pad=40, gw=cols*cell, gh=rows*cell;
  const sc=Math.min((cv.width-pad*2)/gw,(cv.height-pad*2)/gh);
  const ox=pad, oy=cv.height-pad-gh*sc;
  const tx=x=>ox+x*sc, ty=y=>oy+(gh-y)*sc;
  ctx.fillStyle='#1a1f2e';ctx.fillRect(0,0,cv.width,cv.height);
  const vis=maze.visited||[];
  for(let r=0;r<rows;r++)for(let c=0;c<cols;c++){
    const wx=c*cell,wy=r*cell,i=r*cols+c;
    ctx.fillStyle=vis[i]?'#132013':'#0d1117';
    ctx.fillRect(tx(wx),ty(wy+cell),cell*sc,cell*sc);
    ctx.strokeStyle='#21262d';ctx.lineWidth=1;
    ctx.strokeRect(tx(wx),ty(wy+cell),cell*sc,cell*sc);
  }
  ctx.lineCap='round';
  const edge=(x1,y1,x2,y2,w,col)=>{ctx.strokeStyle=col;ctx.lineWidth=w;ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();};
  for(let r=0;r<rows;r++)for(let c=0;c<cols;c++){
    const wx=c*cell,wy=r*cell,wl=(maze.walls||[])[r*cols+c]||0;
    const w=vis[r*cols+c]?5:3,col=vis[r*cols+c]?'#f0f6fc':'#484f58';
    if(wl&1)edge(tx(wx),ty(wy+cell),tx(wx+cell),ty(wy+cell),w,col);
    if(wl&2)edge(tx(wx+cell),ty(wy+cell),tx(wx+cell),ty(wy),w,col);
    if(wl&4)edge(tx(wx),ty(wy),tx(wx+cell),ty(wy),w,col);
    if(wl&8)edge(tx(wx),ty(wy+cell),tx(wx),ty(wy),w,col);
  }
  edge(tx(0),ty(gh),tx(gw),ty(gh),6,'#8b949e');
  edge(tx(0),ty(0),tx(gw),ty(0),6,'#8b949e');
  edge(tx(0),ty(0),tx(0),ty(gh),6,'#8b949e');
  edge(tx(gw),ty(0),tx(gw),ty(gh),6,'#8b949e');
  if(trail.length>1){ctx.strokeStyle='#3fb95088';ctx.lineWidth=2;ctx.beginPath();
    trail.forEach((p,i)=>{const px=tx(p.x),py=ty(p.y);i?ctx.lineTo(px,py):ctx.moveTo(px,py);});ctx.stroke();}
  const L=geom.lenMm||100,W=geom.widMm||85,rear=geom.axleRearMm||40;
  const rad=sim.headingDeg*Math.PI/180;
  const cx=tx(sim.xMm),cy=ty(sim.yMm);
  ctx.save();ctx.translate(cx,cy);ctx.rotate(-rad);
  ctx.fillStyle='#f0883e88';ctx.strokeStyle='#f0883e';ctx.lineWidth=2;
  ctx.fillRect(-W/2*sc,-rear*sc,L*sc,W*sc);ctx.strokeRect(-W/2*sc,-rear*sc,L*sc,W*sc);
  ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(0,0,3,0,7);ctx.fill();
  ctx.restore();
  const gr=geom.goal||[0,2];
  ctx.fillStyle='#a371f7';ctx.beginPath();
  ctx.arc(tx((gr[1]+0.5)*cell),ty((gr[0]+0.5)*cell),6,0,7);ctx.fill();
}
async function loadNvs(){await j('/api/load',{method:'POST'});refresh();}
async function doPlan(){await j('/api/plan',{method:'POST'});refresh();}
async function simCtl(a){await j('/api/sim/'+a,{method:'POST'});refresh();}
setInterval(refresh,200);
refresh();
</script></body></html>
)rawhtml";

static SimPose displayPose() {
    if (!planReady || simSegIdx >= pathLen)
        return sim;
    return interpSeg(path[simSegIdx], simSegProg);
}

static float segmentDurationSec(const PathSeg& seg) {
    if (seg.phase == PH_FORWARD)
        return ticksToMm(seg.ticks) / simSpeedMmS;
    if (seg.phase == PH_PIVOT) return 0.40f;
    if (seg.phase == PH_SPOT)  return 0.70f;
    return 0.3f;
}

static void simBeginSegment() {
    segStartPose = sim;
    simSegProg   = 0;
}

static void simAdvance(float dt) {
    if (!planReady || pathLen == 0) return;
    while (dt > 0.0001f && simSegIdx < pathLen) {
        const PathSeg& seg = path[simSegIdx];
        float dur          = segmentDurationSec(seg);
        float remain       = (1.0f - simSegProg) * dur;
        if (dt >= remain) {
            dt -= remain;
            simCommitSeg(seg);
            simSegIdx++;
            simBeginSegment();
            if (simSegIdx >= pathLen) {
                simPlaying = false;
                break;
            }
        } else {
            simSegProg += dt / dur;
            dt = 0;
        }
    }
}

static String buildStateJson() {
    SimPose disp = displayPose();
    String j     = "{";
    j += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    j += "\"planReady\":" + String(planReady ? "true" : "false") + ",";
    j += "\"planErr\":" + jsonEscape(planErr) + ",";
    j += "\"wallsLoaded\":" + String(wallsLoaded ? "true" : "false") + ",";
    j += "\"sim\":{\"seg\":" + String(simSegIdx) + ",\"prog\":" + String(simSegProg, 3);
    j += ",\"xMm\":" + String(disp.xMm, 1) + ",\"yMm\":" + String(disp.yMm, 1);
    j += ",\"xCm\":" + String(disp.xMm / 10.0f, 2) + ",\"yCm\":" + String(disp.yMm / 10.0f, 2);
    j += ",\"heading\":" + String(disp.heading) + ",\"headingDeg\":" + String(disp.headingDeg, 1);
    j += ",\"playing\":" + String(simPlaying ? "true" : "false") + "},";
    j += "\"geom\":{\"cellMm\":" + String(CELL_MM, 1);
    j += ",\"lenMm\":" + String(ROBOT_LEN_MM) + ",\"widMm\":" + String(ROBOT_WID_MM);
    j += ",\"axleRearMm\":" + String(AXLE_TO_REAR_MM);
    j += ",\"axleFrontMm\":" + String(AXLE_TO_NOSE_MM);
    j += ",\"goal\":[" + String(GOAL_ROW) + "," + String(GOAL_COL) + "]},";
    j += "\"maze\":{\"rows\":" + String(MAZE_ROWS) + ",\"cols\":" + String(MAZE_COLS) + ",\"w\":[";
    for (int r = 0; r < MAZE_ROWS; r++)
        for (int c = 0; c < MAZE_COLS; c++) {
            if (r || c) j += ',';
            j += String(maze.walls[r][c]);
        }
    j += "],\"f\":[";
    for (int r = 0; r < MAZE_ROWS; r++)
        for (int c = 0; c < MAZE_COLS; c++) {
            if (r || c) j += ',';
            j += String(maze.flood[r][c]);
        }
    j += "],\"visited\":[";
    for (int r = 0; r < MAZE_ROWS; r++)
        for (int c = 0; c < MAZE_COLS; c++) {
            if (r || c) j += ',';
            j += maze.visited[r][c] ? "1" : "0";
        }
    j += "]},";
    j += "\"path\":[";
    for (int i = 0; i < pathLen; i++) {
        if (i) j += ',';
        const PathSeg& s = path[i];
        j += "{\"phase\":\"" + String(phaseName(s.phase)) + "\"";
        j += ",\"distCm\":" + String(s.distCm, 2);
        j += ",\"ticks\":" + String(s.ticks);
        j += ",\"dir\":\"";
        j += (s.dir == TURN_RIGHT) ? "R" : (s.dir == TURN_LEFT ? "L" : "");
        j += "\"}";
    }
    j += "],\"trail\":[";
    for (int i = 0; i < trailLen; i++) {
        if (i) j += ',';
        j += "{\"x\":" + String(trail[i].x, 1) + ",\"y\":" + String(trail[i].y, 1) + "}";
    }
    float totalCm = 0;
    for (int i = 0; i < pathLen; i++)
        if (path[i].phase == PH_FORWARD) totalCm += path[i].distCm;
    j += "],\"totalFwdCm\":" + String(totalCm, 1) + "}";
    return j;
}

static void setupHttp() {
    server.on("/", []() {
        server.send_P(200, "text/html", PAGE_HTML);
    });
    server.on("/api/state", HTTP_GET, []() { sendJson(buildStateJson()); });
    server.on("/api/load", HTTP_POST, []() {
        wallsLoaded = nvsLoadWalls();
        if (wallsLoaded) {
            setupMaze();
            prefs.begin(NVS_NS, true);
            prefs.getBytes(NVS_WALLS, maze.walls, sizeof(maze.walls));
            prefs.end();
            maze.floodFill();
            snprintf(planErr, sizeof(planErr), "walls loaded");
        } else {
            snprintf(planErr, sizeof(planErr), "NVS empty — run Explore first");
        }
        sendJson(buildStateJson());
    });
    server.on("/api/plan", HTTP_POST, []() {
        wallsLoaded = nvsLoadWalls();
        planFullFastRun();
        sendJson(buildStateJson());
    });
    server.on("/api/sim/play", HTTP_POST, []() {
        if (planReady) {
            simPlaying = true;
            simLastMs  = millis();
        }
        sendJson(buildStateJson());
    });
    server.on("/api/sim/pause", HTTP_POST, []() {
        simPlaying = false;
        sendJson(buildStateJson());
    });
    server.on("/api/sim/reset", HTTP_POST, []() {
        simPlaying = false;
        simResetPose();
        simBeginSegment();
        sendJson(buildStateJson());
    });
    server.on("/api/sim/step", HTTP_POST, []() {
        if (planReady && simSegIdx < pathLen) {
            simCommitSeg(path[simSegIdx]);
            simSegIdx++;
            simBeginSegment();
        }
        sendJson(buildStateJson());
    });
    server.begin();
}

static void connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] joining %s", WIFI_SSID);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(300);
        Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED)
        Serial.printf("\n[WiFi] http://%s/\n", WiFi.localIP().toString().c_str());
    else
        Serial.println("\n[WiFi] connect failed — AP still offline");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    setupMaze();
    wallsLoaded = nvsLoadWalls();
    if (wallsLoaded) {
        prefs.begin(NVS_NS, true);
        prefs.getBytes(NVS_WALLS, maze.walls, sizeof(maze.walls));
        prefs.end();
        maze.floodFill();
    }
    connectWifi();
    setupHttp();
    simResetPose();
    simBeginSegment();
    Serial.println("[main-wifi] Fast-run planner + sim ready");
    Serial.printf("  cell=%.0f mm  ticks/cell=%ld  offset=%.1f cm\n",
                  CELL_MM, PT.ticksPerCell, START_OFFSET_MM / 10.0f);
}

void loop() {
    server.handleClient();
    uint32_t now = millis();
    if (simPlaying && planReady) {
        float dt = (simLastMs == 0) ? 0.0f : (now - simLastMs) / 1000.0f;
        simLastMs = now;
        if (dt > 0.05f) dt = 0.05f;
        simAdvance(dt);
    } else {
        simLastMs = now;
    }
}