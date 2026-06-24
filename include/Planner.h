// include/Planner.h — maze setup, wall sensing, move-script construction.
//
// `setupMaze` resets the full 16×16 grid (perimeter walls + centre 2×2 goal).
// `senseAndStoreWalls` samples IR + writes F/L/R walls into the maze.
// `buildMoveScript` decides between SPOT 90 / SPOT 180 (dead end) / 1-cell FWD,
// then in fast-run extends the FWD through any straight cells ahead.
//
// `maze` is defined in main.cpp — extern'd here.

#ifndef MM26_PLANNER_H
#define MM26_PLANNER_H

#include <Arduino.h>
#include "Tuning.h"
#include "MicromouseMaze.h"
#include "IRSensors.h"
#include "MotionScript.h"
#include "Pose.h"

extern MicromouseMaze maze;

static void setupMaze() {
    maze.reset();  // 16×16 borders + traditional centre 2×2 goal
}

// Wall sensing sets AND clears edges from IR each cell arrival. Stale "walls"
// that were never cleared caused 2-cell ping-pong (map thought only A↔B was open).
// Readings above WALL_SIDE_MAX are skipped (saturation / too close).
constexpr int WALL_SIDE_MAX = 2000;

// Legacy fixed-threshold side test (used when SIDE_ADAPTIVE=false).
static inline bool sideWallPresent(int raw) {
    return raw > WALL_SIDE_THRESH && raw < WALL_SIDE_MAX;
}

// Adaptive side-wall decision (see Tuning.h [J]). Tri-state so an untrustworthy
// read (ambient near ADC saturation → lit−ambient delta collapses) is SKIPPED
// rather than guessed — the prior map value is kept. Threshold is a FRACTION of
// the per-run reference, so room lighting divides out.
enum SideState { SIDE_OPEN, SIDE_WALL, SIDE_UNKNOWN };

static SideState sideWallState(int raw, int ref, int amb) {
    if (!SIDE_ADAPTIVE) return sideWallPresent(raw) ? SIDE_WALL : SIDE_OPEN;
    if (amb > SIDE_SAT_AMB) return SIDE_UNKNOWN;                 // saturated → don't trust
    if (ref < SIDE_REF_MIN) ref = SIDE_REF_MIN;                 // guard a bad reference
    return (raw > (int)(SIDE_WALL_FRAC * (float)ref)) ? SIDE_WALL : SIDE_OPEN;
}

// Capture this run's side WALL references live so room lighting divides out.
// Robot must be in the start cell (0,0) facing North, where the WEST border
// wall is guaranteed on the LEFT. Right is captured if a wall is there too,
// else scaled from the left by the factory L/R ratio. Implausible reads keep
// the factory IR_CAL. Call once at run start (robot stationary in start cell).
static void calibrateSideRefs() {
    if (!SIDE_ADAPTIVE) return;
    sampleSideMedian();
    int   liveL = irVal[1], liveR = irVal[2];
    float ratio = (float)IR_CAL_R45 / (float)IR_CAL_L45;
    if (liveL >= SIDE_REF_MIN) {
        sideRefL = liveL;
        sideRefR = (liveR >= SIDE_REF_MIN) ? liveR : (int)(liveL * ratio + 0.5f);
    } else {
        sideRefL = IR_CAL_L45;
        sideRefR = (liveR >= SIDE_REF_MIN) ? liveR : IR_CAL_R45;
    }
    Serial.printf("[SIDECAL] live L=%d R=%d amb=%d/%d -> refL=%d refR=%d\n",
                  liveL, liveR, irAmb[1], irAmb[2], sideRefL, sideRefR);
}

// Do not clear perimeter borders (neighbor out of bounds).
static void setWallSensed(int r, int c, AbsDir d, bool present) {
    if (!present) {
        int nr = r + DIR_DR[d], nc = c + DIR_DC[d];
        if (!maze.inBounds(nr, nc)) return;
    }
    maze.setWall(r, c, d, present);
}

// Called at ~50% of the forward move, while robot is between cells.
// At mid-cell the 45° sensors are far enough from any front wall to read true
// side walls of the destination cell without front-wall contamination.
static void senseSideWallsMidCell(int r, int c, int heading) {
    if (!maze.inBounds(r, c)) return;
    sampleSideMedian();
    AbsDir lt = (AbsDir)((heading + 3) % 4);
    AbsDir rt = (AbsDir)((heading + 1) % 4);
    SideState sL = sideWallState(irVal[1], sideRefL, irAmb[1]);
    SideState sR = sideWallState(irVal[2], sideRefR, irAmb[2]);
    if (sL != SIDE_UNKNOWN) setWallSensed(r, c, lt, sL == SIDE_WALL);
    if (sR != SIDE_UNKNOWN) setWallSensed(r, c, rt, sR == SIDE_WALL);
    Serial.printf("[MID] (%d,%d) hd=%c L=%d/%d%s R=%d/%d%s amb=%d/%d\n",
                  r, c, "NESW"[heading],
                  irVal[1], sideRefL, sL == SIDE_UNKNOWN ? "?" : (sL == SIDE_WALL ? "W" : "o"),
                  irVal[2], sideRefR, sR == SIDE_UNKNOWN ? "?" : (sR == SIDE_WALL ? "W" : "o"),
                  irAmb[1], irAmb[2]);
}

static bool irFrontBlocked() {
    return irVal[0] > WALL_FRONT_THRESH || irVal[3] > WALL_FRONT_THRESH;
}

static bool mazeDeadEnd(int r, int c, uint8_t h) {
    AbsDir f = (AbsDir)h;
    AbsDir l = (AbsDir)((h + 3) % 4);
    AbsDir rt = (AbsDir)((h + 1) % 4);
    return maze.hasWall(r, c, f) && maze.hasWall(r, c, l) && maze.hasWall(r, c, rt);
}

static void senseAndStoreWalls() {
    sampleSideMedian();                 // median sides + ambient; also refreshes front irVal[0]/[3]
    bool wF = irFrontBlocked();
    SideState sL = sideWallState(irVal[1], sideRefL, irAmb[1]);
    SideState sR = sideWallState(irVal[2], sideRefR, irAmb[2]);
    AbsDir hd = (AbsDir)robotHeading;
    AbsDir lt = (AbsDir)((robotHeading + 3) % 4);
    AbsDir rt = (AbsDir)((robotHeading + 1) % 4);
    setWallSensed(robotRow, robotCol, hd, wF);
    if (sL != SIDE_UNKNOWN) setWallSensed(robotRow, robotCol, lt, sL == SIDE_WALL);
    if (sR != SIDE_UNKNOWN) setWallSensed(robotRow, robotCol, rt, sR == SIDE_WALL);
    Serial.printf("[SENSE] (%d,%d) hd=%c F=%s(LF=%d RF=%d) L=%s(%d/%d) R=%s(%d/%d) amb=%d/%d\n",
                  robotRow, robotCol, "NESW"[robotHeading],
                  wF ? "wall" : "open", irVal[0], irVal[3],
                  sL == SIDE_UNKNOWN ? "??" : (sL == SIDE_WALL ? "wall" : "open"), irVal[1], sideRefL,
                  sR == SIDE_UNKNOWN ? "??" : (sR == SIDE_WALL ? "wall" : "open"), irVal[2], sideRefR,
                  irAmb[1], irAmb[2]);
}

static bool atDeadEnd() {
    return mazeDeadEnd(robotRow, robotCol, robotHeading);
}

// Build a one-cell-move script: optional spot turn, then forward 1 cell (or
// chain of consecutive straight cells in fast run).
//
// 180° at a dead end (diff==2): SPOT then 1 cell forward along bestDir (exit).
//
// `pendingOffsetTicks` is added to the forward target then cleared — lets
// the first leg from start (or after a 180° re-anchor) compensate for the
// −4.1 cm rear-against-wall reference pose.
// Continuous fast-run route compiler (Smooth mode). Walks the flood gradient
// from the current pose to the goal and emits ONE script of FWD/CURVE segments
// so the mouse flows start→goal with no inter-cell brake. Straights are
// shortened by CURVE_PRE/POST_TICKS around each arc so the centerline path is
// continuous. The first turn from a standstill stays a SPOT (can't arc from
// rest); 180°s stay SPOT. A route longer than MAX_SCRIPT brakes once at the
// seam and EXPLORE_THINK re-plans the remainder.
static void buildFastSmoothRoute() {
    scriptReset();
    long acc = pendingOffsetTicks;          // straight ticks pending → current cell centre
    pendingOffsetTicks = 0;
    int   r = robotRow, c = robotCol;
    AbsDir h = (AbsDir)robotHeading;
    bool  rolling = false;                   // true once the robot is moving at a turn
    int   cap = MAZE_CELLS;
    while (cap-- > 0 && scriptLen < MAX_SCRIPT - 3) {
        if (maze.isGoal(r, c)) break;
        uint8_t dd;
        AbsDir nd = maze.bestDirectionBiased(r, c, h, dd);
        if (dd == FLOOD_INFINITY) break;
        int diff = ((int)nd - (int)h + 4) % 4;

        if (diff == 0) {                     // straight-through: accumulate one cell
            acc += CELL_TICKS;
            r += DIR_DR[h]; c += DIR_DC[h];
        } else if (diff == 1 || diff == 3) { // 90° turn → arc (or SPOT if from rest)
            TurnDir td = (diff == 1) ? TURN_RIGHT : TURN_LEFT;
            long approach = acc - CURVE_PRE_TICKS;          // straight up to the arc entry
            // Arc if already rolling, or if there's room to accelerate from rest
            // to arc-entry speed before the arc (else a stationary in-place spin).
            bool canArc = rolling || approach >= CURVE_MIN_ENTRY_TICKS;
            if (!canArc) {
                scriptPushSpot(td, PIVOT_90_DEG);           // stationary corner → in-place spin
                h = nd;                                     // stay in cell; next iter drives out
            } else {
                if (approach > 20) { scriptPushFwd(approach); }
                scriptPushCurve(td);
                rolling = true;
                h = nd;
                r += DIR_DR[h]; c += DIR_DC[h];
                acc = CELL_TICKS - CURVE_POST_TICKS;        // exit straight already POST-covered
            }
        } else {                             // diff==2: 180° (rare on optimal path) → SPOT
            if (acc > 20) { scriptPushFwd(acc); }
            acc = 0; rolling = false;
            scriptPushSpot(TURN_RIGHT, SPOT_180_DEG);
            h = (AbsDir)(((int)h + 2) % 4);
        }
    }
    if (acc > 20 && scriptLen < MAX_SCRIPT) scriptPushFwd(acc);
    plannedRow     = r;
    plannedCol     = c;
    plannedHeading = h;
}

static void buildMoveScript(AbsDir bestDir) {
    // Smooth fast run: compile the whole continuous route at once.
    if (fastRunMode && g_smoothMode && CURVE_ENABLE) {
        buildFastSmoothRoute();
        return;
    }
    int diff = ((int)bestDir - (int)robotHeading + 4) % 4;
    scriptReset();

    if (diff == 1) {
        // Spot at cell center (no translation). Same in explore and fast run.
        scriptPushSpot(TURN_RIGHT, 90.0f);
    } else if (diff == 3) {
        scriptPushSpot(TURN_LEFT, 90.0f);
    } else if (diff == 2) {
        // Dead end: face the exit, then drive one cell back out.
        scriptPushSpot(TURN_RIGHT, 180.0f);
        long fwd = CELL_TICKS + pendingOffsetTicks;
        pendingOffsetTicks = 0;
        scriptPushFwd(fwd);
        plannedHeading = bestDir;
        plannedRow     = robotRow + DIR_DR[bestDir];
        plannedCol     = robotCol + DIR_DC[bestDir];
        return;
    }

    // diff==0 (straight) or just-pushed SPOT both fall here to push the
    // 1-cell forward. diff==2 (180°) already returned above.
    long fwd = CELL_TICKS + pendingOffsetTicks;
    pendingOffsetTicks = 0;
    scriptPushFwd(fwd);
    plannedHeading = bestDir;
    plannedRow     = robotRow + DIR_DR[bestDir];
    plannedCol     = robotCol + DIR_DC[bestDir];

    // Fast-run straight-chain extension: fuse consecutive straight cells into
    // one PH_FORWARD. The trapezoid in the executor naturally stretches
    // accel → cruise → decel over the whole chain, so longer straights hit
    // higher peak speed. Chain breaks at any turn; endPhase brakes before
    // the next SPOT so wheels are stationary at the turn.
    if (fastRunMode && scriptLen > 0 && script[scriptLen - 1].phase == PH_FORWARD) {
        int rr = plannedRow, cc = plannedCol;
        AbsDir hh = (AbsDir)plannedHeading;
        int safetyCap = MAZE_CELLS;
        while (safetyCap-- > 0) {
            if (maze.isGoal(rr, cc)) break;
            if (maze.hasWall(rr, cc, hh)) break;
            int nr = rr + DIR_DR[hh];
            int nc = cc + DIR_DC[hh];
            if (!maze.inBounds(nr, nc)) break;
            uint8_t d;
            AbsDir next = maze.bestDirectionBiased(nr, nc, hh, d);
            if (d == FLOOD_INFINITY) break;
            if (next != hh) break;

            script[scriptLen - 1].target += CELL_TICKS;
            rr = nr; cc = nc;
        }
        plannedRow     = rr;
        plannedCol     = cc;
        plannedHeading = hh;
        // [FAST chained] print intentionally omitted — fast run keeps serial
        // quiet so the control loop isn't blocked by UART writes at 115200 baud.
    }
}

#endif
