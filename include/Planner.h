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
    maze.reset();                        // 16×16 outer border + default centre goal
    maze.setGoalSingle(GOAL_ROW, GOAL_COL);

    // Seal the 6×3 physical maze inside the 16×16 grid. reset() only walls
    // the outer edge at row/col 15; without this, flood-fill treats rows 6+
    // and cols 3+ as wide-open and routes through fantasy space → ping-pong
    // and BOXED at the goal end of a return leg.
    for (int c = 0; c < MAZE_SIZE; c++)
        maze.setWall(MAZE_ROWS - 1, c, DIR_NORTH, true);
    for (int r = 0; r < MAZE_SIZE; r++)
        maze.setWall(r, MAZE_COLS - 1, DIR_EAST, true);
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
    if (raw < SIDE_OPEN_CEIL) return SIDE_OPEN;                   // user: open when well below touch cal
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
    // 30° side sensors see the front wall at ~104 mm diagonal when that wall is
    // 90 mm ahead, producing ~220 raw counts — enough to trip the side threshold
    // and write a phantom side wall. Mid-cell sensing (senseSideWallsMidCell)
    // ran with the front wall 208 mm away (safe). Skip re-sensing sides here
    // and keep whatever mid-cell wrote; only the front wall write is trustworthy.
    SideState sL, sR;
    if (wF) {
        sL = SIDE_UNKNOWN;   // front contaminating 30° side sensors — keep prior
        sR = SIDE_UNKNOWN;
    } else {
        sL = sideWallState(irVal[1], sideRefL, irAmb[1]);
        sR = sideWallState(irVal[2], sideRefR, irAmb[2]);
    }
    AbsDir hd = (AbsDir)robotHeading;
    AbsDir lt = (AbsDir)((robotHeading + 3) % 4);
    AbsDir rt = (AbsDir)((robotHeading + 1) % 4);
    setWallSensed(robotRow, robotCol, hd, wF);
    if (sL != SIDE_UNKNOWN) setWallSensed(robotRow, robotCol, lt, sL == SIDE_WALL);
    if (sR != SIDE_UNKNOWN) setWallSensed(robotRow, robotCol, rt, sR == SIDE_WALL);
    Serial.printf("[SENSE] (%d,%d) hd=%c F=%s(LF=%d RF=%d) L=%s(%d/%d) R=%s(%d/%d) amb=%d/%d%s\n",
                  robotRow, robotCol, "NESW"[robotHeading],
                  wF ? "wall" : "open", irVal[0], irVal[3],
                  sL == SIDE_UNKNOWN ? "??" : (sL == SIDE_WALL ? "wall" : "open"), irVal[1], sideRefL,
                  sR == SIDE_UNKNOWN ? "??" : (sR == SIDE_WALL ? "wall" : "open"), irVal[2], sideRefR,
                  irAmb[1], irAmb[2],
                  wF ? " [side-skip:frontwall]" : "");
}

static bool atDeadEnd() {
    return mazeDeadEnd(robotRow, robotCol, robotHeading);
}

// Dead-end exit sequence (main-branch): front IR align → 180° → short reverse →
// one cell forward along exitDir. fromR/fromC default to live robot pose.
static void scriptPushDeadEndExit(AbsDir exitDir, int fromR = -1, int fromC = -1) {
    if (fromR < 0) { fromR = robotRow; fromC = robotCol; }
    float ticksPerMm = (float)CELL_TICKS / CELL_PITCH_MM;
    long  revTicks   = (long)(DEADEND_REVERSE_MM * ticksPerMm + 0.5f);
    long  fwdTicks   = (long)(DEADEND_FWD_MM     * ticksPerMm + 0.5f);
    // Explore: skip ALIGN creep — spot 180 and go (ALIGN was a common stall source).
    if (!exploreMode) scriptPushAlignFront();
    scriptPushSpot(TURN_RIGHT, SPOT_180_DEG);
    scriptPushFwd(-revTicks);
    scriptPushFwd( fwdTicks);
    pendingOffsetTicks = 0;
    plannedHeading = exitDir;
    plannedRow     = fromR + DIR_DR[exitDir];
    plannedCol     = fromC + DIR_DC[exitDir];
}

static bool preTurnAlignNeeded(int r, int c, AbsDir h, int diff) {
    (void)r; (void)c; (void)h; (void)diff;
    return false;   // simple safe: never creep-align before turns
}

// Successful drive from → to proves that passage is open; clear any phantom
// wall IR may have written before the cell was first entered from another heading.
static void clearTraversedWall(uint8_t fromR, uint8_t fromC, uint8_t toR, uint8_t toC) {
    for (int d = 0; d < 4; d++) {
        if (toR == fromR + DIR_DR[d] && toC == fromC + DIR_DC[d]) {
            maze.setWall(fromR, fromC, (AbsDir)d, false);
            maze.markProvenOpen(fromR, fromC, (AbsDir)d);
            return;
        }
    }
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
        AbsDir nd = maze.bestDirectionBiased(r, c, h, dd, returnHomeMode);
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
    // Fast run smooth: multi-cell FWD+CURVE arcs.
    if (fastRunMode && g_smoothMode && CURVE_ENABLE) {
        buildFastSmoothRoute();
        return;
    }
    // Explore: always falls through to one-cell stop-pivot below.
    int diff = ((int)bestDir - (int)robotHeading + 4) % 4;
    scriptReset();

    if (diff == 1) {
        if (preTurnAlignNeeded(robotRow, robotCol, (AbsDir)robotHeading, diff))
            scriptPushAlignFront();
        scriptPushSpot(TURN_RIGHT, 90.0f);
    } else if (diff == 3) {
        if (preTurnAlignNeeded(robotRow, robotCol, (AbsDir)robotHeading, diff))
            scriptPushAlignFront();
        scriptPushSpot(TURN_LEFT, 90.0f);
    } else if (diff == 2) {
        scriptPushDeadEndExit(bestDir);
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

    // Straight-chain: fast run only. Explore always one cell per script.
    bool chainAllowed = fastRunMode;
    if (chainAllowed && scriptLen > 0 && script[scriptLen - 1].phase == PH_FORWARD) {
        int rr = plannedRow, cc = plannedCol;
        AbsDir hh = (AbsDir)plannedHeading;
        int safetyCap = MAZE_ROWS * MAZE_COLS;
        while (safetyCap-- > 0) {
            if (returnHomeMode) {
                if (rr == (int)START_ROW && cc == (int)START_COL) break;
            } else if (maze.isGoal((uint8_t)rr, (uint8_t)cc)) {
                break;
            }
            if (maze.hasWall(rr, cc, hh)) break;
            int nr = rr + DIR_DR[hh], nc = cc + DIR_DC[hh];
            if (nr < 0 || nr >= MAZE_ROWS || nc < 0 || nc >= MAZE_COLS) break;
            if (!fastRunMode && !maze.visited[nr][nc]) break;
            uint8_t d;
            AbsDir next = maze.bestDirectionBiased(nr, nc, hh, d, returnHomeMode);
            if (d == FLOOD_INFINITY) break;
            if (next != hh) break;

            script[scriptLen - 1].target += CELL_TICKS;
            rr = nr; cc = nc;
        }
        plannedRow     = rr;
        plannedCol     = cc;
        plannedHeading = hh;
    }
}

#endif
