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

static inline bool sideWallPresent(int raw) {
    return raw > WALL_SIDE_THRESH && raw < WALL_SIDE_MAX;
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
    sampleIR();
    AbsDir lt = (AbsDir)((heading + 3) % 4);
    AbsDir rt = (AbsDir)((heading + 1) % 4);
    setWallSensed(r, c, lt, sideWallPresent(irVal[1]));
    setWallSensed(r, c, rt, sideWallPresent(irVal[2]));
    Serial.printf("[MID] (%d,%d) hd=%c L=%d R=%d\n",
                  r, c, "NESW"[heading], irVal[1], irVal[2]);
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
    sampleIR();
    bool wF = irFrontBlocked();
    bool wL = sideWallPresent(irVal[1]);
    bool wR = sideWallPresent(irVal[2]);
    AbsDir hd = (AbsDir)robotHeading;
    AbsDir lt = (AbsDir)((robotHeading + 3) % 4);
    AbsDir rt = (AbsDir)((robotHeading + 1) % 4);
    setWallSensed(robotRow, robotCol, hd, wF);
    setWallSensed(robotRow, robotCol, lt, wL);
    setWallSensed(robotRow, robotCol, rt, wR);
    Serial.printf("[SENSE] (%d,%d) hd=%c F=%s LF=%d RF=%d L=%s R=%s\n",
                  robotRow, robotCol, "NESW"[robotHeading],
                  wF ? "wall" : "open", irVal[0], irVal[3],
                  wL ? "wall" : "open", wR ? "wall" : "open");
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
static void buildMoveScript(AbsDir bestDir) {
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
