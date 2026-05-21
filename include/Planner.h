// include/Planner.h — maze setup, wall sensing, move-script construction.
//
// `setupMaze` closes the active sub-region's far borders and overrides the
// classical-centre goal to (GOAL_ROW, GOAL_COL).
// `senseAndStoreWalls` samples IR + writes F/L/R walls into the maze.
// `buildMoveScript` decides between SPOT 90 / SPOT 180 + recover / 1-cell FWD,
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
    maze.reset();
    for (int c = 0; c < MAZE_COLS; c++) {
        maze.setWall(MAZE_ROWS - 1, c, DIR_NORTH, true);
    }
    for (int r = 0; r < MAZE_ROWS; r++) {
        maze.setWall(r, MAZE_COLS - 1, DIR_EAST, true);
    }
    maze.setGoalSingle(GOAL_ROW, GOAL_COL);
}

static void senseAndStoreWalls() {
    sampleIR();
    bool wF = irVal[0] > WALL_FRONT_THRESH || irVal[3] > WALL_FRONT_THRESH;
    bool wL = irVal[1] > WALL_SIDE_THRESH;
    bool wR = irVal[2] > WALL_SIDE_THRESH;
    AbsDir hd = (AbsDir)robotHeading;
    AbsDir lt = (AbsDir)((robotHeading + 3) % 4);
    AbsDir rt = (AbsDir)((robotHeading + 1) % 4);
    maze.setWall(robotRow, robotCol, hd, wF);
    maze.setWall(robotRow, robotCol, lt, wL);
    maze.setWall(robotRow, robotCol, rt, wR);
    Serial.printf("[SENSE] (%d,%d) hd=%c F=%d L=%d R=%d irLF=%d irRF=%d irL=%d irR=%d\n",
                  robotRow, robotCol, "NESW"[robotHeading], wF, wL, wR,
                  irVal[0], irVal[3], irVal[1], irVal[2]);
}

// Build a one-cell-move script: optional spot turn, then forward 1 cell (or
// chain of consecutive straight cells in fast run).
//
// 180° turn (diff==2) is followed by a wall re-anchor:
//   align-front → SPOT 180 → reverse 2 cm → forward 1 cell.
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
        // Dead-end exit. Sequence:
        //   1. PH_ALIGN_FRONT — creep until front IR hits ALIGN_LF/RF_TARGET.
        //   2. SPOT 180° — dead-end wall is now behind us.
        //   3. Reverse DEADEND_REVERSE_MM (2 cm) at low speed.
        //   4. Forward DEADEND_FWD_MM (1 cell pitch).
        float ticksPerMm = (float)CELL_TICKS / 180.0f;
        long  revTicks   = (long)(DEADEND_REVERSE_MM * ticksPerMm + 0.5f);
        long  fwdTicks   = (long)(DEADEND_FWD_MM     * ticksPerMm + 0.5f);
        scriptPushAlignFront();
        scriptPushSpot(TURN_RIGHT, 180.0f);
        scriptPushFwd(-revTicks);
        scriptPushFwd( fwdTicks);
        pendingOffsetTicks = 0;
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
