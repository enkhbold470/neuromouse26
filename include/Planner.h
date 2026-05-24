// include/Planner.h — maze setup, wall sensing, move-script construction.
//
// `setupMaze` closes the active sub-region's far borders and sets the
// centre goal cells listed in Tuning.h::GOAL_CENTRE_* (default for the
// 7×7 sub-region is one cell at (GOAL_ROW, GOAL_COL)).
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
    maze.setGoalCentre4();
}

// Fortify every unvisited cell with walls on all four sides. Called once at
// fast-run start so flood-fill cannot route the robot through cells it never
// sensed during explore (those cells default to all-walls-down = 0, which
// the planner would happily treat as open corridors and ram into real walls
// the robot never detected). After fortify, fast-run path planning is
// strictly restricted to the visited subgraph.
//
// Goal cells the robot didn't reach during explore will end up isolated.
// EXPLORE_THINK's flood-fill will then return FLOOD_INFINITY and bail to
// CRASH — a loud "explore was incomplete" signal instead of a silent
// wall-bump loop on the run.
static void fortifyUnvisited() {
    for (int r = 0; r < MAZE_ROWS; r++) {
        for (int c = 0; c < MAZE_COLS; c++) {
            if (maze.visited[r][c]) continue;
            for (int d = 0; d < 4; d++) {
                maze.setWall(r, c, (AbsDir)d, true);  // mirrors to neighbour
            }
        }
    }
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

    // Pre-turn front-wall anchor (explore + return-home only). When the
    // planner wants a 90° turn AND the current cell has a front wall,
    // creep into PH_ALIGN_FRONT first so the SPOT happens from a known
    // position (LF/RF ≈ ALIGN_*_TARGET, ~37.5 mm front gap). Re-zeroes
    // both encoder reference (via phaseEnter) AND yawDeg integration,
    // bounding drift from long straight runs. Fast run skips it because
    // sustained-speed motion has no time for ALIGN creep — fast run
    // trusts the saved map + open-loop tick targets.
    bool preTurnAlign = !fastRunMode
                        && (diff == 1 || diff == 3)
                        && maze.hasWall(robotRow, robotCol, (AbsDir)robotHeading);

    if (diff == 1) {
        // Spot at cell center (no translation). Same in explore and fast run.
        if (preTurnAlign) scriptPushAlignFront();
        scriptPushSpot(TURN_RIGHT, 90.0f);
    } else if (diff == 3) {
        if (preTurnAlign) scriptPushAlignFront();
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

    // Straight-chain extension: fuse consecutive straight cells into one
    // PH_FORWARD so the trapezoid stretches accel → cruise → decel over the
    // whole chain. Chain breaks at any turn; endPhase brakes before the next
    // SPOT so wheels are stationary at the turn.
    //
    //   fastRunMode    — full NVS map, fuse freely.
    //   returnHomeMode — only fuse through cells visited on the forward leg
    //                    (walls known). Stops at any unvisited cell so the
    //                    per-cell sense step still fires on unknown territory.
    //   EXPLORE mode (neither flag) — NEVER chain. Per-cell brake gives IR
    //                    centering a re-anchor opportunity every cell;
    //                    chaining was tested 2026-05-23 (UCLA) and caused
    //                    lateral-drift accumulation across cells with no
    //                    intermediate re-anchor.
    //
    // Goal break point flips to (START_ROW, START_COL) during return-home so
    // we don't fuse past the actual current target.
    bool chainAllowed = fastRunMode || returnHomeMode;
    if (chainAllowed && scriptLen > 0 && script[scriptLen - 1].phase == PH_FORWARD) {
        int rr = plannedRow, cc = plannedCol;
        AbsDir hh = (AbsDir)plannedHeading;
        int safetyCap = MAZE_ROWS * MAZE_COLS;
        while (safetyCap-- > 0) {
            // Stop the chain at the current run's target. Return-home goal is
            // the single start cell; forward-run goal is any of the centre-4
            // cells (use isGoal so we don't fuse past the first one entered).
            if (returnHomeMode) {
                if (rr == (int)START_ROW && cc == (int)START_COL) break;
            } else {
                if (maze.isGoal((uint8_t)rr, (uint8_t)cc)) break;
            }
            if (maze.hasWall(rr, cc, hh)) break;
            int nr = rr + DIR_DR[hh];
            int nc = cc + DIR_DC[hh];
            if (nr < 0 || nr >= MAZE_ROWS || nc < 0 || nc >= MAZE_COLS) break;
            // Return-home: only chain through visited (= sensed) cells.
            // Fast run already has the full NVS map so this guard is
            // skipped entirely.
            if (!fastRunMode && !maze.visited[nr][nc]) break;
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
        // [chain] print intentionally omitted — fast run keeps serial quiet
        // so the control loop isn't blocked by UART writes at 115200 baud.
    }
}

#endif
