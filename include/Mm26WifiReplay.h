#pragma once
// Offline fast-run path planner (mirrors src/main.cpp buildMoveScript chain).
#include "MicromouseMaze.h"

#ifndef MM26_REPLAY_ROWS
#define MM26_REPLAY_ROWS 6
#define MM26_REPLAY_COLS 3
#define MM26_REPLAY_GOAL_R 0
#define MM26_REPLAY_GOAL_C 2
#endif

struct Mm26ReplayStep {
    uint8_t phase;    // 0 FWD 1 PIV 2 SPOT
    uint8_t turnDir;  // 0 - 1 R 2 L
    long    ticks;
    float   distCm;
    float   distMm;
};

constexpr int MM26_REPLAY_MAX_PATH = 160;

struct Mm26ReplayPlanCtx {
    MicromouseMaze maze;
    long           ticksPerCell    = 1405;
    long           startOffsetTicks = 351;
    long           stopBias        = 0;
    bool           fastRunMode     = true;

    uint8_t robotRow = 0, robotCol = 0, robotHeading = 0;
    uint8_t plannedRow = 0, plannedCol = 0, plannedHeading = 0;
    long    pendingOffsetTicks = 0;

    enum { PH_FWD = 0, PH_PIV = 1, PH_SPOT = 2, TURN_NONE = 0, TURN_R = 1, TURN_L = 2 };
    struct Step { uint8_t phase; long target; uint8_t dir; };
    Step   script[8];
    int    scriptLen = 0;

    void setupMaze() {
        maze.reset();
        for (int c = 0; c < MM26_REPLAY_COLS; c++)
            maze.setWall(MM26_REPLAY_ROWS - 1, c, DIR_NORTH, true);
        for (int r = 0; r < MM26_REPLAY_ROWS; r++)
            maze.setWall(r, MM26_REPLAY_COLS - 1, DIR_EAST, true);
        maze.setGoalSingle(MM26_REPLAY_GOAL_R, MM26_REPLAY_GOAL_C);
    }

    float ticksPerMm() const { return (float)ticksPerCell / CELL_MM; }

    void scriptReset() { scriptLen = 0; }
    void scriptPush(uint8_t ph, long target, uint8_t d = 0) {
        if (scriptLen < 8) script[scriptLen++] = { ph, target, d };
    }
    void scriptPushFwd(long t) { scriptPush(PH_FWD, t + stopBias, 0); }
    void scriptPushPivot(uint8_t d) { scriptPush(PH_PIV, 90, d); }
    void scriptPushSpot(uint8_t d) { scriptPush(PH_SPOT, 180, d); }

    void buildMoveScript(AbsDir bestDir) {
        int diff = ((int)bestDir - (int)robotHeading + 4) % 4;
        scriptReset();
        bool startPivot = false;
        if (diff == 1) { scriptPushPivot(TURN_R); startPivot = true; }
        else if (diff == 3) { scriptPushPivot(TURN_L); startPivot = true; }
        else if (diff == 2) { scriptPushSpot(TURN_R); }

        long fwd = ticksPerCell + pendingOffsetTicks;
        if (startPivot) {
            long post = (long)(110.0f * ticksPerMm() + 0.5f);
            fwd = post + pendingOffsetTicks;
        }
        pendingOffsetTicks = 0;
        scriptPushFwd(fwd);
        plannedHeading = bestDir;
        plannedRow     = robotRow + DIR_DR[bestDir];
        plannedCol     = robotCol + DIR_DC[bestDir];

        if (!fastRunMode || scriptLen == 0 || script[scriptLen - 1].phase != PH_FWD)
            return;

        int rr = plannedRow, cc = plannedCol;
        AbsDir hh = (AbsDir)plannedHeading;
        long singleApproach    = (long)(140.0f * ticksPerMm() + 0.5f);
        long postPivotToCenter = (long)(110.0f * ticksPerMm() + 0.5f);
        long pivotFwdOffset    = ticksPerCell - singleApproach;
        long trimTol           = (long)(15.0f * ticksPerMm() + 0.5f);

        if (rr != MM26_REPLAY_GOAL_R || cc != MM26_REPLAY_GOAL_C) {
            uint8_t dPlanned;
            AbsDir nextAtPlanned = maze.bestDirectionBiased(rr, cc, hh, dPlanned);
            int planTurnDiff = ((int)nextAtPlanned - (int)hh + 4) % 4;
            if ((planTurnDiff == 1 || planTurnDiff == 3) && dPlanned != FLOOD_INFINITY
                && !maze.hasWall(rr, cc, nextAtPlanned) && scriptLen + 2 <= 8) {
                int diagR = rr + DIR_DR[nextAtPlanned];
                int diagC = cc + DIR_DC[nextAtPlanned];
                if (diagR >= 0 && diagR < MM26_REPLAY_ROWS
                    && diagC >= 0 && diagC < MM26_REPLAY_COLS) {
                    long& lastFwd = script[scriptLen - 1].target;
                    if (lastFwd <= postPivotToCenter + trimTol)
                        lastFwd = postPivotToCenter;
                    else
                        lastFwd -= pivotFwdOffset;
                    scriptPushPivot(planTurnDiff == 1 ? TURN_R : TURN_L);
                    scriptPushFwd(postPivotToCenter);
                    hh = nextAtPlanned;
                    rr = diagR;
                    cc = diagC;
                }
            }
        }

        int cap = MM26_REPLAY_ROWS * MM26_REPLAY_COLS;
        while (cap-- > 0) {
            if (rr == MM26_REPLAY_GOAL_R && cc == MM26_REPLAY_GOAL_C) break;
            if (maze.hasWall(rr, cc, hh)) break;
            int nr = rr + DIR_DR[hh], nc = cc + DIR_DC[hh];
            if (nr < 0 || nr >= MM26_REPLAY_ROWS || nc < 0 || nc >= MM26_REPLAY_COLS) break;
            uint8_t d;
            AbsDir next = maze.bestDirectionBiased(nr, nc, hh, d);
            if (d == FLOOD_INFINITY) break;
            if (next == hh) {
                script[scriptLen - 1].target += ticksPerCell;
                rr = nr; cc = nc;
                continue;
            }
            int turnDiff = ((int)next - (int)hh + 4) % 4;
            if (turnDiff != 1 && turnDiff != 3) break;
            if (scriptLen + 2 > 8) break;
            int diagR = nr + DIR_DR[next], diagC = nc + DIR_DC[next];
            if (diagR < 0 || diagR >= MM26_REPLAY_ROWS || diagC < 0 || diagC >= MM26_REPLAY_COLS) break;
            if (maze.hasWall(nr, nc, next)) break;
            script[scriptLen - 1].target += singleApproach;
            scriptPushPivot(turnDiff == 1 ? TURN_R : TURN_L);
            scriptPushFwd(postPivotToCenter);
            hh = next; rr = diagR; cc = diagC;
        }
        plannedRow = rr; plannedCol = cc; plannedHeading = hh;
    }

    static Mm26ReplayStep stepFrom(const Step& s, float tpm) {
        Mm26ReplayStep o = {};
        o.phase   = s.phase;
        o.turnDir = s.dir;
        o.ticks   = s.target;
        if (s.phase == PH_FWD) {
            o.distMm = (float)labs(s.target) / tpm;
            o.distCm = o.distMm / 10.0f;
        } else {
            o.distCm = (s.phase == PH_PIV) ? 90.0f : 180.0f;
            o.distMm = o.distCm * 10.0f;
        }
        return o;
    }

    int planFull(Mm26ReplayStep* out, int maxOut, char* err, size_t errLen) {
        robotRow = 0; robotCol = 0; robotHeading = DIR_NORTH;
        pendingOffsetTicks = startOffsetTicks;
        fastRunMode = true;
        maze.floodFill();
        int n = 0, moves = 0;
        while (!maze.isGoal(robotRow, robotCol) && moves < 80) {
            uint8_t bestDist;
            AbsDir bestDir = maze.bestDirectionBiased(robotRow, robotCol,
                                                      (AbsDir)robotHeading, bestDist);
            if (bestDist == FLOOD_INFINITY) {
                snprintf(err, errLen, "no path (%d,%d)", robotRow, robotCol);
                return 0;
            }
            buildMoveScript(bestDir);
            float tpm = ticksPerMm();
            for (int i = 0; i < scriptLen && n < maxOut; i++)
                out[n++] = stepFrom(script[i], tpm);
            robotRow = plannedRow;
            robotCol = plannedCol;
            robotHeading = plannedHeading;
            maze.visited[robotRow][robotCol] = true;
            moves++;
        }
        if (!maze.isGoal(robotRow, robotCol)) {
            snprintf(err, errLen, "goal not reached");
            return 0;
        }
        err[0] = '\0';
        return n;
    }
};
