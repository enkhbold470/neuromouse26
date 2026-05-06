#ifndef MICROMOUSE_MAZE_H
#define MICROMOUSE_MAZE_H

#include "PinConfig.h"

enum AbsDir : uint8_t { DIR_NORTH = 0, DIR_EAST = 1, DIR_SOUTH = 2, DIR_WEST = 3 };

static const uint8_t DIR_WALL[4]     = { WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST };
static const uint8_t DIR_OPPOSITE[4] = { DIR_SOUTH,  DIR_WEST,  DIR_NORTH,  DIR_EAST  };
static const int8_t  DIR_DC[4]       = {  0,  1,  0, -1 };
static const int8_t  DIR_DR[4]       = {  1,  0, -1,  0 };

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
        // Border walls
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
            // BUG-4 fix: use tail % MAZE_CELLS for goal seeding as well
            qRow[tail % MAZE_CELLS] = goalRow[i];
            qCol[tail % MAZE_CELLS] = goalCol[i];
            tail++;
        }

        while (head != tail) {
            uint8_t r = qRow[head % MAZE_CELLS];
            uint8_t c = qCol[head % MAZE_CELLS];
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

#endif
