#ifndef MICROMOUSE_MAZE_H
#define MICROMOUSE_MAZE_H

#include "PinConfig.h"
#include "Tuning.h"

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
    bool    provenOpen[MAZE_SIZE][MAZE_SIZE][4];

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
        if (provenOpen[r][c][d]) return false;
        return (walls[r][c] & DIR_WALL[d]) != 0;
    }

    void markProvenOpen(int r, int c, AbsDir d) {
        if (!inBounds(r, c)) return;
        provenOpen[r][c][d] = true;
        int nr = r + DIR_DR[d], nc = c + DIR_DC[d];
        if (inBounds(nr, nc))
            provenOpen[nr][nc][DIR_OPPOSITE[d]] = true;
    }

    void reset() {
        for (int r = 0; r < MAZE_SIZE; r++)
            for (int c = 0; c < MAZE_SIZE; c++) {
                walls[r][c]   = 0;
                flood[r][c]   = FLOOD_INFINITY;
                visited[r][c] = false;
                for (int d = 0; d < 4; d++)
                    provenOpen[r][c][d] = false;
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

    int countUnvisited(uint8_t maxRow, uint8_t maxCol) const {
        int n = 0;
        for (int r = 0; r < maxRow; r++)
            for (int c = 0; c < maxCol; c++)
                if (!visited[r][c]) n++;
        return n;
    }

    // Multi-source BFS seeded from every unvisited cell in the active region.
    // Robot always routes toward the nearest unexplored territory first.
    // Falls back to normal floodFill if no unvisited cells remain.
    void floodFillExplore(uint8_t maxRow, uint8_t maxCol) {
        for (int r = 0; r < MAZE_SIZE; r++)
            for (int c = 0; c < MAZE_SIZE; c++)
                flood[r][c] = FLOOD_INFINITY;

        static uint8_t qRow[MAZE_CELLS];
        static uint8_t qCol[MAZE_CELLS];
        uint16_t head = 0, tail = 0;

        for (int r = 0; r < maxRow; r++) {
            for (int c = 0; c < maxCol; c++) {
                if (!visited[r][c]) {
                    flood[r][c] = 0;
                    qRow[tail % MAZE_CELLS] = r;
                    qCol[tail % MAZE_CELLS] = c;
                    tail++;
                }
            }
        }
        if (tail == 0) { floodFill(); return; }

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

    // Print ASCII maze to Serial. North at top, row 15 = top row.
    // Each cell shows 2-char hex flood value; visited cells prefixed '*'.
    // Walls: '+--' horizontal, '|' vertical.
    void dump() const {
        // Column header
        Serial.print("   ");
        for (int c = 0; c < MAZE_SIZE; c++) {
            char buf[4]; snprintf(buf, sizeof(buf), "%2d ", c);
            Serial.print(buf);
        }
        Serial.println();

        for (int r = MAZE_SIZE - 1; r >= 0; r--) {
            // North wall row
            Serial.print("   ");
            for (int c = 0; c < MAZE_SIZE; c++) {
                Serial.print(hasWall(r, c, DIR_NORTH) ? "+--" : "+  ");
            }
            Serial.println("+");
            // Cell row
            char rowLabel[4]; snprintf(rowLabel, sizeof(rowLabel), "%2d ", r);
            Serial.print(rowLabel);
            for (int c = 0; c < MAZE_SIZE; c++) {
                Serial.print(hasWall(r, c, DIR_WEST) ? "|" : " ");
                uint8_t f = flood[r][c];
                char cell[3];
                if (f == FLOOD_INFINITY) {
                    snprintf(cell, sizeof(cell), "%s", visited[r][c] ? "**" : "  ");
                } else {
                    snprintf(cell, sizeof(cell), "%02X", f);
                }
                Serial.print(cell);
            }
            Serial.println("|");
        }
        // South border
        Serial.print("   ");
        for (int c = 0; c < MAZE_SIZE; c++) Serial.print("+--");
        Serial.println("+");
    }

    AbsDir bestDirectionBiased(uint8_t r, uint8_t c, AbsDir h, uint8_t &bestDist,
                               bool penalizeVisited = false) const {
        bestDist = FLOOD_INFINITY;
        AbsDir best = h;
        int bestPref = 99;
        for (int d = 0; d < 4; d++) {
            if (hasWall(r, c, (AbsDir)d)) continue;
            int nr = r + DIR_DR[d], nc = c + DIR_DC[d];
            if (!inBounds(nr, nc)) continue;
            uint8_t dist = flood[nr][nc];
            if (dist == FLOOD_INFINITY) continue;
            if (penalizeVisited && visited[nr][nc]) {
                uint16_t bumped = (uint16_t)dist + EXPLORE_VISITED_FLOOD_PENALTY;
                dist = (bumped >= FLOOD_INFINITY) ? (uint8_t)(FLOOD_INFINITY - 1) : (uint8_t)bumped;
            }
            int turn = ((int)d - (int)h + 4) % 4;
            int turnPref;
            if      (turn == 0) turnPref = 0;
            else if (turn == 3) turnPref = 1;
            else if (turn == 1) turnPref = 2;
            else                turnPref = 3;
            int pref = (visited[nr][nc] ? 4 : 0) + turnPref;
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
