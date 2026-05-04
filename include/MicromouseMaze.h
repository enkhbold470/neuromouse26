// =============================================================================
// MicromouseMaze.h
// Flood-fill maze solver for a standard 16x16 micromouse maze.
//
// Coordinate system
//   (0,0) = bottom-left corner (start cell)
//   (15,15) = top-right corner (default goal for half-maze competitions)
//   Competition goal = centre 4 cells: (7,7),(7,8),(8,7),(8,8) — configurable.
//
// Wall encoding (per cell, 4 bits)
//   bit 0 = NORTH wall present
//   bit 1 = EAST  wall present
//   bit 2 = SOUTH wall present
//   bit 3 = WEST  wall present
//
// Flood-fill algorithm
//   1. Set every goal cell's distance to 0.
//   2. BFS outward: for each cell, neighbours without a wall between them get
//      distance = current + 1 (if their distance is not already ≤ current).
//   3. To navigate: robot always steps to the adjacent cell with the lowest
//      flood-fill value.
// =============================================================================
#pragma once
#include <Arduino.h>

// --------------------------------------------------------------------------
// Maze dimensions
// --------------------------------------------------------------------------
#define MAZE_SIZE       16   // 16 x 16 standard maze
#define MAZE_CELLS      (MAZE_SIZE * MAZE_SIZE)  // 256 cells

// --------------------------------------------------------------------------
// Wall bitmasks
// --------------------------------------------------------------------------
#define WALL_NORTH  0x01
#define WALL_EAST   0x02
#define WALL_SOUTH  0x04
#define WALL_WEST   0x08
#define WALL_ALL    0x0F  // all four walls set (unexplored default)

// Opposite direction lookup
#define OPPOSITE_NORTH  WALL_SOUTH
#define OPPOSITE_EAST   WALL_WEST
#define OPPOSITE_SOUTH  WALL_NORTH
#define OPPOSITE_WEST   WALL_EAST

// Absolute directions as indices (matches robot heading enum in main.cpp)
enum AbsDir : uint8_t {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
};

// Wall bit for an absolute direction
static const uint8_t DIR_WALL[4] = {
    WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST
};

// Opposite direction index
static const uint8_t DIR_OPPOSITE[4] = {
    DIR_SOUTH, DIR_WEST, DIR_NORTH, DIR_EAST
};

// Cell offset (col delta, row delta) for each absolute direction
static const int8_t DIR_DC[4] = {  0,  1,  0, -1 };  // col: N stays, E +1, S stays, W -1
static const int8_t DIR_DR[4] = {  1,  0, -1,  0 };  // row: N +1, E stays, S -1, W stays

// --------------------------------------------------------------------------
// Flood-fill value sentinel
// --------------------------------------------------------------------------
#define FLOOD_INFINITY  255   // unreachable / unknown — fits in uint8_t

// --------------------------------------------------------------------------
// MicromouseMaze class
// --------------------------------------------------------------------------
class MicromouseMaze {
public:
    // ---- Public data (accessed directly for speed) ----
    uint8_t walls[MAZE_SIZE][MAZE_SIZE];    // walls[row][col]
    uint8_t flood[MAZE_SIZE][MAZE_SIZE];    // flood-fill distances
    bool    visited[MAZE_SIZE][MAZE_SIZE];  // has cell been physically entered?

    // ---- Goal cells ----
    // Default: competition centre 4 cells for 16x16 maze.
    // Change with setGoal() for testing.
    uint8_t goalCount;
    uint8_t goalRow[4];
    uint8_t goalCol[4];

    // ---- Constructor ----
    MicromouseMaze() {
        reset();
    }

    // ---- reset() ---- clears all knowledge, sets default goal ----
    void reset() {
        Serial.println(F("[MAZE] reset() — clearing all walls, flood values, visited flags"));

        // Every cell starts with all four outer walls potentially present.
        // Interior walls are assumed absent (unknown) initially — only the
        // border of the maze has definite outer walls set.
        for (int r = 0; r < MAZE_SIZE; r++) {
            for (int c = 0; c < MAZE_SIZE; c++) {
                // Start unexplored: no interior walls known → 0 (no wall bits set)
                // Border walls ARE known from maze construction.
                walls[r][c]   = 0x00;
                flood[r][c]   = FLOOD_INFINITY;
                visited[r][c] = false;
            }
        }

        // Apply known outer border walls
        Serial.println(F("[MAZE] reset() — applying outer border walls"));
        for (int c = 0; c < MAZE_SIZE; c++) {
            setWall(0,           c, DIR_SOUTH, true);  // bottom row: south wall
            setWall(MAZE_SIZE-1, c, DIR_NORTH, true);  // top row: north wall
        }
        for (int r = 0; r < MAZE_SIZE; r++) {
            setWall(r, 0,           DIR_WEST,  true);  // left col: west wall
            setWall(r, MAZE_SIZE-1, DIR_EAST,  true);  // right col: east wall
        }

        // Default 16x16 competition goal: centre 4 cells
        setGoalCentre4();

        Serial.println(F("[MAZE] reset() done"));
    }

    // ---- setGoalCentre4() — standard 16x16 competition goal ----
    void setGoalCentre4() {
        goalCount  = 4;
        goalRow[0] = 7;  goalCol[0] = 7;
        goalRow[1] = 7;  goalCol[1] = 8;
        goalRow[2] = 8;  goalCol[2] = 7;
        goalRow[3] = 8;  goalCol[3] = 8;
        Serial.println(F("[MAZE] Goal set to centre 4 cells: (7,7)(7,8)(8,7)(8,8)"));
    }

    // ---- setGoalSingle() — useful for debugging with a small maze ----
    void setGoalSingle(uint8_t row, uint8_t col) {
        goalCount  = 1;
        goalRow[0] = row;
        goalCol[0] = col;
        Serial.printf("[MAZE] Goal set to single cell: (%d,%d)\n", row, col);
    }

    // ---- isGoal() ----
    bool isGoal(uint8_t row, uint8_t col) const {
        for (int i = 0; i < goalCount; i++) {
            if (goalRow[i] == row && goalCol[i] == col) return true;
        }
        return false;
    }

    // ---- inBounds() ----
    bool inBounds(int row, int col) const {
        return row >= 0 && row < MAZE_SIZE && col >= 0 && col < MAZE_SIZE;
    }

    // ---- setWall() — set or clear a wall; also mirrors to neighbour ----
    void setWall(int row, int col, AbsDir dir, bool present) {
        if (!inBounds(row, col)) return;
        uint8_t bit = DIR_WALL[dir];
        if (present) {
            walls[row][col] |= bit;
        } else {
            walls[row][col] &= ~bit;
        }
        // Mirror: set the same wall on the neighbouring cell
        int nr = row + DIR_DR[dir];
        int nc = col + DIR_DC[dir];
        if (inBounds(nr, nc)) {
            uint8_t oppBit = DIR_WALL[DIR_OPPOSITE[dir]];
            if (present) {
                walls[nr][nc] |= oppBit;
            } else {
                walls[nr][nc] &= ~oppBit;
            }
        }
    }

    // ---- hasWall() ----
    bool hasWall(int row, int col, AbsDir dir) const {
        if (!inBounds(row, col)) return true;  // out of bounds acts as a wall
        return (walls[row][col] & DIR_WALL[dir]) != 0;
    }

    // ---- floodFill() — BFS from goal(s) outward ----
    // Recalculates the entire flood array from scratch.
    // Must be called after any wall update.
    void floodFill() {
        Serial.println(F("[MAZE] floodFill() — starting BFS"));
        unsigned long t0 = millis();

        // Step 1: reset all distances to infinity
        for (int r = 0; r < MAZE_SIZE; r++)
            for (int c = 0; c < MAZE_SIZE; c++)
                flood[r][c] = FLOOD_INFINITY;

        // Step 2: seed goal cells with distance 0
        // Use a simple queue implemented as a circular buffer (256 cells max)
        static uint8_t qRow[MAZE_CELLS];
        static uint8_t qCol[MAZE_CELLS];
        uint16_t head = 0, tail = 0;

        for (int i = 0; i < goalCount; i++) {
            flood[goalRow[i]][goalCol[i]] = 0;
            qRow[tail] = goalRow[i];
            qCol[tail] = goalCol[i];
            tail++;
            Serial.printf("[MAZE] floodFill() seeded goal cell (%d,%d) = 0\n",
                          goalRow[i], goalCol[i]);
        }

        // Step 3: BFS
        uint16_t processed = 0;
        while (head != tail) {
            uint8_t r = qRow[head];
            uint8_t c = qCol[head];
            head++;
            processed++;

            uint8_t nextDist = flood[r][c] + 1;

            for (int d = 0; d < 4; d++) {
                if (hasWall(r, c, (AbsDir)d)) continue;  // wall blocks passage
                int nr = r + DIR_DR[d];
                int nc = c + DIR_DC[d];
                if (!inBounds(nr, nc)) continue;
                if (flood[nr][nc] <= nextDist) continue; // already better or equal
                flood[nr][nc] = nextDist;
                qRow[tail % MAZE_CELLS] = nr;
                qCol[tail % MAZE_CELLS] = nc;
                tail++;
            }
        }

        unsigned long elapsed = millis() - t0;
        Serial.printf("[MAZE] floodFill() done — processed %d cells in %lu ms\n",
                      processed, elapsed);
    }

    // ---- bestDirection() — returns the neighbour direction with lowest flood value ----
    // Returns DIR_NORTH (0) by default if all neighbours are blocked/equal.
    // Sets 'bestDist' to the flood value of the best neighbour.
    AbsDir bestDirection(uint8_t row, uint8_t col, uint8_t &bestDist) const {
        bestDist   = FLOOD_INFINITY;
        AbsDir best = DIR_NORTH;
        for (int d = 0; d < 4; d++) {
            if (hasWall(row, col, (AbsDir)d)) continue;
            int nr = row + DIR_DR[d];
            int nc = col + DIR_DC[d];
            if (!inBounds(nr, nc)) continue;
            if (flood[nr][nc] < bestDist) {
                bestDist = flood[nr][nc];
                best     = (AbsDir)d;
            }
        }
        Serial.printf("[MAZE] bestDirection from (%d,%d) → dir=%d floodDist=%d\n",
                      row, col, (int)best, (int)bestDist);
        return best;
    }

    // ---- printFlood() — dump the full 16x16 flood array to Serial ----
    void printFlood() const {
        Serial.println(F("[MAZE] === Flood-fill map (row 15 = top) ==="));
        for (int r = MAZE_SIZE - 1; r >= 0; r--) {
            Serial.printf("[MAZE] %2d | ", r);
            for (int c = 0; c < MAZE_SIZE; c++) {
                if (flood[r][c] == FLOOD_INFINITY)
                    Serial.print(F(" ?? "));
                else
                    Serial.printf("%3d ", (int)flood[r][c]);
            }
            Serial.println();
        }
        Serial.println(F("[MAZE] ======================================"));
    }

    // ---- printWalls() — dump known wall bits for every cell ----
    void printWalls() const {
        Serial.println(F("[MAZE] === Wall map (N/E/S/W bits, row 15 = top) ==="));
        for (int r = MAZE_SIZE - 1; r >= 0; r--) {
            Serial.printf("[MAZE] %2d | ", r);
            for (int c = 0; c < MAZE_SIZE; c++) {
                Serial.printf("0x%01X ", walls[r][c]);
            }
            Serial.println();
        }
        Serial.println(F("[MAZE] ==========================================="));
    }

    // ---- printVisited() — mark visited cells ----
    void printVisited() const {
        Serial.println(F("[MAZE] === Visited cells (row 15 = top) ==="));
        for (int r = MAZE_SIZE - 1; r >= 0; r--) {
            Serial.printf("[MAZE] %2d | ", r);
            for (int c = 0; c < MAZE_SIZE; c++) {
                Serial.print(visited[r][c] ? " V " : " . ");
            }
            Serial.println();
        }
        Serial.println(F("[MAZE] =========================================="));
    }
};
