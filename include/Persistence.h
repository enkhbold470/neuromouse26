// include/Persistence.h — NVS save/load for walls + fast-run speed.
//
// Uses ESP32 Preferences with namespace "mm26". Walls keyed "walls"
// (256 bytes from maze.walls); fast cruise speed keyed "fspeed".
//
// `maze` is defined in main.cpp — extern'd here.

#ifndef MM26_PERSISTENCE_H
#define MM26_PERSISTENCE_H

#include <Arduino.h>
#include <Preferences.h>
#include "MicromouseMaze.h"
#include "Pose.h"
#include "Tuning.h"

extern MicromouseMaze maze;

static Preferences _prefs;
static const char* NVS_NS      = "mm26";
static const char* NVS_WALLS   = "walls";
static const char* NVS_VISITED = "visited";
static const char* NVS_FSPEED  = "fspeed";

static bool nvsSaveWalls() {
    if (!_prefs.begin(NVS_NS, false)) return false;
    _prefs.putBytes(NVS_WALLS, maze.walls, sizeof(maze.walls));
    // Pack visited[][] into a 32-byte bitmap so fast-run can restrict
    // path planning to cells the robot actually traversed (see
    // fortifyUnvisited in Planner.h).
    uint8_t bitmap[(MAZE_SIZE * MAZE_SIZE + 7) / 8] = {0};
    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            if (maze.visited[r][c]) {
                int bit = r * MAZE_SIZE + c;
                bitmap[bit / 8] |= (uint8_t)(1u << (bit % 8));
            }
        }
    }
    _prefs.putBytes(NVS_VISITED, bitmap, sizeof(bitmap));
    _prefs.end();
    return true;
}

static bool nvsLoadWalls() {
    if (!_prefs.begin(NVS_NS, true)) return false;
    bool ok = _prefs.isKey(NVS_WALLS);
    if (ok) {
        _prefs.getBytes(NVS_WALLS, maze.walls, sizeof(maze.walls));
        // Visited bitmap is optional — older saves won't have it. Without
        // it, fortifyUnvisited would block every cell, so treat missing
        // visited as "everything was visited" (legacy behavior).
        if (_prefs.isKey(NVS_VISITED)) {
            uint8_t bitmap[(MAZE_SIZE * MAZE_SIZE + 7) / 8] = {0};
            _prefs.getBytes(NVS_VISITED, bitmap, sizeof(bitmap));
            for (int r = 0; r < MAZE_SIZE; r++) {
                for (int c = 0; c < MAZE_SIZE; c++) {
                    int bit = r * MAZE_SIZE + c;
                    maze.visited[r][c] = (bitmap[bit / 8] & (1u << (bit % 8))) != 0;
                }
            }
        } else {
            for (int r = 0; r < MAZE_SIZE; r++)
                for (int c = 0; c < MAZE_SIZE; c++)
                    maze.visited[r][c] = true;
        }
    }
    _prefs.end();
    return ok;
}

static void nvsClearWalls() {
    if (!_prefs.begin(NVS_NS, false)) return;
    _prefs.remove(NVS_WALLS);
    _prefs.remove(NVS_VISITED);
    _prefs.end();
}

// Print the NVS-saved walls to Serial as (a) a copy-pasteable C++ array and
// (b) an ASCII map. Loads NVS into maze.walls as a side-effect — fine from
// IDLE because setupMaze() resets walls on the next explore/fast run.
static void nvsDumpWalls() {
    if (!nvsLoadWalls()) {
        Serial.println("[DUMP] no NVS walls saved");
        return;
    }
    Serial.println();
    Serial.println("[DUMP] === C++ array (paste into a header) ===");
    Serial.printf("const uint8_t SAVED_WALLS[%d][%d] = {\n", MAZE_SIZE, MAZE_SIZE);
    for (int r = 0; r < MAZE_SIZE; r++) {
        Serial.print("  {");
        for (int c = 0; c < MAZE_SIZE; c++) {
            Serial.printf("0x%02x%s", maze.walls[r][c], (c == MAZE_SIZE - 1) ? "" : ",");
        }
        Serial.printf("}%s  // row %d\n", (r == MAZE_SIZE - 1) ? "" : ",", r);
    }
    Serial.println("};");
    Serial.println();
    Serial.println("[DUMP] === ASCII map (north up, ST=start, **=goal) ===");
    for (int r = MAZE_SIZE - 1; r >= 0; r--) {
        Serial.print("+");
        for (int c = 0; c < MAZE_SIZE; c++)
            Serial.print((maze.walls[r][c] & WALL_NORTH) ? "--+" : "  +");
        Serial.println();
        Serial.print((maze.walls[r][0] & WALL_WEST) ? "|" : " ");
        for (int c = 0; c < MAZE_SIZE; c++) {
            const char* content = "  ";
            if (maze.isGoal((uint8_t)r, (uint8_t)c)) content = "**";
            else if (r == START_ROW && c == START_COL) content = "ST";
            Serial.print(content);
            Serial.print((maze.walls[r][c] & WALL_EAST) ? "|" : " ");
        }
        Serial.println();
    }
    Serial.print("+");
    for (int c = 0; c < MAZE_SIZE; c++)
        Serial.print((maze.walls[0][c] & WALL_SOUTH) ? "--+" : "  +");
    Serial.println();
    Serial.println("[DUMP] done");
}

static void nvsLoadFastSpeed() {
    if (!_prefs.begin(NVS_NS, true)) return;
    if (_prefs.isKey(NVS_FSPEED)) {
        fastRunCruiseTps = _prefs.getFloat(NVS_FSPEED, FAST_RUN_CRUISE_TPS_DEFAULT);
        if (fastRunCruiseTps < FAST_RUN_CRUISE_TPS_MIN) fastRunCruiseTps = FAST_RUN_CRUISE_TPS_MIN;
        if (fastRunCruiseTps > FAST_RUN_CRUISE_TPS_MAX) fastRunCruiseTps = FAST_RUN_CRUISE_TPS_MAX;
    }
    _prefs.end();
}

static void nvsSaveFastSpeed() {
    if (!_prefs.begin(NVS_NS, false)) return;
    _prefs.putFloat(NVS_FSPEED, fastRunCruiseTps);
    _prefs.end();
}

#endif
