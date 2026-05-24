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
static const char* NVS_NS     = "mm26"; 
static const char* NVS_WALLS  = "walls";
static const char* NVS_FSPEED = "fspeed";

static bool nvsSaveWalls() {
    if (!_prefs.begin(NVS_NS, false)) return false;
    _prefs.putBytes(NVS_WALLS, maze.walls, sizeof(maze.walls));
    _prefs.end();
    return true;
}

static bool nvsLoadWalls() {
    if (!_prefs.begin(NVS_NS, true)) return false;
    bool ok = _prefs.isKey(NVS_WALLS);
    if (ok) _prefs.getBytes(NVS_WALLS, maze.walls, sizeof(maze.walls));
    _prefs.end();
    return ok;
}

static void nvsClearWalls() {
    if (!_prefs.begin(NVS_NS, false)) return;
    _prefs.remove(NVS_WALLS);
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
            if (r == GOAL_ROW  && c == GOAL_COL ) content = "**";
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
