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
