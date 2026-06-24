// include/Pose.h — robot pose + mode state.
//
// Updated by:
//   IDLE state (menu dispatch sets exploreMode/fastRunMode, resets pose)
//   buildMoveScript (writes plannedRow/Col/Heading + pendingOffsetTicks)
//   endPhase on script end (commits plannedXxx → robotXxx)
//   FAST_SPEED_EDIT state + nvsLoad/SaveFastSpeed (fastRunCruiseTps)

#ifndef MM26_POSE_H
#define MM26_POSE_H

#include <Arduino.h>
#include "Tuning.h"
#include "MicromouseMaze.h"

static uint8_t robotRow     = START_ROW;
static uint8_t robotCol     = START_COL;
static uint8_t robotHeading = DIR_NORTH;

static uint8_t plannedRow     = START_ROW;
static uint8_t plannedCol     = START_COL;
static uint8_t plannedHeading = DIR_NORTH;

static bool exploreMode = false;
static bool fastRunMode = false;
static bool returnHomeMode = false;    // after explore reaches goal, flips to head back to start

// Set when the final 180° celebration spin has been pushed. Next time
// EXPLORE_THINK runs, transition straight to GOAL instead of sensing/planning.
static bool finalTurnPending = false;

// Runtime-adjustable fast-run cruise speed (NVS-persisted).
static float fastRunCruiseTps = FAST_RUN_CRUISE_TPS_DEFAULT;

// Carries the start-pose offset (or post-180° re-anchor offset) into the
// next forward leg, then clears itself.
static long pendingOffsetTicks = 0;

// Runtime Smooth/Classic toggle (OLED menu). Default OFF so a fresh flash is
// the proven legacy stop-pivot firmware; the user enables Smooth from the menu
// after bench-tuning. When ON (and CURVE_ENABLE), fast run uses continuous
// arc turns. Phase 1 wires this to FAST RUN only; explore stays stop-pivot.
static bool g_smoothMode = false;

#endif
