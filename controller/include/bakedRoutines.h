#pragma once
#include <Arduino.h>

// Two routines taught in the desktop app's Teach dialog, baked in for the
// gamepad's A and B buttons (see playBakedRoutine() in main.cpp). Copied from
// q8botone-python's routines folder: one-hand-greet.json and tilt.json. The
// app keeps its own copies - re-teaching one there doesn't change these.
//
// Each waypoint is a pose - 8 joint angles in tenths of a degree, in MOVE's
// order and units (Q8Bot.move_all() rounds to the same tenths) - or, with
// stance set, the current gait's standing pose (the app's Start/End rows).
// moveMs is how long the glide there takes, holdMs the pause after it.

struct RoutineWaypoint {
  bool stance;
  int16_t angleTenths[8];
  uint16_t moveMs;
  uint16_t holdMs;
};

struct BakedRoutine {
  const char* name;
  const RoutineWaypoint* waypoints;
  uint8_t count;
};

// Button A: "One Hand Greet" (one-hand-greet.json, taught 2026-09-14)
static const RoutineWaypoint ONE_HAND_GREET_WAYPOINTS[] = {
  {true,  {0, 0, 0, 0, 0, 0, 0, 0}, 1000, 0},  // stance (start)
  {false, {386, 1414, 388, 1432, 379, 1427, 388, 1412}, 1000, 0},
  {false, {821, 1028, 451, 1431, 382, 1437, 91, 1642}, 500, 0},
  {false, {-1025, 410, 496, 1455, 240, 744, -65, 1799}, 500, 0},
  {false, {-1027, 410, 506, 1459, 527, 803, -66, 1811}, 500, 0},
  {false, {-39, 413, 520, 1455, 537, 805, -63, 1811}, 200, 0},
  {false, {-1388, 414, 520, 1455, 537, 805, -63, 1811}, 200, 0},
  {false, {-23, 411, 520, 1455, 536, 805, -63, 1811}, 200, 0},
  {false, {-1390, 411, 520, 1455, 536, 805, -63, 1811}, 200, 0},
  {false, {-135, 411, 520, 1455, 536, 805, -63, 1811}, 200, 0},
  {false, {-1360, 412, 520, 1455, 536, 805, -63, 1811}, 200, 500},
  {false, {393, 1410, 387, 1427, 366, 1426, 379, 1425}, 1000, 0},
  {true,  {0, 0, 0, 0, 0, 0, 0, 0}, 1000, 0},  // stance (end)
};
static const BakedRoutine ROUTINE_A = {"One Hand Greet", ONE_HAND_GREET_WAYPOINTS, 13};

// Button B: "Tilt" (tilt.json, taught 2026-09-14)
static const RoutineWaypoint TILT_WAYPOINTS[] = {
  {true,  {0, 0, 0, 0, 0, 0, 0, 0}, 1000, 0},  // stance (start)
  {false, {-198, 1970, -320, 2025, -286, 2056, -312, 2007}, 500, 0},
  {false, {667, 1136, 156, 1651, 659, 1143, 167, 1633}, 500, 0},
  {false, {-395, 2025, -468, 2213, -510, 2163, -556, 2229}, 500, 0},
  {false, {170, 1631, 677, 1134, 146, 1657, 663, 1138}, 500, 0},
  {false, {-420, 2206, -514, 2198, -528, 2247, -420, 2061}, 500, 0},
  {true,  {0, 0, 0, 0, 0, 0, 0, 0}, 500, 0},  // stance (end)
};
static const BakedRoutine ROUTINE_B = {"Tilt", TILT_WAYPOINTS, 7};
