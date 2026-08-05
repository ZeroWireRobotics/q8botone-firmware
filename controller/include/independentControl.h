/*
  independentControl.h - Pure decision logic for standalone (no-host)
  gamepad control, ported directly from python-tools/q8bot's own control
  logic so independent mode behaves the same way operate.py would for the
  same stick position:
    - resolveDirection()  <- control_config.py's get_joystick_direction(analog_mode=True)
    - resolveTrotTier()   <- GaitManager.resolve_tier_index() + TIER_BOUNDARIES
    - ikSolveNeutral()    <- kinematics_solver.py's k_solver.ik_solve() (closed-form, no
                              external math library needed), used with each gait's own
                              (x0, y0) baseline so independent mode can settle to the
                              same neutral stance operate.py's move_xy(pos_x, pos_y, ...)
                              would, instead of snapping straight into a gait's trajectory
                              from wherever the legs happened to be.

  No I/O, no ESP-NOW, no GamepadQT dependency - just math, so it's testable
  independent of hardware. See src/main.cpp for how it's wired to actual
  gamepad reads and GAIT_RUN/MOVE sends.
*/
#ifndef independentControl_h
#define independentControl_h

#include <Arduino.h>

// Canonical direction ordering - MUST match protocol.h's DIRECTION_ORDER
// comment and python-tools/q8bot/protocol.py's DIRECTION_ORDER (also the
// order gaitTables.h's *_DIRECTIONS arrays are built in).
enum class Direction : uint8_t {
  F = 0, B = 1, L = 2, R = 3,
  FL_075 = 4, FL_05 = 5, FR_075 = 6, FR_05 = 7,
  BL_075 = 8, BL_05 = 9, BR_075 = 10, BR_05 = 11,
  NONE = 255,
};

const char* directionName(Direction d);  // for debug logging

// Ports control_config.py's get_joystick_direction(analog_mode=True)
// verbatim (0.25 / 0.6 / 0.85 breakpoints). Expects GamepadQT's axis
// convention: x negative=left/positive=right, y negative=forward/positive=backward.
Direction resolveDirection(float joystickX, float joystickY);

// Ports GaitManager.resolve_tier_index()'s hysteresis climb/drop for the
// TROT_FAST tier chain [TROT_SLOW, TROT_MED, TROT_FAST] (indices 0-2,
// matching TROT_TIER_GAIT_IDS below). currentIndex/return value are
// indices into that chain (0=slowest, 2=fastest) - TIER_BOUNDARIES =
// [(0.60,0.50),(0.85,0.75)], matching gait_manager.py exactly.
uint8_t resolveTrotTier(uint8_t currentIndex, float magnitude);

// gait_id values (see protocol.h's GAIT_IDS comment / protocol.py's
// GAIT_IDS) for the TROT_FAST tier chain, slowest to fastest.
extern const uint8_t TROT_TIER_GAIT_IDS[3];

// Gaits cyclable via SELECT, in cycle order - same list/order operate.py
// cycles through (gait_names in operate.py, GAITS.keys() minus tier
// companions).
extern const uint8_t INDEPENDENT_GAIT_IDS[5];
extern const char* const INDEPENDENT_GAIT_NAMES[5];

// Each cyclable gait's own (x0, y0) baseline - see gait_manager.py's GAITS
// dict entries (format [stacktype, x0, y0, ...]), same indexing as
// INDEPENDENT_GAIT_IDS/NAMES above.
extern const float INDEPENDENT_GAIT_NEUTRAL_X[5];
extern const float INDEPENDENT_GAIT_NEUTRAL_Y[5];

// Ports kinematics_solver.py's k_solver.ik_solve() verbatim (law-of-cosines,
// closed-form) for this robot's leg geometry (CENTER_DIST=19.5, L1=L2P=25,
// L2=L2P=40 - matches operate.py's k_solver(CENTER_DIST, L1, L2, L1, L2)).
// acos() arguments are clamped to [-1,1] to absorb float rounding at the
// domain boundary - every (x,y) this is actually called with is one of the
// gaits' own known-reachable baselines above, so a real out-of-range input
// isn't expected in practice.
void ikSolveNeutral(float x, float y, float& q1Deg, float& q2Deg);

#endif
