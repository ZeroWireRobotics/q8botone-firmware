#include "independentControl.h"
#include <math.h>

const char* directionName(Direction d) {
  switch (d) {
    case Direction::F:      return "f";
    case Direction::B:      return "b";
    case Direction::L:      return "l";
    case Direction::R:      return "r";
    case Direction::FL_075: return "fl_0.75";
    case Direction::FL_05:  return "fl_0.5";
    case Direction::FR_075: return "fr_0.75";
    case Direction::FR_05:  return "fr_0.5";
    case Direction::BL_075: return "bl_0.75";
    case Direction::BL_05:  return "bl_0.5";
    case Direction::BR_075: return "br_0.75";
    case Direction::BR_05:  return "br_0.5";
    default:                return "none";
  }
}

Direction resolveDirection(float axis0, float axis1) {
  if (axis0 == 0.0f && axis1 == 0.0f) return Direction::NONE;

  float absA0 = fabsf(axis0);
  float absA1 = fabsf(axis1);

  // Full turn mode: high horizontal, low vertical (tightened window)
  if (absA0 >= 0.85f && absA1 <= 0.4f) {
    return (axis0 < 0) ? Direction::L : Direction::R;
  }

  if (absA1 > absA0) {
    bool forward = axis1 < 0;
    bool left = axis0 < 0;

    if (absA0 < 0.25f) {
      return forward ? Direction::F : Direction::B;
    } else if (absA0 < 0.6f) {
      if (forward) return left ? Direction::FL_075 : Direction::FR_075;
      else         return left ? Direction::BL_075 : Direction::BR_075;
    } else {
      if (forward) return left ? Direction::FL_05 : Direction::FR_05;
      else         return left ? Direction::BL_05 : Direction::BR_05;
    }
  } else {
    return (axis0 < 0) ? Direction::L : Direction::R;
  }
}

uint8_t resolveTrotTier(uint8_t currentIndex, float magnitude) {
  static const float upThresh[2]   = {0.60f, 0.85f};
  static const float downThresh[2] = {0.50f, 0.75f};
  const uint8_t chainLen = 3;

  int idx = currentIndex;
  while (idx < chainLen - 1 && magnitude >= upThresh[idx]) idx++;
  while (idx > 0 && magnitude < downThresh[idx - 1]) idx--;
  return (uint8_t)idx;
}

// gait_id: 0=TROT_SLOW, 1=TROT_MED, 2=TROT_FAST (see protocol.h)
const uint8_t TROT_TIER_GAIT_IDS[3] = {0, 1, 2};

// gait_id: 2=TROT_FAST, 3=TROT_HIGH, 4=TROT_LOW, 5=CRAWL, 6=BOUND
const uint8_t INDEPENDENT_GAIT_IDS[5] = {2, 3, 4, 5, 6};
const char* const INDEPENDENT_GAIT_NAMES[5] = {
  "TROT_FAST", "TROT_HIGH", "TROT_LOW", "CRAWL", "BOUND"
};

// (x0, y0) per gait - see gait_manager.py's GAITS dict, same order as above.
const float INDEPENDENT_GAIT_NEUTRAL_X[5] = {9.75f, 9.75f, 9.75f, 9.75f, 9.75f};
const float INDEPENDENT_GAIT_NEUTRAL_Y[5] = {43.36f, 60.0f, 25.0f, 40.0f, 33.36f};

void ikSolveNeutral(float x, float y, float& q1Deg, float& q2Deg) {
  const float d = 19.5f, l1 = 25.0f, l2 = 40.0f;

  float c1 = sqrtf((x - d) * (x - d) + y * y);
  float c2 = sqrtf(x * x + y * y);

  float a1arg = (c1 * c1 + d * d - c2 * c2) / (2 * c1 * d);
  float a2arg = (c2 * c2 + d * d - c1 * c1) / (2 * c2 * d);
  float b1arg = (c1 * c1 + l1 * l1 - l2 * l2) / (2 * c1 * l1);
  float b2arg = (c2 * c2 + l1 * l1 - l2 * l2) / (2 * c2 * l1);

  a1arg = constrain(a1arg, -1.0f, 1.0f);
  a2arg = constrain(a2arg, -1.0f, 1.0f);
  b1arg = constrain(b1arg, -1.0f, 1.0f);
  b2arg = constrain(b2arg, -1.0f, 1.0f);

  float a1 = acosf(a1arg);
  float a2 = acosf(a2arg);
  float b1 = acosf(b1arg);
  float b2 = acosf(b2arg);

  float q1 = PI - a1 - b1;
  float q2 = a2 + b2;

  q1Deg = q1 * 180.0f / PI;
  q2Deg = q2 * 180.0f / PI;
}
