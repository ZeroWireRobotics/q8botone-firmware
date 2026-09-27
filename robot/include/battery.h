#pragma once
#include <Arduino.h>

// Coarse charge status, derived from the BQ25887's own status register -
// see batteryGetChargeState().
enum ChargeState : uint8_t {
  CHARGE_NONE   = 0,  // no valid charge source detected - running on battery
  CHARGE_ACTIVE = 1,  // source present, actively charging (trickle/pre/fast/taper/top-off)
  CHARGE_DONE   = 2,  // source present, termination done - full and maintaining
};

void batteryInit();
void batterySetCharging(bool enabled);
float batteryGetADC();
uint16_t batteryGetPercent();
ChargeState batteryGetChargeState();
