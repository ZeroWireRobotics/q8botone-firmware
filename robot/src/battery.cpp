#include "battery.h"
#include <BQ25887.h>

static BQ25887 charger;

// 2S LiPo voltage-to-percent lookup table (30 points, ~0.1V steps).
// Upper portion (8.20–6.60V) approximates a typical discharge curve.
// Lower anchors are derived from discharge testing:
//   20% = 6.20V (low battery warning)
//   10% = 5.80V (disable jumping / reduce performance)
//    5% = 5.55V (stop locomotion, idle only)
//    0% = 5.40V (SW cutoff; HW UVP triggers at 4.8V)
namespace {
  constexpr int kTableSize = 30;

  const float kVoltages[kTableSize] = {
    8.20, 8.10, 8.00, 7.90, 7.80, 7.70, 7.60, 7.50, 7.40, 7.30,
    7.20, 7.10, 7.00, 6.90, 6.80, 6.70, 6.60, 6.50, 6.40, 6.30,
    6.20, 6.10, 6.00, 5.90, 5.80, 5.70, 5.60, 5.55, 5.45, 5.40
  };

  const float kPercentFractions[kTableSize] = {
    1.00, 0.95, 0.90, 0.85, 0.78, 0.73, 0.68, 0.63, 0.57, 0.52,
    0.47, 0.42, 0.37, 0.34, 0.30, 0.27, 0.25, 0.23, 0.22, 0.21,
    0.20, 0.17, 0.14, 0.12, 0.10, 0.08, 0.06, 0.05, 0.02, 0.00
  };
}

static void readADCRegs() {
  charger.readADCIbusReg();
  charger.readADCIchgReg();
  charger.readADCVbusReg();
  charger.readADCVbatReg();
  charger.readADCVCellTopReg();
  charger.readADCTsReg();
  charger.readADCTDieReg();
  charger.readADCVCellBotReg();
}

void batteryInit() {
  charger.begin();
  charger.setWATCHDOG(0);              // Disable watchdog
  charger.wdReset();                   // Clear latch
  charger.setADC_EN(true);            // Keep ADC running continuously
  charger.setChargeCurrentLimit(1.0); // CC mode: 1A charge current
  charger.setEN_CHG(1);               // Enable charging
}

void batterySetCharging(bool enabled) {
  charger.setEN_CHG(enabled);
}

float batteryGetADC() {
  readADCRegs();
  return charger.getADC_VBAT();  // volts
}

uint16_t batteryGetPercent() {
  float vbat = batteryGetADC();
  float percent = 0.0f;

  if (vbat >= kVoltages[0]) {
    percent = 100.0f;
  } else if (vbat <= kVoltages[kTableSize - 1]) {
    percent = 0.0f;
  } else {
    for (int i = 0; i < kTableSize - 1; ++i) {
      if (vbat <= kVoltages[i] && vbat >= kVoltages[i + 1]) {
        float frac = kPercentFractions[i + 1]
          + (kPercentFractions[i] - kPercentFractions[i + 1])
          * (vbat - kVoltages[i + 1]) / (kVoltages[i] - kVoltages[i + 1]);
        percent = frac * 100.0f;
        break;
      }
    }
  }

  if (percent < 0.0f)   percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;
  return static_cast<uint16_t>(percent + 0.5f);
}
