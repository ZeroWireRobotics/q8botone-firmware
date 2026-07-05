#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>

// Manages power latch logic for the Q8Bot PCB.
//
// Power-on:  MCU boots while the SPST button is held (SENSE_PIN HIGH).
//            begin() blocks for up to HOLD_MS, fading the LED in over that
//            window. If the button is still held at HOLD_MS, LATCH_PIN is
//            pulled HIGH to keep power on after release. Returns false (and
//            does not latch) if the button is released early.
//
// Power-off: startMonitorTask() spawns a FreeRTOS task that polls SENSE_PIN.
//            If held HIGH for HOLD_MS, the LED fades out and LATCH_PIN is
//            pulled LOW, cutting power. Releasing early aborts shutdown and
//            restores the LED to full brightness.

class PowerManager {
public:
  PowerManager(uint8_t latchPin, uint8_t sensePin, uint8_t ledPin);

  // Call early in setup(), before FreeRTOS tasks are created.
  // Blocks up to HOLD_MS. Returns true if power is latched, false if the
  // button was released before the hold threshold.
  bool begin();

  // Spawn the shutdown-monitor task. Call after xTaskCreate calls in setup().
  void startMonitorTask();

  // Cut power immediately (e.g. triggered by SW battery cutoff).
  void powerOff();

private:
  uint8_t _latchPin;
  uint8_t _sensePin;
  uint8_t _ledPin;

  static const uint32_t HOLD_MS    = 3000;
  static const uint32_t POLL_MS    = 20;

  static void _monitorTask(void* param);
};

#endif
