#ifndef POWER_RAIL_H
#define POWER_RAIL_H

#include <Arduino.h>

// One of the PCB's switchable buck converters, turned on and off by a
// single MCU pin with its own polarity (see pinMapping.h: the 5V buck is on
// while IO10 is HIGH, the 6V buck while IO1 is LOW). The ESP32 has its own
// supply, so either rail can be off while it keeps running - which is what
// the low-power modes in main.cpp rely on.
class PowerRail {
public:
  PowerRail(uint8_t pin, uint8_t onLevel);

  // Take the pin and drive the rail to `on`. The level is set before the
  // pin becomes an output, so it never passes through the other state.
  void begin(bool on);

  void set(bool on);
  bool isOn() const { return _on; }

private:
  uint8_t _pin;
  uint8_t _onLevel;
  volatile bool _on = false;
};

#endif
