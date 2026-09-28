#include "powerRail.h"

PowerRail::PowerRail(uint8_t pin, uint8_t onLevel)
  : _pin(pin), _onLevel(onLevel) {}

void PowerRail::begin(bool on) {
  set(on);                 // digitalWrite latches the level even on a non-output pin...
  pinMode(_pin, OUTPUT);   // ...so the pin comes up already driving it
}

void PowerRail::set(bool on) {
  digitalWrite(_pin, on ? _onLevel : !_onLevel);
  _on = on;
}
