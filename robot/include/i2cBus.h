#pragma once
#include <Arduino.h>

// The robot's one I2C bus (SDA IO6, SCL IO7): the BQ25887 charger and
// whatever is plugged into the Qwiic connector share it. Any code that
// talks on it holds an I2CBusLock for the whole transaction, since several
// FreeRTOS tasks use it.

// Wire.begin() plus the lock. Call once in setup(), before any I2C use.
void i2cBusBegin();

// Held for its scope. Recursive, so a locked function can call another.
class I2CBusLock {
public:
  I2CBusLock();
  ~I2CBusLock();
  I2CBusLock(const I2CBusLock&) = delete;
  I2CBusLock& operator=(const I2CBusLock&) = delete;
};
