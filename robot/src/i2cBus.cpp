#include "i2cBus.h"
#include <Wire.h>

// NULL until i2cBusBegin() - before that only setup() runs, so there's no
// one to lock against.
static SemaphoreHandle_t busMutex = NULL;

void i2cBusBegin() {
  Wire.begin();
  // Fast mode: every device on the bus supports 400 kHz (BQ25887, INA237,
  // MPU6050), and a sensor sample holds the bus about a quarter as long as
  // at the 100 kHz default.
  Wire.setClock(400000);
  busMutex = xSemaphoreCreateRecursiveMutex();
}

I2CBusLock::I2CBusLock() {
  if (busMutex) xSemaphoreTakeRecursive(busMutex, portMAX_DELAY);
}

I2CBusLock::~I2CBusLock() {
  if (busMutex) xSemaphoreGiveRecursive(busMutex);
}
