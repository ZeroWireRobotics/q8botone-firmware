#pragma once
#include <Arduino.h>

// Sensors on the Qwiic connector, which shares the I2C bus with the charger
// (see i2cBus.h). They're found once, at boot, so plug them in before
// powering on. Supported kinds and their value layouts are listed in
// protocol.h's SensorKind.

// Where sensorsBegin() / sensorsPrint() send their text, one line per call.
typedef void (*SensorPrintFn)(const char* line);

// Scan the bus (printing every address that answers), then look for each
// supported kind at its possible addresses.
void sensorsBegin(SensorPrintFn print);

// Sample every detected sensor into a SENSOR_REPLY payload (layout in
// protocol.h) and return its length. Sensors that would push it past
// maxLen are left out.
size_t sensorsBuildReply(uint8_t* out, size_t maxLen);

// Serial 's': the bus scan, each sensor with a fresh labeled reading, and
// the SENSOR_REPLY payload bytes that reading produces.
void sensorsPrint(SensorPrintFn print);
