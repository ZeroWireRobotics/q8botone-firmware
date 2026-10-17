#pragma once
#include <Arduino.h>
#include <Dynamixel2Arduino.h>

// The robot's 8 DYNAMIXEL XL330-M077 servos, and the one-time setup each
// needs before it can drive a joint: an ID, 1 Mbps, and three settings kept
// in the servo's EEPROM (drive mode, operating mode, homing offset - see
// motorSetup.cpp). A servo straight from the factory is ID 1 at 57600 baud.
//
// The PCB labels the servo slots "joint 1" to "joint 8"; on the bus they
// are IDs 11 to 18, in the same order. Messages name both, so neither
// numbering gets mistaken for the other.

static const uint8_t MOTOR_COUNT = 8;
static const uint8_t MOTOR_FIRST_ID = 11;   // slot 1 = ID 11 ... slot 8 = ID 18

enum MotorStatus : uint8_t {
  MOTOR_OK,               // answers as its ID, settings as they should be
  MOTOR_MISSING,          // nothing answers as its ID at 1 Mbps
  MOTOR_WRONG_SETTINGS,   // answers, but an EEPROM setting differs (or can't be read)
};

struct MotorReport {
  MotorStatus status[MOTOR_COUNT];
  uint8_t found;            // slots whose servo answered at 1 Mbps
  uint8_t wrongSettings;    // of those, how many have a setting that differs
  bool factoryMotorPresent; // something answers as ID 1 at 57600 - a new servo
};

// Where the messages go, one line per call (main.cpp's printLineNow).
typedef void (*MotorPrintFn)(const char* line);

// Ask each slot's servo whether it's there, read and compare its EEPROM
// settings, and look for a factory-default servo; print a line per slot
// and a summary. Writes nothing to any servo. Call before anything else
// uses the bus (it changes the bus baud rate, and leaves it at 1 Mbps).
MotorReport motorSetupCheck(Dynamixel2Arduino& dxl, MotorPrintFn print);
