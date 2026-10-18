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
  MOTOR_UNRELIABLE,       // answers, but reads come back garbled or missing - two servos
                          // sharing the ID, or a loose cable
};

struct MotorReport {
  MotorStatus status[MOTOR_COUNT];
  uint8_t found;            // slots whose servo answered at 1 Mbps
  uint8_t wrongSettings;    // of those, how many have a setting that differs
  uint8_t unreliable;       // of those, how many answer unreliably
  bool factoryMotorPresent; // something answers as ID 1 at 57600 - a new servo
};

// Where the messages go, one line per call (main.cpp's printLineNow).
typedef void (*MotorPrintFn)(const char* line);

// Ask each slot's servo whether it's there, read and compare its EEPROM
// settings, and look for a factory-default servo; print a line per slot
// and a summary. Writes nothing to any servo. Call before anything else
// uses the bus (it changes the bus baud rate, and leaves it at 1 Mbps).
MotorReport motorSetupCheck(Dynamixel2Arduino& dxl, MotorPrintFn print);

// Correct the EEPROM settings of each slot that motorSetupCheck() reported
// as differing: writes only the items that are wrong, then reads them back.
// Updates report; returns how many slots are still wrong.
uint8_t motorSetupFix(Dynamixel2Arduino& dxl, MotorReport& report, MotorPrintFn print);

// Serial F, then Y: put every servo that answers as one of the robot's IDs
// (11-18, at 1 Mbps) back to its factory settings - ID 1, 57600 baud, every
// setting at its default. Returns how many were reset. If any were, it also
// marks the robot to stay stopped (motorSetupHalt) until it's switched fully
// off - the caller then restarts - so motor setup can't set a servo up again
// before there's been a chance to unplug it.
uint8_t motorFactoryResetAll(Dynamixel2Arduino& dxl, MotorPrintFn print);

// Whether a servo reset is waiting for a power cycle. The mark is kept in
// RTC memory, which keeps its contents through a restart (software, or the
// USB reset a firmware flash ends with) but comes up random from power-off.
bool motorSetupResetPending();

// Serial m: how reliably each servo ID answers, at 1 Mbps (IDs 11-18) and at
// 57600 (a new servo, ID 1) - pings and position reads, and whether the
// failures are garbled replies (two servos on one ID) or missing ones (a
// loose cable, a noisy bus). For diagnosing; changes nothing.
void motorSetupScan(Dynamixel2Arduino& dxl, MotorPrintFn print);

class RGBManager;
class PowerRail;

// The stopped state after a servo reset. Never returns: servo power off (so
// servos are safe to unplug), status light solid red, and instructions on
// the serial monitor every 30 s; 'i' still prints firmwareId. Only a power
// cycle - switched off, USB unplugged too - ends it.
[[noreturn]] void motorSetupHalt(RGBManager& rgb, uint8_t ledPin, PowerRail& servoPower,
                                 const char* firmwareId, MotorPrintFn print);

// Motor setup mode, for when motorSetupCheck() finds slots without their
// servo. For "plug a new servo into slot N" (the lowest empty slot) the
// status light blinks green N-1 times - the slots before it, all set up -
// then yellow once, then pauses; when a factory-default
// servo appears, sets it up as that slot's servo, checks it and moves on
// to the next empty slot - one more green blink. A servo that's already set up
// is just counted. Once all 8 answer, waits a moment and runs the full
// check again before returning, so boot carries on with all 8 confirmed.
//
// Two new servos plugged in at once would both take the same ID, so a new
// servo is only set up when exactly one answers; otherwise the light blinks
// nothing but red, fast and steady, and the serial monitor asks for all but
// one to be unplugged.
//
// Serial: 'q' skips setup and returns (boot carries on without the missing
// servos); 'i' prints firmwareId, so the desktop app can still tell this
// board is a robot; 'F' then 'Y' resets the servos to factory settings, as
// in normal running. Returns true if all 8 servos ended up set up.
//
// Blocks - call from setup(), before the tasks that use the bus exist.
bool motorSetupRun(Dynamixel2Arduino& dxl, RGBManager& rgb, uint8_t ledPin,
                   const char* firmwareId, MotorPrintFn print);
