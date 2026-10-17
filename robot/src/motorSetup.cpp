#include "motorSetup.h"

using namespace ControlTableItem;

namespace {
  const uint32_t ROBOT_BAUD = 1000000;
  const uint32_t FACTORY_BAUD = 57600;
  const uint8_t FACTORY_ID = 1;
  const float DXL_PROTOCOL = 2.0;     // (PROTOCOL_VERSION is a ControlTableItem name)

  // What each slot's servo must hold in EEPROM - the values
  // q8bot_motor_config has always written.
  // Drive mode: bit 2 = time-based profile (the firmware times its moves in
  // ms); bit 0 = reverse, on the right-side legs, whose servos are mirrored.
  const uint8_t WANT_DRIVE_MODE[MOTOR_COUNT] = {4, 4, 5, 5, 4, 4, 5, 5};
  const uint8_t WANT_OPERATING_MODE = 4;     // extended position
  const int32_t WANT_HOMING_OFFSET[MOTOR_COUNT] = {2048, 4096, -2048, -4096, 2048, 4096, -2048, -4096};

  const char* const LEG[MOTOR_COUNT] = {
    "front-left", "front-left", "front-right", "front-right",
    "back-left", "back-left", "back-right", "back-right",
  };

  // Reads one item; false if the servo didn't answer properly.
  bool readItem(Dynamixel2Arduino& dxl, uint8_t item, uint8_t id, int32_t& out) {
    out = dxl.readControlTableItem(item, id);
    return dxl.getLastLibErrCode() == DXL_LIB_OK;
  }
}

MotorReport motorSetupCheck(Dynamixel2Arduino& dxl, MotorPrintFn print) {
  MotorReport report = {};
  char line[200];

  dxl.begin(ROBOT_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);
  print("[MOTORS] Checking the servo in each slot (slot 1-8 = internal ID 11-18)");

  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    const uint8_t id = MOTOR_FIRST_ID + i;
    int n = snprintf(line, sizeof(line), "[MOTORS]   slot %u (%s leg), internal ID %u: ",
                     i + 1, LEG[i], id);

    if (!dxl.ping(id)) {
      report.status[i] = MOTOR_MISSING;
      snprintf(line + n, sizeof(line) - n, "not found");
      print(line);
      continue;
    }
    report.found++;

    int32_t drive, op, home;
    bool read = readItem(dxl, DRIVE_MODE, id, drive)
             && readItem(dxl, OPERATING_MODE, id, op)
             && readItem(dxl, HOMING_OFFSET, id, home);
    const uint16_t model = dxl.getModelNumber(id);

    if (!read) {
      report.status[i] = MOTOR_WRONG_SETTINGS;
      report.wrongSettings++;
      snprintf(line + n, sizeof(line) - n, "found, but its settings couldn't be read");
    } else if (drive == WANT_DRIVE_MODE[i] && op == WANT_OPERATING_MODE
               && home == WANT_HOMING_OFFSET[i]) {
      report.status[i] = MOTOR_OK;
      snprintf(line + n, sizeof(line) - n, "ok");
    } else {
      report.status[i] = MOTOR_WRONG_SETTINGS;
      report.wrongSettings++;
      snprintf(line + n, sizeof(line) - n,
               "settings differ - drive mode %ld (should be %u), operating mode %ld (should be %u), "
               "homing offset %ld (should be %ld)",
               (long)drive, WANT_DRIVE_MODE[i], (long)op, WANT_OPERATING_MODE,
               (long)home, (long)WANT_HOMING_OFFSET[i]);
    }
    print(line);

    if (model != XL330_M077) {
      snprintf(line, sizeof(line), "[MOTORS]     note: model number %u - an XL330-M077 is %u",
               model, XL330_M077);
      print(line);
    }
  }

  // A new servo from the factory: ID 1 at 57600 baud.
  dxl.begin(FACTORY_BAUD);
  report.factoryMotorPresent = dxl.ping(FACTORY_ID);
  dxl.begin(ROBOT_BAUD);
  if (report.factoryMotorPresent) {
    print("[MOTORS] A new servo (factory settings: ID 1, 57600 baud) is plugged in");
  }

  if (report.found == MOTOR_COUNT && report.wrongSettings == 0) {
    print("[MOTORS] 8/8 servos found, all set up");
  } else {
    int n2 = snprintf(line, sizeof(line), "[MOTORS] %u/8 servos found", report.found);
    if (report.found < MOTOR_COUNT) {
      n2 += snprintf(line + n2, sizeof(line) - n2, " - missing slot");
      for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (report.status[i] == MOTOR_MISSING) {
          n2 += snprintf(line + n2, sizeof(line) - n2, " %u", i + 1);
        }
      }
    }
    if (report.wrongSettings) {
      snprintf(line + n2, sizeof(line) - n2, "; %u with settings that differ", report.wrongSettings);
    }
    print(line);
  }
  return report;
}
