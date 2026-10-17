#include "motorSetup.h"
#include "RGBManager.h"

#include <stdarg.h>

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

  // Setup mode timing.
  const uint32_t POLL_MS = 500;          // how often the bus is checked
  const uint32_t REMINDER_MS = 15000;    // repeat the "waiting for slot N" line
  const uint32_t BLINK_ON_MS = 250;
  const uint32_t BLINK_PERIOD_MS = 500;  // one blink: on, then off for the rest
  const uint32_t BLINK_GAP_MS = 1500;    // pause between counts
  const uint32_t COOLDOWN_MS = 2000;     // all 8 in: settle, then check once more
  const uint8_t BAUD_1M_CODE = 3;        // BAUD_RATE item value for 1 Mbps
  const uint32_t EEPROM_SETTLE_MS = 50;  // after the ID and baud writes, as the old tool did

  // Do slot i's servo's EEPROM settings read back as they should?
  bool settingsMatch(Dynamixel2Arduino& dxl, uint8_t i) {
    const uint8_t id = MOTOR_FIRST_ID + i;
    int32_t drive, op, home;
    return readItem(dxl, DRIVE_MODE, id, drive) && readItem(dxl, OPERATING_MODE, id, op)
        && readItem(dxl, HOMING_OFFSET, id, home)
        && drive == WANT_DRIVE_MODE[i] && op == WANT_OPERATING_MODE && home == WANT_HOMING_OFFSET[i];
  }

  void say(MotorPrintFn print, const char* fmt, ...) {
    char line[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    print(line);
  }

  // Slot i's servo is at 57600 as its own ID (a new servo part-way through
  // setup, or one whose last step failed): move it to 1 Mbps and confirm it
  // there. Leaves the bus at 1 Mbps.
  bool finishAt1M(Dynamixel2Arduino& dxl, uint8_t i, MotorPrintFn print) {
    const uint8_t id = MOTOR_FIRST_ID + i;
    bool written = dxl.writeControlTableItem(BAUD_RATE, id, BAUD_1M_CODE);
    delay(EEPROM_SETTLE_MS);
    dxl.begin(ROBOT_BAUD);
    if (!written) {
      say(print, "[MOTORS]   couldn't set its baud rate - trying again");
      return false;
    }
    if (!dxl.ping(id)) {
      say(print, "[MOTORS]   it doesn't answer at 1 Mbps as ID %u - trying again", id);
      return false;
    }
    if (!settingsMatch(dxl, i)) {
      say(print, "[MOTORS]   its settings don't read back right - trying again");
      return false;
    }
    say(print, "[MOTORS] Slot %u (%s leg) is set up: internal ID %u, 1 Mbps", i + 1, LEG[i], id);
    return true;
  }

  // Turn the factory-default servo (ID 1 at 57600 - the bus must be there)
  // into slot i's servo: its settings first, then its ID, then its baud
  // rate, so it only leaves 57600 once everything else is in place.
  bool setUpNewServo(Dynamixel2Arduino& dxl, uint8_t i, MotorPrintFn print) {
    const uint8_t id = MOTOR_FIRST_ID + i;
    say(print, "[MOTORS] New servo found - setting it up for slot %u (%s leg), internal ID %u",
        i + 1, LEG[i], id);
    // EEPROM items only take writes with torque off (a new servo's default).
    bool ok = dxl.torqueOff(FACTORY_ID)
           && dxl.writeControlTableItem(DRIVE_MODE, FACTORY_ID, WANT_DRIVE_MODE[i])
           && dxl.writeControlTableItem(OPERATING_MODE, FACTORY_ID, WANT_OPERATING_MODE)
           && dxl.writeControlTableItem(HOMING_OFFSET, FACTORY_ID, WANT_HOMING_OFFSET[i])
           && dxl.writeControlTableItem(ID, FACTORY_ID, id);
    if (!ok) {
      dxl.begin(ROBOT_BAUD);
      say(print, "[MOTORS]   couldn't write its settings - trying again");
      return false;
    }
    delay(EEPROM_SETTLE_MS);
    return finishAt1M(dxl, i, print);
  }

  // Status light: `count` yellow blinks, a pause, repeat. The chip LED
  // blinks along. Only touches the LEDs when the state changes.
  void showBlinks(RGBManager& rgb, uint8_t ledPin, uint8_t count, uint32_t elapsed, int8_t& shown) {
    const uint32_t cycle = count * BLINK_PERIOD_MS + BLINK_GAP_MS;
    const uint32_t t = elapsed % cycle;
    const int8_t on = (t < count * BLINK_PERIOD_MS && (t % BLINK_PERIOD_MS) < BLINK_ON_MS) ? 1 : 0;
    if (on == shown) return;
    shown = on;
    if (on) rgb.setColor(255, 160, 0); else rgb.off();
    digitalWrite(ledPin, on ? HIGH : LOW);
  }

  void flash(RGBManager& rgb, uint8_t ledPin, uint8_t r, uint8_t g, uint8_t b, uint32_t ms) {
    rgb.setColor(r, g, b);
    digitalWrite(ledPin, HIGH);
    delay(ms);
    rgb.off();
    digitalWrite(ledPin, LOW);
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

bool motorSetupRun(Dynamixel2Arduino& dxl, RGBManager& rgb, uint8_t ledPin,
                   const char* firmwareId, MotorPrintFn print) {
  say(print, "[MOTORS] ---- Motor setup ----");
  say(print, "[MOTORS] The light blinks yellow N times for: plug a new servo into slot N.");
  say(print, "[MOTORS] Switch the robot off before plugging a servo in, and add one new servo at a time.");
  say(print, "[MOTORS] Type q to skip setup and start anyway.");

  int8_t announced = -1;   // slot last announced as "next"
  int8_t shown = -1;       // LED state, for showBlinks()
  uint8_t next = 0;
  uint32_t patternStart = millis(), lastPoll = 0, lastReminder = millis();

  while (true) {
    while (Serial.available()) {
      const char c = Serial.read();
      if (c == 'q') {
        rgb.off();
        digitalWrite(ledPin, LOW);
        dxl.begin(ROBOT_BAUD);
        say(print, "[MOTORS] Setup skipped - starting without every servo");
        return false;
      }
      if (c == 'i') print(firmwareId);
    }

    const uint32_t now = millis();
    if (now - lastPoll >= POLL_MS) {
      lastPoll = now;

      // Which slots already have their servo?
      dxl.begin(ROBOT_BAUD);
      uint8_t found = 0;
      int8_t firstMissing = -1;
      for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (dxl.ping(MOTOR_FIRST_ID + i)) {
          found++;
        } else if (firstMissing < 0) {
          firstMissing = i;
        }
      }

      if (found == MOTOR_COUNT) {
        rgb.setColor(0, 255, 0);
        digitalWrite(ledPin, HIGH);
        say(print, "[MOTORS] All 8 servos answer - checking them once more");
        delay(COOLDOWN_MS);
        MotorReport report = motorSetupCheck(dxl, print);
        rgb.off();
        digitalWrite(ledPin, LOW);
        if (report.found == MOTOR_COUNT) {
          say(print, "[MOTORS] ---- Motor setup done ----");
          return true;
        }
        say(print, "[MOTORS] Not every servo answered the second time - back to setup");
        announced = -1;
        continue;
      }

      next = firstMissing;
      if (next != announced) {
        say(print, "[MOTORS] %u/8 servos set up. Next: plug a new servo into slot %u (%s leg) "
            "- the light blinks %u time%s", found, next + 1, LEG[next], next + 1, next ? "s" : "");
        announced = next;
        patternStart = now;
        lastReminder = now;
        shown = -1;
      }

      // A new servo (ID 1), or this slot's servo stuck at 57600 part-way through?
      dxl.begin(FACTORY_BAUD);
      bool done = false, tried = false;
      if (dxl.ping(FACTORY_ID)) {
        tried = true;
        done = setUpNewServo(dxl, next, print);
      } else if (dxl.ping(MOTOR_FIRST_ID + next)) {
        tried = true;
        say(print, "[MOTORS] Slot %u's servo is still at 57600 baud - finishing its setup", next + 1);
        done = finishAt1M(dxl, next, print);
      }
      dxl.begin(ROBOT_BAUD);
      if (tried) {
        if (done) flash(rgb, ledPin, 0, 255, 0, 800); else flash(rgb, ledPin, 255, 0, 0, 800);
        shown = -1;
        lastPoll = millis() - POLL_MS;   // look again straight away
        continue;
      }
    }

    if (now - lastReminder >= REMINDER_MS) {
      lastReminder = now;
      say(print, "[MOTORS] Waiting for a new servo in slot %u (%s leg)...", next + 1, LEG[next]);
    }
    showBlinks(rgb, ledPin, next + 1, now - patternStart, shown);
    delay(10);
  }
}
