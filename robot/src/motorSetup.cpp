#include "motorSetup.h"
#include "RGBManager.h"
#include "powerRail.h"

#include <esp_attr.h>

#include <stdarg.h>

using namespace ControlTableItem;

// See motorSetupResetPending(). Any value but the mark means "no reset
// pending" - which is what power-on's random contents almost surely are.
RTC_NOINIT_ATTR static uint32_t servoResetMark;
static const uint32_t SERVO_RESET_MARK = 0x5E70F0A5;

bool motorSetupResetPending() {
  return servoResetMark == SERVO_RESET_MARK;
}

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
  bool readItem(Dynamixel2Arduino& dxl, uint8_t item, uint8_t id, int32_t& out, uint32_t timeoutMs = 100) {
    out = dxl.readControlTableItem(item, id, timeoutMs);
    return dxl.getLastLibErrCode() == DXL_LIB_OK;
  }

  // A reply takes well under 3 ms even at 57600; the library's 100 ms default
  // would stall the setup loop (and its blinks) on a servo that's not there.
  const uint32_t QUICK_READ_MS = 20;

  // Switch the bus baud rate, and give the line a moment before talking.
  const uint32_t BAUD_SETTLE_MS = 5;
  void useBaud(Dynamixel2Arduino& dxl, uint32_t baud) {
    dxl.begin(baud);
    delay(BAUD_SETTLE_MS);
  }

  // Does `id` answer a ping? Asked a few times, so one lost reply doesn't
  // make a servo that's there look missing.
  bool answers(Dynamixel2Arduino& dxl, uint8_t id, uint8_t tries = 3) {
    for (uint8_t k = 0; k < tries; k++) {
      if (dxl.ping(id)) return true;
    }
    return false;
  }

  // How many of `reads` present-position reads from `id` fail. Two servos
  // sharing an ID reply on top of each other - their positions differ, so
  // the reply arrives garbled; a loose cable loses replies.
  uint8_t badReads(Dynamixel2Arduino& dxl, uint8_t id, uint8_t reads) {
    uint8_t bad = 0;
    int32_t position;
    for (uint8_t k = 0; k < reads; k++) {
      if (!readItem(dxl, PRESENT_POSITION, id, position, QUICK_READ_MS)) bad++;
    }
    return bad;
  }

  bool garbled(DXLLibErrorCode_t err) {
    return err == DXL_LIB_ERROR_CRC || err == DXL_LIB_ERROR_CHECK_SUM || err == DXL_LIB_ERROR_WRONG_PACKET;
  }

  // Setup mode timing.
  const uint32_t POLL_MS = 500;          // how often the bus is checked
  const uint32_t REMINDER_MS = 15000;    // repeat the "waiting for slot N" line
  const uint32_t BLINK_ON_MS = 165;
  const uint32_t BLINK_PERIOD_MS = 330;  // one blink: on, then off for the rest
  const uint32_t BLINK_GAP_MS = 1000;    // pause between counts
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
    char line[320];
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
    useBaud(dxl, ROBOT_BAUD);
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
    if (badReads(dxl, id, 5)) {
      say(print, "[MOTORS]   it answers unreliably as ID %u - is another servo using that ID?", id);
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
      useBaud(dxl, ROBOT_BAUD);
      say(print, "[MOTORS]   couldn't write its settings - trying again");
      return false;
    }
    delay(EEPROM_SETTLE_MS);
    return finishAt1M(dxl, i, print);
  }

  enum NewServos : uint8_t { NO_NEW_SERVO, ONE_NEW_SERVO, SEVERAL_NEW_SERVOS };

  // At 57600: is a factory-default servo plugged in, and only one? Two
  // answering as ID 1 can send identical replies to a ping, so this also
  // reads something that differs from servo to servo - present position -
  // a few times: replies on top of each other arrive garbled.
  NewServos findNewServos(Dynamixel2Arduino& dxl) {
    if (!dxl.ping(FACTORY_ID)) {
      const DXLLibErrorCode_t err = dxl.getLastLibErrCode();
      return garbled(err) ? SEVERAL_NEW_SERVOS : NO_NEW_SERVO;
    }
    for (uint8_t k = 0; k < 3; k++) {
      int32_t position;
      if (!readItem(dxl, PRESENT_POSITION, FACTORY_ID, position)) return SEVERAL_NEW_SERVOS;
    }
    return ONE_NEW_SERVO;
  }

  const uint32_t RESET_CONFIRM_MS = 5000;   // F, then Y within this

  // Nothing at all on the bus usually means no servo power: on USB alone
  // the board runs, but the servos only get power with the robot switched on.
  const char NO_POWER_HINT[] =
      "[MOTORS] No servo answers at all - is the robot switched on? On USB power alone the servos are off.";

  // The setup light runs on a task of its own, a step above the setup loop,
  // so the bus checks - a few hundred ms when servos are missing - can never
  // stretch or cut short a blink. The loop only says what to show.
  enum LightMode : uint8_t { LIGHT_COUNT, LIGHT_RED, LIGHT_GREEN };

  struct SetupLight {
    RGBManager* rgb;
    uint8_t pin;
    volatile LightMode mode;
    volatile uint8_t count;       // LIGHT_COUNT: waiting for slot `count`
    volatile uint32_t start;      // when that count pattern started
    volatile uint32_t redUntil;   // one red flash (a failed step) until then
    volatile bool quit;
    volatile bool stopped;
  };

  // Waiting for slot `count`: a green blink for each slot before it (all set
  // up - it's the lowest empty one), then a yellow blink for the slot it's
  // waiting on, a pause, repeat. 0 off, 1 green, 2 yellow.
  int8_t countState(uint8_t count, uint32_t elapsed) {
    const uint32_t cycle = count * BLINK_PERIOD_MS + BLINK_GAP_MS;
    const uint32_t t = elapsed % cycle;
    const bool lit = t < count * BLINK_PERIOD_MS && (t % BLINK_PERIOD_MS) < BLINK_ON_MS;
    return !lit ? 0 : (t / BLINK_PERIOD_MS == count - 1u ? 2 : 1);
  }

  // Something to fix first - more than one new servo plugged in, or a slot
  // that answers unreliably: nothing but fast, steady red blinks. Red always
  // means "fix this first", never mixed with the green-and-yellow count.
  // 0 off, 3 red.
  int8_t redState(uint32_t now) {
    return (now % BLINK_PERIOD_MS) < BLINK_ON_MS ? 3 : 0;
  }

  void setupLightTask(void* arg) {
    SetupLight& light = *static_cast<SetupLight*>(arg);
    int8_t shown = -1;
    while (!light.quit) {
      const uint32_t now = millis();
      int8_t state;
      if ((int32_t)(light.redUntil - now) > 0) {
        state = 3;
      } else if (light.mode == LIGHT_RED) {
        state = redState(now);
      } else if (light.mode == LIGHT_GREEN) {
        state = 1;
      } else {
        state = countState(light.count, now - light.start);
      }
      if (state != shown) {          // only touch the LEDs when it changes
        shown = state;
        if (state == 1) light.rgb->setColor(0, 255, 0);
        else if (state == 2) light.rgb->setColor(255, 160, 0);
        else if (state == 3) light.rgb->setColor(255, 0, 0);
        else light.rgb->off();
        digitalWrite(light.pin, state ? HIGH : LOW);   // the chip LED blinks along
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    light.rgb->off();
    digitalWrite(light.pin, LOW);
    light.stopped = true;
    vTaskDelete(nullptr);
  }

  void stopLight(SetupLight& light) {
    light.quit = true;
    while (!light.stopped) delay(5);
  }
}

MotorReport motorSetupCheck(Dynamixel2Arduino& dxl, MotorPrintFn print) {
  MotorReport report = {};
  char line[200];

  useBaud(dxl, ROBOT_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);
  print("[MOTORS] Checking the servo in each slot (slot 1-8 = internal ID 11-18)");

  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    const uint8_t id = MOTOR_FIRST_ID + i;
    int n = snprintf(line, sizeof(line), "[MOTORS]   slot %u (%s leg), internal ID %u: ",
                     i + 1, LEG[i], id);

    if (!answers(dxl, id)) {
      report.status[i] = MOTOR_MISSING;
      snprintf(line + n, sizeof(line) - n, "not found");
      print(line);
      continue;
    }
    report.found++;

    const uint8_t bad = badReads(dxl, id, 5);
    if (bad) {
      report.status[i] = MOTOR_UNRELIABLE;
      report.unreliable++;
      snprintf(line + n, sizeof(line) - n, "answers, but %u of 5 reads failed - two servos may share "
               "ID %u, or its cable is loose", bad, id);
      print(line);
      continue;
    }

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
      n += snprintf(line + n, sizeof(line) - n, "settings differ -");
      if (drive != WANT_DRIVE_MODE[i]) {
        n += snprintf(line + n, sizeof(line) - n, " drive mode %ld (should be %u)", (long)drive, WANT_DRIVE_MODE[i]);
      }
      if (op != WANT_OPERATING_MODE) {
        n += snprintf(line + n, sizeof(line) - n, " operating mode %ld (should be %u)", (long)op, WANT_OPERATING_MODE);
      }
      if (home != WANT_HOMING_OFFSET[i]) {
        snprintf(line + n, sizeof(line) - n, " homing offset %ld (should be %ld)", (long)home, (long)WANT_HOMING_OFFSET[i]);
      }
    }
    print(line);

    if (model != XL330_M077) {
      snprintf(line, sizeof(line), "[MOTORS]     note: model number %u - an XL330-M077 is %u",
               model, XL330_M077);
      print(line);
    }
  }

  // A new servo from the factory: ID 1 at 57600 baud.
  useBaud(dxl, FACTORY_BAUD);
  report.factoryMotorPresent = answers(dxl, FACTORY_ID);
  useBaud(dxl, ROBOT_BAUD);
  if (report.factoryMotorPresent) {
    print("[MOTORS] A new servo (factory settings: ID 1, 57600 baud) is plugged in");
  }

  if (report.found == MOTOR_COUNT && report.wrongSettings == 0 && report.unreliable == 0) {
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
      n2 += snprintf(line + n2, sizeof(line) - n2, "; %u with settings that differ", report.wrongSettings);
    }
    if (report.unreliable) {
      snprintf(line + n2, sizeof(line) - n2, "; %u answering unreliably", report.unreliable);
    }
    print(line);
    if (report.found == 0 && !report.factoryMotorPresent) print(NO_POWER_HINT);
  }
  return report;
}

uint8_t motorSetupFix(Dynamixel2Arduino& dxl, MotorReport& report, MotorPrintFn print) {
  useBaud(dxl, ROBOT_BAUD);
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    if (report.status[i] != MOTOR_WRONG_SETTINGS) continue;
    const uint8_t id = MOTOR_FIRST_ID + i;

    int32_t drive, op, home;
    if (!(readItem(dxl, DRIVE_MODE, id, drive) && readItem(dxl, OPERATING_MODE, id, op)
          && readItem(dxl, HOMING_OFFSET, id, home))) {
      say(print, "[MOTORS] Slot %u (%s leg), internal ID %u: its settings can't be read - left as they are",
          i + 1, LEG[i], id);
      continue;
    }

    // EEPROM items only take writes with torque off (as servos power up).
    bool ok = dxl.torqueOff(id);
    if (ok && drive != WANT_DRIVE_MODE[i]) {
      ok = dxl.writeControlTableItem(DRIVE_MODE, id, WANT_DRIVE_MODE[i]);
      say(print, "[MOTORS] Slot %u, internal ID %u: drive mode %ld -> %u", i + 1, id, (long)drive, WANT_DRIVE_MODE[i]);
    }
    if (ok && op != WANT_OPERATING_MODE) {
      ok = dxl.writeControlTableItem(OPERATING_MODE, id, WANT_OPERATING_MODE);
      say(print, "[MOTORS] Slot %u, internal ID %u: operating mode %ld -> %u", i + 1, id, (long)op, WANT_OPERATING_MODE);
    }
    if (ok && home != WANT_HOMING_OFFSET[i]) {
      ok = dxl.writeControlTableItem(HOMING_OFFSET, id, WANT_HOMING_OFFSET[i]);
      say(print, "[MOTORS] Slot %u, internal ID %u: homing offset %ld -> %ld", i + 1, id, (long)home, (long)WANT_HOMING_OFFSET[i]);
    }
    delay(EEPROM_SETTLE_MS);

    if (ok && settingsMatch(dxl, i)) {
      report.status[i] = MOTOR_OK;
      report.wrongSettings--;
      say(print, "[MOTORS] Slot %u (%s leg), internal ID %u: settings corrected", i + 1, LEG[i], id);
    } else {
      say(print, "[MOTORS] Slot %u (%s leg), internal ID %u: couldn't correct its settings", i + 1, LEG[i], id);
    }
  }
  return report.wrongSettings;
}

uint8_t motorFactoryResetAll(Dynamixel2Arduino& dxl, MotorPrintFn print) {
  const uint8_t RESET_EVERYTHING = 0xFF;   // Factory Reset option: ID and baud rate too
  useBaud(dxl, ROBOT_BAUD);
  say(print, "[MOTORS] Resetting the servos to factory settings (ID 1, 57600 baud)...");

  uint8_t reset = 0;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    const uint8_t id = MOTOR_FIRST_ID + i;
    if (!dxl.ping(id)) {
      say(print, "[MOTORS]   slot %u, internal ID %u: not found - skipped", i + 1, id);
      continue;
    }
    dxl.torqueOff(id);
    const bool acked = dxl.factoryReset(id, RESET_EVERYTHING, 100);
    delay(200);                                  // it restarts at its factory settings
    if (acked && !dxl.ping(id)) {
      reset++;
      say(print, "[MOTORS]   slot %u, internal ID %u: reset", i + 1, id);
    } else {
      say(print, "[MOTORS]   slot %u, internal ID %u: reset didn't take", i + 1, id);
    }
  }
  say(print, "[MOTORS] %u servo%s reset.", reset, reset == 1 ? "" : "s");
  if (reset) servoResetMark = SERVO_RESET_MARK;   // stay stopped until a power cycle
  return reset;
}

void motorSetupHalt(RGBManager& rgb, uint8_t ledPin, PowerRail& servoPower,
                    const char* firmwareId, MotorPrintFn print) {
  servoPower.set(false);
  rgb.setColor(255, 0, 0);
  digitalWrite(ledPin, HIGH);
  uint32_t lastSaid = 0;
  bool said = false;
  while (true) {
    if (!said || millis() - lastSaid >= 30000) {
      said = true;
      lastSaid = millis();
      say(print, "[MOTORS] Stopped: the servos were reset to factory settings. Servo power is off, "
          "so they're safe to unplug.");
      say(print, "[MOTORS] To set them up again: switch the robot fully off (unplug USB too), plug in "
          "only slot 1's servo, then switch on - the light asks for each slot in turn.");
    }
    while (Serial.available()) {
      if (Serial.read() == 'i') print(firmwareId);
    }
    delay(20);
  }
}

namespace {
  // One line of motorSetupScan(): pings and position reads for one ID.
  void scanId(Dynamixel2Arduino& dxl, uint8_t id, const char* what, MotorPrintFn print) {
    const uint8_t TRIES = 20;
    uint8_t pinged = 0, clean = 0, mangled = 0, lost = 0;
    for (uint8_t k = 0; k < TRIES; k++) {
      if (dxl.ping(id)) pinged++;
    }
    int32_t position;
    for (uint8_t k = 0; k < TRIES; k++) {
      if (readItem(dxl, PRESENT_POSITION, id, position, QUICK_READ_MS)) clean++;
      else if (garbled(dxl.getLastLibErrCode())) mangled++;
      else lost++;
    }
    const char* verdict =
        (pinged == 0 && clean == 0 && mangled == 0) ? "nothing answers" :
        (clean == TRIES && pinged == TRIES)          ? "ok" :
        mangled                                      ? "garbled replies - two servos may share this ID" :
                                                       "drops replies - a loose cable or a noisy bus";
    say(print, "[MOTORS]   %s, ID %u: pings %u/%u, reads %u ok / %u garbled / %u no reply - %s",
        what, id, pinged, TRIES, clean, mangled, lost, verdict);
  }
}

void motorSetupScan(Dynamixel2Arduino& dxl, MotorPrintFn print) {
  char what[40];
  say(print, "[MOTORS] Servo bus scan - 20 pings and 20 position reads per ID");
  useBaud(dxl, ROBOT_BAUD);
  say(print, "[MOTORS] At 1 Mbps (set-up servos):");
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    snprintf(what, sizeof(what), "slot %u (%s)", i + 1, LEG[i]);
    scanId(dxl, MOTOR_FIRST_ID + i, what, print);
  }
  useBaud(dxl, FACTORY_BAUD);
  say(print, "[MOTORS] At 57600 baud (new servos):");
  scanId(dxl, FACTORY_ID, "new servo", print);
  useBaud(dxl, ROBOT_BAUD);
}

bool motorSetupRun(Dynamixel2Arduino& dxl, RGBManager& rgb, uint8_t ledPin,
                   const char* firmwareId, MotorPrintFn print) {
  say(print, "[MOTORS] ---- Motor setup ----");
  say(print, "[MOTORS] The light blinks green once for each slot before the one it wants, then yellow:");
  say(print, "[MOTORS] green, green, yellow = plug a new servo into slot 3.");
  say(print, "[MOTORS] Switch the robot off before plugging a servo in, and add one new servo at a time.");
  say(print, "[MOTORS] Type q to skip setup and start anyway.");

  int8_t announced = -1;   // slot last announced as "next"
  uint8_t next = 0;
  uint32_t lastPoll = 0, lastReminder = millis();

  SetupLight light = {&rgb, ledPin, LIGHT_COUNT, 1, millis(), 0, false, false};
  xTaskCreate(setupLightTask, "SetupLight", 2048, &light, 2, nullptr);
  uint32_t resetAskedAt = 0;      // when F was typed, 0 if it wasn't
  bool warnedSeveral = false;     // "more than one new servo" said since the last change
  int8_t unreliableSaid = -1;     // slot last reported as answering unreliably
  bool trouble = false;           // something to fix first (see below) - the light shows red
  bool silent = false;            // nothing answered on the last look (see NO_POWER_HINT)

  while (true) {
    while (Serial.available()) {
      const char c = Serial.read();
      if (c == 'q') {
        stopLight(light);
        useBaud(dxl, ROBOT_BAUD);
        say(print, "[MOTORS] Setup skipped - starting without every servo");
        return false;
      }
      if (c == 'i') print(firmwareId);
      if (c == 'm') {
        motorSetupScan(dxl, print);
        lastPoll = millis() - POLL_MS;
      }
      if (c == 'F') {
        resetAskedAt = millis();
        say(print, "[MOTORS] Type Y within 5 s to reset the servos to factory settings - "
            "they'll need setting up again, one at a time");
      }
      if (c == 'Y' && resetAskedAt && millis() - resetAskedAt < RESET_CONFIRM_MS) {
        resetAskedAt = 0;
        if (motorFactoryResetAll(dxl, print)) {
          say(print, "[MOTORS] Restarting - the robot stays stopped until it's switched fully off");
          Serial.flush();
          delay(200);
          ESP.restart();
        }
        announced = -1;
        lastPoll = millis() - POLL_MS;
      }
    }

    const uint32_t now = millis();
    if (now - lastPoll >= POLL_MS) {
      lastPoll = now;

      // Which slots already have their servo - and does each answer cleanly?
      useBaud(dxl, ROBOT_BAUD);
      uint8_t found = 0;
      int8_t firstMissing = -1, firstUnreliable = -1;
      for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        const uint8_t id = MOTOR_FIRST_ID + i;
        if (answers(dxl, id)) {
          found++;
          if (firstUnreliable < 0 && badReads(dxl, id, 3) >= 2) firstUnreliable = i;
        } else if (firstMissing < 0) {
          firstMissing = i;
        }
      }

      // A slot whose replies mostly come back garbled or not at all: two
      // servos probably share its ID, or its cable is loose. Nothing more is
      // set up until that's sorted out.
      if (firstUnreliable >= 0) {
        if (unreliableSaid != firstUnreliable) {
          unreliableSaid = firstUnreliable;
          say(print, "[MOTORS] Slot %u (%s leg), internal ID %u, answers unreliably - two servos may share "
              "that ID, or its cable is loose. Nothing more gets set up until that's fixed (the light blinks red).",
              firstUnreliable + 1, LEG[firstUnreliable], MOTOR_FIRST_ID + firstUnreliable);
          say(print, "[MOTORS] Check its cable, or type F then Y to reset the servos and set them up again "
              "one at a time. Type m for a full servo bus scan.");
        }
        trouble = true;
        light.mode = LIGHT_RED;
        continue;
      }
      unreliableSaid = -1;

      if (found == MOTOR_COUNT) {
        trouble = false;
        light.mode = LIGHT_GREEN;
        say(print, "[MOTORS] All 8 servos answer - checking them once more");
        delay(COOLDOWN_MS);
        MotorReport report = motorSetupCheck(dxl, print);
        if (report.wrongSettings) motorSetupFix(dxl, report, print);
        if (report.found == MOTOR_COUNT && report.unreliable == 0) {
          stopLight(light);
          say(print, "[MOTORS] ---- Motor setup done ----");
          return true;
        }
        say(print, "[MOTORS] Not every servo answered the second time - back to setup");
        light.mode = LIGHT_COUNT;
        announced = -1;
        continue;
      }

      next = firstMissing;
      if (next != announced) {
        if (next == 0) {
          say(print, "[MOTORS] %u/8 servos set up. Next: plug a new servo into slot 1 (%s leg) "
              "- the light blinks yellow", found, LEG[0]);
        } else {
          say(print, "[MOTORS] %u/8 servos set up. Next: plug a new servo into slot %u (%s leg) "
              "- the light blinks green %u time%s, then yellow", found, next + 1, LEG[next], next, next > 1 ? "s" : "");
        }
        announced = next;
        lastReminder = now;
        warnedSeveral = false;
        light.count = next + 1;
        light.start = now;
        if (!trouble) light.mode = LIGHT_COUNT;
      }

      // A new servo (ID 1), or this slot's servo stuck at 57600 part-way through?
      useBaud(dxl, FACTORY_BAUD);
      bool done = false, tried = false;
      const NewServos newServos = findNewServos(dxl);
      if (newServos == SEVERAL_NEW_SERVOS) {
        useBaud(dxl, ROBOT_BAUD);
        if (!warnedSeveral) {
          warnedSeveral = true;
          say(print, "[MOTORS] More than one new servo is plugged in - they'd all get the same ID. "
              "Switch the robot off and leave only slot %u's new servo in (the light blinks red until then).",
              next + 1);
        }
        trouble = true;
        light.mode = LIGHT_RED;
        continue;
      }
      if (trouble) {            // nothing left to fix: back to the count, from its start
        trouble = false;
        light.count = next + 1;
        light.start = now;
        light.mode = LIGHT_COUNT;
      }
      if (newServos == ONE_NEW_SERVO) {
        warnedSeveral = false;
        // A reply lost on the bus mustn't make a set-up slot look empty, or
        // the new servo would take that slot's ID too. Make sure first.
        useBaud(dxl, ROBOT_BAUD);
        if (answers(dxl, MOTOR_FIRST_ID + next, 5)) {
          continue;             // it's there after all - look again
        }
        useBaud(dxl, FACTORY_BAUD);
        tried = true;
        done = setUpNewServo(dxl, next, print);
      } else if (dxl.ping(MOTOR_FIRST_ID + next)) {
        tried = true;
        say(print, "[MOTORS] Slot %u's servo is still at 57600 baud - finishing its setup", next + 1);
        done = finishAt1M(dxl, next, print);
      }
      useBaud(dxl, ROBOT_BAUD);
      if (tried) {
        // Success needs no flash of its own - the count gains a green blink.
        if (!done) light.redUntil = millis() + 800;
        lastPoll = millis() - POLL_MS;   // look again straight away
        continue;
      }
      // Nothing set up and no new servo either: say once why that might be.
      if (found == 0 && !silent) print(NO_POWER_HINT);
      silent = (found == 0);
    }

    if (now - lastReminder >= REMINDER_MS) {
      lastReminder = now;
      say(print, "[MOTORS] Waiting for a new servo in slot %u (%s leg)...", next + 1, LEG[next]);
      if (silent) print(NO_POWER_HINT);
    }
    delay(10);
  }
}
