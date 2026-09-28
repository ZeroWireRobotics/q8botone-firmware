#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Dynamixel2Arduino.h>
// Q8bot-specific Modules
#include "q8Dynamixel.h"
#include "userParams.h"
#include "systemParams.h"
#include "pinMapping.h"
#include "macStorage.h"
#include "powerRail.h"
#include "RGBManager.h"
#include "battery.h"
#include "protocol.h"
#include "gaitRunner.h"

// Initialize global objects
esp_now_peer_info_t peerInfo;
HardwareSerial          ser(0);
Dynamixel2Arduino       q8dxl(ser, DXL_DIR_PIN);
q8Dynamixel             q8(q8dxl);
bool started = false;  // Track robot start state
macStorage storage;
PowerRail               rail5V(BUCK_5V_EN_PIN, BUCK_5V_ON);   // peripherals, e.g. the RGB LED
PowerRail               rail6V(BUCK_6V_EN_PIN, BUCK_6V_ON);   // servos
RGBManager              rgb(RGB_PIN);
GaitRunner              gaitRunner(q8);

// FreeRTOS Handles
QueueHandle_t rxQueue = NULL;
QueueHandle_t debugQueue = NULL;
EventGroupHandle_t eventGroup = NULL;

// Robot State
volatile RobotState robotState = STATE_UNPAIRED;

// Physical pairing button (IO9, active-low, new PCB revision) - long-press
// forces a re-pair, mirroring the controller firmware's button. Unlike a fresh/
// never-paired boot (which accepts a PAIRING request forever - see
// pairingAcceptingRequests's default below), a manual re-pair is bounded to
// PAIRING_TIMEOUT_MS: if nothing pairs within the window, the robot stops
// accepting new PAIRING requests (until the next long-press or a power
// cycle) so a stray broadcast reaching it afterward can't silently claim
// it. See robotStateTask()'s STATE_UNPAIRED case.
const uint32_t BUTTON_PAIRING_HOLD_MS = 3000;
const uint32_t BUTTON_DEBOUNCE_MS = 30;
const uint32_t PAIRING_TIMEOUT_MS = 30000;
bool manualPairingEntry = false;
volatile bool pairingAcceptingRequests = true;

// Default RGB LED brightness (0-255) used only until a host sends
// LED_BRIGHTNESS at least once (see q8Dynamixel.cpp's parseFrame) - 50%,
// dimmer than the controller's own status LED default since this one sits
// right in front of the robot rather than in a handheld controller.
const uint8_t DEFAULT_LED_BRIGHTNESS = 128;

// Joint telemetry (see protocol.h's TelemetryPayload). espnowRxTask only
// raises the flag; gaitRunnerTask does the reading, so the servo bus keeps a
// single owner and the read lands right after the gait write where it uses
// slack rather than delaying a frame.
volatile bool telemetryRequested = false;
uint32_t lastTelemetryRequestMs = 0;

// Temperature is cached rather than read per sample: it sits outside the
// contiguous 126-135 block the main read covers, and moves over seconds. The
// robot refreshes it on this period and every reply carries the last value,
// so the host can ask as fast as it likes without touching a servo for it.
const uint32_t TELEMETRY_TEMP_PERIOD_MS = 1000;
// Only kept fresh while a host is actually asking - standalone mode should
// spend exactly zero bus time on telemetry.
const uint32_t TELEMETRY_IDLE_TIMEOUT_MS = 3000;

// Which firmware this is. The desktop app's flasher reads it two ways: sent
// back as a line when the host writes 'i' over USB serial (what the plugged-in
// device *is*), and found as a literal inside the .bin (what a file *would
// make* it) - so it can refuse to put robot firmware on the controller or
// vice versa. The version after the second '|' is stamped in at build time
// from `git describe` (see scripts/fw_version.py). Must match
// q8botone-python/q8bot/firmware_flasher.py.
#ifndef Q8BOT_FW_VERSION
#define Q8BOT_FW_VERSION "unknown"
#endif
static const char FIRMWARE_ID[] = "Q8BOT_FW|robot|" Q8BOT_FW_VERSION;


// ============================================================================
// Helper Functions
// ============================================================================
void queuePrint(SerialMsgType type, const char* format, ...) {
  SerialMessage msg;
  msg.type = type;
  va_list args;
  va_start(args, format);
  vsnprintf(msg.text, sizeof(msg.text), format, args);
  va_end(args);
  xQueueSend(debugQueue, &msg, 0);  // Non-blocking
}

bool addPeer(const uint8_t* mac) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = chan;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// Serial '5' / '6' (see serialOutputTask): flip a buck by hand, for hardware
// checks. Nothing else is told - switching the 6V rail off under torque
// drops the robot, and its servos come back up with torque off and their
// RAM settings reset.
void toggleRail(PowerRail& rail, const char* name) {
  rail.set(!rail.isOn());
  queuePrint(MSG_INFO, "[POWER] %s buck %s\n", name, rail.isOn() ? "ON" : "OFF");
}

void unpair() {
  queuePrint(MSG_INFO, "[HEARTBEAT] Connection lost - returning to pairing mode\n");

  esp_now_del_peer(clientMac);
  storage.clearPeerMAC();
  queuePrint(MSG_DEBUG, "[STORAGE] Cleared controller MAC from EEPROM\n");
  memset(clientMac, 0, sizeof(clientMac));
  paired = false;
  started = false;
  pairingAcceptingRequests = true;  // Reopen acceptance every time we unpair

  // Update robot state and event group
  robotState = STATE_UNPAIRED;
  xEventGroupClearBits(eventGroup, EVENT_PAIRED | EVENT_STARTED);
  xEventGroupSetBits(eventGroup, EVENT_UNPAIRED);
}

// Manual pairing-mode entry (button long-press) - unlike the pre-existing
// 'p' serial command, this bounds the resulting acceptance window to
// PAIRING_TIMEOUT_MS (see robotStateTask()'s STATE_UNPAIRED case).
void enterPairingModeManually() {
  manualPairingEntry = true;
  q8.toggleTorque(0);        // Disable torque hardware
  q8.resetTorqueState();     // Sync internal flag to match disabled state
  unpair();
}


// ============================================================================
// Boot-time battery check (setup() only) - gives an immediate visual read of
// charge level via one slow breathe, before robotStateTask's RGB (pairing
// rainbow, etc.) takes over. Thresholds are independent of batteryStage's
// (robotStateTask's runtime 20/10/5 bands, used for jump-gating and the
// STATE_STARTED breathe/pulse) - this is a coarser one-shot readout, not a
// standing operational gate.
// ============================================================================
enum BootBatteryBand {
  BOOT_BATTERY_GREEN,     // 60-100%
  BOOT_BATTERY_YELLOW,    // 30-60%
  BOOT_BATTERY_ORANGE,    // 10-30%
  BOOT_BATTERY_RED,       // 5-10% - still boots normally
  BOOT_BATTERY_CRITICAL,  // <5%   - too low to boot; see powerSavingTask()
};

BootBatteryBand classifyBootBattery(uint16_t pct) {
  if (pct >= 60) return BOOT_BATTERY_GREEN;
  if (pct >= 30) return BOOT_BATTERY_YELLOW;
  if (pct >= 10) return BOOT_BATTERY_ORANGE;
  if (pct >= 5)  return BOOT_BATTERY_RED;
  return BOOT_BATTERY_CRITICAL;
}

// One slow 0->1->0 breathe (not a looping pattern - setup() moves on once
// this returns), blocking since nothing else is running yet this early in
// setup().
void breatheOnce(uint8_t r, uint8_t g, uint8_t b, uint32_t durationMs) {
  uint32_t start = millis();
  while (true) {
    uint32_t elapsed = millis() - start;
    if (elapsed >= durationMs) break;
    float level = sinf(PI * (float)elapsed / (float)durationMs);  // single hump
    rgb.setColor((uint8_t)(r * level), (uint8_t)(g * level), (uint8_t)(b * level));
    delay(10);
  }
  rgb.off();
}

// <5% at boot: warn, then hand off to powerSavingTask() instead of booting
// normally.
void blinkCriticalWarning() {
  for (int i = 0; i < 3; i++) {
    rgb.setColor(255, 0, 0);
    delay(150);
    rgb.off();
    delay(150);
  }
}



// ============================================================================
// ESPNOW Callbacks: ISR-Like, Highest Priority
// ============================================================================
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  // Validate length to prevent buffer overrun
  if (len < 1 || len > 250) return;

  // Copy message to queue for processing in espnowRxTask
  ESPNowMessage msg;
  memcpy(msg.mac, mac, 6);
  memcpy(msg.data, data, len);
  msg.len = len;
  msg.timestamp = millis();

  // Non-blocking send - drop message if queue is full
  (void)xQueueSend(rxQueue, &msg, 0);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}


// ============================================================================
// FreeRTOS Tasks (Ranked by Priority)
// ============================================================================
// FreeRTOS Task: ESP-NOW RX Handler (Priority 3 - HIGHEST)
void espnowRxTask(void* parameter) {
  ESPNowMessage msg;

  while (true) {
    // Wait for messages from onRecv() callback (blocking)
    if (xQueueReceive(rxQueue, &msg, portMAX_DELAY) == pdTRUE) {
      uint8_t msgType = msg.data[0];

      // Handle PAIRING message
      if (msgType == PAIRING && !paired && pairingAcceptingRequests) {
        // Validate PAIRING message length
        if (msg.len < sizeof(PairingMessage)) continue;

        memcpy(&pairingData, msg.data, sizeof(pairingData));

        // Ignore pairing requests from other robots
        if (pairingData.role != ROLE_CONTROLLER) continue;

        queuePrint(MSG_DEBUG, "[PAIRING] Pairing request from: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   msg.mac[0], msg.mac[1], msg.mac[2], msg.mac[3], msg.mac[4], msg.mac[5]);

        memcpy(clientMac, msg.mac, 6);
        WiFi.softAPmacAddress(pairingData.macAddr);  // Overwrite with our own MAC
        pairingData.channel = chan;
        pairingData.id = 0;   // Server is ID 0
        pairingData.role = ROLE_ROBOT;
        addPeer(clientMac);
        esp_now_send(clientMac, (uint8_t*)&pairingData, sizeof(pairingData));
        paired = true;
        manualPairingEntry = false;  // Successful pair closes any bounded window
        lastHeartbeatReceived = millis();

        // Update robot state and event group
        robotState = STATE_PAIRED;
        xEventGroupClearBits(eventGroup, EVENT_UNPAIRED);
        xEventGroupSetBits(eventGroup, EVENT_PAIRED);

        // Save the controller MAC address to EEPROM
        storage.savePeerMAC(clientMac);
        queuePrint(MSG_DEBUG, "[STORAGE] Saved controller MAC to EEPROM\n");
        queuePrint(MSG_INFO, "[PAIRING] Paired successfully\n");
      }
      // Handle HEARTBEAT message
      else if (msgType == HEARTBEAT && paired) {
        // Validate HEARTBEAT message length
        if (msg.len < sizeof(HeartbeatMessage)) continue;

        lastHeartbeatReceived = millis();
        queuePrint(MSG_DEBUG, "[HEARTBEAT] Received, echoing back\n");

        // Echo heartbeat back to controller
        esp_now_send(msg.mac, msg.data, msg.len);
      }
      // Handle DATA message
      else if (msgType == DATA && paired) {
        // Validate DATA message length
        if (msg.len < sizeof(CharMessage)) continue;

        lastHeartbeatReceived = millis();  // Any DATA also counts as "alive"
        lastDataReceived = millis();
        memcpy(&theirMsg, msg.data, sizeof(theirMsg));
        uint8_t result = q8.parseFrame((uint8_t*)theirMsg.data, sizeof(theirMsg.data));

        // myMsg params
        myMsg.id = 0;  // Server ID

        if (result == 0) {
          // No special instruction
          continue;
        } else {
          // Controller requests battery level or data
          switch (result) {
            case 4: {
              // Joint telemetry - sampled and answered by gaitRunnerTask.
              telemetryRequested = true;
              lastTelemetryRequestMs = millis();
              break;
            }

            case 1: {
              // Send battery level
              queuePrint(MSG_DEBUG, "[DATA] Send battery level\n");
              // Percent is derived from this same VBAT reading (see
              // battery.cpp's lookup table), so both halves of the reply come
              // from one ADC sample rather than two that could disagree.
              uint8_t payload[3];
              payload[0] = (uint8_t)batteryGetPercent();
              uint16_t millivolts = (uint16_t)lroundf(batteryGetADC() * 1000.0f);
              memcpy(payload + 1, &millivolts, sizeof(millivolts));
              buildFrame((uint8_t)ReplyType::BATTERY_REPLY, payload, sizeof(payload),
                         (uint8_t*)myMsg.data);
              esp_now_send(clientMac, (uint8_t*)&myMsg, sizeof(myMsg));
              break;
            }

          }
        }
      }
    }
  }
}

// FreeRTOS Task: Robot State Manager (Priority 1)
void robotStateTask(void* parameter) {
  TickType_t lastStateChange = xTaskGetTickCount();
  unsigned long lastActivity = 0;
  bool firstRun = true;  // Check saved MAC on first iteration
  uint16_t hue = 0;      // Tracks rainbow position for STATE_UNPAIRED
  uint32_t manualPairingStartMs = 0;  // 0 = no bounded window currently running

  // Pairing button (IO9) debounce state
  bool buttonRaw = false;
  bool buttonDebounced = false;
  uint32_t lastButtonChangeMs = 0;
  uint32_t buttonPressStartMs = 0;
  bool longPressLatched = false;  // Cleared on release so each hold only fires once

  // Battery monitoring state
  // Stages: 3 = normal, 2 = low (<20%), 1 = critical (<10%), 0 = stop (<5%)
  uint8_t batteryStage = 3;
  bool jumpEnabledPrev = true;  // Tracks last setJumpEnabled() call - reversible, unlike batteryStage
  unsigned long lastBatteryCheck = 0;
  ChargeState chargeState = CHARGE_NONE;

  // Give other tasks time to start (especially serialOutputTask)
  vTaskDelay(pdMS_TO_TICKS(100));

  while (true) {
    // On first run, check for saved MAC address
    if (firstRun) {
      if (storage.loadPeerMAC(clientMac)) {
        if (addPeer(clientMac)) {
          firstRun = false;
          queuePrint(MSG_DEBUG, "[PAIRING] Found saved controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                     clientMac[0], clientMac[1], clientMac[2], clientMac[3], clientMac[4], clientMac[5]);
          paired = true;
          robotState = STATE_PAIRED;
          lastHeartbeatReceived = millis();
          xEventGroupClearBits(eventGroup, EVENT_UNPAIRED);
          xEventGroupSetBits(eventGroup, EVENT_PAIRED);
          queuePrint(MSG_INFO, "[PAIRING] Attempting to reconnect to saved controller\n");
        } else {
          // ESP-NOW not ready yet - retry next pass instead of declaring
          // "paired" with no peer actually registered (see setup()'s
          // WiFi/ESP-NOW-before-tasks ordering, which should make this rare).
          queuePrint(MSG_DEBUG, "[PAIRING] addPeer failed for saved MAC - will retry\n");
        }
      } else {
        firstRun = false;
        queuePrint(MSG_INFO, "[PAIRING] No saved MAC found - waiting for pairing request\n");
      }
    }

    unsigned long now = millis();
    RobotState currentState = robotState;

    // Pairing button: debounce + long-press detection (active-low). Runs
    // regardless of current state - a stale pairing to the wrong controller
    // should be forceable from STATE_PAIRED/STATE_STARTED too, not just
    // STATE_UNPAIRED.
    {
      bool raw = (digitalRead(BUTTON_PIN) == LOW);
      if (raw != buttonRaw) {
        buttonRaw = raw;
        lastButtonChangeMs = now;
      }
      if ((now - lastButtonChangeMs) >= BUTTON_DEBOUNCE_MS && buttonDebounced != buttonRaw) {
        buttonDebounced = buttonRaw;
        if (buttonDebounced) {
          buttonPressStartMs = now;
        } else {
          longPressLatched = false;
        }
      }

      if (buttonDebounced && !longPressLatched && (now - buttonPressStartMs) >= BUTTON_PAIRING_HOLD_MS) {
        longPressLatched = true;
        queuePrint(MSG_INFO, "[BUTTON] Held %lums - forcing pairing mode\n", (unsigned long)BUTTON_PAIRING_HOLD_MS);
        enterPairingModeManually();
      }
    }

    // Battery check: every 30s, but only when commands have been quiet for 5s
    // (avoids reading under motor load or during a jump sequence)
    if (now - lastBatteryCheck >= 30000 && now - lastDataReceived >= 5000) {
      lastBatteryCheck = now;
      uint16_t pct = batteryGetPercent();
      queuePrint(MSG_DEBUG, "[BATTERY] %d%%\n", pct);

      // Jump gating: reversible (unlike batteryStage below), re-checked every
      // pass so it recovers if the battery comes back up (e.g. on charge).
      bool jumpEnabledNow = (pct >= 50);
      if (jumpEnabledNow != jumpEnabledPrev) {
        jumpEnabledPrev = jumpEnabledNow;
        q8.setJumpEnabled(jumpEnabledNow);
        queuePrint(MSG_INFO, jumpEnabledNow
          ? "[BATTERY] Jump re-enabled (%d%%)\n"
          : "[BATTERY] Jump disabled - battery below 50%% (%d%%)\n", pct);
      }

      uint8_t newStage;
      if      (pct >= 20) newStage = 3;
      else if (pct >= 10) newStage = 2;
      else if (pct >= 5)  newStage = 1;
      else                newStage = 0;

      if (newStage < batteryStage) {
        batteryStage = newStage;
        switch (batteryStage) {
          case 2:
            queuePrint(MSG_INFO, "[BATTERY] Low battery (%d%%)\n", pct);
            break;
          case 1:
            queuePrint(MSG_INFO, "[BATTERY] Critical battery (%d%%) - reduce activity\n", pct);
            break;
          case 0:
            queuePrint(MSG_INFO, "[BATTERY] Battery critical (%d%%)\n", pct);
            break;
        }
      }

      ChargeState newChargeState = batteryGetChargeState();
      if (newChargeState != chargeState) {
        chargeState = newChargeState;
        switch (chargeState) {
          case CHARGE_NONE:
            queuePrint(MSG_INFO, "[BATTERY] Charger disconnected\n");
            break;
          case CHARGE_ACTIVE:
            queuePrint(MSG_INFO, "[BATTERY] Charging (%d%%)\n", pct);
            break;
          case CHARGE_DONE:
            queuePrint(MSG_INFO, "[BATTERY] Charge complete (%d%%)\n", pct);
            break;
        }
      }
    }

    // Charging RGB override - takes priority over the pairing/idle colors
    // below (rainbow search, solid/blinking blue) since a docked, charging
    // robot is the most common idle state. Deliberately excluded from
    // STATE_STARTED: that color already carries the more operationally
    // useful torque/battery-stage signal, and charging-while-active is an
    // edge case not worth stepping on it for.
    bool chargeOverride = (chargeState != CHARGE_NONE) && (currentState != STATE_STARTED);
    uint8_t chargeR = 0, chargeG = 0, chargeB = 0;
    if (chargeOverride) {
      if (chargeState == CHARGE_DONE) {
        chargeG = 255;  // solid green - full and maintaining, matches the
                         // conventional "green = done" charger LED color
      } else {
        // Breathing amber - actively topping up.
        const uint16_t periodMs = 2000;
        float phase = fmodf((float)now, (float)periodMs) / (float)periodMs;
        float level = (sinf(2.0f * PI * phase - PI / 2.0f) + 1.0f) / 2.0f;
        chargeR = (uint8_t)(255 * level);
        chargeG = (uint8_t)(120 * level);
      }
    }

    switch (currentState) {
      case STATE_UNPAIRED: {
        // A manually-triggered (button) re-pair is bounded to
        // PAIRING_TIMEOUT_MS - a fresh/never-paired boot leaves
        // manualPairingEntry false and keeps accepting forever, same as
        // before this button existed.
        if (manualPairingEntry) {
          if (manualPairingStartMs == 0) manualPairingStartMs = now;
          if (pairingAcceptingRequests && (now - manualPairingStartMs) >= PAIRING_TIMEOUT_MS) {
            pairingAcceptingRequests = false;
            manualPairingEntry = false;
            manualPairingStartMs = 0;
            queuePrint(MSG_INFO, "[PAIRING] No response after %lums - giving up (hold the button again to retry)\n",
                       (unsigned long)PAIRING_TIMEOUT_MS);
          }
        } else {
          manualPairingStartMs = 0;
        }

        if (pairingAcceptingRequests) {
          if (chargeOverride) {
            rgb.setColor(chargeR, chargeG, chargeB);
          } else {
            // Rainbow cycle on RGB LED every loop tick (~100ms per step)
            rgb.setHue(hue);
            hue += 512;  // Full cycle in ~13s
          }

          // Slow blink every 2 seconds
          if (now - lastActivity >= 2000) {
            lastActivity = now;
            digitalWrite(LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(200));
            digitalWrite(LED_PIN, LOW);
            queuePrint(MSG_DEBUG, "[PAIRING] Waiting for pairing...\n");
          }
        } else if (chargeOverride) {
          rgb.setColor(chargeR, chargeG, chargeB);
        } else {
          rgb.off();
        }
        break;
      }

      case STATE_PAIRED: {
        // Solid blue once heartbeats are actively coming in from the
        // controller; brief blink while paired but not yet hearing them
        // (e.g. right after pairing, or if the link has gone quiet). A
        // blink (not a sine breathe) survives the ~1s vTaskDelay block
        // below without visibly stuttering.
        if (chargeOverride) {
          rgb.setColor(chargeR, chargeG, chargeB);
        } else {
          bool heartbeatAlive = (now >= lastHeartbeatReceived) &&
                                 (now - lastHeartbeatReceived <= HEARTBEAT_TIMEOUT_ROBOT);
          if (heartbeatAlive) {
            rgb.setColor(0, 0, 255);
          } else {
            bool blinkOn = (now % 2000) < 500;
            rgb.setColor(0, 0, blinkOn ? 255 : 0);
          }
        }

        // Double blink every 2 seconds (waiting for torque enable)
        if (now - lastActivity >= 2000) {
          lastActivity = now;
          for (int i = 0; i < 2; i++) {
            digitalWrite(LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(200));
            digitalWrite(LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(300));
          }

          // Try to start robot (enable torque)
          if (q8.commStart()) {
            robotState = STATE_STARTED;
            started = true;
            xEventGroupSetBits(eventGroup, EVENT_STARTED);
            queuePrint(MSG_INFO, "[STATE] Robot started!\n");
            lastActivity = now;  // Reset for breathing pattern
          } else {
            queuePrint(MSG_DEBUG, "[STATE] Waiting for robot start...\n");
          }
        }
        break;
      }

      case STATE_STARTED: {
        // RGB reflects torque + battery state. Torque off -> solid blue
        // (matches STATE_PAIRED). Torque on -> breathe/pulse colored by
        // batteryStage, updated once per 100ms task tick.
        if (!q8.isTorqueOn()) {
          rgb.setColor(0, 0, 255);
        } else {
          uint8_t baseR, baseG, baseB;
          uint16_t periodMs;
          switch (batteryStage) {
            case 3:  baseR = 0;   baseG = 255; baseB = 0;   periodMs = 3000; break;  // breathe green
            case 2:  baseR = 255; baseG = 255; baseB = 0;   periodMs = 3000; break;  // breathe yellow
            case 1:  baseR = 255; baseG = 80;  baseB = 0;   periodMs = 900;  break;  // pulse orange
            default: baseR = 255; baseG = 0;   baseB = 0;   periodMs = 900;  break;  // pulse red
          }
          float phase = fmodf((float)now, (float)periodMs) / (float)periodMs;
          float level = (sinf(2.0f * PI * phase - PI / 2.0f) + 1.0f) / 2.0f;  // 0..1 smooth ramp
          rgb.setColor((uint8_t)(baseR * level), (uint8_t)(baseG * level), (uint8_t)(baseB * level));
        }

        // Breathing LED pattern every 10 seconds
        if (now - lastActivity >= 10000) {
          lastActivity = now;

          for (int brightness = 0; brightness <= 255; brightness += 2) {
            analogWrite(LED_PIN, brightness);
            vTaskDelay(pdMS_TO_TICKS(2));
          }
          for (int brightness = 255; brightness >= 0; brightness -= 2) {
            analogWrite(LED_PIN, brightness);
            vTaskDelay(pdMS_TO_TICKS(2));
          }
        }
        break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Check state every 100ms
  }
}

// FreeRTOS Task: Serial Output Handler (Priority 1)
void serialOutputTask(void* parameter) {
  SerialMessage msg;

  while (true) {
    // Check for serial output messages (blocking with timeout)
    if (xQueueReceive(debugQueue, &msg, pdMS_TO_TICKS(10)) == pdTRUE) {
      // Print based on message type
      if (msg.type == MSG_INFO) {
        // INFO messages always printed
        Serial.print(msg.text);
      } else if (msg.type == MSG_DEBUG && debugMode) {
        // DEBUG messages only when debugMode is ON
        Serial.print(msg.text);
      }
    }

    // Check for incoming serial commands
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'p') {
        queuePrint(MSG_INFO, "[PAIRING] Force pairing mode requested\n");
        q8.toggleTorque(0);        // Disable torque hardware
        q8.resetTorqueState();     // Sync internal flag to match disabled state
        unpair();
      } else if (c == 'd') {
        debugMode = !debugMode;
        queuePrint(MSG_INFO, "Debug mode: %s\n", debugMode ? "ON" : "OFF");
      } else if (c == 'i') {
        queuePrint(MSG_INFO, "%s\n", FIRMWARE_ID);  // see FIRMWARE_ID
      } else if (c == '5') {
        toggleRail(rail5V, "5V");   // see toggleRail
      } else if (c == '6') {
        toggleRail(rail6V, "6V");
      }
    }

    // Low priority - yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// FreeRTOS Task: Gait Runner (Priority 2)
// Steps an active native gait (see GaitRunner/gaitTables.h) at a fixed
// cadence matching the host's own per-tick pace (SPEED=200 in operate.py).
// espnowRxTask stays purely reactive - GAIT_RUN/GAIT_STOP/MOVE commands are
// still processed immediately there; this task only advances the phase.
void gaitRunnerTask(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();

  const TickType_t period = pdMS_TO_TICKS(5);

  // Its own reply buffer: espnowRxTask owns myMsg, and both tasks can be
  // building a frame at the same time.
  static DataMessage telemetryMsg;
  static TelemetryPayload telemetry = {};
  static int8_t cachedTemperature[8] = {0};
  static uint32_t lastTempSampleMs = 0;

  while (true) {
    if (gaitRunner.isActive()) {
      gaitRunner.tick();
    }

    // Telemetry goes after the gait write, deliberately: the servos already
    // have their new goal position by this point, so the read only consumes
    // slack that would otherwise sit idle.
    uint32_t nowMs = millis();
    bool hostWantsTelemetry = (nowMs - lastTelemetryRequestMs) < TELEMETRY_IDLE_TIMEOUT_MS;

    if (hostWantsTelemetry && (nowMs - lastTempSampleMs) >= TELEMETRY_TEMP_PERIOD_MS) {
      lastTempSampleMs = nowMs;
      q8.readTemperature(cachedTemperature);
    }

    if (telemetryRequested) {
      telemetryRequested = false;
      if (paired && q8.readJointState(telemetry)) {
        memcpy(telemetry.temperature, cachedTemperature, sizeof(cachedTemperature));
        telemetryMsg.msgType = DATA;
        telemetryMsg.id = 0;
        buildFrame((uint8_t)ReplyType::TELEMETRY_REPLY,
                   (const uint8_t*)&telemetry, sizeof(telemetry),
                   (uint8_t*)telemetryMsg.data);
        esp_now_send(clientMac, (uint8_t*)&telemetryMsg, sizeof(telemetryMsg));
      }
    }

    // If something long ran between ticks - a jump, a greet, or simply this
    // task being starved by the higher-priority espnowRxTask - lastWake is
    // now well in the past, and vTaskDelayUntil would return immediately
    // over and over until it caught up, firing a burst of frames back to
    // back. Resync instead: a gait resumes from where it is rather than
    // sprinting through the backlog.
    TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now - lastWake) > period * 4) {
      lastWake = now;
    }
    vTaskDelayUntil(&lastWake, period);
  }
}

// How long the 5V rail gets to come up (buck soft-start, then the RGB LED's
// own power-on) before the LED is sent a color.
const uint32_t RAIL_5V_SETTLE_MS = 20;

// FreeRTOS Task: Power Saving (Priority 1) - only created when the boot
// battery check reads critical (<5%). WiFi/ESP-NOW/servos are never brought
// up in this mode and both bucks are off (see setup()), so this task is the
// only thing running: a slow "still alive, still critical" red blink until
// the user physically powers off. The 5V buck is switched on only for each
// blink - the RGB LED is its load - so between blinks the ESP32 is the only
// thing drawing current. Next step: deep sleep between blinks instead of
// this task.
void powerSavingTask(void* parameter) {
  while (true) {
    rail5V.set(true);
    vTaskDelay(pdMS_TO_TICKS(RAIL_5V_SETTLE_MS));
    rgb.setColor(255, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    rgb.off();
    rail5V.set(false);
    vTaskDelay(pdMS_TO_TICKS(9950));
  }
}

// ============================================================================
// Arduino Setup. Configures FreeRTOS tasks, ESP-NOW, and other hardware.
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Pairing button

  // Both bucks on before anything they power is touched - the RGB LED just
  // below (5V), the servos from q8.begin() at the end (6V). The 6V rail is
  // on from here, as it was when the board's own pull decided it, so the
  // servos keep the same time to boot before q8.begin() pings them.
  rail5V.begin(true);
  rail6V.begin(true);
  Serial.println("[POWER] 5V and 6V bucks on");
  delay(RAIL_5V_SETTLE_MS);
  rgb.begin(storage.loadLedBrightness(DEFAULT_LED_BRIGHTNESS));

  // BQ25887 init - done here (before WiFi/FreeRTOS bring-up below) so the
  // boot battery check right after can read a real percentage, and the
  // very first RGB activity after boot reflects battery level rather than
  // the pairing rainbow.
  Wire.begin();
  batteryInit();

  // batteryInit() just enabled the BQ25887's continuous ADC - its first
  // conversion isn't ready instantly, and reading VBAT before it completes
  // reads back as ~0V (i.e. 0%), which would misread as critical battery
  // every time. Give it a full cycle to settle before trusting a reading.
  delay(1000);

  bool powerSavingMode = false;
  {
    uint16_t bootPct = batteryGetPercent();
    BootBatteryBand band = classifyBootBattery(bootPct);
    Serial.printf("[BATTERY] Boot check: %d%%\n", bootPct);

    switch (band) {
      case BOOT_BATTERY_GREEN:  breatheOnce(0, 255, 0, 2000);   break;
      case BOOT_BATTERY_YELLOW: breatheOnce(255, 255, 0, 2000); break;
      case BOOT_BATTERY_ORANGE: breatheOnce(255, 80, 0, 2000);  break;
      case BOOT_BATTERY_RED:    breatheOnce(255, 0, 0, 2000);   break;
      case BOOT_BATTERY_CRITICAL:
        Serial.println("[BATTERY] Critical at boot - entering power-saving mode");
        blinkCriticalWarning();
        powerSavingMode = true;
        break;
    }
  }

  if (powerSavingMode) {
    // Skip WiFi/ESP-NOW/servo bring-up entirely - nothing productive can
    // happen safely this low, and staying off the radio saves what little
    // charge is left. Both bucks go off; powerSavingTask() pulses the 5V
    // one only to blink the LED.
    rail6V.set(false);
    rail5V.set(false);
    Serial.println("[POWER] Critical battery - 5V and 6V bucks off");
    xTaskCreate(powerSavingTask, "PowerSaving", 2048, NULL, 1, NULL);
    return;
  }

  bool initSuccess = true;

  // FreeRTOS Initialization
  // Create queues
  rxQueue = xQueueCreate(10, sizeof(ESPNowMessage));
  if (rxQueue == NULL) {
    Serial.println("[RTOS] Failed to create RX queue");
    initSuccess = false;
  }

  debugQueue = xQueueCreate(20, sizeof(SerialMessage));
  if (debugQueue == NULL) {
    Serial.println("[RTOS] Failed to create debug queue");
    initSuccess = false;
  }

  // Create event group for task synchronization
  eventGroup = xEventGroupCreate();
  if (eventGroup == NULL) {
    Serial.println("[RTOS] Failed to create event group");
    initSuccess = false;
  }

  // Init Wi-Fi + ESP-NOW BEFORE creating any tasks. robotStateTask's
  // firstRun EEPROM-reconnect logic (addPeer() from a saved MAC) runs as
  // soon as that task starts - if it ran concurrently with WiFi/ESP-NOW
  // bring-up here (which can take 100ms+), addPeer() could silently fail
  // (ESP-NOW not yet initialized) while the code still declared the robot
  // "paired", with no timeout-driven self-correction from that state (see
  // unpair(), only ever called from the 'p' serial command or a manual
  // pairing-reset trigger) - so it would be a silent, permanent dead end
  // after any reset. Doing this first removes
  // the race entirely; queues/event group above must exist first since
  // onRecv() (registered below) pushes into rxQueue.
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("esp-server", nullptr, chan); // Optional, just to enable softAP mode
  chan = WiFi.channel();
  WiFi.softAPmacAddress(serverMac);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    initSuccess = false;
  } else {
    esp_now_register_recv_cb(onRecv);  // Set up callback when data is received.
  }

  // Create FreeRTOS tasks
  // Create serial output task (Priority 1)
  BaseType_t taskCreated = xTaskCreate(
    serialOutputTask,   // Task function
    "SerialOutput",     // Task name
    2048,               // Stack size (bytes)
    NULL,               // Parameters
    1,                  // Priority (low)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create serial output task");
    initSuccess = false;
  }

  // Create ESP-NOW RX handler task (Priority 3)
  taskCreated = xTaskCreate(
    espnowRxTask,       // Task function
    "ESPNowRX",         // Task name
    4096,               // Stack size (bytes) - larger for message processing
    NULL,               // Parameters
    3,                  // Priority (high - processes incoming messages)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create ESP-NOW RX task");
    initSuccess = false;
  }

  // Create robot state manager task (Priority 1)
  taskCreated = xTaskCreate(
    robotStateTask,     // Task function
    "RobotState",       // Task name
    4096,               // Stack size (bytes) - larger for LED patterns and state logic
    NULL,               // Parameters
    1,                  // Priority (low - manages LED patterns)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create robot state task");
    initSuccess = false;
  }

  // Create gait runner task (Priority 2) - steps an active native gait
  // (see GaitRunner/gaitTables.h) at a fixed cadence, independent of
  // espnowRxTask so a GAIT_STOP/new-direction/MOVE command is always
  // processed promptly rather than starved by gait stepping.
  taskCreated = xTaskCreate(
    gaitRunnerTask,     // Task function
    "GaitRunner",       // Task name
    2048,               // Stack size (bytes)
    NULL,               // Parameters
    2,                  // Priority (medium - below ESPNowRX's 3)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create gait runner task");
    initSuccess = false;
  }

  // Check if all FreeRTOS/WiFi/ESP-NOW initialization succeeded
  if (!initSuccess) {
    Serial.println("[RTOS] Initialization failed - halting system");
    while(1) { delay(1000); }  // Halt system indefinitely
  }

  // Initialize Dynamixel object
  q8.begin();
  q8.attachRGB(&rgb);
  q8.attachGaitRunner(&gaitRunner);
  q8.attachStorage(&storage);
}

// Loop does nothing - all work done in FreeRTOS tasks
void loop() {
  delay(1);
}
