#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>

// Q8bot-specific Modules
#include "systemParams.h"
#include "macStorage.h"
#include "protocol.h"
#include "gamepadQT.h"
#include "independentControl.h"
#include "pinMapping.h"
#include "RGBManager.h"

// Independent (gamepad) mode vs relay mode - see independentControlTask()
// and commandForwardingTask()'s HOST_CONNECTED handling. Independent by
// default at boot; once a host connects, relay mode is one-way until a
// power cycle (ESP32-C3 native USB-CDC has no reliable continuous
// "still connected" signal to auto-revert on).
enum ControllerMode { MODE_INDEPENDENT, MODE_RELAY };
volatile ControllerMode controllerMode = MODE_INDEPENDENT;

// Tunable: how long START+SELECT must be held together (in independent
// mode) to force pairing mode, and how long the resulting broadcast keeps
// retrying before giving up. See independentControlTask()/pairingTask().
const uint32_t PAIRING_HOLD_MS = 5000;
const uint32_t PAIRING_TIMEOUT_MS = 30000;
// Set true right before a manually-triggered unpair() (gamepad combo hold)
// so pairingTask knows to bound its broadcast retry to PAIRING_TIMEOUT_MS -
// the pre-existing 'p' serial command and a fresh boot's "no saved MAC"
// case both leave this false, retrying forever as before.
bool manualPairingEntry = false;

// Default status-LED brightness (0-255) - full brightness, since this LED
// is the controller's only pairing/connection indicator and there's no
// upstream command to change it (unlike the robot's own LED, which defaults
// dimmer - see the robot firmware's DEFAULT_LED_BRIGHTNESS).
const uint8_t DEFAULT_LED_BRIGHTNESS = 255;

// Physical pairing button (IO9, active-low) - separate trigger from the
// gamepad START+SELECT combo above, same enterPairingModeManually() path
// and PAIRING_TIMEOUT_MS bound. See rgbStatusTask().
const uint32_t BUTTON_PAIRING_HOLD_MS = 3000;
const uint32_t BUTTON_DEBOUNCE_MS = 30;
// Tolerates a brief mid-hold contact drop-out (switch bounce, vibration, a
// marginal connection) without restarting the 3s count - only a release
// that stays released for this long cancels an in-progress hold.
const uint32_t BUTTON_RELEASE_GRACE_MS = 150;

// Set/cleared by pairingTask() each time it decides whether to send a
// broadcast this cycle - lets rgbStatusTask() show the "searching" rainbow
// for as long as pairing is actually being attempted (covers both the
// unbounded fresh-boot case and the bounded manual-entry case) and turn it
// off once a bounded window gives up.
volatile bool pairingBroadcasting = false;

// Initialize global objects
esp_now_peer_info_t peerInfo;
Preferences prefs;
macStorage storage;
RGBManager rgb(RGB_PIN);
GamepadQT gamepad;
bool gamepadAvailable = false;  // Set in setup() - relay firmware works fine without one attached

// FreeRTOS Handles
QueueHandle_t rxQueue = NULL;
QueueHandle_t debugQueue = NULL;
QueueHandle_t dataOutputQueue = NULL;
EventGroupHandle_t eventGroup = NULL;
// Guards Serial writes shared between serialOutputTask (debug text) and
// espnowRxTask (binary reply frames) - without this, interleaved writes
// could corrupt a reply frame's boundaries as the host sees them.
SemaphoreHandle_t serialTxMutex = NULL;

// Which firmware this is. The desktop app's flasher reads it two ways: sent
// back as a line when the host writes 'i' over serial (what the plugged-in
// device *is*), and found as a literal inside the .bin (what a file *would
// make* it) - so it can refuse to put robot firmware on the controller or
// vice versa. The version after the second '|' is stamped in at build time
// from `git describe` (see scripts/fw_version.py). Must match
// q8botone-python/q8bot/firmware_flasher.py.
#ifndef Q8BOT_FW_VERSION
#define Q8BOT_FW_VERSION "unknown"
#endif
static const char FIRMWARE_ID[] = "Q8BOT_FW|controller|" Q8BOT_FW_VERSION;


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

  xQueueSend(debugQueue, &msg, 0);
}

bool addPeer(const uint8_t* mac) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = chan;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void unpair() {
  queuePrint(MSG_DEBUG, "[HEARTBEAT] Connection lost - returning to pairing mode\n");

  esp_now_del_peer(serverMac);
  storage.clearPeerMAC();
  queuePrint(MSG_DEBUG, "[STORAGE] Cleared peer MAC from EEPROM\n");

  memset(serverMac, 0, sizeof(serverMac));
  paired = false;
  lastPairAttempt = millis();

  // Signal pairing task to resume broadcasting (if FreeRTOS is running)
  if (eventGroup != NULL) {
    xEventGroupSetBits(eventGroup, EVENT_UNPAIRED);
  }
}

// Manual pairing-mode entry (gamepad START+SELECT hold) - unlike the
// pre-existing 'p' serial command, this bounds the resulting broadcast
// retry to PAIRING_TIMEOUT_MS (see pairingTask()).
void enterPairingModeManually() {
  manualPairingEntry = true;
  unpair();
}

// True only when a host has actually opened the USB CDC port. Worth checking
// before writing: in standalone (gamepad) mode nobody is listening, and
// HWCDC::write() still takes its tx lock and churns the ring buffer for data
// it then discards. Cheap insurance that gets cheaper the more the robot
// sends back.
static inline bool hostAttached() {
  return (bool)Serial;
}

// Builds and sends a command frame directly to the paired robot - used by
// independentControlTask() for all its own sends (GAIT_RUN/JUMP/GREET/
// TORQUE_SET/GAIT_STOP). Uses its own local message buffer (not the
// shared sendMsg global commandForwardingTask uses for relaying) since
// this can run concurrently with that task on a different core.
bool sendCommandToRobot(CmdType cmd, const uint8_t* payload = nullptr, uint8_t payloadLen = 0) {
  if (!paired) return false;
  CharMessage msg;
  msg.msgType = DATA;
  msg.id = 1;
  buildFrame((uint8_t)cmd, payload, payloadLen, (uint8_t*)msg.data);
  esp_now_send(serverMac, (uint8_t*)&msg, sizeof(msg));
  return true;
}

// Moves all 4 legs to the given cyclable gait's own (x0, y0) neutral pose
// (see independentControl.h's ikSolveNeutral()/INDEPENDENT_GAIT_NEUTRAL_*),
// mirroring operate.py's move_xy(pos_x, pos_y, dur) - a raw MOVE, mirrored
// across all 4 legs exactly like move_mirror() does. Payload layout must
// match q8Dynamixel.cpp's MOVE case exactly: 8x int16 (tenths of a degree)
// + uint16 dur + uint8 record flag, native (little-endian) byte order -
// matches Python's struct.pack('<8hHB', ...) since ESP32 is also little-endian.
bool sendMoveToGaitNeutral(uint8_t gaitCycleIndex, uint16_t durMs) {
  float q1Deg, q2Deg;
  ikSolveNeutral(INDEPENDENT_GAIT_NEUTRAL_X[gaitCycleIndex], INDEPENDENT_GAIT_NEUTRAL_Y[gaitCycleIndex], q1Deg, q2Deg);

  int16_t angleTenths[8];
  for (int leg = 0; leg < 4; leg++) {
    angleTenths[leg * 2]     = (int16_t)lroundf(q1Deg * 10.0f);
    angleTenths[leg * 2 + 1] = (int16_t)lroundf(q2Deg * 10.0f);
  }

  uint8_t payload[19];
  memcpy(payload, angleTenths, sizeof(angleTenths));
  memcpy(payload + 16, &durMs, sizeof(durMs));
  payload[18] = 0;  // record = false

  return sendCommandToRobot(CmdType::MOVE, payload, sizeof(payload));
}


// ============================================================================
// ESPNOW Callbacks: ISR-Like, Highest Priority
// ============================================================================
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  // Validate minimum length
  if (len < 1 || len > 250) return;

  // Copy to queue structure
  ESPNowMessage msg;
  memcpy(msg.mac, mac, 6);
  memcpy(msg.data, data, len);
  msg.len = len;
  msg.timestamp = millis();

  // Non-blocking send; drop if full
  (void)xQueueSend(rxQueue, &msg, 0);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

// ============================================================================
// FreeRTOS Tasks (Ranked by Priority)
// ============================================================================
// FreeRTOS Task: Command Forwarding (Priority 4 - HIGHEST)
void commandForwardingTask(void *param) {
  TickType_t lastWake = xTaskGetTickCount();

  while (1) {
    if (Serial.available()) {
      int peeked = Serial.peek();  // 0-255 for a real byte, -1 if none

      if (peeked == 'd') {
        Serial.read();
        debugMode = !debugMode;
        queuePrint(MSG_INFO, "Debug mode: %s\n", debugMode ? "ON" : "OFF");
      }
      else if (peeked == 'p') {
        Serial.read();
        queuePrint(MSG_DEBUG, "[PAIRING] Force pairing mode requested\n");
        unpair();
      }
      else if (peeked == 'i') {
        Serial.read();
        queuePrint(MSG_INFO, "%s\n", FIRMWARE_ID);  // see FIRMWARE_ID
      }
      else if (peeked == FRAME_START) {
        Serial.read();  // consume START

        uint8_t header[FRAME_HEADER_LEN];
        size_t headerRead = Serial.readBytes(header, FRAME_HEADER_LEN);

        if (headerRead != FRAME_HEADER_LEN) {
          // Timed out - drop, resync on the next byte next iteration.
        } else {
          uint8_t length = header[1];
          size_t restLen = (size_t)length + FRAME_CRC_LEN;

          uint8_t* buf = (uint8_t*)sendMsg.data;
          buf[0] = FRAME_START;
          buf[1] = header[0];
          buf[2] = header[1];
          size_t restRead = Serial.readBytes(&buf[3], restLen);

          if (restRead != restLen) {
            // Timed out - drop, resync on the next byte next iteration.
          } else {
            uint8_t msgType; const uint8_t* payload; uint8_t payloadLen;
            if (parseFrameBuf(buf, sizeof(sendMsg.data), &msgType, &payload, &payloadLen)) {
              // HOST_CONNECTED is controller-local - parsed regardless of
              // pairing state (a host may connect before independent mode
              // has paired with a robot yet), never forwarded.
              if ((CmdType)msgType == CmdType::HOST_CONNECTED) {
                if (controllerMode != MODE_RELAY) {
                  controllerMode = MODE_RELAY;
                  queuePrint(MSG_INFO, "[MODE] Host connected - switching to relay mode\n");
                }
              } else if (!paired) {
                // No peer to relay to - drop the frame.
              } else {
                sendMsg.msgType = DATA;
                sendMsg.id = 1;
                esp_now_send(serverMac, (uint8_t*)&sendMsg, sizeof(sendMsg));
              }
            } else {
              queuePrint(MSG_DEBUG, "[PROTOCOL] Dropped command frame - CRC/format invalid\n");
            }
          }
        }
      }
      else {
        // Stray/unknown byte - discard so it can't wedge this loop forever.
        Serial.read();
      }
    }

    // Run at 250 Hz (4ms period)
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(4));
  }
}

// FreeRTOS Task: ESP-NOW RX Handler (Priority 3)
void espnowRxTask(void *param) {
  ESPNowMessage msg;

  while (1) {
    // Block waiting for messages from ISR queue
    if (xQueueReceive(rxQueue, &msg, portMAX_DELAY) == pdTRUE) {

      // Process based on message type
      if (msg.data[0] == PAIRING && !paired) {
        // Validate PAIRING message length
        if (msg.len < sizeof(PairingMessage)) continue;

        memcpy(&pairingData, msg.data, sizeof(PairingMessage));

        // Ignore pairing responses from other controllers
        if (pairingData.role != ROLE_ROBOT) continue;

        queuePrint(MSG_DEBUG, "[PAIRING] Paired with server: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   msg.mac[0], msg.mac[1], msg.mac[2], msg.mac[3], msg.mac[4], msg.mac[5]);

        memcpy(serverMac, msg.mac, sizeof(serverMac));
        addPeer(serverMac);
        paired = true;
        lastHeartbeatReceived = millis();

        // Save the MAC address to EEPROM
        storage.savePeerMAC(serverMac);
        queuePrint(MSG_DEBUG, "[STORAGE] Saved peer MAC to EEPROM\n");
        queuePrint(MSG_DEBUG, "[HEARTBEAT] Connection established, heartbeat timer started\n");

        // Signal pairing task to stop broadcasting
        if (eventGroup != NULL) {
          xEventGroupSetBits(eventGroup, EVENT_PAIRED);
        }

      } else if (msg.data[0] == HEARTBEAT) {
        // Robot echoed heartbeat back
        if (msg.len < sizeof(HeartbeatMessage)) continue;

        lastHeartbeatReceived = millis();

        HeartbeatMessage hbMsg;
        memcpy(&hbMsg, msg.data, sizeof(HeartbeatMessage));
        uint32_t rtt = millis() - hbMsg.timestamp;
        queuePrint(MSG_DEBUG, "[HEARTBEAT] ACK received, RTT: %ums\n", rtt);

      } else if (msg.data[0] == DATA) {
        // Validate DATA message length
        if (msg.len < sizeof(IntMessage)) continue;

        memcpy(&recvMsg, msg.data, sizeof(IntMessage));
        lastHeartbeatReceived = millis();  // Any DATA also counts as "alive"

        // Relay the framed reply the robot built, unchanged, to the host -
        // but only if there is a host. This runs in the highest-priority
        // task, so it should do as little as possible when standalone.
        const uint8_t* buf = (const uint8_t*)recvMsg.data;
        uint8_t msgType; const uint8_t* payload; uint8_t payloadLen;
        if (!hostAttached()) {
          // Nothing to relay to - drop it and move on.
        } else if (parseFrameBuf(buf, sizeof(recvMsg.data), &msgType, &payload, &payloadLen)) {
          if (serialTxMutex != NULL && xSemaphoreTake(serialTxMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            Serial.write(buf, 3 + payloadLen + FRAME_CRC_LEN);
            xSemaphoreGive(serialTxMutex);
          }
        } else {
          queuePrint(MSG_DEBUG, "[PROTOCOL] Dropped reply frame - CRC/format invalid\n");
        }
        memset(&recvMsg, 0, sizeof(recvMsg));
      }
    }
  }
}

// FreeRTOS Task: Heartbeat Manager (Priority 2)
void heartbeatTask(void *param) {
  TickType_t lastWake = xTaskGetTickCount();

  while (1) {
    // Only send heartbeat when paired
    if (paired) {
      // Send heartbeat
      heartbeatMsg.msgType = HEARTBEAT;
      heartbeatMsg.id = 1;
      heartbeatMsg.timestamp = millis();
      lastHeartbeatSent = millis();

      unsigned long timeSinceLastHB = millis() - lastHeartbeatReceived;
      queuePrint(MSG_DEBUG, "[HEARTBEAT] Sending heartbeat (last response: %lums ago)\n", timeSinceLastHB);

      esp_now_send(serverMac, (uint8_t*)&heartbeatMsg, sizeof(heartbeatMsg));
    }

    // Run at frequency defined by constant
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(HEARTBEAT_INTERVAL));
  }
}

// FreeRTOS Task: Serial Output / Debug (Priority 1)
void serialOutputTask(void *param) {
  SerialMessage msg;

  while (1) {
    // Check for serial output messages (blocking with timeout)
    if (xQueueReceive(debugQueue, &msg, pdMS_TO_TICKS(10)) == pdTRUE) {
      // Print based on message type
      bool shouldPrint = hostAttached() &&
                         ((msg.type == MSG_INFO) || (msg.type == MSG_DEBUG && debugMode));
      if (shouldPrint && serialTxMutex != NULL && xSemaphoreTake(serialTxMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.print(msg.text);
        xSemaphoreGive(serialTxMutex);
      }
    }

    // Low priority - yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// FreeRTOS Task: Pairing Manager (Priority 0 - LOWEST)
void pairingTask(void *param) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t broadcastStartMs = 0;  // 0 = not currently in a bounded (manual) broadcast window

  // On first run, check for saved MAC address
  bool reconnected = false;
  if (storage.loadPeerMAC(serverMac)) {
    if (addPeer(serverMac)) {
      char macStr[18];
      sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
              serverMac[0], serverMac[1], serverMac[2],
              serverMac[3], serverMac[4], serverMac[5]);
      queuePrint(MSG_DEBUG, "[PAIRING] Found saved MAC: %s\n", macStr);

      paired = true;
      lastHeartbeatReceived = millis();
      lastHeartbeatSent = millis();

      queuePrint(MSG_DEBUG, "[PAIRING] Attempting to reconnect to saved peer\n");

      // Signal that we're already paired
      xEventGroupSetBits(eventGroup, EVENT_PAIRED);
      reconnected = true;
    } else {
      // ESP-NOW not ready yet (shouldn't happen now that init() runs before
      // task creation, but stay defensive) - fall back to a fresh pairing
      // broadcast instead of declaring "paired" with no peer registered.
      queuePrint(MSG_DEBUG, "[PAIRING] addPeer failed for saved MAC - falling back to fresh pairing\n");
      memset(serverMac, 0, sizeof(serverMac));
      storage.clearPeerMAC();
    }
  }
  if (!reconnected) {
    queuePrint(MSG_DEBUG, "[PAIRING] No saved MAC found - entering pairing mode\n");
  }

  while (1) {
    // Check pairing state (use old global for now, will migrate to atomic later)
    bool isPaired = paired;

    if (!isPaired) {
      bool shouldBroadcast = true;

      if (manualPairingEntry) {
        if (broadcastStartMs == 0) broadcastStartMs = millis();
        if ((millis() - broadcastStartMs) >= PAIRING_TIMEOUT_MS) {
          queuePrint(MSG_INFO, "[PAIRING] No response after %lums - giving up (hold START+SELECT again to retry)\n",
                     (unsigned long)PAIRING_TIMEOUT_MS);
          manualPairingEntry = false;
          broadcastStartMs = 0;
          shouldBroadcast = false;
        }
      } else {
        broadcastStartMs = 0;  // Not a bounded window (fresh boot / 'p' command) - broadcast forever
      }

      pairingBroadcasting = shouldBroadcast;

      if (shouldBroadcast) {
        // Send pairing broadcast every 2s
        if (debugQueue != NULL) {
          queuePrint(MSG_DEBUG, "[PAIRING] Sending broadcast...\n");
        }

        PairingMessage pairingMsg;
        pairingMsg.msgType = PAIRING;
        pairingMsg.id = 1;
        pairingMsg.role = ROLE_CONTROLLER;
        memcpy(pairingMsg.macAddr, clientMac, 6);
        pairingMsg.channel = chan;

        esp_now_send(broadcastMAC, (uint8_t*)&pairingMsg, sizeof(pairingMsg));
      }

      // Still tick every 2s even once given up, so a fresh manual
      // combo-hold request is picked up promptly rather than needing a
      // separate wakeup mechanism.
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2000));
    } else {
      broadcastStartMs = 0;
      pairingBroadcasting = false;

      // Block until unpaired event
      EventBits_t bits = xEventGroupWaitBits(
        eventGroup,
        EVENT_UNPAIRED,
        pdTRUE,   // Clear on exit
        pdFALSE,  // Wait for any bit
        portMAX_DELAY
      );

      // Reset timing after unpair event
      lastWake = xTaskGetTickCount();
    }
  }
}

// ============================================================================
// FreeRTOS Task: RGB Status + Pairing Button (Priority 0 - lowest). Always
// created (unlike independentControlTask, which needs a gamepad) since the
// button and LED are fixed hardware on every unit. Mirrors the robot firmware's
// RGB status language (main.cpp's STATE_UNPAIRED/STATE_PAIRED handling) so
// both ends of the link read the same way: rainbow while searching for a
// peer, solid blue once paired with a live heartbeat, blinking blue if the
// heartbeat has gone stale, off once a bounded manual pairing window gives
// up with no result.
// ============================================================================
void rgbStatusTask(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  const uint32_t TICK_MS = 50;

  bool buttonRaw = false;
  bool buttonDebounced = false;
  uint32_t lastButtonChangeMs = 0;
  uint32_t buttonPressStartMs = 0;  // 0 = no hold currently being counted
  bool longPressLatched = false;  // Cleared on a confirmed release so each hold only fires once
  uint32_t releasePendingMs = 0;  // 0 = not in a post-release grace window (see BUTTON_RELEASE_GRACE_MS)

  uint16_t hue = 0;

  while (1) {
    uint32_t now = millis();

    // --- Pairing button: debounce + long-press detection (active-low) ---
    bool raw = (digitalRead(BUTTON_PIN) == LOW);
    if (raw != buttonRaw) {
      buttonRaw = raw;
      lastButtonChangeMs = now;
    }
    if ((now - lastButtonChangeMs) >= BUTTON_DEBOUNCE_MS && buttonDebounced != buttonRaw) {
      buttonDebounced = buttonRaw;
      if (buttonDebounced) {
        // (Re-)pressed. If this follows a release that's still within its
        // grace window, treat it as the same hold attempt rather than
        // restarting the 3s count - a mechanical switch can momentarily
        // drop contact mid-hold (bounce, vibration, a marginal connection)
        // without the user actually letting go.
        if (releasePendingMs == 0) {
          buttonPressStartMs = now;
        }
        releasePendingMs = 0;
      } else {
        // Debounced release - don't cancel the in-progress hold yet, only
        // start the grace window. Confirmed below once the window expires.
        releasePendingMs = now;
      }
    }

    // Grace window expired with no re-press - now it's a genuine release.
    if (releasePendingMs != 0 && (now - releasePendingMs) >= BUTTON_RELEASE_GRACE_MS) {
      releasePendingMs = 0;
      longPressLatched = false;
      buttonPressStartMs = 0;
    }

    if (buttonDebounced && !longPressLatched && buttonPressStartMs != 0 &&
        (now - buttonPressStartMs) >= BUTTON_PAIRING_HOLD_MS) {
      longPressLatched = true;
      queuePrint(MSG_INFO, "[BUTTON] Held %lums - forcing pairing mode\n", (unsigned long)BUTTON_PAIRING_HOLD_MS);
      enterPairingModeManually();
    }

    // --- RGB status ---
    if (paired) {
      bool heartbeatAlive = (now - lastHeartbeatReceived) <= HEARTBEAT_TIMEOUT_CONTROLLER;
      if (heartbeatAlive) {
        rgb.setColor(0, 0, 255);
      } else {
        bool blinkOn = (now % 2000) < 500;
        rgb.setColor(0, 0, blinkOn ? 255 : 0);
      }
    } else if (pairingBroadcasting) {
      // Rainbow cycle, ~13s per full revolution at this tick rate - matches
      // the robot firmware's STATE_UNPAIRED pace.
      rgb.setHue(hue);
      hue += 256;
    } else {
      rgb.off();
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TICK_MS));
  }
}

// ============================================================================
// FreeRTOS Task: Independent Control (Priority 0 - lowest, diagnostic-
// adjacent). Drives the robot directly from the gamepad whenever
// controllerMode == MODE_INDEPENDENT and paired; goes idle (one clean
// GAIT_STOP handoff) the moment relay mode takes over. See
// independentControl.h for the ported direction/tier decision logic this
// is built on.
// ============================================================================
void independentControlTask(void* parameter) {
  const uint8_t GAIT_COUNT = 5;              // Matches INDEPENDENT_GAIT_IDS/NAMES size
  const uint32_t GAIT_RESEND_MS = 250;       // Matches operate.py's NATIVE_GAIT_RESEND_INTERVAL

  TickType_t lastWake = xTaskGetTickCount();
  uint8_t trotTierIndex = 2;       // Index into TROT_TIER_GAIT_IDS - starts at fastest
  uint8_t gaitCycleIndex = 0;      // Index into INDEPENDENT_GAIT_IDS - starts at TROT_FAST
  bool torqued = false;
  bool gaitActive = false;
  uint8_t lastSentGaitId = 0;
  uint8_t lastSentDirectionId = 0;
  uint32_t lastSendMs = 0;
  bool pairingComboLatched = false;
  bool wasPaired = false;

  while (true) {
    gamepad.update();

    if (controllerMode != MODE_INDEPENDENT) {
      // Host has taken over - one clean handoff, then stay idle until a
      // power cycle brings independent mode back.
      if (gaitActive) {
        sendCommandToRobot(CmdType::GAIT_STOP);
        gaitActive = false;
      }
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(GamepadQT::GAMEPAD_POLL_MS));
      continue;
    }

    // Manual pairing-mode entry: hold START+SELECT for PAIRING_HOLD_MS.
    if (gamepad.comboHeldFor(PAIRING_HOLD_MS) && !pairingComboLatched) {
      pairingComboLatched = true;
      queuePrint(MSG_INFO, "[GAMEPAD] START+SELECT held %lums - forcing pairing mode\n",
                 (unsigned long)PAIRING_HOLD_MS);
      if (gaitActive) {
        sendCommandToRobot(CmdType::GAIT_STOP);
        gaitActive = false;
      }
      enterPairingModeManually();
    }
    if (!gamepad.isPressed(GamepadQT::BTN_START) || !gamepad.isPressed(GamepadQT::BTN_SELECT)) {
      pairingComboLatched = false;
    }

    bool isPaired = paired;
    if (!isPaired) {
      wasPaired = false;
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(GamepadQT::GAMEPAD_POLL_MS));
      continue;
    }
    if (!wasPaired) {
      // Fresh (re)connection - the robot's own torque state resets to off
      // on any boot, so don't trust a stale local flag; gait selection
      // preference is fine to carry over.
      wasPaired = true;
      torqued = false;
      gaitActive = false;
    }

    if (gamepad.wasPressed(GamepadQT::BTN_START)) {
      torqued = !torqued;
      uint8_t val = torqued ? 1 : 0;
      sendCommandToRobot(CmdType::TORQUE_SET, &val, 1);
      if (torqued) {
        // Settle into the current gait's own neutral stance rather than
        // leaving legs wherever they physically were while untorqued -
        // avoids a jerky snap into the trajectory on the next GAIT_RUN.
        sendMoveToGaitNeutral(gaitCycleIndex, 500);
      }
      queuePrint(MSG_INFO, "[GAMEPAD] Torque %s\n", torqued ? "ON" : "OFF");
    }

    if (gamepad.wasPressed(GamepadQT::BTN_SELECT)) {
      gaitCycleIndex = (gaitCycleIndex + 1) % GAIT_COUNT;
      if (gaitActive) {
        sendCommandToRobot(CmdType::GAIT_STOP);
        gaitActive = false;
      }
      if (INDEPENDENT_GAIT_IDS[gaitCycleIndex] == TROT_TIER_GAIT_IDS[2]) {
        trotTierIndex = 2;  // Start fresh at fastest, matching operate.py's switch_gait convention
      }
      // Settle into the new gait's own neutral stance immediately, same as
      // operate.py's switch_gait handler's move_xy(pos_x, pos_y, 500).
      sendMoveToGaitNeutral(gaitCycleIndex, 500);
      queuePrint(MSG_INFO, "[GAMEPAD] Switched to %s\n", INDEPENDENT_GAIT_NAMES[gaitCycleIndex]);
    }

    if (gamepad.wasPressed(GamepadQT::BTN_Y)) {
      sendCommandToRobot(CmdType::JUMP);
      queuePrint(MSG_INFO, "[GAMEPAD] Jump\n");
    }
    if (gamepad.wasPressed(GamepadQT::BTN_X)) {
      sendCommandToRobot(CmdType::GREET);
      queuePrint(MSG_INFO, "[GAMEPAD] Greet\n");
    }

    // Joystick -> movement (always native GAIT_RUN - no host to stream raw positions to).
    float jx = gamepad.joystickX();
    float jy = gamepad.joystickY();
    Direction dir = resolveDirection(jx, jy);

    if (dir == Direction::NONE) {
      if (gaitActive) {
        sendCommandToRobot(CmdType::GAIT_STOP);
        sendMoveToGaitNeutral(gaitCycleIndex, 0);  // Immediate, matching operate.py's idle-transition move_xy(..., 0)
        gaitActive = false;
      }
    } else {
      uint8_t sendGaitId = INDEPENDENT_GAIT_IDS[gaitCycleIndex];

      if (sendGaitId == TROT_TIER_GAIT_IDS[2]) {  // TROT_FAST selected - blend tiers by magnitude
        float magnitude = fminf(1.0f, sqrtf(jx * jx + jy * jy));
        trotTierIndex = resolveTrotTier(trotTierIndex, magnitude);
        sendGaitId = TROT_TIER_GAIT_IDS[trotTierIndex];
      }

      uint8_t directionId = (uint8_t)dir;
      uint32_t now = millis();
      bool changed = !gaitActive || sendGaitId != lastSentGaitId || directionId != lastSentDirectionId;
      bool resendDue = (now - lastSendMs) >= GAIT_RESEND_MS;

      if (changed || resendDue) {
        uint8_t payload[2] = {sendGaitId, directionId};
        sendCommandToRobot(CmdType::GAIT_RUN, payload, 2);
        gaitActive = true;
        lastSentGaitId = sendGaitId;
        lastSentDirectionId = directionId;
        lastSendMs = now;
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(GamepadQT::GAMEPAD_POLL_MS));
  }
}


// ============================================================================
// Arduino Setup. Configures ESP-NOW and FreeRTOS tasks.
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  // delay(2000);  // Useful for debugging

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

  serialTxMutex = xSemaphoreCreateMutex();
  if (serialTxMutex == NULL) {
    Serial.println("[RTOS] Failed to create serial TX mutex");
    initSuccess = false;
  }

  // Init Wi-Fi + ESP-NOW BEFORE creating any tasks. pairingTask's startup
  // EEPROM-reconnect logic (addPeer() from a saved MAC) runs immediately
  // when that task starts, with no delay at all - if it ran concurrently
  // with WiFi/ESP-NOW bring-up here, addPeer() could silently fail (ESP-NOW
  // not yet initialized) while the code still declared the controller
  // "paired", with no timeout-driven self-correction from that state (see
  // unpair(), only ever called from the 'p' serial command) - so it would
  // be a silent, permanent dead end after any reset. Doing this first removes
  // the race entirely; queues/event group above must exist first since
  // onRecv() (registered below) pushes into rxQueue.
  WiFi.mode(WIFI_STA);
  WiFi.macAddress(clientMac);
  esp_wifi_start();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] Init failed");
    initSuccess = false;
  } else {
    esp_now_register_recv_cb(onRecv);
    esp_now_register_send_cb(OnDataSent);
    addPeer(broadcastMAC);
  }

  // Pairing button (IO9, active-low external pull-up) + RGB status LED
  // (IO0, SK6812) - fixed hardware on every unit, unlike the optional
  // gamepad below.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  rgb.begin(storage.loadLedBrightness(DEFAULT_LED_BRIGHTNESS));

  // GamepadQT is optional hardware - not every controller unit has one
  // attached, and the relay firmware must work fine without it, so a
  // missing/failed gamepad does not affect initSuccess.
  gamepadAvailable = gamepad.begin();
  if (gamepadAvailable) {
    Serial.println("[GAMEPAD] Found and calibrated - debug task starting");
  } else {
    Serial.println("[GAMEPAD] Not found at startup - independent-mode debug task disabled");
  }

  // Create FreeRTOS tasks
  // Create serial output task (Priority 1)
  BaseType_t taskCreated = xTaskCreate(
    serialOutputTask,   // Task function
    "SerialOut",        // Task name
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    1,                  // Priority (low - non-critical output)
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
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    3,                  // Priority (high - process messages quickly)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create ESP-NOW RX task");
    initSuccess = false;
  }

  // Create heartbeat manager task (Priority 2)
  taskCreated = xTaskCreate(
    heartbeatTask,      // Task function
    "Heartbeat",        // Task name
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    2,                  // Priority (medium - send/monitor heartbeats)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create heartbeat task");
    initSuccess = false;
  }

  // Create command forwarding task (Priority 4 - HIGHEST)
  taskCreated = xTaskCreate(
    commandForwardingTask, // Task function
    "CmdFwd",              // Task name
    3072,                  // Stack size (bytes)
    NULL,                  // Parameters
    4,                     // Priority (highest - 200 Hz critical timing)
    NULL                   // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create command forwarding task");
    initSuccess = false;
  }

  // Create pairing task (Priority 0 - lowest)
  taskCreated = xTaskCreate(
    pairingTask,        // Task function
    "Pairing",          // Task name
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    0,                  // Priority (lowest)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create pairing task");
    initSuccess = false;
  }

  // Create RGB status + pairing button task (Priority 0 - lowest). Always
  // created - the button and LED are fixed hardware, unlike the gamepad.
  taskCreated = xTaskCreate(
    rgbStatusTask,      // Task function
    "RGBStatus",        // Task name
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    0,                  // Priority (lowest)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create RGB status task");
    initSuccess = false;
  }

  // Create independent control task (Priority 0 - lowest). Only created if
  // the gamepad actually responded; its absence is not a fatal init
  // failure - the relay firmware works fine without one attached.
  if (gamepadAvailable) {
    taskCreated = xTaskCreate(
      independentControlTask,  // Task function
      "IndependentControl",    // Task name
      3072,                    // Stack size (bytes)
      NULL,                    // Parameters
      0,                       // Priority (lowest)
      NULL                     // Task handle
    );
    if (taskCreated != pdPASS) {
      Serial.println("[RTOS] Failed to create independent control task");
    }
  }

  // Check if all FreeRTOS/WiFi/ESP-NOW initialization succeeded
  if (!initSuccess) {
    Serial.println("[RTOS] Initialization failed - halting system");
    while(1) { delay(1000); }  // Halt system indefinitely
  }
}

// Loop does nothing - all work done in FreeRTOS tasks
void loop() {
  delay(1);
}