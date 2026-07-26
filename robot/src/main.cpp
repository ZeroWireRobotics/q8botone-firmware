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
#include "powerManager.h"
#include "RGBManager.h"
#include "battery.h"
#include "protocol.h"

// Initialize global objects
esp_now_peer_info_t peerInfo;
HardwareSerial          ser(0);
Dynamixel2Arduino       q8dxl(ser, DXL_DIR_PIN);
q8Dynamixel             q8(q8dxl);
bool started = false;  // Track robot start state
macStorage storage;
PowerManager            power(LATCH_PIN, SENSE_PIN, LED_PIN);
RGBManager              rgb(RGB_PIN);

// FreeRTOS Handles
QueueHandle_t rxQueue = NULL;
QueueHandle_t debugQueue = NULL;
EventGroupHandle_t eventGroup = NULL;

// Robot State
volatile RobotState robotState = STATE_UNPAIRED;


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

void unpair() {
  queuePrint(MSG_INFO, "[HEARTBEAT] Connection lost - returning to pairing mode\n");

  esp_now_del_peer(clientMac);
  storage.clearPeerMAC();
  queuePrint(MSG_DEBUG, "[STORAGE] Cleared controller MAC from EEPROM\n");
  memset(clientMac, 0, sizeof(clientMac));
  paired = false;
  started = false;

  // Update robot state and event group
  robotState = STATE_UNPAIRED;
  xEventGroupClearBits(eventGroup, EVENT_PAIRED | EVENT_STARTED);
  xEventGroupSetBits(eventGroup, EVENT_UNPAIRED);
}


void addElementToArray(uint16_t*& array, size_t& currentSize, uint16_t newElement) {
    // Allocate a new array with one extra element
    uint16_t* newArray = new uint16_t[currentSize + 1];
    newArray[0] = newElement;
    memcpy(newArray + 1, array, currentSize * sizeof(uint16_t));
    // Delete the old array (free the memory)
    delete[] array;
    // Update the original pointer and size
    array = newArray;
    currentSize++;
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
      if (msgType == PAIRING && !paired) {
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
            case 1: {
              // Send battery level
              queuePrint(MSG_DEBUG, "[DATA] Send battery level\n");
              uint8_t percent = (uint8_t)batteryGetPercent();
              buildFrame((uint8_t)ReplyType::BATTERY_REPLY, &percent, 1, (uint8_t*)myMsg.data);
              esp_now_send(clientMac, (uint8_t*)&myMsg, sizeof(myMsg));
              break;
            }

            case 2: {
              // Sync read position data
              uint16_t* posArray = q8.syncRead();
              if (rData != nullptr) {
                // If rData is not nullptr, resize posArray to append new data
                uint16_t* newData = new uint16_t[masterSize + smallerSize];
                if (newData == nullptr) {
                  queuePrint(MSG_DEBUG, "[ERROR] Memory allocation failed, dropping data\n");
                  delete[] posArray;
                  break;
                }
                memcpy(newData, rData, masterSize * sizeof(uint16_t));
                for (size_t j = 0; j < smallerSize; ++j) {
                  newData[masterSize + j] = posArray[j];
                }
                delete[] rData;
                rData = newData;
                masterSize += smallerSize;
              } else {
                // If rData is nullptr, initialize it for the first time
                rData = new uint16_t[smallerSize];
                if (rData == nullptr) {
                  queuePrint(MSG_DEBUG, "[ERROR] Memory allocation failed\n");
                  delete[] posArray;
                  break;
                }
                for (size_t j = 0; j < smallerSize; ++j) {
                  rData[j] = posArray[j];
                }
                masterSize = smallerSize;
              }
              delete[] posArray;
              break;
            }

            case 3: {
              // Send all recorded data in chunks, each wrapped in its own
              // RECORD_DATA_REPLY frame.
              if (rData != nullptr) {
                queuePrint(MSG_DEBUG, "[DATA] Sending %d recorded data points\n", masterSize);

                size_t totalSize = masterSize;  // total data in rData
                // Chunk size in uint16 elements - leaves room for the 5-byte
                // frame overhead inside the 200-byte myMsg.data buffer.
                size_t chunkSize = 97;
                size_t offset = 0;              // To track the position in rData

                // Loop to send the data in chunks
                while (offset < totalSize) {
                  // If rData is less than chunk size then use its size instead.
                  size_t currentChunkSize = (totalSize - offset < chunkSize) ? (totalSize - offset) : chunkSize;
                  buildFrame((uint8_t)ReplyType::RECORD_DATA_REPLY,
                             (const uint8_t*)&rData[offset],
                             (uint8_t)(currentChunkSize * sizeof(uint16_t)),
                             (uint8_t*)myMsg.data);
                  // Send the chunk via ESP-NOW
                  esp_now_send(clientMac, (uint8_t*)&myMsg, sizeof(myMsg));
                  offset += currentChunkSize;
                }
                delete[] rData;
                rData = nullptr;
                masterSize = 0;
              }
              break;
            }
          }
        }
      }
    }
  }
}

// FreeRTOS Task: Heartbeat Monitor (Priority 2)
void heartbeatMonitorTask(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    // Only monitor heartbeat when paired
    if (paired) {
      unsigned long now = millis();

      // Avoid underflow: only check timeout if now >= lastHeartbeatReceived
      if (now >= lastHeartbeatReceived) {
        unsigned long timeSinceLastMsg = now - lastHeartbeatReceived;

        // Check for timeout (only in auto-pairing mode)
#ifndef PERMANENT_PAIRING_MODE
        if (timeSinceLastMsg > HEARTBEAT_TIMEOUT_ROBOT) {
          queuePrint(MSG_DEBUG, "[HEARTBEAT] Timeout detected (%lums since last message)\n", timeSinceLastMsg);
          q8.toggleTorque(0);        // Disable torque hardware
          q8.resetTorqueState();     // Sync internal flag to match disabled state
          unpair();
        }
#endif
      }
    }

    // Check every 1 second
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
  }
}

// FreeRTOS Task: Robot State Manager (Priority 1)
void robotStateTask(void* parameter) {
  TickType_t lastStateChange = xTaskGetTickCount();
  unsigned long lastActivity = 0;
  bool firstRun = true;  // Check saved MAC on first iteration
  uint16_t hue = 0;      // Tracks rainbow position for STATE_UNPAIRED

  // Battery monitoring state
  // Stages: 3 = normal, 2 = low (<20%), 1 = critical (<10%), 0 = stop (<5%)
  uint8_t batteryStage = 3;
  bool jumpEnabledPrev = true;  // Tracks last setJumpEnabled() call - reversible, unlike batteryStage
  unsigned long lastBatteryCheck = 0;

  // Give other tasks time to start (especially serialOutputTask)
  vTaskDelay(pdMS_TO_TICKS(100));

  // Backup demo cycle: each double-tap (while torque is off) runs exactly
  // one of these, advancing to the next for the following double-tap.
  const uint8_t BACKUP_DEMO_COUNT = 3;
  uint8_t backupDemoIndex = 0;

  while (true) {
    // On first run, check for saved MAC address
    if (firstRun) {
      firstRun = false;
      if (storage.loadPeerMAC(clientMac)) {
        queuePrint(MSG_DEBUG, "[PAIRING] Found saved controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   clientMac[0], clientMac[1], clientMac[2], clientMac[3], clientMac[4], clientMac[5]);
        addPeer(clientMac);
        paired = true;
        robotState = STATE_PAIRED;
        lastHeartbeatReceived = millis();
        xEventGroupClearBits(eventGroup, EVENT_UNPAIRED);
        xEventGroupSetBits(eventGroup, EVENT_PAIRED);
        queuePrint(MSG_INFO, "[PAIRING] Attempting to reconnect to saved controller\n");
      } else {
        queuePrint(MSG_INFO, "[PAIRING] No saved MAC found - waiting for pairing request\n");
      }
    }

    unsigned long now = millis();
    RobotState currentState = robotState;

    // Backup demo trigger: double-tap the power button while torque is off
    // (i.e. no host is actively driving the robot) to run exactly one demo
    // move, cycling to the next on each subsequent double-tap. Ignored
    // entirely if a host is connected, so it can never fight a live
    // teleop session.
    if (power.consumeDoubleTap() && !q8.isTorqueOn()) {
      q8.toggleTorque(true);
      q8.resetTorqueState(true);
      q8.moveToIdle(500);
      delay(600);       // let the move-to-neutral finish before the demo move

      switch (backupDemoIndex) {
        case 0:
          queuePrint(MSG_INFO, "[BACKUP DEMO] Double-tap detected, torque off - trotting\n");
          q8.trotDemo();
          break;
        case 1:
          queuePrint(MSG_INFO, "[BACKUP DEMO] Double-tap detected, torque off - jumping\n");
          q8.jump();
          break;
        case 2:
          queuePrint(MSG_INFO, "[BACKUP DEMO] Double-tap detected, torque off - greeting\n");
          q8.greet();
          break;
      }
      backupDemoIndex = (backupDemoIndex + 1) % BACKUP_DEMO_COUNT;

      q8.moveToIdle(500);
      delay(600);
      q8.toggleTorque(false);
      q8.resetTorqueState(false);
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
    }

    switch (currentState) {
      case STATE_UNPAIRED: {
        // Rainbow cycle on RGB LED every loop tick (~100ms per step)
        rgb.setHue(hue);
        hue += 512;  // Full cycle in ~13s

        // Slow blink every 2 seconds
        if (now - lastActivity >= 2000) {
          lastActivity = now;
          digitalWrite(LED_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(200));
          digitalWrite(LED_PIN, LOW);
          queuePrint(MSG_DEBUG, "[PAIRING] Waiting for pairing...\n");
        }
        break;
      }

      case STATE_PAIRED: {
        // Solid blue once heartbeats are actively coming in from the
        // controller; brief blink while paired but not yet hearing them
        // (e.g. right after pairing, or if the link has gone quiet). A
        // blink (not a sine breathe) survives the ~1s vTaskDelay block
        // below without visibly stuttering.
        bool heartbeatAlive = (now >= lastHeartbeatReceived) &&
                               (now - lastHeartbeatReceived <= HEARTBEAT_TIMEOUT_ROBOT);
        if (heartbeatAlive) {
          rgb.setColor(0, 0, 255);
        } else {
          bool blinkOn = (now % 2000) < 500;
          rgb.setColor(0, 0, blinkOn ? 255 : 0);
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
#ifdef PERMANENT_PAIRING_MODE
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
      }
    }
#endif

    // Low priority - yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


// ============================================================================
// Arduino Setup. Configures FreeRTOS tasks, ESP-NOW, and other hardware.
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Power latch — must run before anything else
  while (!power.begin()) {
    // Serial.println("[POWER] Button released early - hold for 3s to power on");
  }
  Serial.println("[POWER] Latched on");
  rgb.begin();

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

  // Create heartbeat monitor task (Priority 2)
  taskCreated = xTaskCreate(
    heartbeatMonitorTask, // Task function
    "HeartbeatMonitor",   // Task name
    2048,                 // Stack size (bytes)
    NULL,                 // Parameters
    2,                    // Priority (medium - monitors connection)
    NULL                  // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create heartbeat monitor task");
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

  // Check if all FreeRTOS initialization succeeded
  if (!initSuccess) {
    Serial.println("[RTOS] Initialization failed - halting system");
    while(1) { delay(1000); }  // Halt system indefinitely
  }

  // Init Wi-Fi
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("esp-server", nullptr, chan); // Optional, just to enable softAP mode
  chan = WiFi.channel();
  WiFi.softAPmacAddress(serverMac);

  // Init ESP-NOW (after FreeRTOS primitives are ready)
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onRecv);  // Set up callback when data is received.

  // BQ25887 Init
  Wire.begin();
  batteryInit();

  // Start power button monitor task (must be after all xTaskCreate calls)
  power.startMonitorTask();

  // Initialize Dynamixel object
  q8.begin();
  q8.attachRGB(&rgb);
}

// Loop does nothing - all work done in FreeRTOS tasks
void loop() {
  delay(1);
}
