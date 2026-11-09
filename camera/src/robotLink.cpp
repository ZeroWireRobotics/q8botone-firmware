#include "robotLink.h"

#include "wifiSetup.h"

static HardwareSerial& robotSerial = Serial1;
const int ROBOT_RX_PIN = D7;   // GPIO44, from the robot's TX1
const int ROBOT_TX_PIN = D8;   // GPIO7, to the robot's RX1

enum class From : uint8_t { USB, ROBOT };

static const char* firmwareId = "";
static FrameReader usbReader, robotReader;

static void send(From to, uint8_t type, const uint8_t* payload, uint8_t len) {
  static const uint8_t none = 0;
  if (!payload) payload = &none;
  uint8_t frame[FRAME_MAX_LEN];
  size_t n = buildFrame(type, payload, len, frame);
  if (to == From::USB) {
    Serial.write(frame, n);
  } else {
    robotSerial.write(frame, n);
  }
}

static void sendStatus(From to) {
  uint8_t payload[CAM_STATUS_HEADER_LEN + 32];
  size_t len = wifiStatusPayload(payload, sizeof(payload));
  send(to, (uint8_t)ReplyType::CAM_STATUS, payload, (uint8_t)len);
}

void sendToRobot(CmdType type, const uint8_t* payload, uint8_t len) {
  send(From::ROBOT, (uint8_t)type, payload, len);
}

// The robot's replies to sendToRobot()'s commands. Printed for now - camera
// code that drives the robot takes them from here.
static void onRobotReply(uint8_t type, const uint8_t* payload, uint8_t len) {
  if (type == (uint8_t)ReplyType::BATTERY_REPLY && len >= 3) {
    uint16_t millivolts = payload[1] | (payload[2] << 8);
    Serial.printf("[robot] Battery %u%%, %.2f V\n", payload[0], millivolts / 1000.0f);
  } else {
    Serial.printf("[robot] Reply 0x%02X, %u bytes\n", type, len);
  }
}

// {ssid_len, ssid, password_len, password}
static void setWifi(const uint8_t* p, uint8_t len) {
  if (len < 2 || len < 2 + p[0] || len != 2 + p[0] + p[1 + p[0]]) {
    Serial.println("[wifi] CAM_WIFI_SET malformed - kept the old network");
    return;
  }
  uint8_t ssidLen = p[0];
  if (!wifiSet(p + 1, ssidLen, p + 2 + ssidLen, p[1 + ssidLen])) {
    Serial.println("[wifi] CAM_WIFI_SET invalid (SSID 1-32 bytes, password empty or 8-64) - kept the old network");
  }
}

static void handleFrame(const FrameReader& frame, From from) {
  uint8_t type = frame.type();
  if (from == From::ROBOT && isRobotReply(type)) {
    onRobotReply(type, frame.payload(), frame.payloadLen());
    return;
  }
  switch ((CmdType)type) {
    case CmdType::CAM_STATUS_REQUEST:
      sendStatus(from);
      break;
    case CmdType::CAM_WIFI_SET:
      if (from != From::USB) {
        Serial.println("[wifi] Ignored a network sent through the robot - it's only taken over USB");
        break;
      }
      setWifi(frame.payload(), frame.payloadLen());
      sendStatus(from);
      break;
    default:
      // A command for the camera this build doesn't know - newer ones, such
      // as moving a servo of the camera's own, go in this switch.
      break;
  }
}

void linkBegin(const char* id) {
  firmwareId = id;
  robotSerial.setRxBufferSize(4096);
  robotSerial.begin(CAMERA_LINK_BAUD, SERIAL_8N1, ROBOT_RX_PIN, ROBOT_TX_PIN);
}

void linkPoll() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (usbReader.busy() || b == FRAME_START) {
      if (usbReader.feed(b)) handleFrame(usbReader, From::USB);
    } else if (b == 'i') {
      Serial.println(firmwareId);
    } else if (b == 'b' || b == 't' || b == 's') {
      CmdType ask = b == 'b' ? CmdType::BATTERY_REQUEST
                  : b == 't' ? CmdType::TELEMETRY_REQUEST
                             : CmdType::SENSOR_REQUEST;
      Serial.printf("[robot] Asking for its %s\n", b == 'b' ? "battery" : b == 't' ? "joints" : "sensors");
      sendToRobot(ask, nullptr, 0);
    }
  }
  while (robotSerial.available()) {
    if (robotReader.feed(robotSerial.read())) handleFrame(robotReader, From::ROBOT);
  }
}
