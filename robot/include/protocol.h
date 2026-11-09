/*
  protocol.h - Binary framed wire protocol shared with
  q8botone-python/q8bot/protocol.py and the other firmware projects' copies of
  this file (controller/, camera/). KEEP THESE FOUR IN SYNC.

  [START=0xAA][MSG_TYPE][LENGTH][PAYLOAD...][CRC_LO][CRC_HI]

  CRC-16/ARC (poly 0x8005, reflected 0xA001, init 0x0000) computed over
  MSG_TYPE + LENGTH + PAYLOAD.
*/
#ifndef protocol_h
#define protocol_h

#include <Arduino.h>

// Wire protocol version, shared by host, controller and robot. Bump it for
// any change the other side has to know about: a new or changed command or
// reply, a payload layout, gait/direction IDs. Must equal protocol.py's
// PROTOCOL_VERSION. Not yet exchanged or checked on connect.
static const uint8_t PROTOCOL_VERSION = 5;  // 2: SENSOR_REQUEST / SENSOR_REPLY; 3: GAIT_RUN must be repeated;
                                            // 4: TELEMETRY_REPLY adds servo voltage, hardware errors, torque restores (80 bytes),
                                            //    POWER_RAIL, SERVO_REBOOT, LED_BRIGHTNESS's don't-save flag;
                                            // 5: the camera - message ranges, CAM_STATUS_REQUEST, CAM_WIFI_SET, CAM_STATUS

static const uint8_t FRAME_START = 0xAA;
static const size_t FRAME_HEADER_LEN = 2;   // msg_type, length
static const size_t FRAME_CRC_LEN = 2;
static const size_t FRAME_OVERHEAD = 1 + FRAME_HEADER_LEN + FRAME_CRC_LEN;  // start+header+crc
static const size_t FRAME_MAX_LEN = FRAME_OVERHEAD + 255;

// Who a message is for, by its type - the robot routes on this:
//   0x01-0x1F  commands for the robot   from the host over the radio, or from the camera
//   0x20-0x3F  commands for the camera  the robot passes them on to its camera link
//   0x81-0x9F  replies from the robot   back to whoever asked
//   0xA0-0xBF  replies from the camera  the robot passes them on to the host
inline bool isRobotCommand(uint8_t type)  { return type >= 0x01 && type <= 0x1F; }
inline bool isCameraCommand(uint8_t type) { return type >= 0x20 && type <= 0x3F; }
inline bool isRobotReply(uint8_t type)    { return type >= 0x81 && type <= 0x9F; }
inline bool isCameraReply(uint8_t type)   { return type >= 0xA0 && type <= 0xBF; }

// The camera link: a UART between the robot (TX1 IO2, RX1 IO3) and the camera
// (a XIAO ESP32S3: D7 in, D8 out), carrying these same frames both ways.
static const uint32_t CAMERA_LINK_BAUD = 460800;

enum class CmdType : uint8_t {          // host -> robot
  MOVE            = 0x01,
  TORQUE_SET      = 0x02,  // {mode: uint8} 0=off, 1=on (full stiffness), 2=damping (see q8Dynamixel::setJointMode)
  BATTERY_REQUEST = 0x03,
  RECORD_SEND     = 0x04,  // retired - recording removed; id kept reserved
  JUMP            = 0x05,
  GAIT_RUN        = 0x06,  // {gait_id: uint8, direction_id: uint8} - run a local gait/direction;
                           // repeat it while walking - the robot stops after 1s without one
  GAIT_STOP       = 0x07,  // no payload - stop autonomous gait, hand control back to raw MOVE
  GREET           = 0x08,  // no payload - scripted greeting sequence, runs entirely on the robot
  HOST_CONNECTED  = 0x09,  // no payload - controller-local only, never forwarded to the robot
  LED_BRIGHTNESS  = 0x0A,  // {brightness: uint8, 0-255[, flags: uint8]} - sets & persists the robot's
                           // RGB LED brightness; flags bit 0 = this once, don't persist it
  TELEMETRY_REQUEST = 0x0B,  // no payload - ask for one joint snapshot; answered with TELEMETRY_REPLY
  SENSOR_REQUEST  = 0x0C,  // no payload - sample the Qwiic sensors; answered with SENSOR_REPLY
  POWER_RAIL      = 0x0D,  // {rail: uint8 5|6, on: uint8} - switch a buck converter, for power
                           // measurements; 6V off cuts the servos' power, torque off until a restart
  SERVO_REBOOT    = 0x0E,  // {slot: uint8, 1-8} - restart one servo (DYNAMIXEL Reboot), clearing its
                           // hardware error; the robot puts back its profile, gain and torque

  // host or robot -> camera
  CAM_STATUS_REQUEST = 0x20,  // no payload - answered with CAM_STATUS
  CAM_WIFI_SET    = 0x21,  // {ssid_len: uint8, ssid, password_len: uint8, password} - the camera saves
                           // this network and joins it, then answers with CAM_STATUS. Taken only on
                           // the camera's own USB: the radio isn't encrypted, so a password never
                           // crosses it - the camera ignores one that comes over the robot link
};

// Canonical direction ordering for GAIT_RUN's direction_id (0-11). Must
// match q8botone-python/q8bot/protocol.py's DIRECTION_ORDER and the order
// gaitTables.h's *_DIRECTIONS arrays are built in (export_gait_tables.py):
// f=0, b=1, l=2, r=3, fl_0.75=4, fl_0.5=5, fr_0.75=6, fr_0.5=7,
// bl_0.75=8, bl_0.5=9, br_0.75=10, br_0.5=11

enum class ReplyType : uint8_t {        // robot -> host
  BATTERY_REPLY     = 0x81,  // {percent: uint8, millivolts: uint16}
  RECORD_DATA_REPLY = 0x82,  // retired alongside RECORD_SEND; kept reserved
  TELEMETRY_REPLY   = 0x83,  // one joint snapshot - see TelemetryPayload below
  SENSOR_REPLY      = 0x84,  // every detected sensor's reading - see SensorKind below

  // camera -> host or robot
  CAM_STATUS        = 0xA0,  // the camera's Wi-Fi - see CameraWifi below
};

// Payload of CAM_STATUS (little-endian):
//   uint8   protocol   the camera's PROTOCOL_VERSION
//   uint8   wifi       CameraWifi
//   uint8   reason     why the last attempt to join failed (an ESP-IDF
//                      wifi_err_reason_t, e.g. 201 network not found,
//                      202 authentication failed), 0 if none since the last join
//   int8    rssi       signal, dBm (0 unless connected)
//   uint8   ip[4]      its address (0.0.0.0 unless connected)
//   uint8   ssid_len   then that many bytes: the network it uses - never the password
enum class CameraWifi : uint8_t {
  NOT_SET    = 0,  // no network saved yet
  CONNECTING = 1,
  CONNECTED  = 2,
  FAILED     = 3,  // couldn't join - see reason; it keeps trying
};
static const size_t CAM_STATUS_HEADER_LEN = 9;   // up to and including ssid_len

// Payload of SENSOR_REPLY (little-endian, packed):
//   uint32  t_ms     robot millis() when sampled
//   uint8   count    sensor blocks that follow
//   count x block:
//     uint8    kind  SensorKind
//     uint8    addr  its I2C address - tells two of the same kind apart
//     uint8    n     number of values that follow
//     float32  values[n], in the order its SensorKind entry lists (NaN if
//              the sensor stopped answering)
// protocol.py's SENSOR_KINDS gives each kind's value names and units. A kind
// the host doesn't know still parses - n is in the block - so adding a kind
// never changes this format.
enum class SensorKind : uint8_t {
  INA237 = 1,   // bus voltage V, shunt voltage mV, current mA, power mW, die temperature degC
  MPU6050 = 2,  // accel x/y/z m/s^2, gyro x/y/z rad/s, temperature degC
};
static const size_t SENSOR_REPLY_HEADER_LEN = 5;  // t_ms + count
static const size_t SENSOR_BLOCK_HEADER_LEN = 3;  // kind + addr + n

// Payload of TELEMETRY_REPLY: one snapshot of all 8 joints, 80 bytes.
// Position and current are read fresh; temperature, voltage and hwError are
// whatever the robot last sampled (see TELEMETRY_TEMP_PERIOD_MS) since they
// live outside the contiguous 126-135 block and change over seconds, not
// milliseconds. Protocol 3 and earlier sent only the first 56 bytes; the first
// protocol 4 builds, 72 (no restored).
// Layout must match q8botone-python/q8bot/protocol.py's TELEMETRY_STRUCT.
struct __attribute__((packed)) TelemetryPayload {
  int32_t position[8];     // raw Dynamixel ticks
  int16_t current[8];      // mA, signed; reads ~0 with torque off
  int8_t  temperature[8];  // degrees C
  uint8_t voltage[8];      // the servo's supply, 0.1 V
  uint8_t hwError[8];      // Hardware Error Status (70): 0 = fine, else it has shut itself down
  uint8_t restored[8];     // times the torque watchdog turned it back on since boot (wraps at 255)
};
static_assert(sizeof(TelemetryPayload) == 80, "TelemetryPayload must stay 80 bytes");

inline uint16_t crc16Arc(const uint8_t* data, size_t len, uint16_t crc = 0x0000){
  for (size_t i = 0; i < len; i++){
    crc ^= data[i];
    for (int b = 0; b < 8; b++){
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

// Validates a frame already sitting in buf (bufCapacity is a safety bound,
// e.g. the fixed ESP-NOW payload buffer size - NOT necessarily the real
// frame length, since the frame is self-delimiting via its own LENGTH byte).
// Returns true and points *payload/*payloadLen at the frame's payload on
// success; leaves them untouched on failure (bad start byte, truncated
// buffer, or CRC mismatch).
inline bool parseFrameBuf(const uint8_t* buf, size_t bufCapacity,
                          uint8_t* msgType, const uint8_t** payload, uint8_t* payloadLen){
  if (bufCapacity < FRAME_OVERHEAD) return false;
  if (buf[0] != FRAME_START) return false;

  uint8_t type = buf[1];
  uint8_t length = buf[2];
  size_t totalLen = 3 + (size_t)length + FRAME_CRC_LEN;
  if (totalLen > bufCapacity) return false;

  uint16_t crcExpected = crc16Arc(&buf[1], 2 + length);
  uint16_t crcReceived = (uint16_t)buf[3 + length] | ((uint16_t)buf[3 + length + 1] << 8);
  if (crcExpected != crcReceived) return false;

  *msgType = type;
  *payload = &buf[3];
  *payloadLen = length;
  return true;
}

// Builds a complete frame into outBuf (must have capacity >= 3+payloadLen+2).
// Returns the total frame length written.
inline size_t buildFrame(uint8_t msgType, const uint8_t* payload, uint8_t payloadLen, uint8_t* outBuf){
  outBuf[0] = FRAME_START;
  outBuf[1] = msgType;
  outBuf[2] = payloadLen;
  memcpy(&outBuf[3], payload, payloadLen);
  uint16_t crc = crc16Arc(&outBuf[1], 2 + payloadLen);
  outBuf[3 + payloadLen] = crc & 0xFF;
  outBuf[3 + payloadLen + 1] = (crc >> 8) & 0xFF;
  return 3 + (size_t)payloadLen + 2;
}

// Total length of the frame at buf, from its LENGTH byte.
inline size_t frameLen(const uint8_t* buf) {
  return 3 + (size_t)buf[2] + FRAME_CRC_LEN;
}

// Collects frames from a byte stream such as a UART: feed() each byte as it
// arrives, and when it returns true, frame() holds a whole CRC-checked frame.
// A bad frame is dropped, and a frame left half-finished for FRAME_GAP_MS is
// abandoned, so noise or a dropped byte can't wedge it - it just waits for
// the next start byte.
class FrameReader {
 public:
  static const uint32_t FRAME_GAP_MS = 50;

  bool feed(uint8_t b) {
    uint32_t now = millis();
    if (_len && now - _lastByteMs > FRAME_GAP_MS) _len = 0;
    _lastByteMs = now;
    if (_len == 0 && b != FRAME_START) return false;
    _buf[_len++] = b;
    if (_len < 3 || _len < frameLen(_buf)) return false;
    _len = 0;
    uint8_t type; const uint8_t* payload; uint8_t payloadLen;
    return parseFrameBuf(_buf, sizeof(_buf), &type, &payload, &payloadLen);
  }

  bool busy() const { return _len != 0; }   // partway through a frame
  const uint8_t* frame() const { return _buf; }
  size_t length() const { return frameLen(_buf); }
  uint8_t type() const { return _buf[1]; }
  const uint8_t* payload() const { return _buf + 3; }
  uint8_t payloadLen() const { return _buf[2]; }

 private:
  uint8_t _buf[FRAME_MAX_LEN];
  size_t _len = 0;
  uint32_t _lastByteMs = 0;
};

#endif
