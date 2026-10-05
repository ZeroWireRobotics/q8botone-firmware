/*
  protocol.h - Binary framed wire protocol shared with
  q8botone-python/q8bot/protocol.py and the other firmware project's copy of
  this file. KEEP THESE THREE IN SYNC.

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
static const uint8_t PROTOCOL_VERSION = 2;  // 2: SENSOR_REQUEST / SENSOR_REPLY

static const uint8_t FRAME_START = 0xAA;
static const size_t FRAME_HEADER_LEN = 2;   // msg_type, length
static const size_t FRAME_CRC_LEN = 2;
static const size_t FRAME_OVERHEAD = 1 + FRAME_HEADER_LEN + FRAME_CRC_LEN;  // start+header+crc

enum class CmdType : uint8_t {          // host -> robot
  MOVE            = 0x01,
  TORQUE_SET      = 0x02,  // {mode: uint8} 0=off, 1=on (full stiffness), 2=damping (see q8Dynamixel::setJointMode)
  BATTERY_REQUEST = 0x03,
  RECORD_SEND     = 0x04,  // retired - recording removed; id kept reserved
  JUMP            = 0x05,
  GAIT_RUN        = 0x06,  // {gait_id: uint8, direction_id: uint8} - run a local gait/direction
  GAIT_STOP       = 0x07,  // no payload - stop autonomous gait, hand control back to raw MOVE
  GREET           = 0x08,  // no payload - scripted greeting sequence, runs entirely on the robot
  HOST_CONNECTED  = 0x09,  // no payload - controller-local only, never forwarded to the robot
  LED_BRIGHTNESS  = 0x0A,  // {brightness: uint8, 0-255} - sets & persists the robot's RGB LED brightness
  TELEMETRY_REQUEST = 0x0B,  // no payload - ask for one joint snapshot; answered with TELEMETRY_REPLY
  SENSOR_REQUEST  = 0x0C,  // no payload - sample the Qwiic sensors; answered with SENSOR_REPLY
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
};

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

// Payload of TELEMETRY_REPLY: one snapshot of all 8 joints, 56 bytes.
// Position and current are read fresh; temperature is whatever the robot
// last sampled (see TELEMETRY_TEMP_PERIOD_MS) since it lives outside the
// contiguous 126-135 block and changes over seconds, not milliseconds.
// Layout must match q8botone-python/q8bot/protocol.py's TELEMETRY_STRUCT.
struct __attribute__((packed)) TelemetryPayload {
  int32_t position[8];     // raw Dynamixel ticks
  int16_t current[8];      // mA, signed; reads ~0 with torque off
  int8_t  temperature[8];  // degrees C
};
static_assert(sizeof(TelemetryPayload) == 56, "TelemetryPayload must stay 56 bytes");

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

#endif
