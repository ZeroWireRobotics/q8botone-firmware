#include "sensors.h"
#include <Wire.h>
#include <math.h>
#include <Adafruit_INA237.h>
#include <Adafruit_MPU6050.h>
#include "i2cBus.h"
#include "protocol.h"

namespace {

const uint8_t MAX_SENSORS = 8;
const uint8_t MAX_VALUES = 16;     // most values any one kind sends
const uint8_t MAX_BUS_DEVICES = 16;

// Every I2C read here assumes the caller holds an I2CBusLock.

// Read a big-endian 16-bit register and nothing else - for identifying a
// chip before handing it to a library whose begin() may reset or
// reconfigure whatever it finds.
bool readReg16(uint8_t addr, uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)2) != 2) return false;
  uint16_t high = Wire.read();
  uint16_t low = Wire.read();
  value = (high << 8) | low;
  return true;
}

bool readReg8(uint8_t addr, uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool answers(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// One detected sensor. sample() fills valueCount() floats in the order
// protocol.h's SensorKind lists for its kind - all NaN if the sensor has
// stopped answering, so the host sees a gap rather than a stale value.
class Sensor {
public:
  explicit Sensor(uint8_t addr) : addr(addr) {}
  virtual ~Sensor() {}
  virtual SensorKind kind() const = 0;
  virtual const char* name() const = 0;
  virtual uint8_t valueCount() const = 0;
  virtual const char* const* valueLabels() const = 0;  // serial printing only

  void sample(float* values) {
    if (answers(addr)) {
      read(values);
    } else {
      for (uint8_t i = 0; i < valueCount(); i++) values[i] = NAN;
    }
  }

  const uint8_t addr;

protected:
  virtual void read(float* values) = 0;
};

class INA237Sensor : public Sensor {
public:
  using Sensor::Sensor;

  // TI's manufacturer ID, then the INA237's device ID (which the INA238
  // shares - same registers, so it's handled the same).
  static bool identify(uint8_t addr) {
    uint16_t mfg, dev;
    return readReg16(addr, 0x3E, mfg) && mfg == 0x5449 &&
           readReg16(addr, 0x3F, dev) && (dev >> 4) == 0x238;
  }

  bool begin() {
    if (!_ina.begin(addr, &Wire)) return false;
    _ina.setShunt(SHUNT_OHMS, MAX_CURRENT_A);
    return true;
  }

  SensorKind kind() const override { return SensorKind::INA237; }
  const char* name() const override { return "INA237"; }
  uint8_t valueCount() const override { return 5; }
  const char* const* valueLabels() const override {
    static const char* const labels[] = {"bus V", "shunt mV", "current mA", "power mW", "die C"};
    return labels;
  }

protected:
  void read(float* v) override {
    v[0] = _ina.readBusVoltage();
    v[1] = _ina.readShuntVoltage() * 1000.0f;  // the INA237 override returns V, not the base class's mV
    v[2] = _ina.readCurrent();
    v[3] = _ina.readPower();
    v[4] = _ina.readDieTemp();
  }

private:
  // Adafruit's INA237 breakout (their example uses the same values).
  static constexpr float SHUNT_OHMS = 0.015f;
  static constexpr float MAX_CURRENT_A = 10.0f;
  Adafruit_INA237 _ina;
};

class MPU6050Sensor : public Sensor {
public:
  using Sensor::Sensor;

  // WHO_AM_I. Genuine MPU6050s (Adafruit's included) answer 0x68; look-alike
  // chips sold under the name (MPU6500 0x70, MPU9250 0x71) aren't the same
  // part, and the library refuses them too.
  static bool identify(uint8_t addr) {
    uint8_t whoAmI;
    return readReg8(addr, 0x75, whoAmI) && whoAmI == 0x68;
  }

  bool begin() {
    if (!_mpu.begin(addr, &Wire)) return false;
    // +/-8 g rather than the library's 2 g, which a jump or a landing
    // clips; 500 deg/s is the library's own default. A 44 Hz low-pass
    // takes the edge off servo vibration without hiding real motion.
    _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    _mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    return true;
  }

  SensorKind kind() const override { return SensorKind::MPU6050; }
  const char* name() const override { return "MPU6050"; }
  uint8_t valueCount() const override { return 7; }
  const char* const* valueLabels() const override {
    static const char* const labels[] = {"ax", "ay", "az", "gx", "gy", "gz", "temp C"};
    return labels;
  }

protected:
  void read(float* v) override {
    sensors_event_t accel, gyro, temp;
    _mpu.getEvent(&accel, &gyro, &temp);   // one burst read of all three
    v[0] = accel.acceleration.x;
    v[1] = accel.acceleration.y;
    v[2] = accel.acceleration.z;
    v[3] = gyro.gyro.x;
    v[4] = gyro.gyro.y;
    v[5] = gyro.gyro.z;
    v[6] = temp.temperature;
  }

private:
  Adafruit_MPU6050 _mpu;
};

Sensor* tryMPU6050(uint8_t addr) {
  if (!MPU6050Sensor::identify(addr)) return nullptr;
  MPU6050Sensor* sensor = new MPU6050Sensor(addr);
  if (!sensor->begin()) {
    delete sensor;
    return nullptr;
  }
  return sensor;
}

Sensor* tryINA237(uint8_t addr) {
  if (!INA237Sensor::identify(addr)) return nullptr;
  INA237Sensor* sensor = new INA237Sensor(addr);
  if (!sensor->begin()) {
    delete sensor;
    return nullptr;
  }
  return sensor;
}

// Each supported kind: the address range it can be strapped to, and how to
// confirm and start one found there. Only addresses that answered the bus
// scan are tried.
struct Probe {
  uint8_t firstAddr, lastAddr;
  Sensor* (*tryAt)(uint8_t addr);
};
const Probe PROBES[] = {
  {0x40, 0x4F, tryINA237},
  {0x68, 0x69, tryMPU6050},   // AD0 low / high
};

Sensor* sensors[MAX_SENSORS];
uint8_t sensorCount = 0;
uint8_t busDevices[MAX_BUS_DEVICES];
uint8_t busDeviceCount = 0;

void printBusScan(SensorPrintFn print) {
  char line[128];
  int len = snprintf(line, sizeof(line), "[I2C] Devices on the bus:");
  for (uint8_t i = 0; i < busDeviceCount && len < (int)sizeof(line) - 6; i++) {
    len += snprintf(line + len, sizeof(line) - len, " 0x%02X", busDevices[i]);
  }
  if (busDeviceCount == 0) snprintf(line + len, sizeof(line) - len, " none");
  print(line);
}

}  // namespace

void sensorsBegin(SensorPrintFn print) {
  I2CBusLock lock;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (answers(addr) && busDeviceCount < MAX_BUS_DEVICES) busDevices[busDeviceCount++] = addr;
  }
  printBusScan(print);

  char line[64];
  for (const Probe& probe : PROBES) {
    for (uint8_t i = 0; i < busDeviceCount && sensorCount < MAX_SENSORS; i++) {
      uint8_t addr = busDevices[i];
      if (addr < probe.firstAddr || addr > probe.lastAddr) continue;
      Sensor* sensor = probe.tryAt(addr);
      if (sensor == nullptr) continue;
      sensors[sensorCount++] = sensor;
      snprintf(line, sizeof(line), "[SENSOR] %s at 0x%02X", sensor->name(), addr);
      print(line);
    }
  }
  if (sensorCount == 0) print("[SENSOR] No supported sensors found");
}

size_t sensorsBuildReply(uint8_t* out, size_t maxLen) {
  if (maxLen < SENSOR_REPLY_HEADER_LEN) return 0;
  uint32_t nowMs = millis();
  memcpy(out, &nowMs, sizeof(nowMs));   // the ESP32 is little-endian, like the wire format
  size_t len = SENSOR_REPLY_HEADER_LEN;
  uint8_t count = 0;
  float values[MAX_VALUES];

  I2CBusLock lock;   // one lock for the whole snapshot
  for (uint8_t i = 0; i < sensorCount; i++) {
    Sensor* sensor = sensors[i];
    uint8_t n = sensor->valueCount();
    if (len + SENSOR_BLOCK_HEADER_LEN + n * sizeof(float) > maxLen) break;
    sensor->sample(values);
    out[len++] = (uint8_t)sensor->kind();
    out[len++] = sensor->addr;
    out[len++] = n;
    memcpy(out + len, values, n * sizeof(float));
    len += n * sizeof(float);
    count++;
  }
  out[4] = count;
  return len;
}

void sensorsPrint(SensorPrintFn print) {
  printBusScan(print);
  if (sensorCount == 0) {
    print("[SENSOR] No supported sensors found (they're detected at boot only)");
    return;
  }

  uint8_t payload[128];
  uint32_t startUs = micros();
  size_t len = sensorsBuildReply(payload, sizeof(payload));
  uint32_t sampleUs = micros() - startUs;

  // Labeled values, decoded straight from the payload just built.
  char line[128];
  size_t pos = SENSOR_REPLY_HEADER_LEN;
  for (uint8_t i = 0; i < payload[4]; i++) {
    uint8_t kind = payload[pos], addr = payload[pos + 1], n = payload[pos + 2];
    pos += SENSOR_BLOCK_HEADER_LEN;
    const char* const* labels = sensors[i]->valueLabels();
    int lineLen = snprintf(line, sizeof(line), "[SENSOR] %s (kind %u) at 0x%02X:",
                           sensors[i]->name(), kind, addr);
    for (uint8_t v = 0; v < n && lineLen < (int)sizeof(line) - 24; v++) {
      float value;
      memcpy(&value, payload + pos + v * sizeof(float), sizeof(float));
      lineLen += snprintf(line + lineLen, sizeof(line) - lineLen, " %s=%.4f", labels[v], value);
    }
    print(line);
    pos += n * sizeof(float);
  }

  // The raw SENSOR_REPLY payload, for checking a host-side parser against -
  // BYTES_PER_LINE at a time, since a print line holds under 128 chars.
  const size_t BYTES_PER_LINE = 40;
  snprintf(line, sizeof(line), "[SENSOR] payload (%u bytes), sampled in %lu us (I2C bus held throughout):",
           (unsigned)len, (unsigned long)sampleUs);
  print(line);
  for (size_t start = 0; start < len; start += BYTES_PER_LINE) {
    int lineLen = snprintf(line, sizeof(line), "[SENSOR]   ");
    for (size_t i = start; i < len && i < start + BYTES_PER_LINE; i++) {
      lineLen += snprintf(line + lineLen, sizeof(line) - lineLen, "%02X", payload[i]);
    }
    print(line);
  }
}
