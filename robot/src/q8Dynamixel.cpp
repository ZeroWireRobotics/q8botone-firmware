/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  Released into the public domain.
*/

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <q8Dynamixel.h>
#include <RGBManager.h>
#include <protocol.h>
#include <gaitRunner.h>
#include <macStorage.h>

using namespace ControlTableItem;

// Constructor that takes an object of type Dynamixel2Arduino as an argument
q8Dynamixel::q8Dynamixel(Dynamixel2Arduino& dxl) : _dxl(dxl) {
  // Constructor implementation
  // Initialize any other members if needed
}

void q8Dynamixel::attachRGB(RGBManager* rgb){
  _rgb = rgb;
}

void q8Dynamixel::attachGaitRunner(GaitRunner* gaitRunner){
  _gaitRunner = gaitRunner;
}

void q8Dynamixel::attachStorage(macStorage* storage){
  _storage = storage;
}

bool q8Dynamixel::lockBus(TickType_t timeout){
  // Before begin() there is only one task, so an uncreated lock is not a
  // failure - it just means there is nothing to contend with yet.
  if (_busLock == nullptr) return true;
  return xSemaphoreTakeRecursive(_busLock, timeout) == pdTRUE;
}

void q8Dynamixel::unlockBus(){
  if (_busLock != nullptr) xSemaphoreGiveRecursive(_busLock);
}

// Scope guard so an early return can't leave the bus locked.
namespace {
  class BusGuard {
    public:
      explicit BusGuard(q8Dynamixel& q8) : _q8(q8) { _q8.lockBus(); }
      ~BusGuard() { _q8.unlockBus(); }
    private:
      q8Dynamixel& _q8;
  };
}

void q8Dynamixel::begin(){
  _busLock = xSemaphoreCreateRecursiveMutex();

  // The two calls below are the only unguarded bus access in this file, and
  // deliberately so: begin() runs from setup(), before any task exists, so
  // there is nothing to contend with. Everything after startup goes through
  // lockBus() - see the header.
  _dxl.begin(_baudrate);
  _dxl.setPortProtocolVersion(_protocolVersion);
  setOpMode();

  // Fill the members of structure for bulkWrite using internal packet buffer
  _bw_infos.packet.p_buf = nullptr;
  _bw_infos.packet.is_completed = false;
  _bw_infos.p_xels = _info_xels_bw;
  _bw_infos.xel_count = 0;

  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = 0;
    _info_xels_bw[i].id = _DXL[i];
    _info_xels_bw[i].addr = 116; // Goal Position of X serise.
    _info_xels_bw[i].addr_length = 4; // Goal Position
    _info_xels_bw[i].p_data = reinterpret_cast<uint8_t*>(&_bw_data_xel[i]);
    _bw_infos.xel_count++;
  }
  _bw_infos.is_info_changed = true;

  // Fill the members of structure to fastSyncRead using external user packet buffer
  _sr_infos.packet.buf_capacity = _user_pkt_buf_cap;
  _sr_infos.packet.p_buf = _user_pkt_buf;
  _sr_infos.packet.is_completed = false;
  _sr_infos.addr = SR_START_ADDR;
  _sr_infos.addr_length = SR_ADDR_LEN;
  _sr_infos.p_xels = _info_xels_sr;
  _sr_infos.xel_count = 0;  

  for(int i=0; i < _idCount; i++){
    _info_xels_sr[i].id = _DXL[i];;
    _info_xels_sr[i].p_recv_buf = (uint8_t*)&_sr_data[i];
    _sr_infos.xel_count++;
  }
  _sr_infos.is_info_changed = true;

  setProfile(1000);
  expandArrays();
}

bool q8Dynamixel::checkComms(uint8_t ID){
  BusGuard guard(*this);
  return _dxl.ping(ID);
}

bool q8Dynamixel::commStart(){
  // Replace this with an actual check of ESPNow comms later
  return _torqueFlag;
}

uint16_t q8Dynamixel::checkBattery(){
  return 1;
}

void q8Dynamixel::enableTorque(){
  BusGuard guard(*this);
  _dxl.torqueOn(BROADCAST_ID);
}

void q8Dynamixel::disableTorque(){
  BusGuard guard(*this);
  _dxl.torqueOff(BROADCAST_ID);
}

void q8Dynamixel::toggleTorque(bool flag){
  if(flag){
    enableTorque();
  } else{
    disableTorque();
  }
}

void q8Dynamixel::resetTorqueState(bool torqueOn){
  // Reset the internal torque flag to match the given hardware state
  // Used after connection loss when torque was already disabled
  _torqueFlag = torqueOn;
}

bool q8Dynamixel::isTorqueOn() const{
  return _torqueFlag;
}

void q8Dynamixel::setOpMode(){
  BusGuard guard(*this);
  // Set operating mode. Torque off first if needed.
  if (!_torqueFlag){
    for (int i = 0; i < _idCount; i++){
      _dxl.setOperatingMode(_DXL[i], OP_EXTENDED_POSITION);
    }
  } else{
    disableTorque();
    for (int i = 0; i < _idCount; i++){
      _dxl.setOperatingMode(_DXL[i], OP_EXTENDED_POSITION);
    }
    enableTorque();
  }
}

void q8Dynamixel::setProfile(uint16_t dur){
  BusGuard guard(*this);
  setGain(400);
  for (int i = 0; i < _idCount; i++){
    _dxl.writeControlTableItem(PROFILE_VELOCITY, _DXL[i], dur);
    _dxl.writeControlTableItem(PROFILE_ACCELERATION, _DXL[i], dur / 3);
  }
}

void q8Dynamixel::setGain(uint16_t p_gain){
  BusGuard guard(*this);
  // for Time-based Extended Pos, Profile velocity is the move duration (ms).
  for (int i = 0; i < _idCount; i++){
    _dxl.writeControlTableItem(POSITION_P_GAIN, _DXL[i], p_gain);
  }
}

void q8Dynamixel::moveSingle(int32_t val){
  BusGuard guard(*this);
  // 8 motors move to the same position
  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = val;
  }
  _bw_infos.is_info_changed = true;

  _dxl.bulkWrite(&_bw_infos);
}

void q8Dynamixel::bulkWrite(int32_t values[8]){
  BusGuard guard(*this);
  // 8 motors move to their respective positions
  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = values[i];
  }
  _bw_infos.is_info_changed = true;

  _dxl.bulkWrite(&_bw_infos);
}

void q8Dynamixel::bulkWriteDeg(const float degs[8]){
  int32_t ticks[8];
  for (int i = 0; i < 8; i++){
    ticks[i] = _deg2Dxl(degs[i]);
  }
  bulkWrite(ticks);
}

void q8Dynamixel::moveToIdle(uint16_t dur){
  BusGuard guard(*this);  // held across the delay so the profile can't change under us
  setProfile(dur);
  delay(100);
  bulkWrite(_idleArray);
}

uint16_t* q8Dynamixel::syncRead(){
  BusGuard guard(*this);
  // Read relevant registers from all joints into a single array
  int recv_cnt;
  size_t offset = 0;
  uint16_t* byteArray = new uint16_t[_idCount * 2];

  recv_cnt = _dxl.fastSyncRead(&_sr_infos);
  if(recv_cnt = _idCount){
    for (size_t i = 0; i < _idCount; i++){
      // cast to uint16_t since values never exceed 65535 in robot configuration
      byteArray[i*2] = static_cast<uint16_t>(_sr_data[i].present_current + 10000);
      byteArray[i*2+1] = static_cast<uint16_t>(_sr_data[i].present_position);
    }
  } else {
    // Fill ByteArray with zeros
    for (size_t i = 0; i < _idCount*2; ++i){
      byteArray[i] = 0;
    }
  }

  // for (size_t i = 0; i < 4; ++i){
  //   Serial.print(byteArray[i]); Serial.print(" ");
  // } Serial.println();
  return byteArray;
}

void q8Dynamixel::setJumpEnabled(bool enabled) {
  _jumpEnabled = enabled;
}

void q8Dynamixel::warnJumpBlocked() {
  if (!_rgb) return;
  for (int i = 0; i < 3; ++i) {
    _rgb->setColor(255, 0, 0);
    delay(120);
    _rgb->off();
    delay(120);
  }
  // Leaves RGB off - robotStateTask repaints the correct color on its next
  // ~100ms tick, same as jump() does after dimming for the current spike.
}

void q8Dynamixel::jump(){
  BusGuard guard(*this);
  // Dim the RGB LED for the duration of the jump - one less simultaneous
  // current draw during the biggest current spike this robot ever makes.
  if (_rgb) _rgb->off();

  // Crouching Position
  setProfile(500);
  delay(100);
  bulkWrite(_lowArray);
  delay(1000);

  // Jump
  setProfile(0);
  setGain(800);
  delay(100);
  bulkWrite(_highArray);
  delay(100);
  bulkWrite(_restArray);
  delay(2000);

  // Back to idle
  setProfile(500);
  delay(100);
  bulkWrite(_idleArray);
  delay(1000);
  _prevProfile = 500;
}

void q8Dynamixel::greet(){
  BusGuard guard(*this);
  // Scripted via-point greeting sequence (ported from routine_generator.py's greet()).
  static const float G1[8] = {-90, 45, -90, 45, -90, 45, -90, 45};
  static const float G2[8] = {0, 45, 0, 45, 0, 45, 0, 45};
  static const float G3[8] = {-45, 45, -45, 45, 50, 75, 50, 75};
  static const float G4[8] = {45, 90, -45, 45, 50, 75, 50, 75};
  static const float G5[8] = {-45, 45, 45, 90, 50, 75, 50, 75};
  static const float G6[8] = {-90, 45, -90, 45, -90, 45, -90, 45};

  setProfile(1000);
  delay(100);
  bulkWriteDeg(G1);
  delay(1100);
  bulkWriteDeg(G2);
  delay(1000);

  setProfile(500);
  delay(100);
  bulkWriteDeg(G3);
  delay(1000);

  setProfile(200);
  delay(100);
  for (int i = 0; i < 3; i++){
    bulkWriteDeg(G4);
    delay(250);
    bulkWriteDeg(G5);
    delay(250);
  }
  delay(750);

  setProfile(500);
  delay(100);
  bulkWriteDeg(G6);
  delay(700);
}

void q8Dynamixel::_playFrames(const float frames[][8], uint8_t frameCount, uint8_t reps, uint16_t delayMs){
  uint16_t totalFrames = reps * frameCount;
  for (uint16_t i = 0; i < totalFrames; i++){
    bulkWriteDeg(frames[i % frameCount]);
    delay(delayMs);
  }
}

void q8Dynamixel::trotDemo(){
  BusGuard guard(*this);
  // Fast forward, then fast backward, using TROT_FAST's 'f' and 'b'
  // directions (ported from python-tools/q8bot/gait_generator.py's
  // generate_trot_trajectories(), via gait_manager.py's TROT_FAST params).
  static const uint8_t FRAME_COUNT = 36;
  static const uint8_t REPS = 5;  // steps per direction

  static const float TROT_FAST_F[FRAME_COUNT][8] = {
    {74.10f, 178.40f, 23.80f, 121.80f, 23.80f, 121.80f, 74.10f, 178.40f},
    {61.50f, 185.50f, 26.60f, 125.70f, 26.60f, 125.70f, 61.50f, 185.50f},
    {49.00f, 189.80f, 29.60f, 129.60f, 29.60f, 129.60f, 49.00f, 189.80f},
    {35.90f, 190.00f, 32.70f, 133.40f, 32.70f, 133.40f, 35.90f, 190.00f},
    {21.60f, 184.20f, 36.00f, 137.00f, 36.00f, 137.00f, 21.60f, 184.20f},
    {7.30f, 172.70f, 39.40f, 140.60f, 39.40f, 140.60f, 7.30f, 172.70f},
    {-4.20f, 158.40f, 43.00f, 144.00f, 43.00f, 144.00f, -4.20f, 158.40f},
    {-10.00f, 144.10f, 46.60f, 147.30f, 46.60f, 147.30f, -10.00f, 144.10f},
    {-9.80f, 131.00f, 50.40f, 150.40f, 50.40f, 150.40f, -9.80f, 131.00f},
    {-5.50f, 118.50f, 54.30f, 153.40f, 54.30f, 153.40f, -5.50f, 118.50f},
    {1.60f, 105.90f, 58.20f, 156.20f, 58.20f, 156.20f, 1.60f, 105.90f},
    {10.30f, 91.80f, 62.20f, 158.80f, 62.20f, 158.80f, 10.30f, 91.80f},
    {11.50f, 96.40f, 66.30f, 161.20f, 66.30f, 161.20f, 11.50f, 96.40f},
    {13.00f, 100.90f, 70.50f, 163.30f, 70.50f, 163.30f, 13.00f, 100.90f},
    {14.70f, 105.20f, 74.80f, 165.30f, 74.80f, 165.30f, 14.70f, 105.20f},
    {16.70f, 109.50f, 79.10f, 167.00f, 79.10f, 167.00f, 16.70f, 109.50f},
    {18.80f, 113.70f, 83.60f, 168.50f, 83.60f, 168.50f, 18.80f, 113.70f},
    {21.20f, 117.80f, 88.20f, 169.70f, 88.20f, 169.70f, 21.20f, 117.80f},
    {23.80f, 121.80f, 74.10f, 178.40f, 74.10f, 178.40f, 23.80f, 121.80f},
    {26.60f, 125.70f, 61.50f, 185.50f, 61.50f, 185.50f, 26.60f, 125.70f},
    {29.60f, 129.60f, 49.00f, 189.80f, 49.00f, 189.80f, 29.60f, 129.60f},
    {32.70f, 133.40f, 35.90f, 190.00f, 35.90f, 190.00f, 32.70f, 133.40f},
    {36.00f, 137.00f, 21.60f, 184.20f, 21.60f, 184.20f, 36.00f, 137.00f},
    {39.40f, 140.60f, 7.30f, 172.70f, 7.30f, 172.70f, 39.40f, 140.60f},
    {43.00f, 144.00f, -4.20f, 158.40f, -4.20f, 158.40f, 43.00f, 144.00f},
    {46.60f, 147.30f, -10.00f, 144.10f, -10.00f, 144.10f, 46.60f, 147.30f},
    {50.40f, 150.40f, -9.80f, 131.00f, -9.80f, 131.00f, 50.40f, 150.40f},
    {54.30f, 153.40f, -5.50f, 118.50f, -5.50f, 118.50f, 54.30f, 153.40f},
    {58.20f, 156.20f, 1.60f, 105.90f, 1.60f, 105.90f, 58.20f, 156.20f},
    {62.20f, 158.80f, 10.30f, 91.80f, 10.30f, 91.80f, 62.20f, 158.80f},
    {66.30f, 161.20f, 11.50f, 96.40f, 11.50f, 96.40f, 66.30f, 161.20f},
    {70.50f, 163.30f, 13.00f, 100.90f, 13.00f, 100.90f, 70.50f, 163.30f},
    {74.80f, 165.30f, 14.70f, 105.20f, 14.70f, 105.20f, 74.80f, 165.30f},
    {79.10f, 167.00f, 16.70f, 109.50f, 16.70f, 109.50f, 79.10f, 167.00f},
    {83.60f, 168.50f, 18.80f, 113.70f, 18.80f, 113.70f, 83.60f, 168.50f},
    {88.20f, 169.70f, 21.20f, 117.80f, 21.20f, 117.80f, 88.20f, 169.70f},
  };

  static const float TROT_FAST_B[FRAME_COUNT][8] = {
    {1.60f, 105.90f, 58.20f, 156.20f, 58.20f, 156.20f, 1.60f, 105.90f},
    {-5.50f, 118.50f, 54.30f, 153.40f, 54.30f, 153.40f, -5.50f, 118.50f},
    {-9.80f, 131.00f, 50.40f, 150.40f, 50.40f, 150.40f, -9.80f, 131.00f},
    {-10.00f, 144.10f, 46.60f, 147.30f, 46.60f, 147.30f, -10.00f, 144.10f},
    {-4.20f, 158.40f, 43.00f, 144.00f, 43.00f, 144.00f, -4.20f, 158.40f},
    {7.30f, 172.70f, 39.40f, 140.60f, 39.40f, 140.60f, 7.30f, 172.70f},
    {21.60f, 184.20f, 36.00f, 137.00f, 36.00f, 137.00f, 21.60f, 184.20f},
    {35.90f, 190.00f, 32.70f, 133.40f, 32.70f, 133.40f, 35.90f, 190.00f},
    {49.00f, 189.80f, 29.60f, 129.60f, 29.60f, 129.60f, 49.00f, 189.80f},
    {61.50f, 185.50f, 26.60f, 125.70f, 26.60f, 125.70f, 61.50f, 185.50f},
    {74.10f, 178.40f, 23.80f, 121.80f, 23.80f, 121.80f, 74.10f, 178.40f},
    {88.20f, 169.70f, 21.20f, 117.80f, 21.20f, 117.80f, 88.20f, 169.70f},
    {83.60f, 168.50f, 18.80f, 113.70f, 18.80f, 113.70f, 83.60f, 168.50f},
    {79.10f, 167.00f, 16.70f, 109.50f, 16.70f, 109.50f, 79.10f, 167.00f},
    {74.80f, 165.30f, 14.70f, 105.20f, 14.70f, 105.20f, 74.80f, 165.30f},
    {70.50f, 163.30f, 13.00f, 100.90f, 13.00f, 100.90f, 70.50f, 163.30f},
    {66.30f, 161.20f, 11.50f, 96.40f, 11.50f, 96.40f, 66.30f, 161.20f},
    {62.20f, 158.80f, 10.30f, 91.80f, 10.30f, 91.80f, 62.20f, 158.80f},
    {58.20f, 156.20f, 1.60f, 105.90f, 1.60f, 105.90f, 58.20f, 156.20f},
    {54.30f, 153.40f, -5.50f, 118.50f, -5.50f, 118.50f, 54.30f, 153.40f},
    {50.40f, 150.40f, -9.80f, 131.00f, -9.80f, 131.00f, 50.40f, 150.40f},
    {46.60f, 147.30f, -10.00f, 144.10f, -10.00f, 144.10f, 46.60f, 147.30f},
    {43.00f, 144.00f, -4.20f, 158.40f, -4.20f, 158.40f, 43.00f, 144.00f},
    {39.40f, 140.60f, 7.30f, 172.70f, 7.30f, 172.70f, 39.40f, 140.60f},
    {36.00f, 137.00f, 21.60f, 184.20f, 21.60f, 184.20f, 36.00f, 137.00f},
    {32.70f, 133.40f, 35.90f, 190.00f, 35.90f, 190.00f, 32.70f, 133.40f},
    {29.60f, 129.60f, 49.00f, 189.80f, 49.00f, 189.80f, 29.60f, 129.60f},
    {26.60f, 125.70f, 61.50f, 185.50f, 61.50f, 185.50f, 26.60f, 125.70f},
    {23.80f, 121.80f, 74.10f, 178.40f, 74.10f, 178.40f, 23.80f, 121.80f},
    {21.20f, 117.80f, 88.20f, 169.70f, 88.20f, 169.70f, 21.20f, 117.80f},
    {18.80f, 113.70f, 83.60f, 168.50f, 83.60f, 168.50f, 18.80f, 113.70f},
    {16.70f, 109.50f, 79.10f, 167.00f, 79.10f, 167.00f, 16.70f, 109.50f},
    {14.70f, 105.20f, 74.80f, 165.30f, 74.80f, 165.30f, 14.70f, 105.20f},
    {13.00f, 100.90f, 70.50f, 163.30f, 70.50f, 163.30f, 13.00f, 100.90f},
    {11.50f, 96.40f, 66.30f, 161.20f, 66.30f, 161.20f, 11.50f, 96.40f},
    {10.30f, 91.80f, 62.20f, 158.80f, 62.20f, 158.80f, 10.30f, 91.80f},
  };

  // Matches the host's actual full-speed playback rate: operate.py never
  // sets a per-frame DXL profile for gait ticks (move_all always passes
  // dur=0, i.e. max speed/no ramp) - the tick rate is capped at
  // clock.tick(SPEED=200), i.e. ~5ms per frame.
  const uint16_t frameDelay = 5;      // ms between frames - fast, no ramp
  const uint16_t segmentGapMs = 500;  // pause between forward and backward

  setProfile(0);
  delay(100);

  _playFrames(TROT_FAST_F, FRAME_COUNT, REPS, frameDelay);
  delay(segmentGapMs);
  _playFrames(TROT_FAST_B, FRAME_COUNT, REPS, frameDelay);
}

uint8_t q8Dynamixel::parseFrame(const uint8_t* buf, size_t bufCapacity) {
  uint8_t msgType;
  const uint8_t* payload;
  uint8_t payloadLen;

  if (!parseFrameBuf(buf, bufCapacity, &msgType, &payload, &payloadLen)) {
    return 0;  // Bad start byte, truncated buffer, or CRC mismatch - drop.
  }

  switch (static_cast<CmdType>(msgType)) {
    case CmdType::MOVE: {
      if (payloadLen < 19) return 0;  // 8 x int16 + uint16 + uint8

      // A raw MOVE always hands control back to the host, unconditionally -
      // this is what keeps "the robot can still take raw joint positions"
      // true regardless of whether a native gait was running.
      if (_gaitRunner) _gaitRunner->stop();

      int16_t angleTenths[8];
      memcpy(angleTenths, payload, sizeof(angleTenths));
      for (int i = 0; i < 8; i++) {
        _posArray[i] = _deg2Dxl(angleTenths[i] / 10.0f);
      }

      uint16_t profile;
      memcpy(&profile, payload + 16, sizeof(profile));
      _profile = profile;
      if (_profile != _prevProfile) {
        Serial.print("[ROBOT] Profile changed: "); Serial.println(_profile);
        setProfile(_profile);
        _prevProfile = _profile;
      }

      // payload[18] used to be a "record this frame" flag. Recording is gone
      // (it sync-read the bus from this task on every streamed frame), but
      // the byte stays in the layout so the controller's own MOVE builder
      // doesn't have to change in lockstep. Spare for the next flag.
      bulkWrite(_posArray);
      return 0;
    }

    case CmdType::TORQUE_SET: {
      if (payloadLen < 1) return 0;
      _torqueFlag = (payload[0] == 1);
      if (_torqueFlag != _prevTorqueFlag) {
        Serial.println(_torqueFlag ? "[ROBOT] Torque on" : "[ROBOT] Torque off");
        toggleTorque(_torqueFlag);
        _prevTorqueFlag = _torqueFlag;
      }
      return 0;
    }

    case CmdType::BATTERY_REQUEST:
      return 1;

    case CmdType::RECORD_SEND:
      // Retired. Left as a no-op rather than removed so an older host that
      // still sends it gets ignored instead of falling through to default.
      return 0;

    case CmdType::JUMP:
      if (_jumpEnabled) jump();
      else warnJumpBlocked();
      return 0;

    case CmdType::GREET:
      greet();
      return 0;

    case CmdType::GAIT_RUN: {
      if (payloadLen < 2) return 0;
      if (_gaitRunner) _gaitRunner->start(payload[0], payload[1]);
      return 0;
    }

    case CmdType::GAIT_STOP:
      if (_gaitRunner) _gaitRunner->stop();
      return 0;

    case CmdType::LED_BRIGHTNESS: {
      if (payloadLen < 1) return 0;
      if (_rgb) _rgb->setBrightness(payload[0]);
      if (_storage) _storage->saveLedBrightness(payload[0]);
      return 0;
    }

    default:
      return 0;
  }
}

int32_t q8Dynamixel::_deg2Dxl(float deg){
  // Dynamixel joint 0 to 360 deg is 0 to 4096
  const float friendlyPerDxl = 360.0 / 4096.0 / _gearRatio;
  int angleDxl = static_cast<int>(deg / friendlyPerDxl + 0.5) + _zeroOffset;
  return angleDxl;
}

float q8Dynamixel::_dxl2Deg(int32_t dxlRaw){
  // Dynamixel joint 0 to 360 deg is 0 to 4096
  const float friendlyPerDxl = 360.0 / 4096.0 / _gearRatio;
  float angleFriendly = (dxlRaw - _zeroOffset) * friendlyPerDxl;
  return angleFriendly;
}

void q8Dynamixel::expandArrays(){
  for (int i = 0; i < 4; i++){
    _idleArray[i*2] = _deg2Dxl(_idlePos[0]);
    _idleArray[i*2+1] = _deg2Dxl(_idlePos[1]);
  }
  for (int i = 0; i < 4; i++){
    _lowArray[i*2] = _deg2Dxl(_jumpLow[0]);
    _lowArray[i*2+1] = _deg2Dxl(_jumpLow[1]);
  }
  for (int i = 0; i < 4; i++){
    _highArray[i*2] = _deg2Dxl(_jumpHigh[0]);
    _highArray[i*2+1] = _deg2Dxl(_jumpHigh[1]);
  }
  for (int i = 0; i < 4; i++){
    _restArray[i*2] = _deg2Dxl(_jumpRest[0]);
    _restArray[i*2+1] = _deg2Dxl(_jumpRest[1]);
  }
}