/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  MIT License - see LICENSE at the repository root.
*/

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <q8Dynamixel.h>
#include <RGBManager.h>
#include <protocol.h>
#include <gaitRunner.h>
#include <macStorage.h>
#include <powerRail.h>

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

void q8Dynamixel::attachRails(PowerRail* rail5V, PowerRail* rail6V){
  _rail5V = rail5V;
  _rail6V = rail6V;
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

  // Second and third sync-read sets: supply voltage + temperature, and
  // hardware error status (see readServoHealth).
  _tr_infos.packet.buf_capacity = _user_pkt_buf_cap;
  _tr_infos.packet.p_buf = _tr_pkt_buf;
  _tr_infos.packet.is_completed = false;
  _tr_infos.addr = TR_START_ADDR;
  _tr_infos.addr_length = TR_ADDR_LEN;
  _tr_infos.p_xels = _info_xels_tr;
  _tr_infos.xel_count = 0;

  for (int i = 0; i < _idCount; i++){
    _info_xels_tr[i].id = _DXL[i];
    _info_xels_tr[i].p_recv_buf = (uint8_t*)&_tr_data[i];
    _tr_infos.xel_count++;
  }
  _tr_infos.is_info_changed = true;

  _er_infos.packet.buf_capacity = _user_pkt_buf_cap;
  _er_infos.packet.p_buf = _er_pkt_buf;
  _er_infos.packet.is_completed = false;
  _er_infos.addr = ER_START_ADDR;
  _er_infos.addr_length = ER_ADDR_LEN;
  _er_infos.p_xels = _info_xels_er;
  _er_infos.xel_count = 0;

  for (int i = 0; i < _idCount; i++){
    _info_xels_er[i].id = _DXL[i];
    _info_xels_er[i].p_recv_buf = &_er_data[i];
    _er_infos.xel_count++;
  }
  _er_infos.is_info_changed = true;

  setProfile(1000);
  expandArrays();
  learnTurns();
}

bool q8Dynamixel::readJointState(TelemetryPayload& out){
  BusGuard guard(*this);

  if (_dxl.fastSyncRead(&_sr_infos) != _idCount) return false;

  // While limp, keep the turns current from these same readings - the
  // servos may have been power-cycled with the board still running (switched
  // off and on with USB plugged in). With torque on they were set at the
  // last torque-on.
  if (!_torqueFlag) adoptTurns();
  for (uint8_t i = 0; i < _idCount; i++){
    out.position[i] = _sr_data[i].present_position - _turnTicks[i];
    out.current[i]  = _sr_data[i].present_current;
  }
  return true;
}

bool q8Dynamixel::readServoHealth(int8_t temperature[8], uint8_t voltage[8], uint8_t hwError[8]){
  BusGuard guard(*this);

  bool health = _dxl.fastSyncRead(&_tr_infos) == _idCount;
  if (health){
    for (uint8_t i = 0; i < _idCount; i++){
      temperature[i] = (int8_t)_tr_data[i].present_temperature;
      uint16_t v = _tr_data[i].present_input_voltage;
      voltage[i] = v > 255 ? 255 : (uint8_t)v;
    }
  }
  bool errors = _dxl.fastSyncRead(&_er_infos) == _idCount;
  if (errors){
    memcpy(hwError, _er_data, _idCount);
  }
  return health && errors;
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
  // First re-learn which turn each servo counts itself in and make its goal
  // where it is now, so torque coming on moves nothing - not even a servo
  // that restarted, or came up a turn away, since the goals were last sent.
  bool held = false;
  for (int attempt = 0; attempt < 3 && !held; attempt++){
    held = holdPresentPose();
    if (!held) delay(50);
  }
  if (!held) Serial.println("[ROBOT] Torque on: couldn't read every joint's position first");
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

void q8Dynamixel::setJointMode(uint8_t mode){
  // _torqueFlag tracks "is a host/teleop session actively holding the
  // joints", which is true for damping too - only actual torque-off counts
  // as idle for the standalone backup-demo trigger (see main.cpp).
  BusGuard guard(*this);
  _torqueFlag = (mode != 0);
  if (mode == 0) {
    disableTorque();
    return;
  }
  enableTorque();
  setGain(mode == 2 ? DAMPING_GAIN : 400);
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
  // 8 motors move to the same position
  int32_t values[_idCount];
  for (int i = 0; i < _idCount; i++) values[i] = val;
  bulkWrite(values);
}

void q8Dynamixel::bulkWrite(int32_t values[8]){
  // 8 motors move to their respective positions, each in the turn it
  // counts itself in (see _turnTicks)
  int32_t raw[_idCount];
  for (int i = 0; i < _idCount; i++) raw[i] = values[i] + _turnTicks[i];
  writeGoalsRaw(raw);
}

void q8Dynamixel::writeGoalsRaw(const int32_t raw[8]){
  BusGuard guard(*this);
  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = raw[i];
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

  // Limp for the flight and landing instead of driving the legs back -
  // that would be a second current spike, and the push's alone can sag the
  // servo supply enough to restart a servo or two, which then come back
  // with torque off. Turning all eight off, then all back on below, puts
  // every servo - restarted or not - in the same state again.
  disableTorque();
  delay(1000);

  // Back on. enableTorque() first holds wherever each leg landed - a servo
  // that didn't restart still has the push's full extension as its goal -
  // and re-learns the turns, in case a restarted one came up a turn away.
  // setProfile() also restores the gain and profile a restarted servo lost.
  setProfile(500);
  enableTorque();

  // Then ease into the idle pose.
  delay(100);
  bulkWrite(_idleArray);
  delay(1000);
  _prevProfile = 500;
}

bool q8Dynamixel::holdPresentPose(){
  BusGuard guard(*this);
  if (!learnTurns()) return false;
  int32_t present[_idCount];
  for (uint8_t i = 0; i < _idCount; i++){
    present[i] = _sr_data[i].present_position;
  }
  writeGoalsRaw(present);
  return true;
}

bool q8Dynamixel::learnTurns(){
  BusGuard guard(*this);
  if (_dxl.fastSyncRead(&_sr_infos) != _idCount) return false;
  adoptTurns();
  return true;
}

void q8Dynamixel::adoptTurns(){
  for (uint8_t i = 0; i < _idCount; i++){
    const int32_t low = _deg2Dxl(_idlePos[i % 2]) - TICKS_PER_TURN / 2;
    const int32_t d = _sr_data[i].present_position - low;
    const int32_t turns = d >= 0 ? d / TICKS_PER_TURN : -((-d + TICKS_PER_TURN - 1) / TICKS_PER_TURN);
    if (turns * TICKS_PER_TURN != _turnTicks[i]){
      _turnTicks[i] = turns * TICKS_PER_TURN;
      if (turns){
        Serial.printf("[ROBOT] Slot %u counts itself %ld turn%s off - allowing for it\n",
                      i + 1, (long)(turns < 0 ? -turns : turns), (turns == 1 || turns == -1) ? "" : "s");
      } else {
        Serial.printf("[ROBOT] Slot %u counts itself in its own turn again\n", i + 1);
      }
    }
  }
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
  // directions (ported from q8botone-python/q8bot/gait_generator.py's
  // generate_trot_trajectories(), via gait_manager.py's TROT_FAST params).
  static const uint8_t FRAME_COUNT = 36;
  static const uint8_t REPS = 5;  // steps per direction

  static const float TROT_FAST_F[FRAME_COUNT][8] = {
    {77.30f, 176.50f, 26.20f, 118.80f, 26.20f, 118.80f, 77.30f, 176.50f},
    {64.00f, 183.80f, 29.00f, 122.80f, 29.00f, 122.80f, 64.00f, 183.80f},
    {51.20f, 188.40f, 32.00f, 126.80f, 32.00f, 126.80f, 51.20f, 188.40f},
    {37.70f, 188.50f, 35.10f, 130.70f, 35.10f, 130.70f, 37.70f, 188.50f},
    {23.40f, 182.60f, 38.50f, 134.40f, 38.50f, 134.40f, 23.40f, 182.60f},
    {9.00f, 171.00f, 42.00f, 138.00f, 42.00f, 138.00f, 9.00f, 171.00f},
    {-2.60f, 156.60f, 45.60f, 141.50f, 45.60f, 141.50f, -2.60f, 156.60f},
    {-8.50f, 142.30f, 49.30f, 144.90f, 49.30f, 144.90f, -8.50f, 142.30f},
    {-8.40f, 128.80f, 53.20f, 148.00f, 53.20f, 148.00f, -8.40f, 128.80f},
    {-3.80f, 116.00f, 57.20f, 151.00f, 57.20f, 151.00f, -3.80f, 116.00f},
    {3.50f, 102.70f, 61.20f, 153.80f, 61.20f, 153.80f, 3.50f, 102.70f},
    {12.70f, 87.20f, 65.40f, 156.50f, 65.40f, 156.50f, 12.70f, 87.20f},
    {13.90f, 92.20f, 69.60f, 158.90f, 69.60f, 158.90f, 13.90f, 92.20f},
    {15.40f, 97.00f, 73.90f, 161.00f, 73.90f, 161.00f, 15.40f, 97.00f},
    {17.00f, 101.60f, 78.40f, 163.00f, 78.40f, 163.00f, 17.00f, 101.60f},
    {19.00f, 106.10f, 83.00f, 164.60f, 83.00f, 164.60f, 19.00f, 106.10f},
    {21.10f, 110.40f, 87.80f, 166.10f, 87.80f, 166.10f, 21.10f, 110.40f},
    {23.50f, 114.60f, 92.80f, 167.30f, 92.80f, 167.30f, 23.50f, 114.60f},
    {26.20f, 118.80f, 77.30f, 176.50f, 77.30f, 176.50f, 26.20f, 118.80f},
    {29.00f, 122.80f, 64.00f, 183.80f, 64.00f, 183.80f, 29.00f, 122.80f},
    {32.00f, 126.80f, 51.20f, 188.40f, 51.20f, 188.40f, 32.00f, 126.80f},
    {35.10f, 130.70f, 37.70f, 188.50f, 37.70f, 188.50f, 35.10f, 130.70f},
    {38.50f, 134.40f, 23.40f, 182.60f, 23.40f, 182.60f, 38.50f, 134.40f},
    {42.00f, 138.00f, 9.00f, 171.00f, 9.00f, 171.00f, 42.00f, 138.00f},
    {45.60f, 141.50f, -2.60f, 156.60f, -2.60f, 156.60f, 45.60f, 141.50f},
    {49.30f, 144.90f, -8.50f, 142.30f, -8.50f, 142.30f, 49.30f, 144.90f},
    {53.20f, 148.00f, -8.40f, 128.80f, -8.40f, 128.80f, 53.20f, 148.00f},
    {57.20f, 151.00f, -3.80f, 116.00f, -3.80f, 116.00f, 57.20f, 151.00f},
    {61.20f, 153.80f, 3.50f, 102.70f, 3.50f, 102.70f, 61.20f, 153.80f},
    {65.40f, 156.50f, 12.70f, 87.20f, 12.70f, 87.20f, 65.40f, 156.50f},
    {69.60f, 158.90f, 13.90f, 92.20f, 13.90f, 92.20f, 69.60f, 158.90f},
    {73.90f, 161.00f, 15.40f, 97.00f, 15.40f, 97.00f, 73.90f, 161.00f},
    {78.40f, 163.00f, 17.00f, 101.60f, 17.00f, 101.60f, 78.40f, 163.00f},
    {83.00f, 164.60f, 19.00f, 106.10f, 19.00f, 106.10f, 83.00f, 164.60f},
    {87.80f, 166.10f, 21.10f, 110.40f, 21.10f, 110.40f, 87.80f, 166.10f},
    {92.80f, 167.30f, 23.50f, 114.60f, 23.50f, 114.60f, 92.80f, 167.30f},
  };

  static const float TROT_FAST_B[FRAME_COUNT][8] = {
    {3.50f, 102.70f, 61.20f, 153.80f, 61.20f, 153.80f, 3.50f, 102.70f},
    {-3.80f, 116.00f, 57.20f, 151.00f, 57.20f, 151.00f, -3.80f, 116.00f},
    {-8.40f, 128.80f, 53.20f, 148.00f, 53.20f, 148.00f, -8.40f, 128.80f},
    {-8.50f, 142.30f, 49.30f, 144.90f, 49.30f, 144.90f, -8.50f, 142.30f},
    {-2.60f, 156.60f, 45.60f, 141.50f, 45.60f, 141.50f, -2.60f, 156.60f},
    {9.00f, 171.00f, 42.00f, 138.00f, 42.00f, 138.00f, 9.00f, 171.00f},
    {23.40f, 182.60f, 38.50f, 134.40f, 38.50f, 134.40f, 23.40f, 182.60f},
    {37.70f, 188.50f, 35.10f, 130.70f, 35.10f, 130.70f, 37.70f, 188.50f},
    {51.20f, 188.40f, 32.00f, 126.80f, 32.00f, 126.80f, 51.20f, 188.40f},
    {64.00f, 183.80f, 29.00f, 122.80f, 29.00f, 122.80f, 64.00f, 183.80f},
    {77.30f, 176.50f, 26.20f, 118.80f, 26.20f, 118.80f, 77.30f, 176.50f},
    {92.80f, 167.30f, 23.50f, 114.60f, 23.50f, 114.60f, 92.80f, 167.30f},
    {87.80f, 166.10f, 21.10f, 110.40f, 21.10f, 110.40f, 87.80f, 166.10f},
    {83.00f, 164.60f, 19.00f, 106.10f, 19.00f, 106.10f, 83.00f, 164.60f},
    {78.40f, 163.00f, 17.00f, 101.60f, 17.00f, 101.60f, 78.40f, 163.00f},
    {73.90f, 161.00f, 15.40f, 97.00f, 15.40f, 97.00f, 73.90f, 161.00f},
    {69.60f, 158.90f, 13.90f, 92.20f, 13.90f, 92.20f, 69.60f, 158.90f},
    {65.40f, 156.50f, 12.70f, 87.20f, 12.70f, 87.20f, 65.40f, 156.50f},
    {61.20f, 153.80f, 3.50f, 102.70f, 3.50f, 102.70f, 61.20f, 153.80f},
    {57.20f, 151.00f, -3.80f, 116.00f, -3.80f, 116.00f, 57.20f, 151.00f},
    {53.20f, 148.00f, -8.40f, 128.80f, -8.40f, 128.80f, 53.20f, 148.00f},
    {49.30f, 144.90f, -8.50f, 142.30f, -8.50f, 142.30f, 49.30f, 144.90f},
    {45.60f, 141.50f, -2.60f, 156.60f, -2.60f, 156.60f, 45.60f, 141.50f},
    {42.00f, 138.00f, 9.00f, 171.00f, 9.00f, 171.00f, 42.00f, 138.00f},
    {38.50f, 134.40f, 23.40f, 182.60f, 23.40f, 182.60f, 38.50f, 134.40f},
    {35.10f, 130.70f, 37.70f, 188.50f, 37.70f, 188.50f, 35.10f, 130.70f},
    {32.00f, 126.80f, 51.20f, 188.40f, 51.20f, 188.40f, 32.00f, 126.80f},
    {29.00f, 122.80f, 64.00f, 183.80f, 64.00f, 183.80f, 29.00f, 122.80f},
    {26.20f, 118.80f, 77.30f, 176.50f, 77.30f, 176.50f, 26.20f, 118.80f},
    {23.50f, 114.60f, 92.80f, 167.30f, 92.80f, 167.30f, 23.50f, 114.60f},
    {21.10f, 110.40f, 87.80f, 166.10f, 87.80f, 166.10f, 21.10f, 110.40f},
    {19.00f, 106.10f, 83.00f, 164.60f, 83.00f, 164.60f, 19.00f, 106.10f},
    {17.00f, 101.60f, 78.40f, 163.00f, 78.40f, 163.00f, 17.00f, 101.60f},
    {15.40f, 97.00f, 73.90f, 161.00f, 73.90f, 161.00f, 15.40f, 97.00f},
    {13.90f, 92.20f, 69.60f, 158.90f, 69.60f, 158.90f, 13.90f, 92.20f},
    {12.70f, 87.20f, 65.40f, 156.50f, 65.40f, 156.50f, 12.70f, 87.20f},
  };

  // Matches the host's full-speed playback rate: the desktop app's Q8Bot
  // streams gait frames with dur=0 (max speed, no ramp) at TICK_HZ = 200,
  // i.e. ~5ms per frame.
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
      uint8_t mode = payload[0];  // 0=off, 1=on, 2=damping
      if (mode != 0 && _servosUnpowered) {
        Serial.println("[ROBOT] Torque on refused - the servos lost power (POWER_RAIL); restart the robot first");
        return 0;
      }
      if (mode != _prevJointMode) {
        const char* label = (mode == 2) ? "damping" : (mode == 1 ? "on" : "off");
        Serial.printf("[ROBOT] Torque %s\n", label);
        setJointMode(mode);
        _prevJointMode = mode;
      }
      return 0;
    }

    case CmdType::BATTERY_REQUEST:
      return 1;

    case CmdType::TELEMETRY_REQUEST:
      // Answered by gaitRunnerTask rather than here: the servo bus has one
      // owner, and sampling there puts the read right after the gait write
      // where it uses slack instead of delaying a frame.
      return 4;

    case CmdType::SENSOR_REQUEST:
      // Answered by sensorTask (see main.cpp) - the I2C reads shouldn't
      // hold up the radio task that parses this.
      return 5;

    case CmdType::RECORD_SEND:
      // Retired. Left as a no-op rather than removed so an older host that
      // still sends it gets ignored instead of falling through to default.
      return 0;

    case CmdType::JUMP:
      if (_servosUnpowered) return 0;  // jump() turns torque on itself - see POWER_RAIL
      if (_jumpEnabled) jump();
      else warnJumpBlocked();
      return 0;

    case CmdType::GREET:
      if (_servosUnpowered) return 0;
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
      // Flags bit 0: just for now, don't save it (a measurement turning the
      // light off) - a restart brings back the saved brightness.
      bool keep = payloadLen >= 2 && (payload[1] & 0x01);
      if (_storage && !keep) _storage->saveLedBrightness(payload[0]);
      return 0;
    }

    case CmdType::POWER_RAIL: {
      // For power measurements, like the serial '5' / '6' toggles but over
      // the radio. Cutting the 6V buck takes the servos' power: torque goes
      // off first, and stays off until the robot restarts - a servo that
      // lost power comes back with its RAM settings (the motion profile
      // among them) reset, and nothing here sets them again.
      if (payloadLen < 2) return 0;
      PowerRail* rail = payload[0] == 5 ? _rail5V : (payload[0] == 6 ? _rail6V : nullptr);
      if (!rail) return 0;
      bool on = payload[1] != 0;
      if (rail == _rail6V && !on) {
        if (_torqueFlag) {
          setJointMode(0);
          _prevJointMode = 0;
        }
        _servosUnpowered = true;
      }
      rail->set(on);
      Serial.printf("[POWER] %uV buck %s (POWER_RAIL)\n", payload[0], on ? "ON" : "OFF");
      if (rail == _rail6V && on) {
        Serial.println("[POWER] Servos powered again - restart the robot before turning the motors on");
      }
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