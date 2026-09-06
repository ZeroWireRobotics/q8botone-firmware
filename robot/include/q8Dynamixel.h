/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  Released into the public domain.
*/
#ifndef q8Dynamixel_h
#define q8Dynamixel_h

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

class RGBManager;  // Forward declaration - see RGBManager.h
class GaitRunner;  // Forward declaration - see gaitRunner.h
class macStorage;  // Forward declaration - see macStorage.h

class q8Dynamixel
{
  public:
    q8Dynamixel(Dynamixel2Arduino& dxl);
    void begin();
    void attachRGB(RGBManager* rgb);  // Optional - lets jump() dim the LED during its current spike
    void attachGaitRunner(GaitRunner* gaitRunner);  // Lets parseFrame() dispatch GAIT_RUN/GAIT_STOP
    void attachStorage(macStorage* storage);  // Lets parseFrame() persist LED_BRIGHTNESS to NVS
    bool checkComms(uint8_t ID);
    bool commStart();
    uint16_t checkBattery();
    void enableTorque();
    void disableTorque();
    void toggleTorque(bool flag);
    void resetTorqueState(bool torqueOn = false);  // Reset internal torque flag without changing hardware state
    bool isTorqueOn() const;  // Current internal torque flag (not a hardware read)
    void setOpMode();
    void setProfile(uint16_t dur);
    void setGain(uint16_t p_gain);
    void moveSingle(int32_t val);
    void bulkWrite(int32_t values[8]);
    void bulkWriteDeg(const float degs[8]);  // Converts to raw ticks via _deg2Dxl, then bulkWrite
    void moveToIdle(uint16_t dur = 500);  // Move all legs to the precomputed idle/neutral pose
    uint16_t* syncRead();
    void jump();
    void greet();      // Scripted greeting sequence (no host required)
    void trotDemo();   // A few trot steps forward, then a few steps backward (no host required)
    void setJumpEnabled(bool enabled);
    void warnJumpBlocked();  // A few red RGB pulses - jump requested while disabled (low battery)
    // Parses/validates a framed command (see protocol.h) already sitting in
    // buf (bufCapacity is a safety bound, not necessarily the real frame
    // length - the frame is self-delimiting via its own LENGTH byte).
    // Returns 0 (no-op/handled), 1 (battery requested), 2 (move + record),
    // or 3 (send recorded data) - same meaning as the old parseData().
    uint8_t parseFrame(const uint8_t* buf, size_t bufCapacity);

  private:
    Dynamixel2Arduino& _dxl; // Member variable to store the object of Dynamixel2Arduino

    const uint8_t BROADCAST_ID = 254;
    const uint16_t SR_START_ADDR = 126; //Present current, then velocity and position
    const uint16_t SR_ADDR_LEN = 10;
    const uint16_t SW_START_ADDR = 116; //Goal position
    const uint16_t SW_ADDR_LEN = 4;

    uint32_t _baudrate = 1000000;
    float _protocolVersion = 2.0;
    static const uint8_t _idCount = 8;
    const uint8_t _DXL[_idCount] = {11, 12, 13, 14, 15, 16, 17, 18};
    const uint8_t _directionPin = 8;
    static const uint16_t _user_pkt_buf_cap = 128;
    uint8_t _user_pkt_buf[_user_pkt_buf_cap];
    const int16_t _zeroOffset = 4096;
    const uint8_t _gearRatio = 1;
    int32_t _posArray[8];
    uint16_t _profile = 0;
    uint16_t _prevProfile;
    bool _torqueFlag = false;
    bool _prevTorqueFlag = false;
    bool _jumpEnabled = true;
    RGBManager* _rgb = nullptr;
    GaitRunner* _gaitRunner = nullptr;
    macStorage* _storage = nullptr;
    int32_t _deg2Dxl(float deg);
    float _dxl2Deg(int32_t dxlRaw);
    // Plays a frame table at a constant per-frame delay. Used by trotDemo().
    void _playFrames(const float frames[][8], uint8_t frameCount, uint8_t reps, uint16_t delayMs);
    void expandArrays();
    const float _idlePos[2]   = {30, 150};
    int32_t _idleArray[8];
    const float _jumpLow[2]   = {-90, 270};
    int32_t _lowArray[8];
    const float _jumpHigh[2] = {90, 90};
    int32_t _highArray[8];
    const float _jumpRest[2]  = {30, 150};
    int32_t _restArray[8];

    // Struct definitions for br (bulk read) and bw (bulk write)
    struct br_data_xel{
      int32_t present_position;
    } __attribute__((packed));
    struct bw_data_xel{
      int32_t goal_position;
    } __attribute__((packed));

    // Struct definitions for sr (sync read) and sw (sync write)
    typedef struct sr_data{
      int16_t present_current;
      int32_t present_velocity;
      int32_t present_position;
    } __attribute__((packed)) sr_data_t;
    typedef struct sw_data{
      int32_t goal_position;
    } __attribute__((packed)) sw_data_t;

    struct br_data_xel _br_data_xel[_idCount];
    DYNAMIXEL::InfoBulkReadInst_t _br_infos;
    DYNAMIXEL::XELInfoBulkRead_t _info_xels_br[_idCount];
    struct bw_data_xel _bw_data_xel[_idCount]; 
    DYNAMIXEL::InfoBulkWriteInst_t _bw_infos;
    DYNAMIXEL::XELInfoBulkWrite_t _info_xels_bw[_idCount];

    sr_data_t _sr_data[_idCount];
    DYNAMIXEL::InfoSyncReadInst_t _sr_infos;
    DYNAMIXEL::XELInfoSyncRead_t _info_xels_sr[_idCount];
    sw_data_t _sw_data[_idCount];
    DYNAMIXEL::InfoSyncWriteInst_t _sw_infos;
    DYNAMIXEL::XELInfoSyncWrite_t _info_xels_sw[_idCount];
};

#endif