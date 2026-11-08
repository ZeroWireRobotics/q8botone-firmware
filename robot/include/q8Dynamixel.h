/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  MIT License - see LICENSE at the repository root.
*/
#ifndef q8Dynamixel_h
#define q8Dynamixel_h

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

using namespace ControlTableItem;

class RGBManager;  // Forward declaration - see RGBManager.h
class GaitRunner;  // Forward declaration - see gaitRunner.h
class macStorage;  // Forward declaration - see macStorage.h
class PowerRail;   // Forward declaration - see powerRail.h

class q8Dynamixel
{
  public:
    q8Dynamixel(Dynamixel2Arduino& dxl);
    void begin();
    void attachRGB(RGBManager* rgb);  // Optional - lets jump() dim the LED during its current spike
    void attachGaitRunner(GaitRunner* gaitRunner);  // Lets parseFrame() dispatch GAIT_RUN/GAIT_STOP
    void attachStorage(macStorage* storage);  // Lets parseFrame() persist LED_BRIGHTNESS to NVS
    void attachRails(PowerRail* rail5V, PowerRail* rail6V);  // Lets parseFrame() switch the bucks (POWER_RAIL)
    bool checkComms(uint8_t ID);
    bool commStart();
    uint16_t checkBattery();
    void enableTorque();
    void disableTorque();
    void toggleTorque(bool flag);
    void resetTorqueState(bool torqueOn = false);  // Reset internal torque flag without changing hardware state
    bool isTorqueOn() const;  // Current internal torque flag (not a hardware read)
    // TORQUE_SET's mode byte: 0=off, 1=on (full stiffness), 2=damping - still
    // torqued, but at DAMPING_GAIN instead of the usual 400, so a hand can
    // push a joint into a new pose against soft resistance instead of either
    // fighting full stiffness or getting no resistance at all (torque off).
    // Any value other than 0/2 is treated as "on", so a malformed byte can't
    // strand the robot in whatever mode it was already in.
    void setJointMode(uint8_t mode);
    void setOpMode();
    void setProfile(uint16_t dur);
    void setGain(uint16_t p_gain);
    void moveSingle(int32_t val);
    void bulkWrite(int32_t values[8]);
    void bulkWriteDeg(const float degs[8]);  // Converts to raw ticks via _deg2Dxl, then bulkWrite
    void moveToIdle(uint16_t dur = 500);  // Move all legs to the precomputed idle/neutral pose
    // Reads present position and current for all 8 joints into out (see
    // protocol.h's TelemetryPayload). Leaves out.temperature and the rest alone -
    // those are sampled separately by readServoHealth(), since they sit outside the
    // contiguous 126-135 block and changes far too slowly to be worth the
    // extra bus time on every sample.
    //
    // Works with torque off; current simply reads near zero, because none is
    // flowing. Returns false if the servos didn't all answer.
    bool readJointState(struct TelemetryPayload& out);

    // Samples what each servo says about its own health: temperature (degC),
    // supply voltage (0.1 V) and Hardware Error Status (register 70, nonzero
    // once it has shut itself down). Their own transactions rather than a
    // wider readJointState(), since they change over seconds. A servo in
    // error still answers reads, so this keeps working after one faults. An
    // array is left as it was if its read fails; false if either did.
    //
    // Also the torque watchdog: with the motors on, a servo found with its
    // torque off and no hardware error - restarted, most likely - gets its
    // settings and torque back. restored counts that per servo since boot.
    bool readServoHealth(int8_t temperature[8], uint8_t voltage[8], uint8_t hwError[8],
                         uint8_t restored[8]);
    // DYNAMIXEL's Reboot instruction for one servo (slot 1-8) - short of a
    // power cycle, the only way to clear its Hardware Error Status and let
    // its torque on again. Puts back what the restart reset (see SERVO_REBOOT).
    bool rebootServo(uint8_t slot);
    // Call often (gaitRunnerTask, every tick): puts a servo that just eased
    // back into its pose onto the others' profile again - see returnToCommanded().
    void serviceReturn(uint32_t nowMs);
    void jump();
    void greet();      // Scripted greeting sequence (no host required)
    void trotDemo();   // A few trot steps forward, then a few steps backward (no host required)
    void setJumpEnabled(bool enabled);
    void warnJumpBlocked();  // A few red RGB pulses - jump requested while disabled (low battery)
    // Parses/validates a framed command (see protocol.h) already sitting in
    // buf (bufCapacity is a safety bound, not necessarily the real frame
    // length - the frame is self-delimiting via its own LENGTH byte).
    // Returns 0 (no-op/handled), 1 (battery requested) or 4 (joint
    // telemetry requested). 2 and 3 used to mean "move + record" and "send
    // recorded data"; recording is gone.
    uint8_t parseFrame(const uint8_t* buf, size_t bufCapacity);

    // The servo bus is half duplex and shared by more than one task -
    // gaitRunnerTask steps a gait and samples telemetry every 5ms while
    // espnowRxTask handles MOVE and JUMP - so every transaction below takes
    // this lock. Public
    // so a caller that must not stall (the gait tick) can try for it and skip
    // its turn rather than block behind a multi-second jump().
    //
    // Recursive: the composite operations are built out of the simple ones
    // (moveToIdle -> bulkWrite, jump -> setProfile + bulkWrite + ...).
    bool lockBus(TickType_t timeout = portMAX_DELAY);
    void unlockBus();

  private:
    SemaphoreHandle_t _busLock = nullptr;
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
    // Last TORQUE_SET payload actually acted on (0/1/2) - dedupes repeated
    // sends the same way _prevProfile does, so re-sending the current mode
    // doesn't re-run enableTorque()/setGain() on every packet.
    uint8_t _prevJointMode = 0;
    // Untested starting point - the right value for "resists a push but
    // doesn't fight hand-posing" depends on gearing/friction/leg mass and
    // needs tuning on the actual robot. Lower = less resistance.
    static const uint16_t DAMPING_GAIN = 30;
    bool _jumpEnabled = true;
    RGBManager* _rgb = nullptr;
    GaitRunner* _gaitRunner = nullptr;
    macStorage* _storage = nullptr;
    PowerRail* _rail5V = nullptr;
    PowerRail* _rail6V = nullptr;
    bool _servosUnpowered = false;  // POWER_RAIL cut the 6V buck - torque stays off until a restart
    // What setProfile() / setGain() last wrote to every servo - what a
    // rebooted one gets back.
    uint16_t _appliedProfile = 0;
    uint16_t _appliedGain = 400;
    // A restarted servo's RAM is back at its defaults: put back the profile
    // and gain the others run with.
    void restoreSettings(uint8_t id);
    uint8_t _restored[8] = {0};   // torque watchdog recoveries per slot (wraps at 255)

    // The pose every joint was last told to take, in its own turn - what
    // bulkWrite() last wrote, or where holdPresentPose() held it. Not valid
    // until one of them has run.
    int32_t _commanded[8] = {0};
    bool _commandedValid = false;
    // After a servo comes back (the torque watchdog, SERVO_REBOOT): every
    // joint back to `commanded`, the ones in slowMask (bit = slot - 1) -
    // limp until just now, their legs dropped - over RETURN_MS rather than
    // at full speed. Not during a gait: its next frames drive every leg.
    void returnToCommanded(const int32_t commanded[8], uint8_t slowMask);
    static const uint16_t RETURN_MS = 1000;
    volatile uint8_t _slowMask = 0;   // servos still on that slow profile...
    uint32_t _slowUntil = 0;          // ...until about then - see serviceReturn()
    int32_t _deg2Dxl(float deg);
    float _dxl2Deg(int32_t dxlRaw);
    // Plays a frame table at a constant per-frame delay. Used by trotDemo().
    void _playFrames(const float frames[][8], uint8_t frameCount, uint8_t reps, uint16_t delayMs);
    void expandArrays();
    // Make every servo's goal its present position, so torque coming on
    // moves nothing - re-learning the turns first. False if they didn't all
    // answer.
    bool holdPresentPose();

    // Which turn each servo counts itself in, in ticks (a whole number of
    // turns). After a power-up a servo rebuilds its multi-turn count from
    // its single-turn encoder, and a reversed one (the right legs) can come
    // up whole turns away from the turn its joint's angles are written for -
    // how many depends on the servo's own firmware and even on how it was
    // last restarted, so it can't be fixed reliably in its homing offset.
    // Instead every goal written gets this added and every position read
    // has it taken off, so the rest of the firmware - and the host - always
    // see each joint in its own turn.
    static const int32_t TICKS_PER_TURN = 4096;
    int32_t _turnTicks[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // Set _turnTicks from positions already in _sr_data: whole turns outside
    // half a turn either side of the joint's idle angle - a leg can't go a
    // full turn, and that window still holds the jump's crouch and full
    // extension. Says which joints changed.
    void adoptTurns();
    // Read every servo's position and adoptTurns(). False if they didn't all answer.
    bool learnTurns();
    // The goal-position write itself, in the servos' own (raw) ticks.
    void writeGoalsRaw(const int32_t raw[8]);
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

    // Second and third sync-read sets, for readServoHealth(). Separate from
    // _sr_* so the reads don't have to reconfigure each other's
    // address/length every time they alternate.
    const uint16_t TR_START_ADDR = 144;  // Present Input Voltage, then Present Temperature
    const uint16_t TR_ADDR_LEN = 3;
    typedef struct tr_data{
      uint16_t present_input_voltage;    // 0.1 V
      uint8_t present_temperature;
    } __attribute__((packed)) tr_data_t;
    tr_data_t _tr_data[_idCount];
    uint8_t _tr_pkt_buf[_user_pkt_buf_cap];
    DYNAMIXEL::InfoSyncReadInst_t _tr_infos;
    DYNAMIXEL::XELInfoSyncRead_t _info_xels_tr[_idCount];
    // Torque Enable (64) through Hardware Error Status (70) in one read -
    // what the torque watchdog needs. 66-67 aren't used; they read as 0.
    const uint16_t ER_START_ADDR = 64;
    const uint16_t ER_ADDR_LEN = 7;
    typedef struct er_data{
      uint8_t torque_enable;             // 64
      uint8_t led;                       // 65
      uint8_t unused[2];                 // 66-67
      uint8_t status_return_level;       // 68
      uint8_t registered_instruction;    // 69
      uint8_t hardware_error_status;     // 70
    } __attribute__((packed)) er_data_t;
    er_data_t _er_data[_idCount];
    uint8_t _er_pkt_buf[_user_pkt_buf_cap];
    DYNAMIXEL::InfoSyncReadInst_t _er_infos;
    DYNAMIXEL::XELInfoSyncRead_t _info_xels_er[_idCount];
    sw_data_t _sw_data[_idCount];
    DYNAMIXEL::InfoSyncWriteInst_t _sw_infos;
    DYNAMIXEL::XELInfoSyncWrite_t _info_xels_sw[_idCount];
};

#endif