#include "gaitRunner.h"
#include "gaitTables.h"

GaitRunner::GaitRunner(q8Dynamixel& q8) : _q8(q8) {}

void GaitRunner::start(uint8_t gaitId, uint8_t directionId){
  bool freshStart = !_active || gaitId != _gaitId;

  if (freshStart){
    _phaseIndex = 0;
    _q8.setProfile(0);  // max speed/no ramp, matching the host's per-tick dur=0 semantics
  }

  _gaitId = gaitId;
  _directionId = directionId;
  _active = true;
}

void GaitRunner::stop(){
  _active = false;
}

bool GaitRunner::isActive() const {
  return _active;
}

void GaitRunner::tick(){
  if (!_active) return;
  if (_gaitId >= GAIT_TABLE_COUNT || GAIT_TABLES[_gaitId] == nullptr) return;
  if (_directionId >= 12) return;

  const GaitDirectionTable& table = GAIT_TABLES[_gaitId][_directionId];
  if (table.count == 0) return;

  const GaitFrame& frame = table.frames[_phaseIndex % table.count];
  _q8.bulkWriteDeg(frame.degs);
  _phaseIndex = (_phaseIndex + 1) % table.count;
}
