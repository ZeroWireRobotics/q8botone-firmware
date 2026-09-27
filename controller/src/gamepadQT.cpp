#include "gamepadQT.h"
#include <Wire.h>

namespace {
  const uint8_t GP_JOYSTICK_X = 14;
  const uint8_t GP_JOYSTICK_Y = 15;
}

const uint8_t GamepadQT::_pins[GamepadQT::BUTTON_COUNT] = {
  6,   // BTN_X
  2,   // BTN_Y
  5,   // BTN_A
  1,   // BTN_B
  0,   // BTN_SELECT
  16,  // BTN_START
};

const uint32_t GamepadQT::_buttonMask =
    (1UL << 6) | (1UL << 2) | (1UL << 5) | (1UL << 1) | (1UL << 0) | (1UL << 16);

bool GamepadQT::begin(uint8_t addr) {
  Wire.begin();  // Board default I2C pins

  if (!_ss.begin(addr)) {
    return false;
  }

  _ss.pinModeBulk(_buttonMask, INPUT_PULLUP);

  // Calibrate joystick center from a few readings at startup - manufacturing
  // variance means it's rarely exactly 512 (e.g. one observed unit sat at
  // X=499, Y=511), and this avoids hardcoding a single unit's numbers.
  long sumX = 0, sumY = 0;
  const int samples = 8;
  for (int i = 0; i < samples; i++) {
    sumX += _ss.analogRead(GP_JOYSTICK_X);
    sumY += _ss.analogRead(GP_JOYSTICK_Y);
    delay(5);
  }
  _joyXCenter = sumX / samples;
  _joyYCenter = sumY / samples;

  _began = true;
  return true;
}

void GamepadQT::update() {
  if (!_began) return;

  uint32_t buttons = _ss.digitalReadBulk(_buttonMask);
  uint32_t now = millis();

  for (int i = 0; i < BUTTON_COUNT; i++) {
    bool raw = !(buttons & (1UL << _pins[i]));  // Active-low (pulled up, pressed = pulled to ground)
    _pressedEdge[i] = false;

    if (raw != _rawState[i]) {
      _rawState[i] = raw;
      _lastChangeMs[i] = now;
    } else if (raw != _debouncedState[i] && (now - _lastChangeMs[i]) >= BUTTON_DEBOUNCE_MS) {
      _debouncedState[i] = raw;
      if (raw) _pressedEdge[i] = true;
    }
  }

  if (_debouncedState[BTN_START] && _debouncedState[BTN_SELECT]) {
    if (_comboStartMs == 0) _comboStartMs = now;
  } else {
    _comboStartMs = 0;
  }

  // Joystick: normalize so X negative=left/positive=right, Y
  // negative=forward(up)/positive=backward(down) - matches
  // control_config.py's convention. Raw seesaw reading has right=0/left=1023
  // and up=0/down=1023 (verified on hardware), hence the X inversion below.
  int rawX = _ss.analogRead(GP_JOYSTICK_X);
  int rawY = _ss.analogRead(GP_JOYSTICK_Y);

  float normX = (float)(_joyXCenter - rawX) / JOY_HALF_RANGE;
  float normY = (float)(rawY - _joyYCenter) / JOY_HALF_RANGE;

  _joyXNorm = (fabsf(normX) < JOY_DEADZONE) ? 0.0f : constrain(normX, -1.0f, 1.0f);
  _joyYNorm = (fabsf(normY) < JOY_DEADZONE) ? 0.0f : constrain(normY, -1.0f, 1.0f);
}

bool GamepadQT::wasPressed(Button b) const {
  return _pressedEdge[b];
}

bool GamepadQT::isPressed(Button b) const {
  return _debouncedState[b];
}

bool GamepadQT::comboHeldFor(uint32_t holdMs) const {
  return _comboStartMs != 0 && (millis() - _comboStartMs) >= holdMs;
}

float GamepadQT::joystickX() const {
  return _joyXNorm;
}

float GamepadQT::joystickY() const {
  return _joyYNorm;
}
