/*
  gamepadQT.h - Wrapper for the Adafruit Mini I2C Gamepad (seesaw
  co-processor, STEMMA QT, default I2C addr 0x50), wired to this board's
  default I2C pins (Wire.begin() with no explicit SDA/SCL args).

  Adds software debounce on top of the raw seesaw button reads, and
  normalizes/deadzones the joystick to match q8botone-python's
  convention (control_config.py): X negative=left/positive=right,
  Y negative=forward(up)/positive=backward(down). Pin numbers below are on
  the seesaw's own GPIO space (Adafruit's Gamepad QT pinout reference) and
  the axis sign convention was verified live on hardware
  (rt_probe.py-style bring-up testing) - not the seesaw datasheet alone.
*/
#ifndef gamepadQT_h
#define gamepadQT_h

#include <Arduino.h>
#include <Adafruit_seesaw.h>

class GamepadQT {
  public:
    enum Button { BTN_X, BTN_Y, BTN_A, BTN_B, BTN_SELECT, BTN_START, BUTTON_COUNT };

    // Poll cadence this class is designed to be called at.
    static const uint32_t GAMEPAD_POLL_MS = 20;

    bool begin(uint8_t addr = 0x50);

    // Reads the gamepad and updates debounced button/joystick state - call
    // once per GAMEPAD_POLL_MS from a dedicated task, before consuming any
    // of the accessors below.
    void update();

    // True for exactly one update() call - the one where a button's
    // debounced state transitions to pressed (edge-triggered "tap", not
    // level - a held button fires once until released).
    bool wasPressed(Button b) const;

    // Raw debounced level (true while held).
    bool isPressed(Button b) const;

    // True if BTN_START and BTN_SELECT have both been continuously held
    // (post-debounce) for at least holdMs. A level check, not edge-
    // triggered - stays true every poll for as long as the hold continues
    // past the threshold. Callers that want a one-shot action (e.g.
    // "enter pairing mode") need their own latch, cleared once either
    // button releases.
    bool comboHeldFor(uint32_t holdMs) const;

    // Joystick axes, deadzoned, range -1.0..1.0 (0 = centered). Convention
    // matches control_config.py: X negative=left/positive=right,
    // Y negative=forward/positive=backward.
    float joystickX() const;
    float joystickY() const;

  private:
    static const uint32_t BUTTON_DEBOUNCE_MS = 30;  // Raise if you see false triggers, lower if presses feel laggy
    static const int JOY_HALF_RANGE = 512;          // ADC counts from center to full deflection (0-1023 range)
    static constexpr float JOY_DEADZONE = 0.08f;    // Normalized units

    Adafruit_seesaw _ss;
    bool _began = false;

    bool _rawState[BUTTON_COUNT] = {};
    bool _debouncedState[BUTTON_COUNT] = {};
    bool _pressedEdge[BUTTON_COUNT] = {};
    uint32_t _lastChangeMs[BUTTON_COUNT] = {};

    uint32_t _comboStartMs = 0;  // 0 = not currently holding the combo

    int _joyXCenter = 512;
    int _joyYCenter = 512;
    float _joyXNorm = 0.0f;
    float _joyYNorm = 0.0f;

    static const uint8_t _pins[BUTTON_COUNT];
    static const uint32_t _buttonMask;
};

#endif
