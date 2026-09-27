#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class RGBManager {
public:
  RGBManager(uint8_t pin);
  void begin(uint8_t brightness);
  void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0);
  void setHue(uint16_t hue);  // 0–65535, full HSV wheel
  void off();
  // Global brightness scale (0-255) applied to every subsequent color -
  // re-shows the current color immediately so a live update is visible
  // right away rather than waiting for the next status change.
  void setBrightness(uint8_t brightness);

private:
  Adafruit_NeoPixel _pixel;
};
