#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class RGBManager {
public:
  RGBManager(uint8_t pin);
  void begin();
  void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0);
  void setHue(uint16_t hue);  // 0–65535, full HSV wheel
  void off();

private:
  Adafruit_NeoPixel _pixel;
};
