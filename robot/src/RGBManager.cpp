#include "RGBManager.h"

RGBManager::RGBManager(uint8_t pin)
  : _pixel(1, pin, NEO_GRBW + NEO_KHZ800) {}

void RGBManager::begin() {
  _pixel.begin();
  _pixel.setBrightness(64);
  off();
}

void RGBManager::setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  _pixel.setPixelColor(0, _pixel.Color(r, g, b, w));
  _pixel.show();
}

void RGBManager::setHue(uint16_t hue) {
  _pixel.setPixelColor(0, _pixel.gamma32(_pixel.ColorHSV(hue)));
  _pixel.show();
}

void RGBManager::off() {
  _pixel.setPixelColor(0, 0);
  _pixel.show();
}
