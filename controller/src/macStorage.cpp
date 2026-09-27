#include "macStorage.h"

macStorage::macStorage() {
  // Constructor
}

bool macStorage::loadPeerMAC(uint8_t* mac) {
  _prefs.begin(_namespace, false);
  size_t len = _prefs.getBytesLength(_macKey);
  if (len == 6) {
    _prefs.getBytes(_macKey, mac, 6);
    _prefs.end();
    return true;
  }
  _prefs.end();
  return false;
}

void macStorage::savePeerMAC(const uint8_t* mac) {
  _prefs.begin(_namespace, false);
  _prefs.putBytes(_macKey, mac, 6);
  _prefs.end();
}

void macStorage::clearPeerMAC() {
  _prefs.begin(_namespace, false);
  _prefs.remove(_macKey);
  _prefs.end();
}

uint8_t macStorage::loadLedBrightness(uint8_t defaultValue) {
  _prefs.begin(_namespace, false);
  uint8_t brightness = _prefs.getUChar(_ledBrightKey, defaultValue);
  _prefs.end();
  return brightness;
}

void macStorage::saveLedBrightness(uint8_t brightness) {
  _prefs.begin(_namespace, false);
  _prefs.putUChar(_ledBrightKey, brightness);
  _prefs.end();
}
