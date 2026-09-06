#ifndef MACSTORAGE_H
#define MACSTORAGE_H

#include <Arduino.h>
#include <Preferences.h>

class macStorage {
private:
  Preferences _prefs;
  const char* _namespace = "q8bot";
  const char* _macKey = "peerMAC";
  const char* _ledBrightKey = "ledBright";

public:
  macStorage();

  // Load peer MAC address from NVS
  // Returns true if MAC found, false otherwise
  bool loadPeerMAC(uint8_t* mac);

  // Save peer MAC address to NVS
  void savePeerMAC(const uint8_t* mac);

  // Clear saved peer MAC from NVS
  void clearPeerMAC();

  // Load RGB LED brightness (0-255) from NVS, or defaultValue if never saved.
  uint8_t loadLedBrightness(uint8_t defaultValue);

  // Save RGB LED brightness (0-255) to NVS.
  void saveLedBrightness(uint8_t brightness);
};

#endif
