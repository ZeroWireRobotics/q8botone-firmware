#pragma once

#include <Arduino.h>

// The camera's Wi-Fi network, saved in flash (NVS). The desktop app sets it
// over the camera's USB (CAM_WIFI_SET in protocol.h); a build with an
// include/wifi_config.h starts out on that file's network until then.

// Starts joining the saved network, if there is one - doesn't wait for it.
void wifiBegin();

// Call every half second or so: starts the join again when the router
// refused it (the Wi-Fi library then gives up for good) or it's taking too
// long, so the camera never stays off the network until a restart.
void wifiKeepJoined();

// Saves a network and joins it. False, keeping the old one, unless the SSID
// is 1-32 bytes and the password empty (an open network) or 8-64 characters.
bool wifiSet(const uint8_t* ssid, size_t ssidLen, const uint8_t* password, size_t passwordLen);

// CAM_STATUS's payload (see protocol.h) into out; returns its length.
size_t wifiStatusPayload(uint8_t* out, size_t capacity);
