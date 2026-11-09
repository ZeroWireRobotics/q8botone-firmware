#include "wifiSetup.h"

#include <Preferences.h>
#include <WiFi.h>

#include "protocol.h"

#if __has_include("wifi_config.h")
#include "wifi_config.h"   // optional, gitignored: a network to start with
#define HAVE_WIFI_CONFIG 1
#endif

const uint32_t WIFI_RETRY_MS = 15000;      // not joined for this long: start the join again
const uint32_t WIFI_FAILED_RETRY_MS = 3000;   // ...or this long after the library gave up

static const char* PREFS_NAMESPACE = "wifi";
static char savedSsid[33] = "";
static char savedPassword[65] = "";
static uint32_t joinStart = 0;
// Why the last attempt to join failed (an ESP-IDF wifi_err_reason_t), 0 once
// joined or since a new network was set.
static volatile uint8_t lastFailure = 0;
// join() abandons any attempt in progress, whose failure then still arrives
// a moment later - it isn't the new attempt's, so it's ignored. A real
// failure takes over a second (a timeout, or a scan).
const uint32_t STALE_FAILURE_MS = 500;
static volatile uint32_t joinedAt = 0;

static void onDisconnected(WiFiEvent_t, WiFiEventInfo_t info) {
  uint8_t reason = info.wifi_sta_disconnected.reason;
  if (reason != WIFI_REASON_ASSOC_LEAVE && millis() - joinedAt > STALE_FAILURE_MS) {
    lastFailure = reason;
  }
}

static void onGotIp(WiFiEvent_t, WiFiEventInfo_t) {
  lastFailure = 0;
}

static void join() {
  joinedAt = millis();
  WiFi.disconnect();
  WiFi.begin(savedSsid, savedPassword);
  joinStart = millis();
}

void wifiBegin() {
  WiFi.persistent(false);   // the network lives in our own NVS namespace - one flash write, not two
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.onEvent(onDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.onEvent(onGotIp, ARDUINO_EVENT_WIFI_STA_GOT_IP);

  Preferences prefs;
  prefs.begin(PREFS_NAMESPACE, false);
  if (prefs.isKey("ssid")) {
    prefs.getString("ssid", savedSsid, sizeof(savedSsid));
    prefs.getString("password", savedPassword, sizeof(savedPassword));
  }
  prefs.end();
#ifdef HAVE_WIFI_CONFIG
  if (!savedSsid[0]) {
    strlcpy(savedSsid, ssid, sizeof(savedSsid));
    strlcpy(savedPassword, password, sizeof(savedPassword));
  }
#endif

  if (savedSsid[0]) {
    Serial.printf("[wifi] Joining %s\n", savedSsid);
    WiFi.begin(savedSsid, savedPassword);
    joinStart = millis();
  } else {
    Serial.println("[wifi] No network set - set one from the desktop app (Tools > Camera Wi-Fi)");
  }
}

void wifiKeepJoined() {
  if (!savedSsid[0]) return;
  if (WiFi.status() == WL_CONNECTED) {
    joinStart = millis();
    return;
  }
  uint32_t trying = millis() - joinStart;
  if ((WiFi.status() == WL_CONNECT_FAILED && trying > WIFI_FAILED_RETRY_MS) || trying > WIFI_RETRY_MS) {
    Serial.printf("[wifi] %s not joined (status %d, last failure %u) - trying again\n",
                  savedSsid, WiFi.status(), lastFailure);
    join();
  }
}

bool wifiSet(const uint8_t* ssid, size_t ssidLen, const uint8_t* password, size_t passwordLen) {
  if (ssidLen < 1 || ssidLen > 32 || memchr(ssid, 0, ssidLen)) return false;
  if (passwordLen != 0 && (passwordLen < 8 || passwordLen > 64)) return false;
  if (memchr(password, 0, passwordLen)) return false;

  memcpy(savedSsid, ssid, ssidLen);
  savedSsid[ssidLen] = 0;
  memcpy(savedPassword, password, passwordLen);
  savedPassword[passwordLen] = 0;

  Preferences prefs;
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putString("ssid", savedSsid);
  prefs.putString("password", savedPassword);
  prefs.end();

  lastFailure = 0;
  Serial.printf("[wifi] Network set: joining %s\n", savedSsid);
  join();
  return true;
}

size_t wifiStatusPayload(uint8_t* out, size_t capacity) {
  size_t ssidLen = strlen(savedSsid);
  if (capacity < CAM_STATUS_HEADER_LEN + ssidLen) return 0;

  bool connected = WiFi.status() == WL_CONNECTED;
  CameraWifi state = !savedSsid[0] ? CameraWifi::NOT_SET
                   : connected     ? CameraWifi::CONNECTED
                   : lastFailure   ? CameraWifi::FAILED
                                   : CameraWifi::CONNECTING;
  IPAddress ip = connected ? WiFi.localIP() : IPAddress(0, 0, 0, 0);

  out[0] = PROTOCOL_VERSION;
  out[1] = (uint8_t)state;
  out[2] = lastFailure;
  out[3] = connected ? (uint8_t)(int8_t)WiFi.RSSI() : 0;
  for (int i = 0; i < 4; i++) out[4 + i] = ip[i];
  out[8] = (uint8_t)ssidLen;
  memcpy(out + CAM_STATUS_HEADER_LEN, savedSsid, ssidLen);
  return CAM_STATUS_HEADER_LEN + ssidLen;
}
