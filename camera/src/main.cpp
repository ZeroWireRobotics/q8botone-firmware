#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

#include "stream_stats.h"
#include "wifiSetup.h"
#include "robotLink.h"

const uint32_t STATUS_PERIOD_MS = 5000;   // a status line this often, over USB serial
const uint32_t WIFI_CHECK_MS = 500;       // how often wifiKeepJoined() looks

// Which firmware this is - sent back as a line when the host writes 'i' over
// USB, like the robot's and the controller's, so the desktop app can tell the
// camera from them (they share a USB ID). The version is stamped in at build
// time from `git describe` (see scripts/fw_version.py). Must match
// q8botone-python/q8bot/firmware_flasher.py.
#ifndef Q8BOT_FW_VERSION
#define Q8BOT_FW_VERSION "unknown"
#endif
static const char FIRMWARE_ID[] = "Q8BOT_FW|camera|" Q8BOT_FW_VERSION;

void startCameraServer();
void setupLedFlash(int pin);
void printStatus();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  // Wi-Fi joins in the background: the camera must take its network from the
  // desktop app (robotLink) even while it has none. The web server listens
  // from the start and answers once the camera is on the network.
  wifiBegin();
  startCameraServer();
  linkBegin(FIRMWARE_ID);
  Serial.printf("%s ready - its address shows in the status line once it's on Wi-Fi\n", FIRMWARE_ID);
}

void loop() {
  static uint32_t lastWifiCheck = 0, lastStatus = 0;
  linkPoll();
  uint32_t now = millis();
  if (now - lastWifiCheck >= WIFI_CHECK_MS) {
    lastWifiCheck = now;
    wifiKeepJoined();
  }
  if (now - lastStatus >= STATUS_PERIOD_MS) {
    lastStatus = now;
    printStatus();
  }
  delay(1);
}

// Every STATUS_PERIOD_MS: where to find the camera, its memory, and what the
// stream did since the last line - the worst gap between frames, split into
// waiting for the camera and sending over Wi-Fi, shows which one stalls.
void printStatus() {
  static uint32_t lastFrames = 0, lastBytes = 0, lastMs = 0;
  uint32_t now = millis();
  uint32_t frames = streamStats.frames, bytes = streamStats.bytes;
  uint32_t newFrames = frames - lastFrames;
  float fps = lastMs ? newFrames * 1000.0f / (now - lastMs) : 0;
  uint32_t kbPerFrame = newFrames ? (bytes - lastBytes) / newFrames / 1024 : 0;
  uint32_t gap = streamStats.worstGapMs, cam = streamStats.worstCamMs, send = streamStats.worstSendMs;
  streamStats.worstGapMs = streamStats.worstCamMs = streamStats.worstSendMs = 0;
  lastFrames = frames;
  lastBytes = bytes;
  lastMs = now;

  Serial.printf("[cam] %s RSSI %d | PSRAM %u KB | heap %u KB (min %u) | %u stream(s) %.1f fps %u KB/frame"
                " | worst gap %u ms (camera %u, send %u) | fails: camera %u, send %u | ended %u\n",
                WiFi.isConnected() ? WiFi.localIP().toString().c_str() : "no Wi-Fi", WiFi.RSSI(),
                ESP.getPsramSize() / 1024,
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
                heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024,
                streamStats.clients, fps, kbPerFrame, gap, cam, send,
                streamStats.camFails, streamStats.sendFails, streamStats.ended);
}