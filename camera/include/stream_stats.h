#pragma once
#include <stdint.h>

// What the video stream has been doing. The stream handler (app_httpd.cpp)
// counts; main.cpp's loop() prints a status line every few seconds and
// resets the worst* fields after each one.
struct StreamStats {
  volatile uint32_t clients;      // streams open now
  volatile uint32_t frames;       // frames sent since boot
  volatile uint32_t bytes;        // bytes of them
  volatile uint32_t camFails;     // esp_camera_fb_get() gave no frame
  volatile uint32_t sendFails;    // a send to the browser failed
  volatile uint32_t ended;        // streams that have ended
  volatile uint32_t worstGapMs;   // longest time between two frames sent...
  volatile uint32_t worstCamMs;   // ...longest wait for the camera's frame
  volatile uint32_t worstSendMs;  // ...longest send of one frame
};

extern StreamStats streamStats;
