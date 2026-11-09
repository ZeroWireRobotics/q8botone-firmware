#include "cameraLink.h"
#include "pinMapping.h"
#include "protocol.h"

static HardwareSerial& cameraSerial = Serial1;   // UART1 - UART0 is the servo bus

void cameraLinkBegin() {
  cameraSerial.setRxBufferSize(1024);
  cameraSerial.setTxBufferSize(1024);   // a reply never waits on the wire
  cameraSerial.begin(CAMERA_LINK_BAUD, SERIAL_8N1, CAMERA_RX_PIN, CAMERA_TX_PIN);
}

void cameraLinkSend(const uint8_t* frame, size_t len) {
  cameraSerial.write(frame, len);
}

void cameraLinkTask(void* parameter) {
  CameraFrameHandler onFrame = (CameraFrameHandler)parameter;
  static FrameReader reader;   // only this task uses it

  while (true) {
    int n = cameraSerial.available();
    while (n-- > 0) {
      if (reader.feed((uint8_t)cameraSerial.read())) {
        onFrame(reader.frame(), reader.length());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
