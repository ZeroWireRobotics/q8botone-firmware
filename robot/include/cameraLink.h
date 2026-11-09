#ifndef cameraLink_h
#define cameraLink_h

#include <Arduino.h>

// The camera link: a UART to the camera (a XIAO ESP32S3) on the ESP32-C3's
// second UART - the first drives the servos - carrying protocol.h's frames
// both ways at CAMERA_LINK_BAUD. Pins in pinMapping.h.
//
// The robot routes by message type (see protocol.h): commands for the camera
// from the host go out here unchanged; the camera's replies go on to the
// host; and commands for the robot that come from the camera are carried out
// like the host's, with any reply going back to the camera.

// Called with each whole, CRC-checked frame the camera sends.
typedef void (*CameraFrameHandler)(const uint8_t* frame, size_t len);

void cameraLinkBegin();

// Sends one complete frame to the camera. Safe from any task: a frame goes
// out in a single write, which the UART driver keeps whole.
void cameraLinkSend(const uint8_t* frame, size_t len);

// FreeRTOS task: reads the camera's frames and hands each one to the
// CameraFrameHandler passed as its parameter.
void cameraLinkTask(void* parameter);

#endif
