#pragma once

#include <Arduino.h>

#include "protocol.h"

// The camera's two ways in for protocol.h's frames: its USB, where the
// desktop app connects, and the UART to the robot (D7 in, from the robot's
// TX1; D8 out, to its RX1). linkPoll() reads both and carries out what
// arrives, answering the way it came:
//
//   CAM_STATUS_REQUEST   from either
//   CAM_WIFI_SET         from USB only - the robot's side reaches the radio,
//                        which isn't encrypted, so a password never comes that way
//   the robot's replies  to sendToRobot()'s commands: onRobotReply() in robotLink.cpp
//
// Over USB the camera also takes single-letter keys: i (which firmware this
// is), and b, t, s - ask the robot for its battery, joints or sensors, to
// check the link both ways.

// firmwareId: what 'i' answers - see main.cpp's FIRMWARE_ID.
void linkBegin(const char* firmwareId);

// Call often, from loop().
void linkPoll();

// Sends the robot one of its own commands (protocol.h, 0x01-0x1F) - just as
// the desktop app would: MOVE, GAIT_RUN, TORQUE_SET... Its reply, if it has
// one, comes back to onRobotReply().
void sendToRobot(CmdType type, const uint8_t* payload, uint8_t len);
