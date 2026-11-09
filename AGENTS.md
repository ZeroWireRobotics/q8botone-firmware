# AGENTS.md

A brief for AI coding assistants - Claude Code, Codex and others read this
file when they start in this repository. People: see [README.md](README.md).

## What this is

Firmware for Q8botOne's boards, Arduino framework under PlatformIO - the robot
and the controller are each a Seeed Studio XIAO ESP32-C3, the camera a XIAO
ESP32S3 Sense:

- `robot/` - the robot: its eight DYNAMIXEL XL330 servos (`q8Dynamixel`), gait
  playback (`gaitRunner`), battery and charger (`battery`), power rails
  (`powerRail`), sensors on the Qwiic port (`sensors`, `i2cBus`), motor setup
  (`motorSetup`), the UART link to the camera (`cameraLink`)
- `controller/` - the handheld controller: its joystick and buttons
  (`gamepadQT`), driving the robot on its own (`independentControl`), and
  relaying a computer's commands from USB serial to the robot by radio (ESP-NOW)
- `camera/` - the camera: video in a browser over Wi-Fi (`app_httpd`), its
  Wi-Fi network (`wifiSetup`, set from the desktop app over USB), and the
  UART link to the robot (`robotLink`)
- `scripts/fw_version.py` - stamps the version into the build from `git describe`

Messages are routed by type (see `protocol.h`): the robot passes the host's
camera commands to the camera and the camera's replies back, and carries out
robot commands that come from the camera as it does the host's.

The other end of the protocol - the Python library, the desktop app and the
simulator - is `q8botone-python`, usually cloned next to this repository
(`../q8botone-python`).

## Building

```
cd robot && pio run                  # env robot_permanent
cd controller && pio run             # env controller_permanent
cd camera && pio run                 # env camera
```

The version comes from the latest `vX.Y.Z` git tag - never write one by hand.

## Rules

- **Uploading moves hardware.** `pio run -t upload` flashes whichever board is
  plugged in. Only upload when the user asks and says the robot is ready;
  building is always fine.
- **The protocol has four copies.** `robot/include/protocol.h`,
  `controller/include/protocol.h` and `camera/include/protocol.h` are
  identical - change one, copy it to the other two - and
  `../q8botone-python/q8bot/protocol.py` must agree with them: command IDs,
  payload layouts, direction order. Bump `PROTOCOL_VERSION` in all of them for
  any change the other side has to know about.
- **Wi-Fi passwords stay off the radio.** The robot-controller radio isn't
  encrypted, so the camera takes `CAM_WIFI_SET` only over its own USB and
  ignores one that arrives through the robot. Keep it that way.
- **Generated file.** `robot/include/gaitTables.h` is written by
  `q8botone-python` (`python -m q8bot.export_gait_tables`) - change the gaits
  there, not here.
- **The simulator copies this firmware.** `../q8botone-python/q8botone_sim/firmware.py`
  redoes the robot's gait player and servo motion; a change to how the robot
  behaves belongs there too.
- **The controller copies the library.** `controller/src/independentControl.cpp`
  is ported from q8botone-python's `control_config.py`, `GaitManager` speed
  tiers and `kinematics_solver.py` (see its header), so it drives as the
  desktop app does; keep them in step.
- **Commits.** `git config core.hooksPath .githooks` turns on hooks that stamp
  the firmware and protocol versions into each commit message.
