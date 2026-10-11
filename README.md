# Q8botOne Firmware

Firmware for [Q8botOne](https://www.crowdsupply.com/zerowire-robotics/q8botone), a
quadruped robot with no physical wires that fits in the palm of your hand.

> **Preliminary release.** This code is shared early so backers can see where
> it stands. It will be updated constantly throughout the campaign and before
> delivery, so expect things to change.

## What's here

- `robot/` - firmware for the robot
- `controller/` - firmware for the handheld controller

Both are [PlatformIO](https://platformio.org/) projects for the Seeed Studio
XIAO ESP32-C3. To build one, open its folder and run `pio run`.

## Questions and feedback

Open an [issue](../../issues) here on GitHub, or email
[yufeng.wu0902@gmail.com](mailto:yufeng.wu0902@gmail.com).

## Thanks

Thanks to Pranay Sanghai for a fix that lets the firmware build on Linux.

## License

MIT - see [LICENSE](LICENSE).
