# Q8botOne Firmware

Firmware for **Q8botOne**, a
quadruped robot with no physical wires that fits in the palm of your hand. Currently on sale at [**Crowd Supply**](https://www.crowdsupply.com/zerowire-robotics/q8botone).

> **Preliminary release.** This code is shared early so backers can see where
> it stands. It will be updated constantly throughout the campaign and before
> delivery, so please expect things to change.

## What's here

- `robot/` - firmware for the robot
- `controller/` - firmware for the handheld controller

Both are [PlatformIO](https://platformio.org/) projects for the Seeed Studio
XIAO ESP32-C3. To build one, open its folder and run `pio run`. More detailed instructions can be found on PlatformIO's [website](https://platformio.org/).

## Questions and feedback

If you have any questions, please open an [issue](https://github.com/ZeroWireRobotics/q8botone-firmware/issues) here on GitHub, or email
[yufeng.wu0902@gmail.com](mailto:yufeng.wu0902@gmail.com).

## Thanks

Thanks to [Pranay Sanghai](https://github.com/gamingdoom) for
[a fix](https://github.com/EricYufengWu/q8bot/commit/d5f4f80031dff06f436802c8470826c2b9954855) that lets the firmware build on Linux. He made it in the original [Q8bot](https://github.com/EricYufengWu/q8bot) repository, and it came along when part of the firmware moved here.

## License

MIT - see [LICENSE](LICENSE).
