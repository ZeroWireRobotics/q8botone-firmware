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

## Working with an AI coding assistant

An AI coding assistant such as Claude Code or Codex can read this firmware
for you: explain how a part of it works, find where something happens, or
make a change with you and build it.

Install one - each needs its own account, and signs you in the first time
you start it:

- **Claude Code** ([quickstart](https://code.claude.com/docs/en/quickstart)) -
  a Claude subscription or an Anthropic Console account:

  ```
  irm https://claude.ai/install.ps1 | iex                  # Windows, in PowerShell
  curl -fsSL https://claude.ai/install.sh | bash           # macOS / Linux
  ```

- **Codex** ([README](https://github.com/openai/codex)) - a ChatGPT plan or an
  OpenAI API key:

  ```
  powershell -ExecutionPolicy ByPass -c "irm https://chatgpt.com/codex/install.ps1 | iex"   # Windows
  curl -fsSL https://chatgpt.com/codex/install.sh | sh                                      # macOS / Linux
  ```

Both also come as a VS Code extension, if you'd rather not use a terminal.

Then open a terminal in this folder and run `claude` or `codex`. You don't
need to explain the project first: both read [`AGENTS.md`](AGENTS.md) when
they start - a short brief on what's where, how to build, and which files
have to stay in step with the Python side. With
[q8botone-python](https://github.com/ZeroWireRobotics/q8botone-python) cloned
next to this folder, it can follow a command from the desktop app all the
way to the servos.

Some things to ask:

- *What happens on the robot from a walk command arriving to the legs moving?*
- *How does the controller drive the robot on its own, without a computer?*
- *Build the robot firmware and explain any warnings.*

It edits files and runs commands as it works - how much it asks first
depends on its settings - so look over what it changed (`git diff`) before
you keep it. Upload new firmware yourself, with the robot somewhere it can
move safely.

## Questions and feedback

If you have any questions, please open an [issue](https://github.com/ZeroWireRobotics/q8botone-firmware/issues) here on GitHub, or email
[yufeng.wu0902@gmail.com](mailto:yufeng.wu0902@gmail.com).

## Thanks

Thanks to [Pranay Sanghai](https://github.com/gamingdoom) for
[a fix](https://github.com/EricYufengWu/q8bot/commit/d5f4f80031dff06f436802c8470826c2b9954855) that lets the firmware build on Linux. He made it in the original [Q8bot](https://github.com/EricYufengWu/q8bot) repository, and it came along when part of the firmware moved here.

## License

MIT - see [LICENSE](LICENSE).
