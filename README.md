# Red-gobbo

Small PlatformIO firmware project skeleton for the "Red-gobbo" embedded device.

## Overview
Minimal, well-documented starting point for building, flashing and debugging firmware with PlatformIO. Replace placeholders and notes with your project's specifics.

## Features
- PlatformIO build system and environment examples
- Flash / monitor commands
- Basic wiring and configuration notes
- Contributing and license template

## Prerequisites
- PlatformIO IDE or PlatformIO Core (CLI)
- USB cable and compatible development board
- Relevant toolchain installed by PlatformIO

## Example platformio.ini
Replace the environment with the board you actually use.

```ini
[platformio]
default_envs = esp32dev

[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
build_flags = 
    ; adjust defines here
    -D DEBUG
lib_deps =
    ; list dependencies here, e.g. some-library/1.2.3
```

## Quick commands
- Build: pio run
- Build + upload: pio run -t upload
- Clean: pio run -t clean
- Serial monitor: pio device monitor (or pio device monitor -b 115200)

## Flashing steps
1. Connect board via USB.
2. Select the correct env in `platformio.ini` or use `-e <env>`.
3. Run `pio run -t upload`. If manual boot mode is required, follow your board's reset/boot sequence.

## Serial output
Use the serial monitor to view logs:
- Default baud: 115200 (change `monitor_speed` in platformio.ini)
- Example: pio device monitor --port COM3 --baud 115200

## Wiring (example)
- Power: 5V or 3.3V depending on board
- GND: common ground
- GPIOs: annotate pins in code and map to sensors/actuators
(Replace with a full wiring diagram for your hardware)

## Configuration
- Put device-specific settings in a config header or use `build_flags` to inject macros.
- Keep secrets out of the repository; use environment variables or a local file ignored by git.

## Troubleshooting
- Build failures: run `pio run -v` for verbose output.
- Upload issues: check port (`pio device list`) and cable, try manual reset.
- Dependency issues: clear `~/.platformio` or run `pio update`.

## Contributing
- Open issues for bugs or feature requests.
- Use branches and make small, focused pull requests.
- Include a short changelog for non-trivial changes.

## License
MIT — see LICENSE file.

Replace the placeholders (board, wiring, libraries) with details specific to Red-gobbo.