# PlatformIO Command Reference

Run these commands from the `esp32-drivetrain` project directory, which contains
`platformio.ini`. The long command is `platformio`; `pio` is its shorter alias.

## Project environments

| Environment | Purpose | Entry point |
| --- | --- | --- |
| `esp32-s3-devkitm-1` | Normal/default ESP32 firmware | `src/main.cpp` |
| `tuning` | Wheel PI tuning harness | `src/harnesses/tuning_main.cpp` |
| `drive` | Four-wheel debug drive harness | `src/harnesses/drive_main.cpp` |
| `drivetrain-test` | Interactive motor/encoder and movement acceptance test | `src/harnesses/drivetrain_test_main.cpp` |
| `native` | Unit tests that run on the development computer | Files under `test/` |

The `-e` or `--environment` option chooses an environment. Because
`default_envs = esp32-s3-devkitm-1` is set in `platformio.ini`, commands that
omit `-e` use the normal firmware environment.

## Build firmware

```powershell
pio run
```

Build the normal firmware without uploading it.

```powershell
pio run -e tuning
pio run -e drive
pio run -e drivetrain-test
```

Build one of the hardware test harnesses. Building does not modify the ESP32.

```powershell
pio run -e esp32-s3-devkitm-1 -e tuning -e drive
```

Build all ESP32 firmware variants in one command.

```powershell
pio run -e drive -v
```

Build with verbose compiler and linker output. This is useful when diagnosing a
dependency, include-path, or linker problem.

## Upload firmware

```powershell
pio run -t upload
```

Build and upload the normal firmware to the configured upload port.

```powershell
pio run -e tuning -t upload
pio run -e drive -t upload
pio run -e drivetrain-test -t upload
```

Build and upload a specific harness. The drive harness can move all four
motors, so raise the robot and make the drivetrain safe before uploading or
issuing commands.

```powershell
pio run -e drive -t upload --upload-port COM12
```

Override `upload_port` for one command without editing `platformio.ini`.

## Serial monitor

```powershell
pio device list
```

List detected serial devices and their COM ports.

```powershell
pio device monitor -e esp32-s3-devkitm-1
pio device monitor -e tuning
pio device monitor -e drive
pio device monitor -e drivetrain-test
```

Open the serial monitor using the selected environment's monitor settings.
Press `Ctrl+C` to exit.

```powershell
pio device monitor -e drive --port COM12 --baud 115200
```

Override the serial port and baud rate for one monitor session.

```powershell
pio run -e drive -t upload -t monitor
```

Upload the drive harness and then open its serial monitor.

## Run tests

```powershell
pio test -e native
```

Build and run all host-based unit tests. No ESP32 is required.

```powershell
pio test -e native --list-tests
```

List the tests PlatformIO discovers without running them.

```powershell
pio test -e native -f "test_wheel_velocity_pi"
```

Run only tests whose name matches the filter.

```powershell
pio test -e native -vv
```

Run tests with additional build and test-runner diagnostics.

## Clean generated build files

```powershell
pio run -t clean
pio run -e tuning -t clean
pio run -e drive -t clean
```

Delete generated files for the selected environment. The next build will be a
full rebuild. Source files are not deleted.

```powershell
pio system prune --dry-run
```

Preview unused PlatformIO packages and caches that could be removed. Review the
output before running `pio system prune` without `--dry-run`.

## Inspect the project

```powershell
pio run --list-targets
```

List build targets supported by the current project, such as `upload`, `clean`,
and `monitor`.

```powershell
pio project config
```

Print the resolved PlatformIO configuration after environment settings and
defaults have been applied.

```powershell
pio boards esp32-s3-devkitm-1
```

Show PlatformIO's board information for the ESP32-S3 DevKitM-1.

```powershell
pio pkg list
pio pkg list -e drive
```

List installed project packages globally or for a selected environment.

## Static analysis and debugging

```powershell
pio check -e esp32-s3-devkitm-1
```

Run PlatformIO static analysis for the normal firmware. Additional analyzer
packages may be installed the first time this runs.

```powershell
pio debug -e esp32-s3-devkitm-1
```

Start PlatformIO's hardware debugger. This requires a supported debug probe and
the corresponding `debug_tool` configuration; it is separate from the `drive`
debug harness.

## Help and version information

```powershell
pio --version
pio --help
pio run --help
pio test --help
pio device monitor --help
```

Show the installed PlatformIO version or detailed help for a command.

## Common workflow

For ordinary firmware development:

```powershell
pio test -e native
pio run
pio run -t upload
pio device monitor -e esp32-s3-devkitm-1
```

For closed-loop drivetrain testing:

```powershell
pio test -e native
pio run -e tuning
pio run -e tuning -t upload
pio device monitor -e tuning
```

For motor/encoder acceptance testing, first lift and secure the robot, then:

```powershell
pio run -e drivetrain-test -t upload
pio device monitor -e drivetrain-test
```

Type `help` for the serial protocol. Start with `pair fl 0.25 1500` (then `fr`,
`bl`, and `br`) and verify that measured encoder velocity has the same sign as
the commanded duty. Use `sequence 0.2 1500` to run forward, backward, left,
right, all four 45-degree diagonals, CW, and CCW with coast pauses. Direction
changes made with `invert motor|encoder <wheel> [0|1]` last only until reboot.

Only use the `drive` or `drivetrain-test` upload after the robot is physically
secured.
