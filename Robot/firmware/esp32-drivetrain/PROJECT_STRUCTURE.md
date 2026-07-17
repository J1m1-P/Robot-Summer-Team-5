# Project Structure

## Purpose

This repository contains the ESP32-S3 firmware for the robot drivetrain board. It combines low-level motor and encoder access, closed-loop wheel velocity control, drivetrain kinematics, tape sensing, board configuration, and several hardware test applications.

The project follows a layered structure: application code calls control modules, control modules call drivers, and configuration supplies board-specific constants to each layer. Pure control math is kept separate from ESP32 hardware where practical so it can be tested on a desktop.

## Directory tree

Generated directories such as `.pio/` and editor metadata under `.vscode/` are intentionally omitted.

```text
esp32-drivetrain/
|-- .gitignore
|-- platformio.ini
|-- PLATFORMIO_COMMANDS.md
|-- PROJECT_STRUCTURE.md
|-- FILE_RESPONSIBILITIES.md
|
|-- include/                         Public project headers
|   |-- README                       PlatformIO header-directory notes
|   |-- config/
|   |   |-- pin_map.h
|   |   |-- drivetrain/
|   |   |   |-- motor_config.h
|   |   |   |-- encoder_config.h
|   |   |   `-- drivetrain_config.h
|   |   |-- tape_following/
|   |   |   `-- tape_following_config.h
|   |   `-- communication/
|   |       |-- i2c_bus_config.h
|   |       `-- uart_link_config.h
|   |-- control/
|   |   |-- drivetrain/
|   |   |   |-- drivetrain.h
|   |   |   |-- velocity_kinematics.h
|   |   |   |-- wheel_velocity_pi.h
|   |   |   `-- odometry.h
|   |   `-- tape_following/
|   |       |-- tape_follower.h
|   |       `-- tape_following_controller.h
|   |-- drivers/
|   |   |-- motor/
|   |   |   `-- motor_driver.h
|   |   |-- encoder/
|   |   |   `-- encoder_driver.h
|   |   `-- tape_sensor/
|   |       `-- tape_sensor_driver.h
|   `-- sensing/
|       `-- tape_following/
|           |-- tape_line_estimator.h
|           `-- tape_task_detection.h
|
|-- src/                             Implementations and applications
|   |-- main.cpp
|   |-- config/
|   |   |-- drivetrain/
|   |   |   |-- motor_config.c
|   |   |   |-- encoder_config.c
|   |   |   `-- drivetrain_config.c
|   |   |-- tape_following/
|   |   |   `-- tape_following_config.c
|   |   `-- communication/
|   |       |-- i2c_bus_config.c
|   |       `-- uart_link_config.c
|   |-- control/
|   |   |-- drivetrain/
|   |   |   |-- drivetrain.c
|   |   |   |-- velocity_kinematics.c
|   |   |   |-- wheel_velocity_pi.c
|   |   |   `-- odometry.c
|   |   `-- tape_following/
|   |       |-- tape_follower.c
|   |       `-- tape_following_controller.c
|   |-- drivers/
|   |   |-- motor/
|   |   |   `-- motor_driver.c
|   |   |-- encoder/
|   |   |   `-- encoder_driver.c
|   |   `-- tape_sensor/
|   |       `-- tape_sensor_driver.c
|   |-- sensing/
|   |   `-- tape_following/
|   |       |-- tape_line_estimator.c
|   |       `-- tape_task_detection.c
|   `-- harnesses/
|       |-- drive_main.cpp
|       |-- drivetrain_test_main.cpp
|       `-- tuning_main.cpp
|
|-- test/                            Native PlatformIO unit tests
|   |-- README
|   |-- native_stubs/
|   |   |-- esp_err.h
|   |   `-- driver/gpio.h
|   |-- test_velocity_kinematics/
|   |   `-- test_velocity_kinematics.cpp
|   |-- test_wheel_velocity_pi/
|   |   `-- test_wheel_velocity_pi.cpp
|   |-- test_drivetrain_odometry/
|   |   `-- test_drivetrain_odometry.cpp
|   `-- test_tape_following/
|       `-- test_tape_following.cpp
|
|-- tools/                           Developer-side browser tools
|   |-- drivetrain_test_dashboard.html
|   |-- drive_dashboard.html
|   `-- tuning_dashboard.html
|
`-- lib/
    `-- README                       Placeholder for private local libraries
```

## Architectural layers

| Layer | Location | Responsibility | Dependency direction |
|---|---|---|---|
| Application and harnesses | `src/main.cpp`, `src/harnesses/` | Select a firmware behavior, process operator commands, schedule updates, and report telemetry. | May depend on configuration, control, drivers, and external communication libraries. |
| Sensing | `include/sensing/`, `src/sensing/` | Estimate tape-line position and detect task markers from sampled inputs. | Depends on sampled sensor state, but not on GPIO operations or application code. |
| Control | `include/control/`, `src/control/` | Convert body commands to wheel targets, regulate wheel speed, integrate pose deltas, and coordinate drivetrain safety and hardware. | Depends on drivers and control configuration. Pure math modules should not depend on application code. |
| Hardware abstraction | `include/drivers/`, `src/drivers/` | Wrap ESP32 LEDC, GPIO, PCNT, and timing details behind motor and encoder APIs. | Depends on ESP-IDF and small shared utilities, but not on control or application code. |
| Board configuration | `include/config/`, `src/config/` | Bind generic module types to this board's pins, peripherals, geometry, limits, and calibration values. | May depend on the type definitions being configured. Runtime modules consume these constants. |
| Shared communication/utilities | sibling `../lib/robot-common` | Provides logging, UART-link, I2C-bus, math, and packet utilities shared across firmware projects. | This repository consumes it through `lib_extra_dirs`; it is not implemented in this repository. |
| Tests | `test/` | Exercise hardware-independent control behavior using PlatformIO's native environment. | Depend on public headers and selected pure C implementations. |
| Developer tools | `tools/` | Provide browser dashboards for serial/WebSocket tuning and telemetry. | Depend on the text protocols implemented by the harnesses, not on firmware headers. |

The intended dependency flow is:

```text
application / harness
        |
        v
control facade and robot logic
        |
        v
hardware drivers
        |
        v
ESP-IDF / Arduino hardware APIs

configuration --------> supplies constants to all applicable layers
robot-common ---------> supplies shared communication and utilities
```

Lower layers should never include or call application code. Pure math modules should remain free of Arduino and hardware headers so native tests stay fast and portable.

## Folder descriptions and placement rationale

### `include/`

Contains interfaces that more than one translation unit may include. Its subfolders mirror `src/`, making an implementation easy to find from its header. The organization is layer first and subsystem second: for example, drivetrain algorithms live under `control/drivetrain/`, while motor hardware access lives under `drivers/motor/`. Public structures, enums, constants declared with `extern`, and function prototypes belong here. Private helper functions and file-local state do not.

### `include/config/` and `src/config/`

These files separate reusable algorithms and drivers from one robot's physical wiring and tuning. Headers expose named immutable configuration objects; sources construct those objects from pin assignments, dimensions, peripheral IDs, and calibration values. Changing a connector or gain should normally affect configuration rather than driver logic.

`config/drivetrain/` groups motor, encoder, and complete drivetrain settings. `config/tape_following/` groups tape hardware, estimation, controller, and task-detection settings. `config/communication/` groups the board settings consumed by shared I2C and UART abstractions. Board-wide `pin_map.h` remains at the config root because several subsystems use it.

Important files include:

- `pin_map.h`: the single board-level GPIO map.
- `motor_config.*` and `encoder_config.*`: physical wheel-device assignments.
- `drivetrain_config.*`: composition root for drivetrain hardware, geometry, PI gains, and safety limits.
- `tape_following/tape_following_config.*`: tape module pins, channel-position weights, controller limits, and task-detection debounce.
- `i2c_bus_config.*` and `uart_link_config.*`: board-specific settings for shared `robot-common` communication abstractions.

### `include/drivers/` and `src/drivers/`

Drivers own direct hardware interaction. Each hardware family has a subfolder so new drivers do not produce one large flat directory. `drivers/motor/` translates signed duty into direction GPIO and LEDC output. `drivers/encoder/` configures ESP32 pulse counters and turns quadrature counts into distance and velocity. `drivers/tape_sensor/` owns shared-multiplexer GPIO setup and sampling. Keeping these details here prevents control and sensing code from knowing register, channel, or GPIO mechanics.

### `include/control/` and `src/control/`

This is the motion-control layer. Motion modules are grouped by subsystem under `control/drivetrain/` and `control/tape_following/`:

- `velocity_kinematics.*` is pure geometry: it converts between body velocity and four wheel angular velocities in both directions.
- `wheel_velocity_pi.*` is pure closed-loop math for one wheel.
- `odometry.*` integrates already-computed body-frame displacement into a world-frame pose.
- `drivetrain.*` is the hardware-facing facade. It owns four motors, four encoders, four PI states, watchdog timing, braking/coasting behavior, body-command limits, and telemetry.

The facade belongs above the individual drivers because it coordinates them as one subsystem. Kinematics, PI, and odometry remain separate because they are reusable, testable mathematical responsibilities.

`control/tape_following/` contains the bounded tape correction controller and the stateful follower that chooses the leading sensor, handles line loss, and emits a `DrivetrainBodyVelocity` compatible with the drivetrain facade.

### `include/sensing/` and `src/sensing/`

`sensing/tape_following/` contains hardware-independent interpretation of sampled tape inputs. `tape_line_estimator.*` computes weighted line position and remembers a lost-line direction. `tape_task_detection.*` debounces broad left-module observations into task-marker events.

### `src/main.cpp`

PlatformIO's default application entry point. It is currently a two-motor bench test, not a complete robot application. Entry points belong at the top of `src/` because they compose modules rather than implement a reusable layer.

### `src/harnesses/`

Contains alternative C++ application entry points selected by PlatformIO source filters. `drive_main.cpp` exercises the complete velocity drivetrain with serial/WebSocket commands. `drivetrain_test_main.cpp` provides timed and encoder-relative acceptance tests, runtime control tuning, and complete-robot movement. `tuning_main.cpp` accesses one motor/encoder control loop at a time for gain identification. Keeping these out of the normal source root prevents multiple `setup()`/`loop()` definitions and keeps debug-only networking or tuning logic out of production firmware.

### `test/`

Contains native unit tests arranged one suite per folder. `native_stubs/` replaces the small ESP-IDF error and GPIO type dependencies required by hardware-independent modules. Tests are separate from firmware sources so they are never linked into deployable images.

### `tools/`

Contains host-side HTML dashboards rather than embedded code. `drivetrain_test_dashboard.html` is the USB-only button interface for the drivetrain acceptance harness. They belong outside `src/` because PlatformIO must not compile them, and because their runtime is a browser communicating with a harness.

### `lib/`

Reserved by PlatformIO for project-private libraries. It currently contains only the generated README. The actual shared library used by this project is the sibling `../lib/robot-common`, configured through `lib_extra_dirs`.

## Root files

| File | Role and reason for location |
|---|---|
| `.gitignore` | Repository-wide generated-file exclusions, so it belongs at the root. |
| `platformio.ini` | Defines the board, dependencies, build flags, source filters, and the `esp32-s3-devkitm-1`, `native`, `tuning`, and `drive` environments. PlatformIO expects it at the project root. |
| `PLATFORMIO_COMMANDS.md` | Developer command reference used across the whole repository. |
| `PROJECT_STRUCTURE.md` | High-level onboarding map for the complete project. |
| `FILE_RESPONSIBILITIES.md` | Detailed ownership, dependency, and improvement reference. |

## Where new code should go

| New code type | Preferred location |
|---|---|
| New GPIO/peripheral driver | `include/drivers/<device>/<name>.h`, `src/drivers/<device>/<name>.c` |
| New hardware-independent controller or motion transform | `include/control/<subsystem>/<name>.h`, `src/control/<subsystem>/<name>.c` |
| New board pin, gain, limit, or immutable hardware instance | matching subsystem files under `include/config/` and `src/config/` |
| New sensor hardware abstraction | `include/drivers/<sensor>/`, `src/drivers/<sensor>/` |
| New hardware-independent sensor interpretation | `include/sensing/<subsystem>/`, `src/sensing/<subsystem>/` |
| New complete firmware behavior or diagnostic program | `src/main.cpp` or a separate `src/harnesses/*_main.cpp` plus a PlatformIO environment |
| New native unit suite | `test/test_<module>/` |
| New browser/host diagnostic interface | `tools/` |
| Code shared by multiple robot firmware projects | sibling `robot-common` library rather than this project's `src/` |

Do not put board pin numbers in generic drivers, direct motor duty writes in high-level robot behaviors, or Arduino/WebSocket code in pure control modules. Those boundaries preserve testability and make hardware changes local.
