# File Responsibilities

## How to read this document

This document describes the repository as it currently exists. Items under **Recommendations** are proposals, not existing files or completed integrations.

For every header/source pair, the header is the public contract: shared types, immutable-object declarations, and callable functions. The source owns executable behavior, hardware calls, validation details, and `static` helpers. A caller should not depend on a source file's private symbols or internal sequencing.

## Root and build files

| Path | Layer | Responsibility and interaction |
|---|---|---|
| `.gitignore` | Repository tooling | Excludes PlatformIO output, generated VS Code databases, and `misc/`. It should contain only repository-wide ignore rules and should not encode build behavior. Git and developer tools consume it. |
| `platformio.ini` | Build configuration | Selects ESP32-S3/Arduino and native toolchains, `robot-common`, WebSockets, C++17, entry-point source filters, monitor settings, and five environments. It should not contain firmware behavior or runtime constants. PlatformIO consumes it; its filters determine which files become each image. |
| `PLATFORMIO_COMMANDS.md` | Developer documentation | Lists common build, test, upload, monitor, and clean commands. It affects no firmware and depends only on the environment names in `platformio.ini`. |
| `include/README`, `lib/README`, `test/README` | Template documentation | Generic PlatformIO explanations for those directories. They are onboarding aids but do not describe this robot. Replacing or supplementing them with project-specific notes would be more useful. |

## Configuration layer

Configuration files bind reusable types to this board. Their source objects are immutable `const` data and should not contain runtime state machines.

### `include/config/pin_map.h`

- **Why it exists:** provides one authoritative map from board functions to GPIO numbers.
- **Defines:** tape inputs and selector pins, motor PWM/direction/brake pins, encoder pins, I2C pins, UART pins, and ToF shutdown pins.
- **Should contain:** board-revision pin assignments and only very small compile-time pin aliases.
- **Should not contain:** driver initialization, controller gains, runtime state, or application decisions.
- **Dependencies:** none.
- **Used by:** board configuration sources, including the tape-following composition root.
- **Tape isolation:** the tape driver receives selector pins through `TapeSensorMuxConfig`; it does not consume this board map directly.

### `include/config/drivetrain/motor_config.h` and `src/config/drivetrain/motor_config.c`

- **Layer:** hardware configuration.
- **Why they exist:** describe the four physical motor channels without baking board wiring into `motor_driver.c`.
- **Header owns:** `extern` declarations for `FL_MOTOR_CONFIG`, `FR_MOTOR_CONFIG`, `BL_MOTOR_CONFIG`, and `BR_MOTOR_CONFIG`.
- **Source owns:** PWM pins/channels/frequency/resolution, direction inversion, and per-motor maximum duty.
- **Should not own:** live duty, enable state, PI gains, chassis geometry, or command sequencing.
- **Dependencies:** `motor_driver.h` for `MotorDriverConfig`; `pin_map.h` for GPIO assignments.
- **Consumers:** `drivetrain_config.c`, `main.cpp`, and indirectly both harnesses through `DRIVETRAIN_CONFIG`.
- **Interaction:** configuration is passed to `motor_driver_init`; the driver then retains a pointer to it.
- **Current note:** comments document the physical swap between connectors 2 and 3. That calibration belongs here, although the stale reference to `misc/algo_testing.py` points to an ignored/non-project path.

### `include/config/drivetrain/encoder_config.h` and `src/config/drivetrain/encoder_config.c`

- **Layer:** hardware and calibration configuration.
- **Why they exist:** bind each logical wheel to PCNT units/channels, pins, direction, counts per revolution, wheel diameter, counter bounds, and filtering.
- **Header owns:** four named `extern` configurations and the `ENCODER_CONFIGS` lookup indexed by `EncoderId`.
- **Source owns:** calibration constants and physical connector mapping.
- **Should not own:** accumulated counts, timestamps, velocity calculations, or drivetrain pose.
- **Dependencies:** `encoder_driver.h`, `pin_map.h`, and ESP-IDF PCNT/GPIO types exposed by the driver header.
- **Consumers:** `drivetrain_config.c`; direct users may also use `ENCODER_CONFIGS`.
- **Interaction:** `encoder_driver_init` stores a pointer to one of these immutable objects.
- **Current overlap:** wheel diameter here and wheel radius in `drivetrain_config.c` describe the same physical wheel and can drift apart. They are currently consistent at `0.070 m` diameter and `0.035 m` radius.

### `include/config/drivetrain/drivetrain_config.h` and `src/config/drivetrain/drivetrain_config.c`

- **Layer:** subsystem composition and control configuration.
- **Why they exist:** create the single complete configuration needed to initialize the velocity-controlled drivetrain.
- **Header owns:** only the `extern const DrivetrainConfig DRIVETRAIN_CONFIG` declaration.
- **Source owns:** motor/encoder object references, chassis geometry, wheel PI gains, duty and body-velocity bounds, maximum control interval, command timeout, and brake pin.
- **Should not own:** mutable PI state, encoder measurements, motor objects, application commands, or telemetry.
- **Dependencies:** public drivetrain types plus motor, encoder, and pin configurations.
- **Consumers:** `drive_main.cpp`, `tuning_main.cpp`, and any future production drivetrain application.
- **Interaction:** it is the subsystem's composition root; `drivetrain_init` validates and retains it.

### `include/config/tape_following/tape_following_config.h` and `src/config/tape_following/tape_following_config.c`

- **Layer:** tape-sensor and behavior configuration.
- **Why they exist:** compose the tape subsystem in one place, parallel to `config/drivetrain/`.
- **Header owns:** declarations for the shared mux, three module configs, front/back estimator configs, complete follower config, and task-detector config.
- **Source owns:** board pin bindings, channel weights, controller gains and limits, lost-line behavior, and task-marker debounce values.
- **Should contain:** immutable wiring, calibration, controller tuning, and behavior thresholds.
- **Should not contain:** sensor samples, controller history, GPIO operations, task state, or drivetrain commands.
- **Dependencies:** tape driver, sensing, follower types, and `pin_map.h`.
- **Consumers:** future production tape-following composition code; the definitions currently compile into the default firmware but are not invoked by `main.cpp`.

### `include/config/communication/i2c_bus_config.h` and `src/config/communication/i2c_bus_config.c`

- **Layer:** communication-bus configuration.
- **Why they exist:** configure the drivetrain board's shared sensor I2C master without duplicating the generic bus implementation.
- **Header owns:** `SENSOR_I2C_BUS_CONFIG` declaration.
- **Source owns:** I2C port, SDA/SCL pins, clock, timeout, and pull-up setting.
- **Should not own:** I2C transactions, individual sensor protocols, or application polling.
- **Dependencies:** `robot_common/i2c_bus.h` and `pin_map.h`.
- **Consumers:** none in the current `src/` tree.
- **Interaction:** intended to be passed to the shared `robot-common` I2C bus API when sensors are integrated.

### `include/config/communication/uart_link_config.h` and `src/config/communication/uart_link_config.c`

- **Layer:** communication-link configuration.
- **Why they exist:** configure UART1 for the link to the upper/arm ESP32 using the shared UART abstraction.
- **Header owns:** `TOP_ESP_UART_LINK_CONFIG` declaration.
- **Source owns:** UART peripheral, pins, baud rate, and buffer sizes.
- **Should not own:** packet meaning, link state, callbacks, or robot commands.
- **Dependencies:** `robot_common/uart_link.h` and `pin_map.h`.
- **Consumers:** none in the current application.
- **Interaction:** intended to initialize a `robot-common` UART link; there is no current application-level communication module connecting received packets to drivetrain commands.

## Driver layer

### `include/drivers/motor/motor_driver.h` and `src/drivers/motor/motor_driver.c`

- **Layer:** hardware abstraction.
- **Why they exist:** present one consistent signed-duty motor API over ESP32 GPIO and LEDC details.
- **Header owns:** `MotorDriverConfig`, runtime `MotorDriver`, validation/lifecycle/duty/coast functions, and read-only state getters. C linkage guards allow C++ harnesses to call the C implementation.
- **Source owns:** configuration validation, duty-to-LEDC conversion, direction logic, GPIO/LEDC initialization, state transitions, and private `static` hardware helpers.
- **Should contain:** behavior for exactly one motor channel.
- **Should not contain:** wheel kinematics, velocity PI, four-wheel coordination, command timeouts, or robot behavior.
- **Dependencies:** ESP-IDF GPIO/LEDC, `esp_err`, and `robot_common/math_utils.h`.
- **Consumers:** `drivetrain.c`, `main.cpp`, and `tuning_main.cpp`; configuration sources depend on its config type.
- **Interaction:** `drivetrain.c` is the normal owner. It calls `motor_driver_set_duty` only through its private four-wheel duty application helper.
- **Encapsulation note:** runtime fields are publicly visible because the C object is caller-allocated. Callers should still prefer the API getters and mutators.

### `include/drivers/encoder/encoder_driver.h` and `src/drivers/encoder/encoder_driver.c`

- **Layer:** hardware abstraction and measurement conversion.
- **Why they exist:** hide legacy ESP32 PCNT quadrature setup and expose accumulated count, travel, and velocity.
- **Header owns:** `EncoderId`, `EncoderDriverConfig`, runtime `EncoderDriver`, lifecycle/read/update functions, and state/velocity getters.
- **Source owns:** PCNT channel configuration, glitch-filter conversion, raw counter flushing, overflow-safe accumulation, timing, and count-to-motion conversion.
- **Should contain:** behavior for one encoder and its wheel calibration.
- **Should not contain:** chassis kinematics, body velocity, pose integration, PI control, or motor commands.
- **Dependencies:** ESP-IDF GPIO, PCNT, timer, math, and error APIs.
- **Consumers:** `drivetrain.c` and `tuning_main.cpp`; encoder configuration depends on its public types.
- **Interaction:** callers periodically invoke `encoder_driver_update`, then read cached velocity or accumulated position.

## Control layer

### `include/control/drivetrain/velocity_kinematics.h` and `src/control/drivetrain/velocity_kinematics.c`

- **Layer:** pure control mathematics.
- **Why they exist:** convert between body-frame `vx`, `vy`, angular velocity and front-left/front-right/back-left/back-right wheel angular velocities.
- **Header owns:** `DrivetrainVelocityKinematicsConfig`, `DrivetrainBodyVelocity`, `DrivetrainWheelVelocity`, and both conversion functions.
- **Source owns:** finite/geometry/singularity validation plus forward and inverse wheel equations.
- **Should not contain:** motor duty, encoder reads, PI state, timing, GPIO, or application commands.
- **Dependencies:** only math, `esp_err`, and its own public types.
- **Consumers:** `drivetrain.c`, `drivetrain_test_main.cpp`, and native kinematics tests.
- **Interaction:** the drivetrain facade converts the result from radians per second to linear wheel speed using wheel radius before invoking each wheel PI controller.
- **Position-test use:** the acceptance harness applies the inverse transform to relative encoder wheel angles to estimate translation and heading change.

### `include/control/drivetrain/wheel_velocity_pi.h` and `src/control/drivetrain/wheel_velocity_pi.c`

- **Layer:** pure closed-loop control mathematics.
- **Why they exist:** calculate bounded signed motor duty for one wheel from target and measured linear velocity.
- **Header owns:** tunable `WheelVelocityPiConfig`, runtime `WheelVelocityPi` history, config validation, update, and reset functions.
- **Source owns:** clamping, feedforward/offset, proportional/integral behavior, anti-windup, safe-stop direction behavior, duty slew limiting, and state reset.
- **Should not contain:** motor-driver calls, encoder reads, wheel selection, chassis geometry, or command watchdog logic.
- **Dependencies:** math, memory utilities, and `esp_err`.
- **Consumers:** `drivetrain.c`, `tuning_main.cpp`, and native PI tests.
- **Interaction:** the drivetrain owns four independent PI state objects but one active gain configuration.
- **Naming note:** the implementation is PI plus feedforward and slew limiting, not only a minimal PI equation; the current name remains reasonable because PI is its feedback core.

### `include/control/drivetrain/odometry.h` and `src/control/drivetrain/odometry.c`

- **Layer:** pure state estimation mathematics.
- **Why they exist:** integrate a body-frame displacement and heading change into world-frame `x`, `y`, and heading while recording invalid-cycle status.
- **Header owns:** body delta, pose, odometry fault enum, odometry state, update, and reset declarations.
- **Source owns:** finite validation, body-to-world rotation, pose accumulation, invalid-cycle handling, and reset behavior.
- **Should not contain:** encoder hardware reads, wheel-to-body kinematics, motor control, global timestamps, or communications.
- **Dependencies:** math, memory utilities, and `esp_err`.
- **Consumers:** currently only native odometry tests.
- **Interaction gap:** `Drivetrain` does not own or update odometry, and no module converts four encoder deltas into the `DrivetrainBodyDelta` this module expects.
- **Fault terminology:** this local enum reports odometry-cycle validity. It is not the drivetrain fault state that has intentionally not been added.

### `include/control/drivetrain/drivetrain.h` and `src/control/drivetrain/drivetrain.c`

- **Layer:** drivetrain subsystem facade, hardware coordinator, and safety boundary.
- **Why they exist:** expose body-velocity control as one subsystem rather than forcing applications to coordinate eight devices and four controllers.
- **Header owns:** wheel IDs, complete configuration, caller-facing status, statically allocated drivetrain state, lifecycle/command/update/tuning/status APIs, and read-only wheel telemetry.
- **Source owns:** complete config validation, partial-initialization rollback, motor/encoder ownership, enable/disable/brake/coast sequencing, command watchdog, maximum body-velocity enforcement, maximum control-interval handling, kinematics calls, four PI updates, and telemetry updates.
- **Private implementation:** `apply_wheel_duties` is `static`; no public caller can command four raw duties through the drivetrain facade. Validation, cleanup, encoder-update, clamping, and reset helpers are also private.
- **Should not contain:** serial/WebSocket parsing, tape-following decisions, browser telemetry formatting, or board-specific numeric configuration values.
- **Dependencies:** motor and encoder drivers, velocity kinematics, wheel PI, GPIO, ESP timer, and `robot-common` logging.
- **Consumers:** `drive_main.cpp`; tuning includes its types/config but deliberately controls individual devices directly.
- **Current public state:** `Drivetrain` has four grouped top-level members—`config`, `devices`, `control`, and `status`—instead of many flat fields. The device and control layouts remain visible so callers can statically allocate a C object. This creates more coupling than a truly opaque handle.
- **Scope note:** at roughly 500 lines it has several responsibilities, but they are currently cohesive around subsystem coordination. If it grows, private lifecycle, safety, and telemetry helpers could move to internal files without expanding the public API.

## Tape driver, sensing, and control layers

### `include/drivers/tape_sensor/tape_sensor_driver.h` and `src/drivers/tape_sensor/tape_sensor_driver.c`

- **Layer:** sensor hardware abstraction.
- **Why they exist:** operate three tape modules that share two multiplexer selector pins and each expose one selected-channel input.
- **Header owns:** channel/module constants, mux and module configuration, runtime sampled state, initialization, current-channel read, full scan, and packed raw scan APIs.
- **Source owns:** active electrical level, mux settling, channel selection, GPIO setup, sampling, and bit packing.
- **Should not contain:** estimation weights, controller gains, steering decisions, motor commands, or task-marker policy.
- **Dependencies:** ESP-IDF GPIO, delay, and error APIs.
- **Consumers:** the tape sensing and control modules use the sampled `TapeSensor` type; no application currently initializes the physical sensors.
- **Isolation:** selector pins are supplied through `TapeSensorMuxConfig`, so the driver no longer reads board pin macros directly.

### `include/sensing/tape_following/tape_line_estimator.h` and `src/sensing/tape_following/tape_line_estimator.c`

- **Layer:** hardware-independent sensing.
- **Why they exist:** calculate a weighted line centroid and retain a useful search direction when the line disappears.
- **Header owns:** estimator configuration/state plus reset and compute APIs.
- **Source owns:** active-channel aggregation, centroid calculation, and lost-line fallback.
- **Should not contain:** GPIO reads, PID state, drivetrain commands, or board-specific weights.
- **Consumers:** `tape_follower.c` and native tape-following tests.

### `include/sensing/tape_following/tape_task_detection.h` and `src/sensing/tape_following/tape_task_detection.c`

- **Layer:** hardware-independent sensing/event detection.
- **Why they exist:** debounce broad left-module observations into stable task-marker state and edge events.
- **Header owns:** detector configuration, runtime state, output events, and lifecycle/update APIs.
- **Source owns:** active-channel counting, saturated counters, confirmation, and release transitions.
- **Consumers:** future robot-manager integration and native tape-following tests.

### `include/control/tape_following/tape_following_controller.h` and `src/control/tape_following/tape_following_controller.c`

- **Layer:** pure control mathematics.
- **Why they exist:** convert estimated lateral tape error into a bounded lateral velocity correction.
- **Header owns:** PID configuration/history plus reset and update APIs.
- **Source owns:** finite-input validation, integration, derivative calculation, and output clamping.
- **Should not contain:** sensor sampling, travel-direction selection, task detection, or drivetrain calls.
- **Consumers:** `tape_follower.c` and native tape-following tests.

### `include/control/tape_following/tape_follower.h` and `src/control/tape_following/tape_follower.c`

- **Layer:** stateful tape-following behavior.
- **Why they exist:** select the leading sensor from travel direction, coordinate estimation and feedback, recover briefly after tape loss, and report a lost state.
- **Header owns:** follower configuration, input/output, status, state, and lifecycle/update APIs.
- **Source owns:** configuration validation, direction changes, controller resets, timeout handling, and search behavior.
- **Drivetrain compatibility:** output uses `DrivetrainBodyVelocity` (`vx`, `vy`, `omega`) and can be passed to `drivetrain_set_body_velocity` when `motion_valid` is true.
- **Should not contain:** GPIO operations, direct drivetrain calls, task detection, or board-specific tuning constants.
- **Consumers:** future robot-manager integration and native tape-following tests.

## Application and harness files

### `src/main.cpp`

- **Layer:** default application entry point.
- **Current responsibility:** initializes logging and the front-left/front-right motors, enables them, and repeatedly applies `0.5` duty as a bench test.
- **Why it exists here:** Arduino expects one compiled pair of global `setup()` and `loop()` functions; the default PlatformIO environment includes this file.
- **Should contain:** top-level composition, startup ordering, periodic scheduling, and high-level handling for the selected production behavior.
- **Should not contain:** reusable motor logic, controller equations, pin definitions, or hardware-driver implementation.
- **Dependencies:** Arduino, logging, motor driver, and motor configuration.
- **Consumers:** none; it is an entry point.
- **Misplacement/status:** despite being selected by the default environment, it is a diagnostic motor program rather than complete drivetrain firmware. It would fit better as `src/harnesses/motor_bench_main.cpp`, with a new production `main.cpp` built around `Drivetrain`.

### `src/harnesses/drive_main.cpp`

- **Layer:** full-subsystem diagnostic application.
- **Responsibility:** initializes `Drivetrain`, accepts serial and WebSocket drive/turn/stop and PI-tuning commands, schedules drivetrain updates, handles timed motion, and publishes wheel telemetry.
- **Should contain:** diagnostic protocol parsing, Wi-Fi/WebSocket setup, presentation formatting, and harness scheduling.
- **Should not contain:** raw PWM operations, PI equations, encoder implementation, or board configuration constants.
- **Dependencies:** Arduino, WiFi, WebSockets, ESP timer, logging, `DRIVETRAIN_CONFIG`, and the public drivetrain API.
- **Consumers:** the `drive` PlatformIO environment and `tools/drive_dashboard.html` protocol.
- **Interaction:** this is the best current example of how a caller should use the drivetrain facade.

### `src/harnesses/tuning_main.cpp`

- **Layer:** component-level calibration application.
- **Responsibility:** selects individual wheel motor/encoder pairs, runs a local wheel PI loop, accepts serial gain/target commands, and prints measurements for tuning.
- **Should contain:** experimental command parsing, test sequencing, and diagnostic output.
- **Should not contain:** reusable control equations or permanent robot behavior.
- **Dependencies:** Arduino, drivetrain configuration, motor/encoder drivers, and wheel PI.
- **Consumers:** the `tuning` PlatformIO environment and `tools/tuning_dashboard.html` protocol.
- **Intentional bypass:** it calls drivers directly instead of `Drivetrain` because single-wheel experimentation is its purpose. That would be inappropriate in production application code.

### `src/harnesses/drivetrain_test_main.cpp`

- **Layer:** interactive hardware acceptance application.
- **Responsibility:** runs timed and encoder-relative translation/rotation tests, duty-limited individual motor/encoder checks, tape-sensor monitoring and timed `0110` lateral centering, runtime PI/ramp/tolerance tuning, and serial telemetry.
- **Configuration behavior:** starts from test-local copies of `DRIVETRAIN_CONFIG`, allowing RAM-only motor or encoder direction inversion without changing production configuration.
- **Safety behavior:** provides controlled stop, immediate coast, hardware brake/re-enable, motion timeout, acceleration/deceleration ramps, an 80% individual-test cap, timed tape centering, no-tape motion suppression, and pauses between automatic sequence steps.
- **Consumers:** the `drivetrain-test` PlatformIO environment.

## Tests

### `test/native_stubs/esp_err.h`

- **Layer:** test portability support.
- **Responsibility:** provide the small `esp_err_t` type and constants needed to compile pure control modules on the host.
- **Should not contain:** hardware behavior or broad ESP-IDF emulation.
- **Consumers:** native-compiled headers and control sources via the native include path.

### `test/test_velocity_kinematics/test_velocity_kinematics.cpp`

- **Responsibility:** verify exact forward/strafe/turn/combined wheel values, geometric scaling, linearity, and invalid input rejection.
- **Dependencies:** Unity and the C velocity-kinematics API.
- **Should not test:** motors, encoders, timing, or application protocols.
- **C++ role:** small reference wrappers keep assertions readable while exercising the pointer-based C interface.

### `test/test_wheel_velocity_pi/test_wheel_velocity_pi.cpp`

- **Responsibility:** verify integral accumulation/reset, safe stopping, anti-push behavior, slew limiting, and configuration validation.
- **Dependencies:** Unity and the C wheel PI API.
- **Should not test:** motor hardware, encoder sampling, or four-wheel coordination.

### `test/test_drivetrain_odometry/test_drivetrain_odometry.cpp`

- **Responsibility:** verify pose integration, frame rotation, invalid-cycle handling, finite validation, and reset behavior.
- **Dependencies:** Unity and the C odometry API.
- **Should not test:** the not-yet-existing encoder-to-body-delta path.

## Developer tools

### `tools/drivetrain_test_dashboard.html`

- **Responsibility:** provides a local point-and-click controller for every command exposed by `drivetrain_test_main.cpp`.
- **Transport:** uses the browser Web Serial API at 115200 baud through the CH340K USB-to-UART port; it has no Wi-Fi or server dependency.
- **Safety:** exposes prominent stop controls, preserves the harness's input limits, and displays all firmware responses in a serial log.
- **Telemetry:** plots rolling target/measured wheel velocity and applied duty for all four wheels, displays all three four-channel tape patterns, and reports tape detection, centering error, lateral correction, and exact `0110` matches.
- **Consumers:** operators running the `drivetrain-test` PlatformIO environment in desktop Chrome or Edge.

### `tools/drive_dashboard.html`

- **Layer:** host-side diagnostics UI.
- **Responsibility:** provide controls and visualization for the serial/WebSocket protocol exposed by `drive_main.cpp`.
- **Should not contain:** authoritative firmware limits or control equations.
- **Dependency:** its command and telemetry formats must remain synchronized with the drive harness.

### `tools/tuning_dashboard.html`

- **Layer:** host-side calibration UI.
- **Responsibility:** send tuning commands and display wheel-controller measurements produced by `tuning_main.cpp`.
- **Should not contain:** production controller state or embedded hardware logic.
- **Dependency:** tightly coupled to the tuning harness's textual protocol.

## Current dependency and responsibility findings

### Overlapping responsibilities

1. Wheel identity is represented by both `DrivetrainMotorId` and `EncoderId`. They currently share FL/FR/BL/BR ordering, and `drivetrain.c` validates that assumption, but two enums can diverge.
2. Wheel geometry appears as encoder diameter and kinematics radius. This duplication is necessary for the current APIs but needs a single calibrated source or explicit cross-validation.
3. `drivetrain.c` combines lifecycle, safety, control scheduling, and telemetry. This is acceptable for a facade today, but private submodules may be warranted as it grows.

### Misplaced or inconsistent files

1. `src/main.cpp` is a motor bench harness in the production/default entry-point location.
2. Generic PlatformIO README files do not document this project's conventions.
3. `lib/` is empty while the actual private shared library is a sibling directory. This is valid PlatformIO configuration but can surprise new developers.

### Unnecessary coupling

1. `drivetrain.h` includes both driver headers and publicly exposes `DrivetrainDevices` and `DrivetrainControlState`. This is required by the present caller-allocated C object design, but it exposes implementation layout.
2. `encoder_driver.h` exposes ESP-IDF `gpio_num_t`, `pcnt_unit_t`, and `pcnt_channel_t`, so any host code including it needs ESP-IDF types or stubs.
3. The HTML tools and C++ harnesses share protocols only by convention; there is no versioned schema or common protocol description.

### Missing integration or modules

These are observations and recommendations, not existing files:

1. **Production application:** replace the current bench-test `main.cpp` with a real application that initializes `Drivetrain`, receives commands, and schedules `drivetrain_update`.
2. **Motor bench harness:** move the present `main.cpp` behavior to a recommended `src/harnesses/motor_bench_main.cpp` and add a matching PlatformIO environment.
3. **Wheel-to-body odometry transform:** add a hardware-independent forward-kinematics/delta module so encoder changes can produce `DrivetrainBodyDelta`; then decide whether `Drivetrain` or a higher state-estimation layer owns odometry.
4. **Communication application layer:** add a module that owns UART-link initialization, packet parsing, timeouts, and translation from upper-controller messages to drivetrain commands. The current repository has only UART configuration.
5. **Tape-following integration:** add application/course logic that initializes the tape driver, updates the follower and task detector, and forwards valid body-velocity requests to the drivetrain. The primitives are implemented and natively tested but unused by `main.cpp`.
6. **Integration and driver tests:** add tests for drivetrain state transitions/limits with hardware fakes, tape scan behavior, and driver validation. Pure tape estimation, control, follower, and task detection now have native coverage.
7. **Root README:** add a project-specific `README.md` covering purpose, hardware, prerequisites, quick build/test commands, and links to these architecture documents.

## Suggested improvement order

1. Move the motor bench behavior into its own harness and make `main.cpp` an explicit production composition root.
2. Add native drivetrain state/limit tests using hardware fakes.
3. Establish one shared wheel identity and one authoritative wheel-geometry calibration source.
4. Add wheel-to-body kinematics and connect odometry only after its ownership and update timing are defined.
5. Add application-level UART command handling and tape-following behavior as separate modules rather than expanding drivers.
6. Consider an opaque or fixed-storage drivetrain handle only if public-layout coupling becomes a maintenance problem; do not introduce heap allocation merely for encapsulation on this embedded target.
