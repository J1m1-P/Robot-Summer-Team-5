# Firmware Fix Plan

> Re-audited on 2026-07-16 against the current repository.
>
> Scope: `Robot/firmware/esp32-drivetrain`, `Robot/firmware/esp32-arm`,
> `Robot/firmware/lib/robot-common`, firmware configuration, calibration tooling,
> documentation, and test directories.
>
> This was a static code and clean-build review. Hardware behavior has not been
> verified by this audit.

## Executive Status

The reusable foundations are substantially improved: shared UART and I2C
transports exist, cumulative odometry has a defined wire format, drivetrain
safety behavior is implemented, and the optical sensor calibration/debug tools
build. The firmware is not production-complete because neither production
`main.cpp` integrates those modules.

The most immediate functional blocker is the deployed optical calibration file.
`data/calibration.json` does not contain the required `baseline_mm`, so
`static_calibration_load()` rejects it and optical fusion remains disabled.

### Audit Results

- Clean drivetrain production build: **passes**.
- Clean arm production build: **passes**.
- Clean arm `optical_debug` build: **passes**.
- Clean arm `calibration` build: **passes**.
- Arm builds warn that the four inline stepper configuration variables require
  C++17, while the current build uses an older language mode.
- `tools/calibrate_optical.py --help`: **passes**, including the `pyserial`
  import in the current environment.
- `data/calibration.json` parses as JSON but is missing required
  `baseline_mm`: **fails application schema**.
- Both PlatformIO `test/` directories contain only template READMEs: **no
  automated firmware tests exist**.
- Working tree was clean before this plan update.

## Verified Completed Work

### Shared `robot-common` Library

- Logging tags and convenience macros are shared by both boards.
- `app_log_tag()` safely returns `"unknown"` for invalid or unmapped tags.
- Common robot state and command types are centralized.
- Float `clamp()` has one shared implementation.
- UART framing, parsing, checksum validation, latest-packet storage, and link
  counters are centralized.
- The I2C master bus and addressed-device API were moved out of the drivetrain
  project into `robot-common`, so both ESP32 targets can use it.
- Cumulative odometry packets are encoded and decoded explicitly in
  little-endian order rather than transmitting a padded C struct.
- The odometry payload includes cumulative position, heading, a sequence number,
  and a sensor-validity flag.
- Shared and board-specific C/C++ files now have file descriptions and concise
  comments for functions and definitions.

### Drivetrain Safety and Drivers

- Motor initialization returns `esp_err_t` consistently.
- Motor GPIO and LEDC failures propagate to callers.
- Motor, encoder, and drivetrain configurations receive basic validation before
  use.
- Drivetrain initialization engages the brake before device setup.
- Partial drivetrain initialization stops started encoders, disables initialized
  motors, re-engages the brake, and clears runtime state.
- Partial drivetrain enablement disables already-enabled motors and restores the
  braked state.
- Motor duty setters do not silently release the brake.
- A partial four-motor command failure invokes the safe braked/disabled state.
- `drivetrain_brake()` and `drivetrain_coast()` have distinct explicit behavior.
- `drivetrain_tick()` implements a 250 ms command watchdog and coasts once on
  timeout.
- X-drive body-to-wheel mixing validates inputs and scales all wheel duties
  together to the configured maximum.
- Encoder accumulation avoids the legacy PCNT 16-bit range by periodically
  flushing hardware deltas into a 32-bit software count.

### Arm Optical Sensing and Calibration Support

- The PMW3610 driver supports two devices on a shared bit-banged three-wire bus.
- Poll order alternates to reduce fixed left/right timing bias.
- Motion validity checks overflow, laser status, and surface quality.
- Fusion uses per-sensor calibration matrices and physical sensor separation.
- Pose integration skips invalid deltas, preserves the last valid pose, and
  resumes on the next valid sample.
- Separate production, optical-debug, and calibration PlatformIO environments
  exist.
- The calibration firmware emits machine-readable raw sensor deltas.
- The laptop calibration helper collects multiple passes and fits unit rotation
  matrices.
- The optical debug firmware reports full sensor diagnostics and fused pose when
  calibration loads successfully.

## Current Critical Gaps

### 1. Production Entry Points Are Still Placeholders

- Arm `src/main.cpp` is the default Arduino arithmetic stub. It does not
  initialize PMW3610 sensors, load calibration, fuse pose, send odometry, service
  steppers, or initialize I2C lidar devices.
- Drivetrain `src/main.cpp` directly runs one front-left motor at fixed duty. It
  does not construct `Drivetrain`, call `drivetrain_tick()`, receive odometry,
  update encoders, read tape sensors, or run a controller.
- That bench loop commands `0.5` duty directly through `MotorDriver`, bypassing
  the full drivetrain's `0.4` limit and shared brake-pin ownership.
- The implemented drivetrain safety layer is therefore not exercised by the
  production executable.

### 2. Optical Calibration Deployment Is Invalid

- `static_calibration_load()` requires a finite positive `baseline_mm`.
- The checked-in `data/calibration.json` contains only `left` and `right`
  matrices, so loading always fails.
- The calibration helper prints only the two matrices. It tells the operator to
  preserve a measured baseline, but the current file has no baseline to preserve.
- Measure the sensor center-to-center separation, add it to the JSON, upload the
  LittleFS image, and verify the debug environment enables fusion.
- Consider adding a required `--baseline-mm` option to the helper so its output is
  always a complete deployable document.

### 3. End-to-End Odometry Transport Is Not Integrated

- The shared packet module exists, but the arm production loop never sends it.
- No drivetrain odometry receiver exists.
- `RobotOdometry` is defined but unused.
- No 100 ms transport freshness timeout is implemented.
- Sequence gaps and arm resets are not detected or reported.
- UART counters are collected but never surfaced periodically.
- `UartLink` stores only the latest complete frame, not a queue. Calling
  `uart_link_update()` can overwrite earlier frames before one
  `uart_link_take_packet()` call. Receiver design must explicitly accept
  latest-only semantics or change the UART layer to queue/callback delivery.

### 4. Hardware Safety Is Not Yet Verified

- Motor PWM polarity is still marked uncertain in the implementation.
- Brake active level and coast/brake behavior have not been confirmed on hardware.
- Motor and encoder direction inversion settings are unverified.
- X-drive wheel ordering and body-command signs are unverified.
- The 250 ms watchdog exists but has no production-loop caller or hardware test.

## Remaining Workstreams

### A. Repair Calibration and Documentation First

1. Measure and add `baseline_mm` to `data/calibration.json`.
2. Upload the filesystem and confirm `static_calibration_load()` succeeds.
3. Run known forward, lateral, and in-place rotation checks in `optical_debug`.
4. Update `optical_readme.md`: invalid optical samples preserve pose; they do not
   reset it to zero.
5. Update the arm `platformio.ini` comment that currently claims the production
   environment streams odometry packets.
6. Make the calibration helper emit or validate the complete JSON schema.

### B. Integrate Arm Odometry Sending

At a fixed cadence, the arm production loop must:

1. Initialize logging, LittleFS calibration, both PMW3610 sensors, fusion, pose,
   and the drivetrain UART link with checked return values.
2. Poll both sensors.
3. Fuse the current raw deltas.
4. Integrate only a fully valid sample; retain prior pose otherwise.
5. Send one cumulative `OdometryPacket` per cycle, including invalid cycles with
   `valid=false`.
6. Increment the sequence number for every transmitted sample.
7. Rate-limit initialization, sensor, calibration, and transmit fault logs.
8. Continue servicing other arm devices without blocking.

Define policy for UART send failure: the loop should retain pose and continue
sampling, while diagnostics expose the transport fault.

### C. Add a Drivetrain Odometry Receiver

Create a receiver module that owns:

- Latest decoded `RobotOdometry`.
- Timestamp of the last validly decoded odometry frame.
- Last sequence number and a `has_sequence` flag.
- Sequence-gap, sequence-reset, decode-error, and freshness-timeout counters.
- Sensor validity separately from transport freshness.

Required behavior:

1. Call `uart_link_update()` every control iteration.
2. Consume the latest available frame and ignore unrelated packet types safely.
3. Decode odometry only through `odometry_packet_decode()`.
4. Convert millimeters to meters at the receiver boundary.
5. Do not refresh freshness after checksum, type, length, or payload failure.
6. Mark odometry invalid after 100 ms without a decoded odometry packet.
7. Detect sequence gaps and likely arm reboot/reset events.
8. Decide whether latest-only delivery is sufficient. If every frame matters,
   replace `UartLink.latest_packet` with a bounded queue or parser callback.

### D. Replace the Drivetrain Bench Main

The production loop should:

1. Initialize the shared UART link and odometry receiver.
2. Initialize the full `Drivetrain` object with `DRIVETRAIN_CONFIG`.
3. Update all encoders every iteration.
4. Read tape sensors only at the cadence needed by the active controller.
5. Run the robot state machine/controller using explicit sensor-validity states.
6. Apply one coherent body or four-wheel command.
7. Call `drivetrain_tick()` on every iteration, including error paths where the
   main loop continues.
8. Enter an explicit safe state on initialization, receiver, encoder, or controller
   failure.

Do not copy the current bench behavior into production control: the bench main
bypasses drivetrain brake and watchdog ownership.

### E. Rework the Stepper Module

### Header and Configuration Ownership

- Break the circular include between `stepper_config.h` and
  `stepper_driver.h`.
- Keep `StepperConfig` in the config header and include it only from the driver.
- Remove private `static` helper declarations from the public driver header.
- Replace C++17 inline configuration variables with `extern const`
  declarations and one `.cpp` definition file, or explicitly adopt C++17.
- Use pin macros from `pin_map.h` instead of duplicating numeric GPIO values.
- Normalize naming (`step_pin`, `step_pulse_us`, and
  `stepper_move_distance_mm`).

### Non-Blocking Runtime

Replace the blocking loop and delays with a service-based state machine:

```c
typedef struct {
    const StepperConfig *config;
    long steps_remaining;
    long position_steps;
    int8_t direction;
    int64_t next_transition_us;
    bool pulse_high;
    bool initialized;
} StepperDriver;
```

Recommended API:

```c
esp_err_t stepper_init(StepperDriver *driver, const StepperConfig *config);
esp_err_t stepper_start_move(StepperDriver *driver, long steps);
esp_err_t stepper_service(StepperDriver *driver, int64_t now_us);
esp_err_t stepper_stop(StepperDriver *driver);
bool stepper_is_moving(const StepperDriver *driver);
```

Also validate null pointers and timing, handle `LONG_MIN`, detect distance-to-step
overflow, define stop semantics, and update position only after emitted pulses.

### F. Harden Tape Sensing and PID

- Export `FRONT_PID_WEIGHTS`, `BACK_PID_WEIGHTS`, and `LEFT_PID_WEIGHTS` from
  `tape_following_config.h`.
- Validate tape GPIOs before shifting them into pin masks.
- Propagate errors from channel-select `gpio_set_level()` calls.
- Confirm active level, channel mapping, channel weight order, and the 5 us mux
  settling time on hardware.
- Reject non-finite PID gains, inputs, and `dt_s`.
- Require a non-negative integral limit and ordered output limits.
- Add `has_last_known_error`; initial line loss must not arbitrarily choose right.
- Distinguish tracked, lost, and ambiguous/intersection patterns.
- Reset or suppress derivative state when tape is reacquired.
- Use conditional integration so saturation does not continue windup.
- Return `esp_err_t` plus an output parameter from the PID update so invalid input
  is distinguishable from a valid zero correction.

### G. Finish Shared I2C and Lidar Integration

- Keep the generic I2C implementation in `robot-common`.
- Add an arm-specific `I2cBusConfig` using the arm pin map; only the drivetrain
  currently defines a board-specific sensor bus config.
- Add the actual lidar/ToF device driver and lifecycle code for each board; pin
  definitions alone are present today.
- No production entry point currently initializes or uses the shared I2C API.
- Define XSHUT/address-assignment sequencing for multiple devices on one bus.
- Validate I2C port, SDA/SCL pins, clock, and timeout before driver installation.
- Return `ESP_ERR_INVALID_STATE` rather than `ESP_ERR_INVALID_ARG` when probing an
  uninitialized bus.
- Decide ownership when multiple modules share one hardware I2C port and prevent
  duplicate driver installation.
- Confirm external pull-ups, bus voltage, address plan, and 100/400 kHz operation
  on hardware.

### H. Driver and API Hardening

- Make PMW3610 initialization and bus operations return errors instead of ignoring
  GPIO failures and continuing after sensor setup problems.
- Add null/state validation to PMW3610, fusion, pose, and stepper APIs.
- Finish consistent error-code semantics: several encoder calls currently return
  `ESP_ERR_INVALID_ARG` for valid pointers in the wrong lifecycle state.
- Consider output-parameter status APIs where returning zero currently hides
  invalid drivetrain, motor, or encoder state.
- Extend drivetrain configuration validation to detect duplicate GPIO assignments
  and cross-subsystem pin collisions, not only duplicate PWM channels/PCNT units.
- Define and validate UART port, pin, and buffer constraints before installing the
  driver.
- Decide whether packet version/type errors need separate diagnostics from the
  aggregate parser error counter.
- Add packet modules before using the existing `COMMAND` and `STATUS` enum values;
  they currently have no payload definitions or consumers.

### I. Tests, Tooling, and Repository Configuration

Add host-testable or PlatformIO tests for:

- UART framing, parser resynchronization, checksum failures, and overwrite/queue
  behavior.
- Odometry byte order, round trips, malformed validity values, and non-finite data.
- Drivetrain kinematics normalization and sign conventions.
- Drivetrain rollback and watchdog state transitions using hardware wrappers or
  fakes.
- Encoder conversion math and glitch-filter bounds.
- PMW3610 matrix validation, fusion, invalid-sample pose retention, and recovery.
- Tape centroid, loss, ambiguity, reacquisition, saturation, and anti-windup.
- Calibration matrix fitting and complete JSON schema generation.

Repository cleanup:

- Remove machine-specific `upload_port = COM5` and `monitor_port = COM5` from the
  drivetrain PlatformIO configuration, or move them to an untracked local override.
- Correct stale documentation before treating it as an implementation reference.
- Consider pinning the resolved ArduinoJson version if reproducible builds matter;
  the declared `^7.4.2` currently resolves to 7.4.3.
- Add CI that performs clean builds for all four environments and runs tests.

## Hardware Acceptance Checklist

These items require the assembled robot:

- Confirm brake active level and startup behavior.
- Confirm coast versus active braking behavior.
- Confirm PWM polarity and duty-to-speed direction.
- Verify every motor and encoder direction setting.
- Verify X-drive wheel mapping, forward, strafe, and turn signs.
- Stop commands for more than 250 ms and confirm watchdog coasting.
- Verify UART wiring, shared ground, sustained packet rate, corrupt frames, arm
  reboot, and unplug behavior.
- Verify PMW3610 forward/lateral signs, scale, baseline, invalid samples, and
  recovery on representative surfaces.
- Verify each stepper direction, travel conversion, limits, simultaneous motion,
  and stop behavior.
- Verify tape channel order, active level, intersection patterns, and settling
  time.
- Verify lidar power, XSHUT sequencing, addresses, pull-ups, range data, and bus
  recovery.

## Recommended Implementation Order

1. Repair `calibration.json`, correct stale optical/build documentation, and
   verify fused optical output on hardware.
2. Integrate the arm odometry sender.
3. Implement the drivetrain receiver, sequence diagnostics, and 100 ms freshness
   timeout, including an explicit latest-only versus queued UART decision.
4. Replace the drivetrain bench main and exercise the existing safety layer and
   watchdog.
5. Repair stepper ownership/build warnings, then implement non-blocking motion.
6. Harden tape sensing and PID behavior.
7. Add board-specific I2C configs and lidar/ToF device integration.
8. Harden remaining driver error handling and API state semantics.
9. Add automated tests and clean-build CI.
10. Complete the hardware acceptance checklist before enabling unrestricted robot
    motion.

## Completion Definition

Firmware is production-ready only when:

- Both production mains use the intended shared modules rather than test code.
- Every clean build succeeds without project warnings.
- The deployed calibration schema is valid and hardware-verified.
- Odometry validity covers both sensor validity and transport freshness.
- Motor watchdog, brake, rollback, and direction behavior pass hardware tests.
- Stepper motion is non-blocking and does not starve sensing or UART service.
- Tape and lidar fault states are explicit and handled by the controller.
- Automated tests cover transport, math, state transitions, and malformed inputs.
- No machine-specific serial port is required in tracked project configuration.
