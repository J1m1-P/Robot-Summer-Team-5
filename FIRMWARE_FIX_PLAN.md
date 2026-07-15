# Firmware Fix Plan

> Updated from the firmware review on 2026-07-15.
>
> Scope: `Robot/firmware/esp32-drivetrain`, `Robot/firmware/esp32-arm`, and
> `Robot/firmware/lib/robot-common`.
>
> Out of scope for this plan: calibration files and FreeRTOS/task/mutex changes.

## Current Status

Both production `main.cpp` files are temporary test programs and are not treated as
the final firmware entry points. Their replacement should happen only after the
shared odometry protocol and driver safety behavior are agreed upon.

### Completed Fixes

- `motor_driver_init()` now returns `esp_err_t` consistently.
- UART, packet framing, logging, and common robot types have been moved into the
  shared `robot-common` library.
- The shared logging tag table contains the union of tags used by both boards.
- `app_log_tag()` now returns `"unknown"` for invalid or missing table entries.
- PMW3610 pose faults now preserve the last valid pose, skip the invalid delta,
  and resume from the retained pose on the next valid sample.
- Motor direction and PWM operations now propagate errors from `gpio_set_level()`,
  `ledc_set_duty()`, and `ledc_update_duty()`.
- UART and I2C headers now document that runtime objects must be zero-initialized
  before their first initialization call.
- The temporary drivetrain `main.cpp` no longer logs initialization or enablement
  success after a failure, and it stops issuing commands after a runtime error.
- Drivetrain configuration is validated before hardware initialization.
- Drivetrain initialization and enablement now roll back earlier successful steps
  when a later motor or encoder operation fails.
- Drivetrain startup keeps the brake engaged until enablement completes.
- Duty setters no longer release the brake implicitly.
- Partial motor-command failures force the entire drivetrain into a braked,
  disabled state.
- `drivetrain_tick()` now coasts the drivetrain after 250 ms without a successful
  motor command.
- Float `clamp()` is provided once by `robot-common` and reused by the drivetrain
  and motor driver.

### Still Incomplete

- Arm production `main.cpp` is still an Arduino stub.
- Drivetrain production `main.cpp` is still a single-motor bench program.
- The odometry packet is not shared; the arm owns `delta_pose_packet.*`, while the
  drivetrain's `odometry_packet.*` files are empty.
- No drivetrain-side odometry receiver exists.
- Stepper movement is blocking and the stepper module has header/configuration
  problems.
- Tape PID configuration and fault handling need hardening.
- UART diagnostic counters exist but are not surfaced periodically.
- The drivetrain PlatformIO configuration contains a machine-specific upload port.

## Adopted Drivetrain Decisions

The drivetrain implementation now uses these decisions:

1. Startup and failure rollback engage the brake. Normal duty setters cannot
   release it; movement resumes through `drivetrain_enable()` or an explicit coast.
2. Motor commands time out after 250 ms and cause the drivetrain to coast.

The following communication choices still require confirmation:

1. The shared odometry protocol carries cumulative pose rather than per-cycle
   deltas.
2. The odometry payload includes a sequence number for dropped-packet and reset
   detection.
3. Odometry becomes stale after 100 ms without a valid decoded packet.

## Session 1: Drivetrain Safety and State (Implemented)

### 1. Define Brake Semantics

Recommended behavior:

- `drivetrain_init()` engages the brake before initializing motors or encoders.
- `drivetrain_enable()` enables all motors and releases the brake only after every
  motor enables successfully.
- Duty setters change duty only; they do not silently change brake state.
- `drivetrain_brake()` sets all duties to zero and engages the brake.
- The brake remains engaged until another explicit `drivetrain_enable()` call.
- `drivetrain_coast()` releases the brake and sets all PWM outputs to zero because
  free movement is part of the meaning of coasting.

This keeps movement natural while ensuring that an intentional brake command cannot
be silently undone by a later duty setter.

### 2. Implement the Command Watchdog

`drivetrain_tick()` is a dead-man watchdog. It must be called every main-loop cycle,
even when no new command arrives.

Add the following runtime state to `Drivetrain`:

```c
int64_t last_command_us;
bool command_timeout_active;
```

Every successful duty command records the current timestamp. A failed command must
not refresh the timestamp.

Recommended behavior:

```c
#define DRIVETRAIN_COMMAND_TIMEOUT_US 250000

esp_err_t drivetrain_tick(Drivetrain *drivetrain, int64_t now_us) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->initialized) return ESP_ERR_INVALID_STATE;
    if (!drivetrain->enabled) return ESP_OK;

    if (drivetrain->last_command_us == 0 ||
        now_us - drivetrain->last_command_us > DRIVETRAIN_COMMAND_TIMEOUT_US) {
        if (!drivetrain->command_timeout_active) {
            esp_err_t err = drivetrain_coast(drivetrain);
            if (err != ESP_OK) return err;
            drivetrain->command_timeout_active = true;
        }
    }

    return ESP_OK;
}
```

A successful new command clears `command_timeout_active`.

### 3. Add Initialization Rollback

Before touching hardware, validate every drivetrain, motor, and encoder
configuration.

During initialization:

1. Engage the brake.
2. Track how many motors and encoders initialize or start successfully.
3. If a later operation fails, stop all started encoders in reverse order.
4. Coast and disable all initialized motors in reverse order.
5. Keep the brake engaged.
6. Clear the drivetrain runtime object.
7. Return the original error that triggered rollback.

Driver cleanup should be best effort: attempt every cleanup operation while
retaining the first error for reporting.

Use the same approach in `drivetrain_enable()`. If motor three fails to enable,
disable motors zero through two and leave the brake engaged.

For `drivetrain_set_all_motor_duty()`:

- Validate and clamp every input before applying any output.
- If one motor update fails after earlier motors changed, immediately brake or coast
  all motors rather than leaving mixed wheel duties.
- Return the original motor update error.

### 4. Align Error Codes

- Null pointers and invalid values return `ESP_ERR_INVALID_ARG`.
- Calls made before initialization or while disabled return
  `ESP_ERR_INVALID_STATE`.
- Hardware errors propagate unchanged.

## Session 2: Shared Odometry Protocol

### 1. Replace the Arm-Specific Packet

Do not move `delta_pose_packet.*` into the common library unchanged. It depends on
the arm-only `DeltaPose` sensing type.

Create instead:

```text
Robot/firmware/lib/robot-common/
├── include/robot_common/odometry_packet.h
└── src/odometry_packet.c
```

The shared wire type should depend only on common types:

```c
typedef struct {
    float x_mm;
    float y_mm;
    float theta_rad;
    uint32_t sequence;
    bool valid;
} OdometryPacket;
```

Provide:

```c
esp_err_t odometry_packet_send(UartLink *link, const OdometryPacket *packet);
bool odometry_packet_is(const PacketFrame *frame);
esp_err_t odometry_packet_decode(const PacketFrame *frame,
                                 OdometryPacket *packet_out);
```

Serialize each field explicitly. Do not transmit a C struct directly because struct
padding is compiler-dependent.

After both projects use the shared packet:

- Remove arm `delta_pose_packet.*`.
- Remove the empty drivetrain `odometry_packet.*` placeholders.
- Confirm that packet encoding and decoding exist only in `robot-common`.

### 2. Arm-Side Cumulative Pose

The arm should:

1. Poll both PMW3610 sensors.
2. Fuse the current sample into a `DeltaPose`.
3. Update `Pmw3610PoseManager` only when the sample is valid.
4. Preserve the previous pose when either sensor is invalid.
5. Increment the packet sequence number for each transmitted packet.
6. Send cumulative pose with `valid=false` during invalid sensor samples.

The private `DeltaPose` type remains in the arm sensing module. Only cumulative pose
crosses the board boundary.

### 3. Drivetrain Receiver and UART Freshness

Create a drivetrain odometry receiver that owns the last decoded packet timestamp.
Sensor validity and transport freshness should ideally be tracked separately, with
`RobotOdometry.valid` representing both conditions.

Recommended timeout implementation:

```c
#define ODOMETRY_TIMEOUT_US 100000

typedef struct {
    RobotOdometry odometry;
    int64_t last_packet_us;
    uint32_t last_sequence;
    bool has_sequence;
} OdometryReceiver;

esp_err_t odometry_update(OdometryReceiver *receiver, UartLink *link) {
    if (receiver == NULL || link == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t err = uart_link_update(link);
    if (err != ESP_OK) return err;

    PacketFrame frame;
    while (uart_link_take_packet(link, &frame) == ESP_OK) {
        OdometryPacket packet;
        if (odometry_packet_decode(&frame, &packet) != ESP_OK) continue;

        receiver->last_packet_us = esp_timer_get_time();
        receiver->last_sequence = packet.sequence;
        receiver->has_sequence = true;

        receiver->odometry.pose.x_m = packet.x_mm / 1000.0f;
        receiver->odometry.pose.y_m = packet.y_mm / 1000.0f;
        receiver->odometry.pose.theta_rad = packet.theta_rad;
        receiver->odometry.valid = packet.valid;
        receiver->odometry.timestamp_ms =
            (uint32_t)(receiver->last_packet_us / 1000);
    }

    int64_t now_us = esp_timer_get_time();
    if (receiver->last_packet_us == 0 ||
        now_us - receiver->last_packet_us > ODOMETRY_TIMEOUT_US) {
        receiver->odometry.valid = false;
    }

    return ESP_OK;
}
```

Sequence gaps should increment a diagnostic counter. A sequence reset should be
logged because it may indicate that the arm board rebooted.

### 4. Surface UART Diagnostics

Periodically report changes to:

- `packets_received`
- `packets_overwritten`
- `checksum_errors`
- `parse_errors`
- Odometry timeout count
- Sequence gap count

Rate-limit these logs so an unhealthy link does not flood the control loop.

## Session 3: Non-Blocking Stepper Drivers

### 1. Repair Header and Configuration Ownership

- Remove `#include "drivers/stepper_driver.h"` from `stepper_config.h`.
- Keep `StepperConfig` in `stepper_config.h`.
- Remove private `static` helper declarations from `stepper_driver.h`.
- Replace inline configuration objects with `extern const` declarations.
- Add `src/config/stepper_config.cpp` containing the definitions.
- Use `PIN_STEP1`, `PIN_STEP1_DIR`, and the remaining pin macros from
  `pin_map.h`; do not duplicate numeric pin values.

### 2. Replace Blocking Movement

Recommended runtime state:

```c
typedef struct {
    uint8_t step_pin;
    uint8_t dir_pin;
    uint32_t step_pulse_us;
    uint32_t step_delay_us;
    long steps_remaining;
    long position_steps;
    int8_t direction;
    int64_t next_step_us;
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

`stepper_service()` emits at most one pulse per call. The arm loop services all four
drivers every iteration, allowing simultaneous movement without blocking UART or
sensor polling.

### 3. Validate Movement Inputs

- Reject zero pulse or delay values.
- Handle `LONG_MIN` without negating it.
- Detect overflow when converting millimeters to steps.
- Keep distance conversion separate from the low-level driver.
- Update `position_steps` only after a pulse is emitted successfully.
- Define whether `stepper_stop()` preserves or clears the queued move.

### 4. Stepper Verification

- Confirm each motor's direction on hardware.
- Confirm one pulse produces one commanded step.
- Run two or more axes simultaneously.
- Confirm UART and optical polling continue during long moves.
- Confirm stop behavior and position accounting.

## Session 4: Tape PID Hardening

### Current Problems

- `FRONT_PID_WEIGHTS`, `BACK_PID_WEIGHTS`, and `LEFT_PID_WEIGHTS` are defined but
  not declared in the public configuration header.
- PID gains and inputs are not checked for NaN or infinity.
- A negative `integral_limit` produces incorrect clamping.
- `output_min > output_max` is accepted.
- Before tape has ever been detected, line loss selects the positive/right fallback
  because `last_known_error` starts at zero.
- `line_was_present` is recorded but not used to suppress derivative spikes when
  tape is reacquired.
- Integral accumulation continues while the output is saturated.
- Returning `0.0f` for invalid arguments is indistinguishable from a valid zero
  correction.
- Symmetric and all-active patterns produce zero error, so an intersection can be
  mistaken for a centered line.

### Recommended Changes

1. Export all three weight configurations from `tape_following_config.h`.
2. Add a gain validation function that requires finite gains, a non-negative
   integral limit, and ordered finite output limits.
3. Add `has_last_known_error` to `TapePidState`.
4. Return an explicit line state such as `LINE_TRACKED`, `LINE_LOST`, or
   `LINE_AMBIGUOUS` instead of only a boolean.
5. On initial line loss, return a caller-selected search behavior rather than always
   turning right.
6. Suppress or reset derivative history when transitioning from lost to tracked.
7. Use conditional integration: do not integrate farther into output saturation.
8. Change `tape_pid_update()` to return `esp_err_t` and write correction through an
   output pointer.
9. Define application behavior for intersections and all-active patterns.
10. Verify sensor weight order and sign on the physical robot.

## Session 5: Production Loop Integration

After the previous sessions are verified, replace the temporary mains.

### Arm Loop

At a fixed cadence:

1. Poll optical sensors.
2. Fuse the current sample.
3. Update or retain cumulative pose based on validity.
4. Send the shared cumulative odometry packet.
5. Service every active stepper.
6. Report rate-limited faults.

### Drivetrain Loop

At a fixed cadence:

1. Update the UART receiver and odometry freshness.
2. Update wheel encoders.
3. Read tape sensors when required by the active controller.
4. Run the state machine or controller.
5. Apply the resulting motor command.
6. Call `drivetrain_tick()` last on every iteration.

## Hardware Checks

These checks cannot be resolved through code review:

- Determine the real PWM polarity of the motor driver.
- Verify motor and encoder direction settings.
- Verify X-drive wheel mapping and signs.
- Verify tape sensor channel order and active level.
- Check whether the 5 microsecond tape mux settling time is sufficient.
- Review ESP32-S3 pin restrictions for encoder and mux-select pins.
- Verify UART wiring, shared ground, packet rate, and unplug behavior.

## Validation Checklist

### Static Checks

- Both PlatformIO projects build.
- No arm-only sensing headers are included by `robot-common`.
- Packet encoding and decoding exist only in `robot-common`.
- `drivetrain_tick()` has exactly one declaration and one implementation.
- No stepper runtime function performs a complete multi-step move in a blocking loop.

### Drivetrain Safety

- Startup leaves the brake engaged.
- Partial initialization leaves all motors stopped and the brake engaged.
- Partial enable rolls back previously enabled motors.
- A multi-motor update failure triggers a safe all-motor state.
- Motor commands stopping for more than 250 ms causes coasting.

### Odometry Link

- Normal motion produces cumulative pose on the drivetrain.
- Invalid optical samples retain the previous pose and set `valid=false`.
- Unplugging UART makes odometry invalid within 100 ms.
- A checksum failure does not update pose or freshness.
- Sequence gaps and arm resets are reported.

### Steppers

- Multiple axes move concurrently.
- UART and optical polling continue during long moves.
- Stop and completion states are reported correctly.
- Position accounting matches emitted steps.

### Tape PID

- Invalid gains are rejected.
- Initial line loss does not arbitrarily choose right.
- Reacquiring tape does not create a derivative spike.
- Saturation does not continue winding up the integral.
- Intersections are distinguishable from a centered line.

## Recommended Implementation Order

1. Confirm brake, timeout, and packet decisions.
2. Implement drivetrain rollback and command watchdog.
3. Add the shared cumulative odometry packet.
4. Add arm sender and drivetrain receiver with freshness timeout.
5. Rework steppers into the service-based API.
6. Harden tape PID behavior and configuration.
7. Replace both temporary production mains.
8. Perform hardware acceptance checks.
