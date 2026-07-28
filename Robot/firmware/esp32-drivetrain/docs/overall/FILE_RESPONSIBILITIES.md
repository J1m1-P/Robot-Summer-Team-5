# File Responsibilities

Headers define public contracts. Sources own validation, state transitions,
hardware calls, and private helpers.

## Application

| Files | Responsibility |
|---|---|
| `src/main.cpp` | Owns production state, initializes and connects subsystems, and implements Arduino `setup()`/`loop()` scheduling and fault handling. |
| `src/harnesses/*_main.cpp` | Alternative diagnostic applications selected by PlatformIO source filters. |

## Configuration

Files under `config/` contain immutable board wiring, geometry, limits, and
control settings. They must not own runtime state or behavior.

- `config/drivetrain/`: motors, encoders, drivetrain geometry and limits.
- `config/tape_following/`: tape mux and module wiring.
- `config/communication/`: UART and I2C peripheral settings.
- `config/time_of_flight/`: ToF hardware settings.

## Drivers

Drivers own direct hardware access and one device family's local state.

| Module | Responsibility |
|---|---|
| `drivers/motor/` | Signed duty, direction, PWM, coast, and lifecycle for one motor. |
| `drivers/encoder/` | Quadrature counting and wheel velocity for one encoder. |
| `drivers/tape_sensor/` | Shared-mux selection and sampling for the three tape modules. |
| `drivers/time_of_flight/` | ToF device communication and samples. |

Drivers do not make course decisions or coordinate the complete drivetrain.

## Drivetrain and motion

| Module | Responsibility |
|---|---|
| `control/drivetrain/drivetrain.*` | Owns four motors, four encoders, wheel controllers, safety limits, watchdog, and body-velocity commands. |
| `control/drivetrain/x_drive_kinematics.*` | Converts body motion and wheel motion. |
| `control/drivetrain/odometry.*` | Integrates a body-frame delta into world pose. |
| `control/drivetrain/drivetrain_odometry_source.*` | Converts encoder-count changes into body deltas. |
| `control/drivetrain/pmw3610_odometry_source.*` | Converts cumulative optical packets into body deltas. |
| `control/odometry/pose_tracker.*` | Selects fresh optical deltas and encoder fallback to maintain one fused pose. |
| `control/odometry/pose_service.*` | Sole reader of the shared arm UART; dispatches frames and advances `PoseTracker`. |
| `control/motion/translator.*` | Implements blocking body-relative `precision_move()`. |
| `control/pid/bounded_pid.*` | Reusable bounded PID math with no hardware access. |

## Tape following

| Module | Responsibility |
|---|---|
| `control/line_following/line_follower.*` | Implements blocking `follow_tape()`: sensor reads, line error, steering, stop detection, pose servicing, and drivetrain commands. |

The application owns the sensors, drivetrain, and pose service. The follower
borrows them through `LineFollowerContext`.

## Robot actions

| Module | Responsibility |
|---|---|
| `control/task/movement_action_controller.*` | Maps sequence movement actions to `follow_tape()` or `precision_move()`. |
| `control/task/robot_sequence_controller.*` | Starts ordered movement/arm steps, handles arm status and Pi reports, and stops on terminal failure. |
| `comm/odometry_link.*` | Decodes and caches PMW3610 packets already routed from UART. |

`PoseService` is the only production reader of the arm UART. Other modules
receive already-dequeued frames or cached packets.

## Tests and tools

- `test/`: native unit tests and minimal ESP-IDF stubs.
- `tools/`: browser-based diagnostics for the matching harness.
- `tools/deprecated/`: retained historical diagnostics; not production APIs.

## Dependency direction

```text
main -> task/motion/pose control -> drivetrain -> drivers
                   |
configuration -----+
robot-common ------+-- shared packets, UART, timing, and math
```

Lower layers must not include application code. Hardware-independent control
modules should remain buildable in native tests.
