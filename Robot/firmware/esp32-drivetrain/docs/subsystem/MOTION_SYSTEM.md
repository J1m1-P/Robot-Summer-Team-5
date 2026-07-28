# Motion System

## 1. Feature Overview

The motion subsystem executes a body-relative translation and heading change
using the shared drivetrain and fused pose service. It captures the current
pose when an action starts, converts the relative target into world
coordinates, and closes the loop at 200 Hz.

The controller produces body-frame velocity requests. The drivetrain's wheel
controller remains responsible for wheel feasibility, acceleration response,
and motor output.

## 2. Usage

```cpp
#include "control/motion/translator.hpp"

PrecisionMoveContext ctx = {&drivetrain, &pose_service};
PrecisionMoveTarget target = {
    .dx_body_m = 0.25f,
    .dy_body_m = 0.0f,
    .delta_heading_rad = 0.0f,
    .body_velocity = {.vx = 0.15f, .vy = 0.0f, .omega = 0.0f},
};

const esp_err_t result = precision_move(&ctx, &target, /*timeout_s=*/15.0f);
```

- `dx_body_m` is forward/backward distance; positive is body `+x`.
- `dy_body_m` is lateral distance; positive is body `+y`/left.
- `delta_heading_rad` is relative rotation; positive is counterclockwise.
- `body_velocity` is the cruise/feed-forward body velocity in m/s and rad/s.
- The target is anchored to the pose returned by the shared `PoseService` at
  the beginning of the call. The robot does not need to start at `(0, 0, 0)`.
- `precision_move()` blocks for the maneuver and services `PoseService` every
  control cycle, matching `follow_tape()`'s ownership model.
- It returns `ESP_OK` after the position, heading, and measured-speed checks
  pass; timeout, invalid pose, and drivetrain errors are returned directly.
- The drivetrain is always stopped before the function returns.

## 3. Control Behaviour

The controller rotates toward the translation path, blends toward the final
heading while translating, then approaches the target over the final 30 mm.
It stops directly once the target is within the configured position and
heading tolerances and measured motion is below the practical wheel deadband.

There is no endpoint-hold or rotational-settle subsystem. Small residual PID
commands below the wheel controller's effective motion threshold are not used
to keep a maneuver alive.

## 4. Sequence Integration

The application owns one global `Drivetrain`, `PoseTracker`, and `PoseService`.
`main.cpp` creates one borrowed `PrecisionMoveContext` and injects it into the
movement-action controller:

```cpp
movement_action_controller_set_precision_move_context(&precision_move_ctx);
```

Sequence entries use body-relative action values:

```c
{ROBOT_STEP_MOVEMENT,
 {.movement = MOVEMENT_ACTION_GO_FORWARD}, 0.25f},
{ROBOT_STEP_MOVEMENT,
 {.movement = MOVEMENT_ACTION_ROTATE}, 90.0f},
```

`GO_FORWARD`, `GO_LEFT_DISTANCE`, `GO_RIGHT_DISTANCE`, and `ROTATE` are
blocking actions. The sequence advances only after the action returns success.

## 5. Current Limitations

- The public motion call is blocking; it is not a scheduler or asynchronous
  command handle.
- The movement-action controller currently supplies its own default cruise
  velocity. Direct callers can set `PrecisionMoveTarget::body_velocity`.
- The motion controller depends on an initialized shared `PoseService`; it
  does not own or create a second pose tracker.
