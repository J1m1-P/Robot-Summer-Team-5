# Motion System

Executes a body-relative translation/heading change against the shared
drivetrain and fused pose tracker, closing the loop at 200 Hz. The controller
produces body-frame velocity requests; the drivetrain handles wheel
feasibility, acceleration response, and motor output.

## precision_move()

```cpp
PrecisionMoveContext ctx = {
    .drivetrain = &drivetrain,
    .sequence_controller = &robot_sequence_controller,
    .sensors = {&front_sensor, &back_sensor, &left_sensor},
};
PrecisionMoveTarget target = {
    .dx_body_m = 0.25f,           // +x forward
    .dy_body_m = 0.0f,            // +y left
    .delta_heading_rad = 0.0f,    // + counterclockwise
    .body_velocity = {.vx = 0.15f, .vy = 0.0f, .omega = 0.75f},
};
const esp_err_t result = precision_move(&ctx, &target, /*timeout_s=*/15.0f);
```

The target is relative to the pose at call time. The call blocks until it
settles, times out, or encounters an error, and stops the drivetrain before
returning. During the maneuver it calls
`robot_sequence_controller_update()` every control cycle to keep UART and
pose processing live.

### Tape stops

Set `tape_stop_enabled` and `tape_stop_spec` to end early on a sensor pattern
using the same `TapeStopCondition` evaluator as `follow_tape()` lateral stops.
The position/heading target becomes a safety bound; reaching it first returns
`ESP_ERR_TIMEOUT`. This currently triggers on first detection and does not
center on a strip or gap. All three sensors are required.

### align_on_tape()

After a `follow_tape()` `RISE_ONE` or `RISE_TWO` stop, the guide sensor
is on the feature but the body can still be skewed. `align_on_tape()` pivots
about the guide sensor (`travel_dir`) while correcting from a different
sensor (`feedback_dir`) until that sensor is centered:

```cpp
align_on_tape(&align_ctx, /*travel_dir=*/Direction::PY,
              /*feedback_dir=*/Direction::MX,
              /*on_gap=*/false, /*timeout_s=*/3.0f);
```

`on_gap` must match the preceding stop: `false` after `LATERAL_ONE` and `true`
after `RISE_TWO`. The feedback error is EMA-filtered before PID and the
output omega is EMA-smoothed, matching `follow_tape()` steering.

## Sequence integration

Production `main.cpp` owns the drivetrain, pose tracker, odometry link, and
sequence controller. It injects borrowed line-following and precision-motion
contexts before explicitly starting the sequence:

```cpp
movement_action_controller_set_line_follower_context(&line_follower_ctx);
movement_action_controller_set_precision_move_context(&precision_move_ctx);
movement_action_controller_begin_sequence();
robot_sequence_controller_start(&robot_sequence_controller, millis());
```

Sequence entries pass one body-relative `action_value` (distance, degrees, or
speed depending on the action). All movement actions block until resolved;
the sequence advances only on success.

## Limitations

- Motion calls are blocking, not asynchronous command handles.
- The movement-action controller supplies default cruise velocities; direct
  callers can choose lower values in `PrecisionMoveTarget`.
- Motion depends on the initialized sequence controller and does not own or
  create another pose tracker.
