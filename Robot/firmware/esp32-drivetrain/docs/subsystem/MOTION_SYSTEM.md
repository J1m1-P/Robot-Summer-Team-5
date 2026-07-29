# Motion System

Executes a body-relative translation/heading change against the shared
drivetrain and fused pose service, closing the loop at 200 Hz. Produces
body-frame velocity requests; the drivetrain handles wheel feasibility and
motor output.

## precision_move()

```cpp
PrecisionMoveContext ctx = {
    .drivetrain = &drivetrain,
    .pose_service = &pose_service,
    .sensors = {&front_sensor, &back_sensor, &left_sensor},
};
PrecisionMoveTarget target = {
    .dx_body_m = 0.25f,           // +x forward
    .dy_body_m = 0.0f,            // +y left
    .delta_heading_rad = 0.0f,    // + counterclockwise
    .body_velocity = {.vx = 0.15f, .vy = 0.0f, .omega = 0.75f},  // cruise speed
};
const esp_err_t result = precision_move(&ctx, &target, /*timeout_s=*/15.0f);
```

Target is relative to the pose at call time. Blocks until settled, timeout,
or error; always stops the drivetrain before returning.

### Tape stops

Set `tape_stop_enabled` + `tape_stop_spec` to end early on a sensor pattern
(shared `TapeStopCondition` evaluator, same as `follow_tape()`'s LATERAL
stops). The position/heading target becomes a safety bound -- reaching it
first returns `ESP_ERR_TIMEOUT`. Uncalibrated: triggers on first detection,
doesn't center on a strip/gap. Requires all three sensors in the context.

### align_on_tape()

After a `follow_tape()` `LATERAL_ONE`/`LATERAL_TWO` stop, the guide sensor is
on the feature but the body can still be skewed -- and centering that same
sensor's channels can't reveal skew, since it's already on target. This
pivots about the guide sensor (`travel_dir`) while correcting off a second,
different sensor (`feedback_dir`), until *that* sensor also centers:

```cpp
align_on_tape(&align_ctx, /*travel_dir=*/Direction::PY,
              /*feedback_dir=*/Direction::MX,
              /*on_gap=*/false, /*timeout_s=*/3.0f);
```

`on_gap` must match the stop that preceded it: `false` after `LATERAL_ONE`,
`true` after `LATERAL_TWO`.

## Sequence integration

```cpp
movement_action_controller_set_line_follower_context(&line_follower_ctx);
movement_action_controller_set_precision_move_context(&precision_move_ctx);
```

Sequence entries pass one body-relative `action_value` (distance, degrees,
or speed depending on the action -- see the comment on each
`movement_action_controller.cpp` case). All movement actions block until
resolved; the sequence advances only on success.

## Limitations

- Blocking call, not an async command handle.
- Depends on an initialized shared `PoseService`; doesn't own a pose tracker.
