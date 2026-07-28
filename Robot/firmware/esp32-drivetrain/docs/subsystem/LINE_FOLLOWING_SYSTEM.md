# Line Following

`follow_tape()` is one blocking maneuver. It reads the tape sensors, updates
the shared pose/UART service, and commands the drivetrain at 200 Hz.

## Setup

The application owns all hardware and passes borrowed pointers:

```cpp
LineFollowerContext context = {
    .drivetrain = &drivetrain,
    .sensors = {&front_sensor, &back_sensor, &left_sensor},
    .pose_service = &pose_service,
};
```

Keep the context and every referenced object alive for the entire call.

## Calling `follow_tape()`

```cpp
const bool reached = follow_tape(
    &context,
    Direction::PX,
    0.20f,
    StopCondition::DISTANCE,
    1.5f,
    12.0f);
```

Directions:

- `PX`: positive body X, guided by the front module.
- `MX`: negative body X, guided by the back module.
- `PY`: positive body Y, guided by the left-side module.

Stop conditions:

- `DISTANCE`: `stop_value` is cumulative path length in meters.
- `TIME_ONLY`: `stop_value` is seconds; `timeout_s` must be greater.
- `LATERAL_ONE`: center the array on one crossing strip.
- `LATERAL_TWO`: center it in the gap between two crossing strips.

The current hardware geometry supports marker stops only during `PX` travel.
Callers must not combine marker stops with another direction.

`timeout_s` is always a safety deadline. The call returns `true` only when the
requested stop is reached. Lost tape for two seconds, timeout, pose failure,
or sensor failure returns `false`.

## Control behavior

The active guide module produces a weighted four-channel line error. The
controller filters that error, applies a deadband and bounded PD steering, and
filters the final angular command. If the line disappears briefly, the robot
turns toward the last observed side while searching.

For distance and armed marker stops, speed ramps down over the last 30 mm.
Stopping is immediate once the estimated target is reached; there is no
post-stop overshoot correction.
