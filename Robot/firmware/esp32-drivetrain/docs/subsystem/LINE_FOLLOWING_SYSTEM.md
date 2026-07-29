# Line Following

`follow_tape()` is one blocking maneuver: reads the tape sensors, updates the
shared pose/UART service, and commands the drivetrain at 200 Hz.

```cpp
LineFollowerContext context = {
    .drivetrain = &drivetrain,
    .sensors = {&front_sensor, &back_sensor, &left_sensor},
    .sequence_controller = &robot_sequence_controller,
};
const bool reached = follow_tape(
    &context, Direction::PX, 0.20f, StopCondition::DISTANCE, 1.5f, 12.0f);
```

Directions: `PX` (front module), `MX` (back module), `PY` (left-side module).

Stop conditions:

- `DISTANCE`: `stop_value` is cumulative path length in meters.
- `TIME_ONLY`: `stop_value` is seconds; `timeout_s` must be greater.
- `LATERAL_ONE`: center the array on one crossing strip.
- `LATERAL_TWO`: after tape has been seen, stop only when the selected sensor
  sees tape on its two outer channels and no tape on its two centre channels.

Marker detection uses the sensor 90 degrees counter-clockwise from travel:
`PX` uses side/+Y, `PY` uses back/-X. No -Y module exists, so don't combine
marker stops with `MX`.

Returns `true` only when the requested stop is reached; `timeout_s` is always
a safety deadline. Returns `false` on timeout, pose/sensor failure, or lost
tape past the search sweep.

## Control behavior

A weighted four-channel line error is filtered, deadbanded, and steered with
a bounded PD controller. If the line disappears, the robot sweeps toward the
last observed side by 45 degrees, then reverses through the start heading to
45 degrees on the other side before giving up.

Speed ramps down over the last 30 mm before a distance/marker stop; stopping
is immediate at the estimated target, with no overshoot correction.

The reusable `TapeStopCondition` module (shared with the precision
translator) is deliberately uncalibrated: `LATERAL_ONE`/`LATERAL_TWO` stop at
the first qualifying detection rather than centering on a strip/gap.
