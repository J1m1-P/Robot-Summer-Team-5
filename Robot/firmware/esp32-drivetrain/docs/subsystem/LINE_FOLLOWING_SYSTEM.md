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

## Tape-stop masks

`TapeStopSpec` uses two independent bitmasks. `sensor_mask` selects tape
modules; `channel_mask` selects channels within each selected module.

| Bit | Sensor module | Direction |
| --- | --- | --- |
| `1 << 0` | Front module | `PX` |
| `1 << 1` | Back module | `MX` |
| `1 << 2` | Side module | `PY` |

The channel bits use the same pattern inside the selected module:
`1 << 0` selects channel 0, through `1 << 3` for channel 3. For example:

```cpp
TapeStopSpec spec = {
    .sensor_mask = 1U << 2,  // select the PY/side module
    .required_sensor_count = 1,
    .channel_mask = 1U << TAPE_SENSOR_CHANNEL_0,
};
```

This means “stop when channel 0 on the PY/side module sees tape.” A spec can
select multiple modules by OR-ing sensor bits together. In that case,
`required_sensor_count` says how many selected modules must be active.

For lateral stops, the marker sensor is perpendicular to the travel direction:

- Traveling `PX` uses the `PY`/side module (`1 << 2`).
- Traveling `PY` uses the `MX`/back module (`1 << 1`).
- Traveling `MX` has no perpendicular marker module installed.

With `stop_on_gap = false`, a stop triggers on the selected sensor's rising
tape edge. With `stop_on_gap = true`, the condition records a first tape edge,
waits for a gap, then requires a second tape edge within
`max_gap_distance_m` (normally 80 mm).

The alignment routines use the same module order but do not use `sensor_mask`:
`PX` maps to `sensors[0]`, `MX` to `sensors[1]`, and `PY` to `sensors[2]`.
They calculate a weighted channel-centre error using weights `{-3, -1, 1, 3}`
and drive a bounded correction until the selected tape or gap is centered.

Stop conditions:

- `DISTANCE`: `stop_value` is cumulative path length in meters.
- `TIME_ONLY`: `stop_value` is seconds; `timeout_s` must be greater.
- `RISE_ONE`: stop on one detected tape edge.
- `RISE_TWO`: after tape has been seen, stop on a second tape edge after a
  gap, within `max_gap_distance_m`.

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

The reusable `TapeStopCondition` module is shared with the precision
translator. `RISE_ONE` is a one-edge stop; `RISE_TWO` is the two-edge
edge-gap-edge detector described above.
