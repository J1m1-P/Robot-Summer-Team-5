# Drivetrain Runtime Test Commands

> Temporary command reference for `src/harnesses/drivetrain_test_main.cpp`.
> Lift and secure the robot before issuing movement commands.

## Connect

Upload the test harness and open its 115200-baud serial monitor:

```powershell
pio run -e drivetrain-test -t upload
pio device monitor -e drivetrain-test
```

Send each command followed by Enter. The firmware displays a `> ` prompt,
echoes typed characters, and supports both Backspace and Delete. Do not add
PlatformIO's `--echo` option; monitor-side echo would display every character
twice because the firmware handles echo and line editing itself.

### USB button interface

For point-and-click control, close the PlatformIO serial monitor and open
`tools/drivetrain_test_dashboard.html` in desktop Chrome or Edge. Click
**Connect USB**, select `USB-SERIAL CH340K`, and use the movement or
individual-wheel buttons. The page communicates directly over Web Serial and
does not connect to the robot's Wi-Fi.

## Wheel Identifiers

| Wheel | Name | Numeric identifier | Hardware pair |
| --- | --- | --- | --- |
| Front-left | `fl` | `1` | M1/E1 |
| Back-left | `bl` | `2` | M2/E2 |
| Front-right | `fr` | `3` | M3/E3 |
| Back-right | `br` | `4` | M4/E4 |

## Individual Motor/Encoder Tests

```text
pair <wheel> <duty> [duration_ms]
```

Duty must be nonzero and between `-0.80` and `+0.80`. A negative duty commands
the opposite direction. The encoder velocity should have the same sign as the
commanded duty.

Examples:

```text
pair fl 0.25 1500
pair fr 0.25 1500
pair bl -0.25 1500
pair br 0.25 3000
```

## Translation

```text
forward [speed_mps] [duration_ms]
backward [speed_mps] [duration_ms]
left [speed_mps] [duration_ms]
right [speed_mps] [duration_ms]
```

Examples:

```text
forward 0.20 1500
backward 0.15 2000
left 0.20 1000
right 0.20 1000
```

## 45-Degree Diagonal Translation

```text
forward-left [speed_mps] [duration_ms]
forward-right [speed_mps] [duration_ms]
backward-left [speed_mps] [duration_ms]
backward-right [speed_mps] [duration_ms]
```

Examples:

```text
forward-left 0.20 1500
forward-right 0.20 1500
backward-left 0.15 1000
backward-right 0.15 1000
```

## Rotation

```text
cw [angular_speed_rad_s] [duration_ms]
ccw [angular_speed_rad_s] [duration_ms]
```

Examples:

```text
cw 0.60 1500
ccw 0.60 1500
```

## Encoder-Relative Distance and Angle

Move a measured distance in any translation direction:

```text
distance <direction> <meters> [maximum_speed_mps]
```

Turn through a measured encoder-relative angle:

```text
turn-angle cw|ccw <degrees> [maximum_rad_s]
```

Examples:

```text
distance forward 1.0 0.30
distance forward-right 0.5 0.20
turn-angle cw 90 0.60
turn-angle ccw 45 0.40
```

These commands snapshot all four encoders, transform wheel displacement back
into robot-frame motion, slow down as the goal approaches, and stop at the
configured tolerance. Wheel slip, wheel diameter error, and incorrect encoder
direction still affect accuracy.

## Automatic Complete Test

The sequence runs forward, backward, left, right, all four diagonals, clockwise,
and counterclockwise. It inserts a 750 ms coast pause between movements.

```text
sequence [speed_mps] [duration_ms_per_movement]
```

Example:

```text
sequence 0.20 1500
```

## Runtime Direction Inversion

Toggle the current setting:

```text
invert motor <wheel>
invert encoder <wheel>
```

Set it explicitly:

```text
invert motor <wheel> 0
invert motor <wheel> 1
invert encoder <wheel> 0
invert encoder <wheel> 1
```

Examples:

```text
invert motor fl
invert encoder fl 1
```

These changes are held only in RAM and reset to the configured values after a
reboot.

## Runtime PI and Feedforward Tuning

The wheel controller is feedforward plus PI. All four wheels share the runtime
values:

```text
pi show
pi kff <value>
pi offset <value>
pi kp <value>
pi ki <value>
pi slew <duty_per_second>
pi reset
```

Examples:

```text
pi kp 0.45
pi ki 0.18
pi kff 1.10
pi offset 0.065
pi slew 2.5
```

Changes are RAM-only. `pi reset` restores the values compiled into
`drivetrain_config.c`.

## Motion and Reporting Settings

```text
ramp <acceleration> <deceleration>
distance-tolerance <meters>
angle-tolerance <degrees>
timeout <milliseconds>
telemetry <period_ms>
limits
```

`ramp` uses the same numeric rates as m/s² for translation and rad/s² for
rotation. `timeout` applies to encoder-relative position commands. `telemetry`
accepts periods from 20 to 5000 ms. `limits` prints all live settings and PI
values.

## Utility Commands

```text
stop
coast
brake
enable
reset-encoders
status
config
limits
help
```

- `stop`: ramp to zero, use closed-loop braking, then coast.
- `coast`: immediately command zero PWM while leaving the brake released.
- `brake`: coast, engage the hardware brake, and disable motor outputs.
- `enable`: re-enable outputs and release the brake after `brake`.
- `reset-encoders`: stop motion and reset all four accumulated counts.
- `status`: print encoder counts, target speeds, measured speeds, and duties.
- `config`: print current motor and encoder inversion settings.
- `limits`: print live motion settings and PI/feedforward values.
- `help`: print the built-in command summary.

## Defaults and Limits

| Setting | Value |
| --- | --- |
| Translation speed | `0.20 m/s` |
| Rotation speed | `0.60 rad/s` |
| Command duration | `1500 ms` |
| Maximum command duration | `10000 ms` |
| Maximum translation speed | `1.00 m/s` |
| Maximum individual-test duty | `0.80` |
| Sequence coast pause | `750 ms` |
| Acceleration | `0.50 m/s²` or `rad/s²` |
| Deceleration | `0.80 m/s²` or `rad/s²` |
| Distance tolerance | `0.01 m` |
| Angle tolerance | `2 degrees` |
| Position timeout | `10000 ms` |
| Telemetry period | `100 ms` |

All movement commands stop automatically when their duration expires.
