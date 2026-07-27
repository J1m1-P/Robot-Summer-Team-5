# Line Following System

## 1. Feature Overview

The line-following subsystem reads three four-channel tape modules, interprets a lateral line position, and computes a drivetrain body-velocity request.

The module in (+x) guides "forward" travel, the (-x) module "reverse", and (-y) module for "strafe" to the left.

The module located 90 degrees CCW relative to the direction of travel is intended to detect broad task markers (one or two lateral lines of tape).

## 2. Usage

```cpp
#include "control/line_following/line_follower.hpp"

LineFollowerContext ctx{&drivetrain, {&front_sensor, &back_sensor, &side_sensor}, &pose_service};

bool ok = follow_tape(&ctx, Direction::PX, /*speed_mps=*/0.2f,
                       StopCondition::DISTANCE, /*stop_value=*/1.5f,
                       /*timeout_s=*/12.0f);
```

- Caller owns and initializes `drivetrain`, `sensors[3]` (front/back/side), and `pose_service` once; reuse the same `ctx` across calls.
- `follow_tape()` **blocks** the calling task for the whole maneuver and drives `pose_service` itself, so pose/UART keep updating even outside `loop()`.
- `Direction`: `PX` = forward, `MX` = reverse, `PY` = strafe left.
- `StopCondition`:
  - `DISTANCE` — stop after `stop_value` meters.
  - `LATERAL_ONE` — center on a single tape strip (side sensor only, `dir == PX`).
  - `LATERAL_TWO` — center in the gap between two strips (side sensor only, `dir == PX`).
  - `TIME_ONLY` — run until `timeout_s`.
- Returns `true` if the stop condition was reached, `false` on timeout, lost tape, or error.
- Overshoot past the true stop point is corrected internally with a low-speed reverse crawl — no caller-side compensation needed.

