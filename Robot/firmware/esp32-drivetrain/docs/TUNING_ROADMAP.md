# Tuning Branch Roadmap

Outstanding work for the `tuning` branch, captured as a checklist. Items are listed in
no particular order except where a dependency is called out explicitly.

This doc is written to be self-contained for a fresh session with no prior context —
read "0. Context for implementers" first before starting any item below.

## 0. Context for implementers

### Architecture conventions already established in this repo

Follow these unless there's a specific reason not to — they're consistent across
every existing module (`drivetrain`, `encoder_driver`, `tape_follower`,
`wheel_velocity_controller`, `odometry`):

- **Config vs. state separation.** Every stateful module splits into an immutable
  `const` `*Config` struct (board/tuning values, never mutated) and a separate
  mutable runtime struct (retained history across calls). Example:
  `TapeFollowerConfig` vs. `TapeFollower`, `EncoderDriverConfig` vs. `EncoderDriver`,
  `WheelVelocityPiConfig` vs. `WheelVelocityPi`. Runtime objects hold a `const
  Config*` pointer; callers own the config's lifetime (global `const` objects in
  `src/config/**` satisfy this).
- **Pure math stays hardware-free and host-testable.** Modules like
  `tape_following_controller.c`, `x_drive_kinematics.c`, and `odometry.c` take
  plain structs/floats in and out, no GPIO/ESP-IDF calls, so they can run in the
  `[env:native]` PlatformIO environment and be unit tested without hardware. Any
  new PID/kinematics/planning math (§1, §3, §4) should follow this pattern and get
  added to `[env:native]`'s `build_src_filter` in `platformio.ini`.
- **`config_is_valid()` + `esp_err_t` pattern.** Every module exposes a
  `*_config_is_valid()` checker (rejects NaN/inf, out-of-range, null) called from
  init, and every fallible function returns `esp_err_t` (`ESP_OK` /
  `ESP_ERR_INVALID_ARG` / `ESP_ERR_INVALID_STATE`, etc.) rather than throwing or
  asserting.
- **A stateful behavior module never drives hardware directly.** `tape_follower.c`
  explicitly documents "never calls the drivetrain directly" — it fills an output
  struct with a requested `DrivetrainBodyVelocity` plus a `motion_valid` flag, and
  the *caller* (currently only `drivetrain_test_main.cpp`) decides whether to apply
  it via `drivetrain_set_body_velocity()`. Follow the same contract for the
  off-tape movement module (§1) and `MoveS`/`MoveL`/`MoveP`/`MoveC` (§2) — they
  compute, the harness/application layer applies.
- **No dynamic allocation, designated initializers for config, namespace-scope
  runtime objects in harnesses.** Match existing style in
  `src/config/drivetrain/*.c` and `src/harnesses/*.cpp`.
- **Any non-`ESP_OK` return from an update function fed into
  `drivetrain_update()`'s call chain currently brakes the whole drivetrain** (see
  `update_encoders()` in `src/control/drivetrain/drivetrain.c`). Any new module
  whose per-cycle update can legitimately be "nothing to do yet" (e.g. waiting on a
  sensor) must return `ESP_OK` for that case, not an error — this bit the encoder
  SM/T low-speed fallback and is easy to get wrong.

### Existing building blocks — reuse these, don't re-derive them

| Module | Path | Status | Relevance |
|---|---|---|---|
| **Body↔wheel kinematics** | `include/control/drivetrain/x_drive_kinematics.h` | Implemented, tested | `DrivetrainBodyVelocity {vx, vy, omega}` is the shared output contract for all motion sources. `x_drive_kinematics_wheel_to_body_velocities()` already converts wheel deltas back to a body-frame delta — see next row. |
| **World-frame pose integration** | `include/control/drivetrain/odometry.h` / `src/control/drivetrain/odometry.c` | **Implemented and unit-tested (`test/test_drivetrain_odometry/`), but not called anywhere in production code.** | This is most of §1's "error source" module already. `DrivetrainOdometry` accumulates a `DrivetrainPose {x_mm, y_mm, heading_rad}` from repeated `DrivetrainOdometryDelta {forward_mm, lateral_mm, heading_delta_rad}` calls via `drivetrain_odometry_update()`. **What's missing:** something to produce that `DrivetrainOdometryDelta` from encoder counts each cycle and call this on a schedule. |
| **Wheel-delta → body-delta bridge** | `include/control/drivetrain/drivetrain_odometry_source.h` / `src/control/drivetrain/drivetrain_odometry_source.c` | **Implemented and unit-tested (`test/test_drivetrain_odometry_source/`); §1's error-source module.** | Promoted from the harness-local `get_relative_body_motion()` prototype (`drivetrain_test_main.cpp:279`, still there as a reference/comparison, not removed) into a real module: takes plain `DrivetrainWheelCounts` (no `Drivetrain*`/hardware dependency, so it's `[env:native]`-testable) each cycle, diffs against the previous cycle's counts, and calls `drivetrain_odometry_update()` continuously. **Still not called from any harness or main loop** — nothing yet reads live encoder counts into it on a schedule. |
| **Off-tape motion PID** | `include/control/drivetrain/off_tape_motion.h` / `src/control/drivetrain/off_tape_motion.c` | **Implemented and unit-tested (`test/test_off_tape_motion/`); §1's shared PID.** | Reuses `tape_following_controller`'s generic bounded PID directly (nested `TapeFollowingControllerConfig`) rather than re-deriving one. Takes a caller-computed scalar `error` + `travel_velocity_mps`, outputs a `DrivetrainBodyVelocity` with the correction applied laterally (`vy`) and `omega` left at `0.0f` for a later per-primitive heading layer (e.g. `MoveP`'s terminal phase). **Not yet consumed by any motion primitive** — §2 doesn't exist yet. |
| **Jerk-bounded speed ramp** | `include/control/drivetrain/speed_profile.h` / `src/control/drivetrain/speed_profile.c` | **Implemented and unit-tested (`test/test_speed_profile/`); §4's S-curve deliverable.** | Simplified jerk-bounded ramp (not a full 7-segment S-curve — a deliberate scope choice). Rate-limits acceleration each cycle toward whatever bang-bang acceleration would reach the target speed, bounded by `max_accel_mps2` and `SpeedProfileConfig.max_jerk_mps3`. **Not yet called by anything** — no motion primitive exists yet to drive it. |
| **Encoder velocity + position** | `src/drivers/encoder/encoder_driver.c` | Implemented (includes the SM/T quadrature-grouping + low-speed-timeout work already merged to `tuning`) | `drivetrain_get_encoder_accumulated_count()` gives exact raw ticks (unaffected by the SM/T velocity smoothing); `encoder_driver_get_velocity_mps()` gives smoothed velocity. Use accumulated count for odometry, not velocity. |
| **Wheel-velocity inner loop** | `include/control/drivetrain/wheel_velocity_controller.h` | Implemented | The existing FF+PI loop `MoveS` relies on solely, and that `MoveL`/`MoveP`/`MoveC` sit on top of via the drivetrain facade. |
| **Drivetrain facade** | `include/control/drivetrain/drivetrain.h` | Implemented | `drivetrain_set_body_velocity()` / `drivetrain_update()` is the single entry point every motion source (tape following, and eventually the new Move APIs) should command through. Handles watchdog/timeout/coast/brake safety already — don't reimplement this. |
| **Closest existing analog of a PID-driven behavior module** | `src/control/tape_following/tape_follower.c` + `tape_following_controller.c` | Implemented, tested, **not wired into any production main loop either** | Template mirrored for structuring `off_tape_motion.c` (§1, see above): config/state split, a `*_update()` that takes input + `dt_s` and fills an output struct, bounded PID with clamped output. |
| **Diagnostic harness + dashboard pattern to extend** | `src/harnesses/drivetrain_test_main.cpp` + `tools/drivetrain_test_dashboard.html` | Implemented | This is the pattern §5's tuning harness/webpage and §6's jog/program-builder webpage should extend, not a new harness built from scratch. Serial command protocol + HTML dashboard already exist for the drivetrain/tape-following diagnostics. |

### Build environments (`platformio.ini`)

- `[env:native]` — host-compiled, no hardware. Pure modules only (currently
  `x_drive_kinematics.c`, `wheel_velocity_controller.c`, `odometry.c`,
  `drivetrain_odometry_source.c`, `off_tape_motion.c`, `speed_profile.c`,
  `tape_following_controller.c`, `tape_follower.c`, `tape_following_kinematics.c`,
  `tape_line_estimator.c`, `tape_task_detection.c`). **Add any new pure
  PID/kinematics/planning module here** so it gets unit tested without hardware.
- `[env:drivetrain-test]` — the harness referenced throughout this doc
  (`drivetrain_test_main.cpp`, now also builds `odometry.c` so §1's pose bridge
  can run on hardware); this is almost certainly where §2's Move APIs get
  exercised first and where §5/§6's webpages attach.
- `[env:tuning]` / `[env:drive]` — single-wheel PI tuning and full closed-loop
  drive harnesses; not directly relevant to this roadmap but shows the existing
  environment-per-purpose convention to follow if a new dedicated harness turns out
  to be needed.

### Reference papers (in `misc/`)

- `misc/M_T_method_based_incremental_encoder_velocity_measurement_error_analysis_and_self-adaptive_error_elimination_algorithm.pdf`
  — **already implemented**, see the SM/T quadrature-grouping logic in
  `encoder_driver_update()` (`src/drivers/encoder/encoder_driver.c`). No action
  needed here; referenced only for context on why the encoder velocity code looks
  the way it does.
- `misc/Calibration_Method_of_Mecanum_Wheeled_Mobile_Robot_Odometer.pdf` — **not
  yet implemented**, informs §3 below (`F_lon`/`F_lat`/`F_ang` static correction +
  dynamic max-acceleration index). Read this paper before implementing §3.

### Now-merged reference module

`tape_following_kinematics.c/h` (`src/control/tape_following/`,
`include/control/tape_following/`) is merged into `tuning` as of the
`TapeFollowing`/`DrivetrainCleanup` PRs. It maps a requested `{vx, vy}` into a
coordinated turn `omega` (turns the robot to face the direction it's currently
strafing, rate-limited). It's a useful reference pattern for `MoveC`'s arc-tracking
math (§2), even though its actual purpose there is tape-following, not off-tape
motion. `docs/subsystem/TAPE_FOLLOWING_SYSTEM.md` is also merged and worth matching
in style if this roadmap's modules get similar documentation later.

## 1. Off-tape movement module

- [x] PID-based movement controller for when the robot is **not** following tape
      (open-field point-to-point / vector / arc moves). The shared PID itself is
      implemented (`off_tape_motion.c`/`.h`, reusing `tape_following_controller`'s
      generic bounded PID rather than re-deriving one, per §0's reuse guidance) and
      unit-tested (`test/test_off_tape_motion/`). Still open: no motion primitive
      (§2) calls it yet, so it's unexercised outside its own tests.
- [x] The controller must consume an **error function from a separate module**,
      not compute it inline — mirrors the existing split between
      `tape_line_estimator` (pure math) and `tape_sensor_driver` (hardware), so the
      PID stays agnostic to where the error comes from. `off_tape_motion_update()`
      takes a caller-computed `error` field (`OffTapeMotionInput`); the module
      itself never computes an error.
- [x] For now, the error source is **encoder-based odometry only**. Most of this
      already exists (see §0's building-blocks table) — `DrivetrainOdometry`
      (`odometry.c`) already integrates world-frame pose from body-frame deltas,
      and `get_relative_body_motion()` in `drivetrain_test_main.cpp` already shows
      how to turn encoder ticks into a body-frame delta. What's missing is
      packaging that bridge into a real module that calls
      `drivetrain_odometry_update()` continuously every cycle (not just as a
      since-start snapshot), rather than reimplementing odometry from scratch.
      Done: `drivetrain_odometry_source.c`/`.h` is that module (takes plain
      `DrivetrainWheelCounts` + config, no hardware/`Drivetrain*` dependency, so it
      stays `[env:native]`-testable); unit-tested in
      `test/test_drivetrain_odometry_source/`. Not yet wired into any harness or
      main loop that actually reads live encoder counts each cycle (`platformio.ini`
      only builds it, doesn't call it from anywhere yet).
- [ ] Design the error-source interface so a **PMW3610 optical flow sensor** can
      replace or supplement encoder odometry later without changing the PID or the
      motion API layer above it. Partially addressed: `off_tape_motion_update()`'s
      input is already a caller-computed scalar `error`, so a future optical-flow
      error source only has to produce that same scalar. No second implementation
      exists yet to confirm the interface actually generalizes.
- [ ] This single PID instance is reused by `MoveL`, `MoveP`, and `MoveC` (§2) —
      only the error function each one tracks differs (distance-along-line vs.
      distance-to-point vs. distance-off-arc). `MoveS` intentionally does **not**
      use it. Not yet true: none of `MoveL`/`MoveP`/`MoveC` exist.

## 2. Motion API layer

Four motion primitives, all taking `speed` and `max_accel` parameters:

| API | Behavior | Feedback | Params |
|---|---|---|---|
| **MoveS** ("simple") | Open-loop-ish move relying solely on the existing wheel-velocity PI (no outer position/heading correction) | Wheel velocity PI only | distance, heading, speed, max accel |
| **MoveL** ("linear") | Straight-line move with outer PID correction to stay on the commanded vector | Off-tape PID (§1) | distance, heading, speed, max accel |
| **MoveP** ("point") | Move to a point and finish at a specific ending heading; may need a path-planning helper to blend translation and final rotation | Off-tape PID (§1) + path planning | distance, heading, end heading, speed, max accel |
| **MoveC** ("circular") | Move along a circular arc (turns, smooth curves) — coordinated `vx` + `omega` tracking a defined radius, not a straight segment or a single end-orientation target | Off-tape PID (§1) | radius, arc angle (or start/end heading), speed, max accel |

- [ ] Implement `MoveS`
- [ ] Implement `MoveL`
- [ ] Implement `MoveP`
- [ ] Implement `MoveC`
- [x] Determine whether `MoveP` needs a dedicated path-planning helper module, or
      whether it can reuse `MoveL` plus a terminal heading-correction phase.
      Decided: dedicated `path_planner.c` module (not yet implemented — this is a
      design decision, not a completed module).
- [ ] `MoveC` needs its own arc-tracking error function (distance/heading off the
      target arc) feeding the shared off-tape PID (§1) — this is a distinct control
      problem from `MoveP`'s "reach a point, end at a heading," so keep it a
      separate primitive rather than folding it into `MoveP`'s path planner.

## 3. MoveS calibration methodology

Reference: *"Calibration Method of Mecanum Wheeled Mobile Robot Odometer"*
(Tu & Min, 2019). Since `MoveS` is pure dead-reckoning (no outer PID), it's exactly
the failure mode this paper targets, and its procedure gives a concrete calibration
recipe rather than just ad-hoc gain tweaking.

- [ ] **Static/kinematic calibration** — three correction factors applied to the
      wheel Jacobian (body velocity → per-wheel target speed conversion):
  - [ ] `F_lon` (longitudinal, one scalar): repeated straight-line `MoveS` runs of
        known distance → measure endpoint error → uniformly rescale all wheel
        speeds to fix systematic under/overshoot.
  - [ ] `F_lat` (lateral, diagonal 4×4 — **one term per wheel**): from the same
        straight-line drift data, back out a per-wheel gain correction via the
        inverse Jacobian. This is the answer to "do we need separate per-motor
        curves" from §4 below — a cheap per-wheel scale factor instead of fully
        separate PID tuning.
  - [ ] `F_ang` (angular, one scalar): repeated in-place rotation `MoveS` runs of a
        known angle → measure actual rotation (protractor, or a gyro if available)
        → `F_ang = commanded / actual`. Needed as a second pass because `F_lat`
        alone doesn't fully correct heading error.
- [ ] **Dynamic calibration** — bench-measure per-wheel current/voltage/speed,
      least-squares fit viscous + Coulomb friction and effective mass/inertia, and
      derive a maximum commanded acceleration that keeps required wheel traction
      under available floor friction. This becomes the default `max_accel` ceiling
      the S-curve profile generator (§4) should respect — exceeding it causes wheel
      slip, which is non-systematic error that no static correction can fix.
- [ ] Use **radial position error** `δr_e` (Euclidean distance between commanded
      and actual endpoint, averaged over repeated trials) and percent improvement
      `δr_m` as the objective metric — same statistic the tuning harness (§5)
      should report for locking in FF/Kp/Ki.

## 4. Speed profile shaping (S-curves)

- [x] Replace/augment simple accel limiting with **S-curve (jerk-limited) speed
      profiles** so `MoveS`/`MoveL`/`MoveP`/`MoveC` don't feed the wheel-velocity PI
      step changes in target speed. Implemented as a **simplified jerk-bounded
      ramp** (`speed_profile.c`/`.h`), not a full 7-segment S-curve state machine —
      a deliberate scope call, not a missed requirement: it rate-limits
      acceleration toward whatever bang-bang acceleration would reach the target
      speed, rather than pre-planning distinct accel/cruise/decel segments.
      Unit-tested in `test/test_speed_profile/`.
- [x] This effectively adds a `max_jerk` (or equivalent) dimension on top of the
      existing `speed` / `max_accel` parameters — needs a decision on where that
      value lives (per-call parameter vs. fixed config). Decided: config default
      (`SpeedProfileConfig.max_jerk_mps3`); no per-call override yet since no
      caller (§2) exists to need one — add one when a motion primitive does.
- [ ] This is a dependency of §2 — the motion APIs should call into the S-curve
      profile generator rather than commanding raw step targets. Not yet true:
      `speed_profile.c` is built and tested but unused, since `MoveS`/`MoveL`/
      `MoveP`/`MoveC` don't exist yet.
- [ ] `max_accel` defaults should respect the slip-avoidance ceiling derived in §3's
      dynamic calibration.

## 5. Pre-lock-in tuning harness + webpage

- [ ] Extend the existing diagnostic harness/dashboard pattern
      (`src/harnesses/drivetrain_test_main.cpp` +
      `tools/drivetrain_test_dashboard.html`) to support tuning:
  - [ ] Final wheel-velocity PI parameters (`kff`, `kff_offset`, `kp`, `ki`, currently
        in `src/config/drivetrain/drivetrain_config.c`).
  - [ ] Off-tape PID path-travel parameters (§1).
  - [ ] `MoveS` static/dynamic calibration procedure from §3 (`F_lon`, `F_lat`,
        `F_ang`, max-acceleration index).
- [ ] Add calibration/validation trajectories, matching the paper's methodology:
  - [ ] **Straight-line** — used to *derive* `F_lon`/`F_lat`/`F_ang` (§3).
  - [ ] **Square** — validation trajectory only; confirms the derived factors
        generalize, does not re-derive them.
  - [ ] **Hemicycle (semicircle)** — validation trajectory only, same as square.
        Depends on `MoveC` (§2) existing, since it requires a true continuous arc,
        not discrete straight segments + point turns.
- [ ] Once tuned, **lock in** the FF/Kp/Ki values as the shipped config.
- [ ] Evaluate whether individual motors need **separate tuning curves** if hardware
      response differs noticeably wheel-to-wheel, or whether `F_lat` (§3) is
      sufficient compensation instead of fully separate gain sets.

## 6. Post-tuning jog + program-builder webpage (end goal)

- [ ] Webpage to **jog** the robot manually (direct velocity/vector commands).
- [ ] Webpage support for composing **simple predefined paths** out of the
      `MoveS` / `MoveL` / `MoveP` / `MoveC` primitives (§2).
- [ ] Output should be easily **copyable into the main loop** (i.e. generates
      code/config the firmware can consume directly, not just a visualization).
- [ ] This is the overall end goal of this roadmap — depends on §2–§5 being done
      first.

## Suggested dependency order

Not mandated, but a reasonable build order given the dependencies above:

1. Off-tape error-source module (§1) — needed before any outer-loop PID exists.
2. S-curve profile generator (§4) — needed by all four motion APIs.
3. `MoveS` (§2), then its calibration procedure (§3) — establishes `F_lon`,
   `F_lat`, `F_ang`, and the max-acceleration ceiling used everywhere else.
4. `MoveL` → `MoveP` → `MoveC` (§2), in that order (each adds a layer on the last).
5. Tuning harness/webpage (§5), including straight-line/square/hemicycle
   validation — needed to actually lock in gains before relying on the motion
   APIs for real paths.
6. Jog + program-builder webpage (§6) — final layer once everything above is tuned
   and stable.
