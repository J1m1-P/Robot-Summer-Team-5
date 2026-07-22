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
| **`MoveS`** | `include/control/drivetrain/move_s.h` / `src/control/drivetrain/move_s.c` | **Implemented and unit-tested (`test/test_move_s/`); §2's first motion primitive.** | Open-loop dead-reckoned straight move: `speed_profile.c` for jerk-bounded ramping, a stopping-distance target for the final approach, progress tracked against a `DrivetrainPose` the caller supplies each cycle. No outer PID (relies solely on the drivetrain facade's own wheel-velocity PI). **Not yet called from any harness or main loop**, and `F_lon`/`F_lat`/`F_ang` aren't threaded in yet (§3). |
| **`RotS`** | `include/control/drivetrain/rot_s.h` / `src/control/drivetrain/rot_s.c` | **Implemented and unit-tested (`test/test_rot_s/`); §2's rotation primitive, not in the original roadmap table — added to give §3's `F_ang` trials a real primitive.** | `MoveS`'s in-place-rotation counterpart, same shape (jerk-bounded stopping-distance target, no outer PID), reusing `speed_profile.c` directly for the angular ramp since its math is unit-agnostic. `angle_rad`'s sign is the only direction input. **Not yet called from any harness or main loop.** |
| **Encoder velocity + position** | `src/drivers/encoder/encoder_driver.c` | Implemented (includes the SM/T quadrature-grouping + low-speed-timeout work already merged to `tuning`) | `drivetrain_get_encoder_accumulated_count()` gives exact raw ticks (unaffected by the SM/T velocity smoothing); `encoder_driver_get_velocity_mps()` gives smoothed velocity. Use accumulated count for odometry, not velocity. |
| **Wheel-velocity inner loop** | `include/control/drivetrain/wheel_velocity_controller.h` | Implemented | The existing FF+PI loop `MoveS` relies on solely, and that `MoveL`/`MoveP`/`MoveC` sit on top of via the drivetrain facade. |
| **Drivetrain facade** | `include/control/drivetrain/drivetrain.h` | Implemented | `drivetrain_set_body_velocity()` / `drivetrain_update()` is the single entry point every motion source (tape following, and eventually the new Move APIs) should command through. Handles watchdog/timeout/coast/brake safety already — don't reimplement this. |
| **Closest existing analog of a PID-driven behavior module** | `src/control/tape_following/tape_follower.c` + `tape_following_controller.c` | Implemented, tested, **not wired into any production main loop either** | Template mirrored for structuring `off_tape_motion.c` (§1, see above): config/state split, a `*_update()` that takes input + `dt_s` and fills an output struct, bounded PID with clamped output. |
| **Diagnostic harness + dashboard pattern to extend** | `src/harnesses/drivetrain_test_main.cpp` + `tools/drivetrain_test_dashboard.html` | Implemented | This is the pattern §5's tuning harness/webpage and §6's jog/program-builder webpage should extend, not a new harness built from scratch. Serial command protocol + HTML dashboard already exist for the drivetrain/tape-following diagnostics. |

### Build environments (`platformio.ini`)

- `[env:native]` — host-compiled, no hardware. Pure modules only (currently
  `x_drive_kinematics.c`, `wheel_velocity_controller.c`, `odometry.c`,
  `drivetrain_odometry_source.c`, `off_tape_motion.c`, `speed_profile.c`,
  `move_s.c`, `rot_s.c`, `tape_following_controller.c`, `tape_follower.c`, `tape_following_kinematics.c`,
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

Five motion primitives, all taking `speed`/`omega` parameters; `max_accel`/
`max_alpha` are **config-level defaults, not per-call parameters** (see the
`MoveS`/`RotS` bullets below) — a traction/hardware ceiling that doesn't vary
call to call, same reasoning §4 already applied to `max_jerk`.

| API | Behavior | Feedback | Params |
|---|---|---|---|
| **MoveS** ("simple") | Open-loop-ish move relying solely on the existing wheel-velocity PI (no outer position/heading correction) | Wheel velocity PI only | distance, heading, speed |
| **RotS** ("rotate simple") | `MoveS`'s in-place-rotation counterpart, added to give §3's `F_ang` calibration trials a real primitive instead of the ad hoc harness `turn-angle` command. Open-loop, relies solely on the wheel-velocity PI | Wheel velocity PI only | angle (signed), max omega |
| **MoveL** ("linear") | Straight-line move with outer PID correction to stay on the commanded vector | Off-tape PID (§1) | distance, heading, speed |
| **MoveP** ("point") | Move to a point and finish at a specific ending heading; may need a path-planning helper to blend translation and final rotation | Off-tape PID (§1) + path planning | distance, heading, end heading, speed |
| **MoveC** ("circular") | Move along a circular arc (turns, smooth curves) — coordinated `vx` + `omega` tracking a defined radius, not a straight segment or a single end-orientation target | Off-tape PID (§1) | radius, arc angle (or start/end heading), speed |

- [x] Implement `MoveS`. `move_s.c`/`.h` — takes `{distance, heading, speed}`,
      drives purely open-loop (no outer PID; `off_tape_motion` is never
      called), using `speed_profile.c` for jerk-bounded accel/decel and a
      stopping-distance target (`sqrt(2*max_accel*remaining)`) to come to a
      smooth stop at `distance`. `max_accel_mps2` lives in `MoveSConfig`
      (currently `0.5f`, matching the value `drivetrain_test_main.cpp`'s own
      ramping already uses — a placeholder, not yet calibrated) rather than
      being a `move_s_start()` parameter: it's a traction ceiling shared by
      every call, not something that varies move to move. The drivetrain
      facade's own `drivetrain_update()` still does body→wheel kinematics and
      the wheel-velocity PI — `MoveS` only produces a `DrivetrainBodyVelocity`
      for the caller to apply via `drivetrain_set_body_velocity()`, same
      output contract as `tape_follower`. Progress is tracked against a
      world-frame direction captured once at `move_s_start()`, from a
      caller-supplied `DrivetrainPose` (i.e. `drivetrain_odometry_source`,
      §1) each cycle — deliberately not re-derived from the live heading each
      cycle, so heading drift isn't silently absorbed instead of measured.
      `F_lon`/`F_lat`/`F_ang` are not threaded in yet (see §3 —
      `move_calibration.c` doesn't exist, and `F_lat` needs a per-wheel hook
      the drivetrain facade doesn't currently expose). Unit-tested in
      `test/test_move_s/`, including a full simulated run to completion.
      **Not yet exercised on hardware or from any harness.**
- [x] Implement `RotS`. `rot_s.c`/`.h` — takes `{angle_rad (signed),
      max_omega}`; mirrors `MoveS` exactly (progress tracked against a
      heading captured once at `rot_s_start()`, jerk-bounded stopping-distance
      target, no outer PID, `max_alpha_rad_s2` in `RotSConfig` rather than a
      parameter — currently `1.5f`, matching `TAPE_FOLLOWER_CONFIG`'s heading
      config, also a placeholder) but for rotation instead of translation.
      Directly reuses `speed_profile.c` for the angular ramp — that module's
      math is unit-agnostic, so no new ramp code was needed. `angle_rad`'s
      sign is the only direction input (positive = counterclockwise, matching
      `DrivetrainBodyVelocity.omega`'s convention) — there's no separate
      heading parameter the way `MoveS` has one, since a signed scalar is a
      complete description of "how far to turn." Progress uses a plain
      subtraction against the caller-supplied current heading each cycle
      (`DrivetrainPose.heading_rad` accumulates continuously rather than
      wrapping, so no unwrap logic is needed). `F_ang` is not threaded in yet,
      same reasoning as `MoveS`'s `F_lon`/`F_lat`. Unit-tested in
      `test/test_rot_s/`, including full simulated runs for both rotation
      directions. **Not yet exercised on hardware or from any harness.**
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

- [ ] **Static/kinematic calibration** — three correction factors, only two of
      which are Jacobian-level (paper eq. 22-23: `J_c⁻¹ = F_lon · F_lat · J⁻¹`,
      applied to the body velocity → per-wheel target speed conversion);
      `F_ang` is a separate command-level correction, not part of that matrix
      (see its own bullet below):
  - [ ] `F_lon` (longitudinal, one scalar, eq. 18): repeated straight-line
        `MoveS` runs of known distance `L1` → measure the endpoint's
        longitudinal/lateral error `x̄e`/`ȳe` (averaged over trials) →
        `F_lon = L1 / √((L1-x̄e)² + ȳe²)` → uniformly rescale all wheel
        speeds to fix systematic under/overshoot.
  - [ ] `F_lat` (lateral, diagonal 4×4 — **one term per wheel**, eq. 19-20):
        from the *same* straight-line trial, also directly protractor-measure
        `θe` — the robot's own heading drift from where it started to where
        it stopped (mark the robot's heading before the run; this is not
        derived from `x̄e`/`ȳe` via trig, it's measured on its own). Compute
        per-wheel wheel-speed error `φ̇e = J⁻¹·[0,0,-θe]ᵀ` (the same inverse
        Jacobian `x_drive_kinematics.c` implements), then
        `F_lat = diag[1 + φ̇e,i/φ̇i]` (`φ̇i` = each wheel's commanded angular
        velocity during the trial). This is the answer to "do we need
        separate per-motor curves" from §4 below — a cheap per-wheel scale
        factor instead of fully separate PID tuning.
  - [ ] `F_ang` (angular, one scalar, eq. 21): a **separate** repeated
        in-place rotation `RotS` trial (§2 — `RotS` was added specifically so
        this trial has a real dead-reckoned primitive instead of the
        diagnostic harness's ad hoc `turn-angle` command) of a known angle
        `θ1` → protractor-measure the actual rotation `θ2` → `F_ang = θ1/θ2`
        (averaged over k trials). Needed as a second pass because `F_lat`
        alone doesn't fully correct heading error. **Not baked into the
        Jacobian like F_lon/F_lat** — applied as a scalar on the *commanded
        target angle* passed to `rot_s_start()` (`angle_rad = θ_want *
        F_ang`), not on angular speed and not via `x_drive_kinematics.c`.
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
- [x] This is a dependency of §2 — the motion APIs should call into the S-curve
      profile generator rather than commanding raw step targets. `MoveS` and
      `RotS` (§2) now both call `speed_profile_update()` directly.
      `MoveL`/`MoveP`/`MoveC` still don't exist.
- [ ] `max_accel` defaults should respect the slip-avoidance ceiling derived in §3's
      dynamic calibration. `MoveSConfig.max_accel_mps2` / `RotSConfig.max_alpha_rad_s2`
      exist now (as config defaults, not per-call params — see §2), currently set to
      placeholder values (`0.5f` / `1.5f`, borrowed from existing harness/tape-following
      defaults), not yet the real calibrated ceiling this bullet is about.

## 5. Wheel-velocity PI tuning harness + calibration workflow

### 5a. Manual tuning harness (DONE)

- [x] Redesigned `src/harnesses/tuning_main.cpp` for simple manual single-run
      testing (replaces the old automatic step-response harness):
  - [x] **Duty mode** — open-loop: set raw duty directly, hold for a fixed
        duration, measure peak speed. Used to characterize the raw duty→speed
        curve (feedforward calibration) without feedback masking the response.
  - [x] **PI mode** — closed-loop: set target speed + FF/Kp/Ki gains, run for
        fixed duration, measure settled speed. Used to tune feedback after
        feedforward is locked.
  - [x] **Turn mode** — negates setpoint on FL/BL (vs FR/BR) to drive in-place
        rotation instead of straight line, enabling peak omega measurement for
        the paper's `F_ang` calibration.
  - [x] All parameters (mode, duty, target, duration, turn, ff/ffo/kp/ki) are
        set via serial commands or the dashboard, run only on explicit `start`,
        auto-stop and freeze the chart after duration expires.
  - [x] **S-curve (jerk-limited) speed profile**, PI mode only: `jerk
        <value>` sets `SpeedProfileConfig.max_jerk_mps3` (0 disables —
        PI sees the raw step target), `accel <value>` sets a dedicated
        `max_accel_mps2` ceiling for the ramp (independent of `jerk`,
        which only bounds how fast that ceiling is approached; defaults
        to `0.5`, matching `MoveSConfig`'s own placeholder). The shared
        target speed is ramped through `speed_profile_update()` once per
        cycle (not per motor), then the turn sign-flip is applied per
        motor to the already-ramped value — every enabled motor sees the
        same smoothed target, not just whichever motor happens to be
        index 0. Duty mode is intentionally unaffected: it commands raw
        duty directly by design, so there's nothing to ramp. This gives
        the S-curve a value **before** the paper's calibration trials run
        (§5d) — see the note there on why it isn't itself derived from
        the paper's procedure.
- [x] Updated `tools/tuning_dashboard.html` to match:
  - [x] Mode toggle (Duty ↔ PI), field visibility toggling based on mode.
  - [x] Turn toggle (force-enables all four motors on next start).
  - [x] Start/Stop buttons (nothing moves until start).
  - [x] Jerk/accel fields (PI mode only, hidden in Duty mode).
  - [x] Real-time charting of velocity + duty per motor during active run.

### 5b. Feedforward characterization via Duty-mode sweeps (DONE)

- [x] Ran manual Duty-mode sweeps (all four motors enabled) and PI-mode step
      response to locate the wheel-velocity PI's gains. **Locked into
      `src/config/drivetrain/drivetrain_config.c`'s `wheel_controller`:**
      `kff = 1.2`, `kff_offset = 0.15`, `kp = 0.4`, `ki = 0.1`.
  - [x] Measured achievable wheel speed range at these gains: **~0.05 m/s**
        (deadband floor) to **~0.54 m/s** (`output_max` saturation ceiling)
        — documented in a comment above `wheel_controller` in
        `drivetrain_config.c`.
  - [ ] **Follow-up, not yet done:** `DRIVETRAIN_CONFIG.max_vx_mps` /
        `max_vy_mps` are still `1.0` — both exceed what a single wheel can
        deliver even for pure-forward motion (~0.62 m/s via the X-drive
        Jacobian's `cos(30°)` factor at this wheel-speed ceiling, before
        even accounting for combined vx/vy/omega motions that load wheels
        harder). Commanding anywhere near the current ceiling will saturate
        before reaching the requested body speed. Needs a deliberate call
        on what body-frame ceiling to set (worst-case combined motion, not
        just pure-forward), not a blind plug-in of the wheel number.
  - [ ] Reverse-direction sweep not yet run — current gains assume
        forward/reverse symmetry.

### 5c. PI loop tuning (DONE — see 5b)

- [x] `kp`/`ki` tuned together with the feedforward sweep above (same
      session, all four motors enabled) — see locked-in values in 5b.

### 5c-note. S-curve values from this session

- [x] `jerk = 1.0` (`max_jerk_mps3`), `accel = 2.5` (`max_accel_mps2`) tested
      and settled on via the tuning harness's PI-mode fields.
  - [x] Now have a home: `src/harnesses/calibration_main.cpp`'s
        `MoveSConfig`/`RotSConfig` RAM defaults (§5d) — still not in a
        `src/config/**` const definition, since nothing outside that harness
        consumes them yet, but no longer literally unused.

### 5d. Calibration harness (DONE — wiring) / procedure (NEXT — data + factors)

**Note on the S-curve (§4/§5a):** the paper's `F_lon`/`F_lat`/`F_ang`
correction factors are measured from *endpoint* error (distance/angle
actually traveled vs. commanded), which doesn't depend on how smoothly the
robot accelerated to get there — so the S-curve is not itself something the
paper's procedure calibrates. It should still be **on and set to a
reasonable value** (not left at infinite/unbounded jerk) during the
calibration trials, since `max_jerk_mps3`/`max_accel_mps2` are part of the
real operating configuration the robot will run calibrated values under —
calibrating against an unrealistic (instant-step) acceleration profile risks
factors that don't transfer to normal operation. `calibration_main.cpp`
defaults to the §5c-note values above; overridable live via `jerk`/`accel`.

- [x] **`MoveS`/`RotS` wired into a hardware harness**
      (`src/harnesses/calibration_main.cpp`, `[env:calibration]` in
      `platformio.ini`) — the piece every earlier version of this roadmap
      section listed as the blocker. Runs `MoveS`/`RotS` through the
      production `Drivetrain` facade (`drivetrain_set_body_velocity()` +
      `drivetrain_update()`, same cycle `main.cpp` will eventually use) with
      `drivetrain_odometry_source` continuously feeding live encoder counts
      into a `DrivetrainPose` each cycle — the "not yet wired into any
      harness or main loop" gap §0/§1 called out is closed for this harness.
      Serial commands: `move <distance_m> <heading_deg> <speed_mps>`,
      `rotate <angle_deg> <max_omega_deg_s>`, `tol`/`angtol`/`accel`/`alpha`/
      `jerk` to adjust `MoveSConfig`/`RotSConfig` live, `stop`/`brake`/
      `enable` for safety. Streams pose/remaining-distance telemetry during
      a run and prints a completion summary.
  - [x] **First hardware run found and fixed a real `RotS` bug**: `rotate`
        never completed — a commanded 90° rotation spun past 600° and kept
        going. Root cause (`rot_s.c`'s direction sign picked from the
        *originally commanded* angle instead of the *current* remaining
        error — once real PI tracking lag overshot the target, the module
        kept commanding the original direction forever instead of
        reversing to correct). Fixed with `copysignf(magnitude,
        remaining_rad)`; regression tests added
        (`test_overshoot_past_{positive,negative}_target_reverses_direction`
        in `test/test_rot_s/`) since the existing self-consistent
        Euler-integration tests couldn't reproduce it (their simulated
        heading tracks the commanded profile perfectly and never diverges
        enough to overshoot). `move`/`MoveS` doesn't share this bug — its
        always-positive `distance_m` means `fmaxf(0.0f, ...)` alone clamps
        overshoot to a stop.
  - [x] **Light logic review (no new hardware run) found and fixed a second,
        more severe latent bug in `odometry.c`** — the module `MoveS` (and
        eventually `MoveL`/`MoveP`/`MoveC`) depends on for progress
        tracking. `odometry.c`'s world-frame Y update had the wrong sign:
        `y_mm += -sine*forward + cosine*lateral` instead of
        `y_mm += sine*forward - cosine*lateral`. Traced by cross-checking
        against the documented conventions (`vy` positive = right,
        `omega` positive = CCW, both from `x_drive_kinematics.h`) and
        `MoveS`'s own progress dot-product math: the combined effect made
        `MoveS`'s tracked progress equal `distance × cos(2·θ0)`, where
        `θ0` is the robot's actual world heading when the move starts —
        only correct at `θ0 = 0`. At `θ0 = 90°` (e.g. immediately after a
        `RotS` turn) progress would compute with the *wrong sign*
        (thinks it's moving backward while moving forward, never
        completes) — the same "runs forever" symptom as the `RotS` bug,
        but latent, since every `move` run so far happened to start from
        heading 0 (fresh boot) where the bug is invisible. Directly
        threatens the square/multi-leg validation trajectories and any
        future chained `Move*`/`Rot*` sequence. Fixed by negating the
        `y_mm` update; existing tests that encoded the old (wrong) sign
        were corrected (`test_rotates_body_delta_into_world_frame`,
        `test_integrates_delta_at_origin`,
        `test_integrates_known_displacement`), and new regression
        coverage added: `test_rotates_lateral_delta_into_world_frame`
        (`test/test_drivetrain_odometry/`) — a companion `MoveS` test
        feeding this through real odometry math was added at the time and
        later removed, superseded by the architecture change below, which
        makes the whole scenario structurally unreachable rather than
        just correctly handled.
  - [x] **Architecture change: `MoveS`/`RotS` made genuinely open-loop.**
        Both previously used *live* odometry each cycle (a caller-supplied
        pose/heading) to decide when to decelerate/stop via a
        stopping-distance formula — technically feedback, even though
        billed as "open-loop, dead-reckoned." That's a problem specifically
        *for calibration*: odometry is exactly what §3 is trying to
        validate, so letting real mechanical error (wheel slip, radius
        mismatch) influence when a move ends would quietly absorb some of
        that error into the move's own duration instead of it showing up
        as measurable endpoint error — undermining the systemic-error
        measurement the whole harness exists for. Both `move_s_update()`
        and `rot_s_update()` no longer take a pose/heading parameter at
        all: "remaining" is now tracked via a new `planned_progress_m`/
        `planned_progress_rad` field, self-integrated purely from each
        module's own commanded output (`planned_progress += commanded *
        dt_s`) every cycle. Real odometry is now used only by the *caller*
        (`calibration_main.cpp`), captured separately at `move`/`rotate`
        start purely for the post-hoc completion-summary comparison —
        never fed back into the primitives. Side effects, both positive:
        the `RotS` runaway-direction bug class from the fix above is now
        structurally unreachable (there's no external signal left to
        disagree with the plan, so its two regression tests were removed
        as untestable-by-construction, replaced by a note in
        `test/test_rot_s/` explaining why); and `move_s.h`/`move_s.c` no
        longer depend on `odometry.h`/`DrivetrainPose` at all, so the
        `odometry.c` Y-sign bug class above is also now unreachable from
        `MoveS`'s side (though `odometry.c` itself is still correct and
        still used elsewhere, e.g. by `calibration_main.cpp`'s own
        reporting). `test/test_move_s/` and `test/test_rot_s/` updated
        throughout for the new signatures; two tests that only made sense
        with external pose/heading input were replaced with a
        self-integration check
        (`test_remaining_{distance,angle}_tracks_self_integrated_progress`).
  - [ ] Still only lightly exercised on real hardware — one `move` and one
        `rotate` run so far (both under the pre-open-loop design), not yet
        a full trial session. Neither the `odometry.c` fix nor the
        open-loop redesign has been verified on hardware yet — only
        unit-tested.
- [x] **`tools/calibration_helper.html`** — standalone calculator (no serial
      connection, unlike `tuning_dashboard.html`/`drivetrain_test_dashboard.html`
      — just takes hand-entered trial measurements and computes locally).
      Implements the paper's eq. 18 (`F_lon`), eq. 19–20 (`F_lat`, via a JS
      port of `x_drive_kinematics_body_to_wheel_velocities()`'s exact
      sign/order convention), eq. 21 (`F_ang`), and eq. 24–25
      (`δr_e`/`δr_m` before/after validation). Add-a-row tables for
      straight-line trials (`xe`/`ye`/`θe`) and rotation trials (`θ1`/`θ2`),
      averages automatically, outputs a copy-pastable comment block with
      the final numbers. Sanity-checked against known edge cases (zero
      error → identity factors; undershoot → correction factor > 1 in the
      expected direction) but not yet validated against a real trial's
      numbers end-to-end.
  - [ ] **Important limitation, documented in the harness's own header
        comment:** the completion summary's `odom_longitudinal_mm`/
        `odom_lateral_mm`/`odom_delta_deg` are odometry's own estimate of
        what happened, derived from the same encoder ticks and Jacobian
        `MoveS`/`RotS` use to decide when to stop — by construction this
        will always read close to zero against the commanded target and
        **cannot** detect real-world error (wheel slip, radius mismatch,
        per-wheel response differences). It is a sanity check that the move
        completed and behaved reasonably, not the paper's `x_e`/`y_e`/`θ_e`.
- [ ] Run the actual trials (needs the harness above, now available):
  - [ ] **Straight-line** — used to *derive* `F_lon`/`F_lat` (paper's eq. 18–20).
        Before running, mark both the robot's start **position** and its
        **heading** (a line off the front) — you need both for the three
        measurements this trial produces. Command `move <L1> 0 <speed>`,
        then measure: `xe`/`ye` (longitudinal/lateral endpoint error, tape
        measure against `commanded_mm`) *and* `θe` (heading drift, protractor
        against the marked starting heading — measured directly, not derived
        from `xe`/`ye`). Repeat for an average of each.
  - [ ] **In-place rotation (`RotS`)** — used to derive `F_ang` (paper's eq. 21).
        Command `rotate <angle> <max_omega>`, measure the real rotation with
        a protractor against `commanded_deg`, repeat for an average.
  - [ ] **Square** — validation only; confirms factors generalize.
  - [ ] **Hemicycle (semicircle)** — validation only; requires `MoveC` (§2).
- [ ] Compute radial position error metrics (`δr_e`, `δr_m` per paper eq. 24–25)
      to quantify improvement before/after correction factors applied.
- [ ] Evaluate whether individual motors need **separate tuning curves** if
      per-motor spreads are large after `F_lat` applied, or accept per-wheel
      `F_lat` scaling as sufficient compensation.

## 6. Post-tuning jog + program-builder webpage (end goal)

- [ ] Webpage to **jog** the robot manually (direct velocity/vector commands).
- [ ] Webpage support for composing **simple predefined paths** out of the
      `MoveS` / `MoveL` / `MoveP` / `MoveC` primitives (§2).
- [ ] Output should be easily **copyable into the main loop** (i.e. generates
      code/config the firmware can consume directly, not just a visualization).
- [ ] This is the overall end goal of this roadmap — depends on §2–§5 being done
      first.

## Suggested next steps (current state)

The motion APIs (§1–§4) and tuning harness (§5a) are **implemented and unit-tested**.
The wheel-velocity PI gains **are now tuned and locked into config** (§5b/§5c:
`kff=1.2`, `kff_offset=0.15`, `kp=0.4`, `ki=0.1`; achievable wheel speed range
~0.05–0.54 m/s). The calibration harness (§5d) is **wired and compiling**
(`[env:calibration]`, `src/harnesses/calibration_main.cpp`) but **not yet run
on hardware**. Here's the path forward:

1. **Run the calibration harness on hardware for the first time** — flash
   `[env:calibration]`, verify `move`/`rotate` behave sanely (completes,
   doesn't stall/oscillate) before trusting it for actual trials.
2. **Run straight-line and rotation trials (§5d)** — physically measure real
   endpoints against the harness's `commanded_mm`/`commanded_deg`, derive
   `F_lon`, `F_lat`, `F_ang` per the paper.
3. **Reconcile `max_vx_mps`/`max_vy_mps`/`max_omega_rad_s`** in
   `drivetrain_config.c` against the measured ~0.54 m/s wheel ceiling (§5b's
   follow-up) — currently `1.0`/`1.0`/`2.0`, which saturate before reaching
   the requested body speed for any combined motion.
4. **Run a reverse-direction Duty sweep** (§5b) to confirm forward/reverse
   symmetry, or capture separate gains if it doesn't hold.
5. **Implement `MoveL`/`MoveP`/`MoveC` (§2)** — as needed for the square/hemicycle
   validation trajectories, or postpone until after calibration is complete.
6. **Add jog + program-builder webpage (§6)** — final layer for real operation.
