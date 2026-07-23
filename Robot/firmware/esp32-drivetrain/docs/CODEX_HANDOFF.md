# Handoff: Calibration Methodology + Open Issue

Written for a fresh session with no prior context. Read this before touching
`rot_s.c`, `move_s.c`, `speed_profile.c`, or `calibration_main.cpp`.

## 1. What this project is doing

### Coordinate convention (authoritative)

All drivetrain and world-frame motion uses a right-handed planar frame:
`+X` is robot front, `+Y` is robot left, and positive heading/omega is
counter-clockwise. `MoveP` targets are world-frame coordinates; `MoveS` and
`MoveL` headings are body-relative, with positive heading toward body `+Y`.
The wheel Jacobian, odometry, path planner, calibration direction tables, and
hardware-test direction names all follow this convention.

### Current implementation addendum

The current hardware calibration set is single-speed and direction-specific:
0.30 m/s translation and 60 deg/s rotation, with three trials per condition.
The authoritative values are in src/config/drivetrain/move_calibration_config.c:

- longitudinal: +x 0.9225, -x 0.9030, +y 1.2808, -y 1.2709;
- lateral: four per-wheel tables, one for each translation direction;
- angular target factor: 0.93755.

The advanced drivetrain applies the X and Y longitudinal factors to the
corresponding body-velocity components and applies the lateral table selected
by the dominant translation axis. F_ang is stored for target-angle use only;
it is not applied to instantaneous omega, so it does not distort MoveP or
MoveC curvature. MoveS and RotS remain uncalibrated measurement primitives.

MoveR is the closed-loop in-place rotation primitive. The calibration harness
command rotl <target_heading_deg> uses an absolute world-frame heading.
All harness motion commands reject a new command while another motion is
active; use stop before replacing an active command.

Final endpoint adjustment for both `MoveP` and `MoveC` is pulse-based
(shared `endpoint_settle.h`/`.c`): commands the minimum effective correction
for a short pulse, waits for odometry to settle, repeats only if still outside
tolerance. Intentional: the wheel velocity loop has a practical low-speed
deadband, so a continuous sub-deadband command cannot correct precisely.

`esp32-drivetrain` is PlatformIO firmware for a 4-wheel X-drive (mecanum-style)
robot. The `tuning` branch's current goal is calibrating out systematic
dead-reckoning error, following the procedure in Tu & Min (2019), *"Calibration
Method of Mecanum Wheeled Mobile Robot Odometer"*
(`misc/Calibration_Method_of_Mecanum_Wheeled_Mobile_Robot_Odometer.pdf`). Full
background, architecture conventions, and a running checklist of what's
done/pending live in `docs/TUNING_ROADMAP.md` — that doc is the source of
truth for project state; this file is a narrower snapshot for the specific
issue below.

## 2. The calibration methodology (short version)

The paper derives three correction factors from physical trials:

- **`F_lon`** (scalar): repeated straight-line runs of known distance `L1` →
  measure real endpoint error (`xe` longitudinal, `ye` lateral, tape measure)
  → `F_lon = L1 / sqrt((L1-xe)^2 + ye^2)`. Applied as a uniform scalar on
  commanded wheel speed (scales the whole Jacobian output).
- **`F_lat`** (per-wheel diagonal, 4 values): from the *same* straight-line
  trial, also protractor-measure `theta_e` (heading drift from a marked
  starting heading — a **direct** reading, not derived from `xe`/`ye`).
  `phi_e = J^-1 * [0,0,-theta_e]^T` (same inverse Jacobian
  `x_drive_kinematics.c` implements) gives a per-wheel angular-velocity
  error; `F_lat = diag[1 + phi_e_i/phi_i]`. Applied as a per-wheel
  post-Jacobian scalar.
- **`F_ang`** (scalar): a **separate** experiment — repeated in-place
  rotations of known angle `theta1` → protractor-measure actual rotation
  `theta2` → `F_ang = theta1/theta2`. **Not** baked into the Jacobian like
  `F_lon`/`F_lat` (paper is explicit that `F_lat` alone wasn't sufficient for
  heading error, hence the second, separate correction) — applied as a
  scalar on the *commanded target angle* passed to the rotation primitive.

The factors now have one authoritative config and an opt-in advanced-motion
command path; `MoveS`/`RotS` deliberately bypass it because they remain
calibration measurement primitives. We extended the paper's single-speed procedure to a
**dual-speed** methodology (calibrate at a low and high speed/omega, linearly
interpolate between them, clamped — never extrapolated) because deadband/
feedforward nonlinearity near low speed means a single scalar doesn't
transfer well across the operating range. `tools/calibration_helper.html` is
a standalone (no serial connection) calculator implementing this — enter
trial measurements, it computes `F_lon`/`F_lat`/`F_ang` per speed group plus
the interpolation coefficients.

**Speed values in use this session**: straight-line trials at 0.15 and 0.45
m/s; rotation trials at 34.1 and 102.2 deg/s (chosen so each wheel's
contact-point speed matches the straight-line speeds: `wheel_speed_mps = arm
* omega_rad_s`, `arm = chassis_half_length*sin(wheel_angle) +
chassis_half_width*cos(wheel_angle)`).

## 3. The hardware harness

`src/harnesses/calibration_main.cpp` (`[env:calibration]` in
`platformio.ini`) runs `MoveS`/`RotS` on real hardware through the
production `Drivetrain` facade. Serial commands: `move <distance_m>
<heading_deg> <speed_mps>`, `rotate <angle_deg> <max_omega_deg_s>`,
`tol`/`angtol`/`accel`/`alpha`/`jerk` to adjust config live, `stop`/`brake`/
`enable` for safety. Streams CSV telemetry
(`millis,mode,pose_x_mm,pose_y_mm,heading_deg,remaining,vx,vy,omega`) at 20Hz
while a move is active.

**Important**: the harness's own completion summary (`odom_longitudinal_mm`
etc.) is *not* the paper's `xe`/`ye`/`theta_e` — it's odometry's own belief,
useful only as a sanity check. Real calibration numbers require physically
measuring where the robot ended up (tape measure / protractor) against the
`commanded_mm`/`commanded_deg` printed at move start.

## 4. `MoveS`/`RotS` are genuinely open-loop (recent, deliberate redesign)

Originally `MoveS`/`RotS` read live odometry each cycle to decide when to
decelerate/stop (a stopping-distance formula). This was changed: **neither
takes a pose/heading parameter anymore.**

```c
esp_err_t move_s_start(MoveS *, const MoveSConfig *, float distance_m, float heading_rad, float max_speed_mps);
esp_err_t move_s_update(MoveS *, float dt_s, MoveSOutput *);

esp_err_t rot_s_start(RotS *, const RotSConfig *, float angle_rad, float max_omega_rad_s);
esp_err_t rot_s_update(RotS *, float dt_s, RotSOutput *);
```

"Remaining distance/angle" is tracked via a new self-integrated field
(`planned_progress_m` / `planned_progress_rad`) — each module sums its own
commanded output over time (`planned_progress += commanded * dt_s`), never
touching real odometry during the move. `calibration_main.cpp` still reads
odometry every cycle, but purely for its own post-hoc reporting (captured at
`move`/`rotate` start, compared once the primitive reports complete) —
**never fed back into the primitives.**

**Why**: odometry is exactly what's being calibrated. If `MoveS`/`RotS` used
it to decide when to stop, real mechanical error (wheel slip, radius
mismatch) would get silently absorbed into the move's own duration instead
of showing up as measurable endpoint error — defeating the calibration.

Both `move_s.h` and `rot_s.h` have detailed header comments explaining this;
read them before changing either module's control logic.

## 5. Bugs found and fixed this session, in order

All three were found via real hardware telemetry, not code review alone —
each one only manifested once actually running on the robot.

### 5a. `RotS` runaway-in-one-direction (fixed)

`rot_s.c` picked its commanded direction from the sign of the *originally
requested angle*, frozen at `rot_s_start()`, not from the sign of the
*current remaining error*. Once real-world tracking overshot the target
(remaining flips sign), the module kept commanding the original direction
forever instead of reversing — observed as a commanded 90° rotation spinning
past 600°. Fixed with `copysignf(magnitude, remaining_rad)` instead of a
frozen `direction` variable. (This predates the open-loop redesign in §4,
which itself makes this specific failure mode structurally unreachable —
see the header comment in `rot_s.c`.)

### 5b. `odometry.c` world-frame Y-axis sign bug (fixed)

Unrelated to (a) — found via code review, not a new hardware run.
`odometry.c`'s Y-axis update had the wrong sign (`y_mm += -sine*forward +
cosine*lateral` instead of `y_mm += sine*forward - cosine*lateral`),
inconsistent with the documented conventions (`vy` positive = left, `omega`
positive = CCW). This didn't affect the open-loop `MoveS`/`RotS` (they don't
read odometry anymore per §4), but was still wrong for anything else
consuming odometry (e.g. this harness's own reporting). Fixed; three
existing tests that encoded the same wrong sign were corrected
(`test_rotates_body_delta_into_world_frame`, `test_integrates_delta_at_origin`,
`test_integrates_known_displacement` in `test/test_drivetrain_odometry*`).

### 5c. Deceleration overshoot / oscillation (partially understood, config workaround known)

After (a) and (b), `rotate 90 30` still didn't settle — telemetry showed
`remaining` oscillating between roughly +13° and -13° indefinitely (never
converging), later escalating into full runaway again in a second test.

**Contributing factor 1 (config mismatch, confirmed by the numbers)**: the
harness's default `jerk=1.0 rad/s^3` is too low relative to `alpha=1.5
rad/s^2` — ramping acceleration from 0 to the full `alpha` takes 1.5s, but
the actual stopping event (cruise speed to zero) only takes ~0.35s. So the
deceleration phase never reaches anywhere near its assumed rate, causing
systematic under-deceleration and overshoot far past the 2° tolerance band —
worked out to a predicted ~22° stopping distance vs. the intended ~5°,
matching the observed oscillation amplitude closely. **Workaround**: raise
`jerk` substantially via the harness's `jerk <value>` command before
rotating (e.g. `jerk 10` or higher) — this measurably improved the ramp-up
shape in the next test.

**Contributing factor 2 (real bug, fixed but not fully root-caused)**: even
after raising jerk, telemetry showed `omega` climbing to nearly 3x the
configured `max_omega_rad_s` ceiling and never recovering — objectively
impossible given `rot_s.c`'s explicit `fminf(max_omega_rad_s, ...)` clamp on
the *target*. Root cause: `speed_profile_update()`'s overshoot-clamp (in
`speed_profile.c`) only guarantees its output won't cross *the current
cycle's target* — that's a relative guarantee, not an absolute ceiling. Right
where deceleration begins, the target itself is dropping cycle-to-cycle;
something (suspected: dt_s irregularity across many un-printed internal
cycles, or an accumulation effect not yet isolated by hand-tracing) let the
output escape the relative check. **Fixed as defense-in-depth**: added an
explicit absolute clamp in both `rot_s.c` and `move_s.c` right after
`speed_profile_update()` returns, and wrote the clamped value back into
`profile.commanded_speed_mps` so a bad step can't leave phantom excess speed
baked into the next cycle's calculation. Verified as a real regression (not
guesswork) — reverted the fix, ran a new test with a synthetic irregular-dt_s
pattern
(`test_commanded_omega_never_exceeds_max_even_with_irregular_dt` /
`test_commanded_speed_never_exceeds_max_even_with_irregular_dt` in
`test/test_rot_s/` and `test/test_move_s/`), confirmed it fails without the
fix (`Expected TRUE Was FALSE`) and passes with it.

A separate, smaller fix went in alongside these: `calibration_main.cpp`'s
own `loop()` computes `dt_s` from wall-clock time with no upper bound (unlike
`drivetrain.c`'s `drivetrain_update()`, which already guards against an
oversized internal dt by coasting instead of integrating it). Added the same
style of clamp, capped to `DRIVETRAIN_CONFIG.max_control_dt_s`.

### 5d. Final resolution after the follow-up hardware trace (implemented)

The oscillation was not a physical calibration signal. It came from using an
instantaneous-deceleration stopping-distance formula with a jerk-limited
profile: once planned progress crossed the target, `RotS` changed the target
omega sign and repeatedly tried to correct itself.

`MoveS` and `RotS` now use a one-way profile: accelerate, cruise if needed,
then brake to a zero target and complete. A new
`speed_profile_predict_stopping_distance()` simulates the profile's current
speed and acceleration forward with a zero target, so braking starts early
enough for the configured jerk. Once braking starts, neither primitive may
reverse. Regression tests cover low-jerk runs for both modules and prove the
command cannot reverse. The calibration harness now defaults both profiles to
`jerk=10.0`; re-run the straight-line data set under this profile before
treating any `F_lon`/`F_lat` values as final.

The final robot will not use `MoveS`/`RotS`; they remain calibration-only
tools. The finalized integration plan is in `docs/TUNING_ROADMAP.md` §3:
final advanced motion APIs use an error-source-agnostic outer PID, apply
wheel calibration only in their own output path, and leave tape following
uncalibrated.

## 6. What's NOT yet done / open questions for you

1. **The exact multi-cycle mechanism behind 5c's overshoot has not been
   fully hand-derived.** Single-step algebra on the overshoot-clamp doesn't
   obviously reproduce it; suspect the issue only appears after several
   cycles of interaction between the changing target and the jerk-limited
   ramp, possibly worsened by irregular real-hardware `dt_s`. The regression
   test proves the *symptom* (output exceeding max) and that the fix closes
   it, but doesn't pin down *why* the relative overshoot-clamp fails
   specifically. If you want to actually understand (not just guard against)
   this, instrument `speed_profile_update()`'s internal state
   (`commanded_speed_mps`, `commanded_accel_mps2`) cycle-by-cycle under the
   same irregular-dt_s pattern the new tests use, and look for where the
   invariant "output only approaches target monotonically" first breaks.

2. **Not yet verified on hardware since the absolute-clamp fix.** Everything
   in §5 up through the clamp is unit-tested and builds clean
   (`pio test -e native`, `pio run -e calibration`), but the person running
   this asked me to prepare this handoff instead of waiting for the next
   hardware test — so as of this writing, `rotate 90 30` (with a raised
   `jerk`) has not been retried against the fixed firmware. That's the
   immediate next step once you or the user has hardware access.

3. **The shared jerk/accel profile is a hardware-validation item.** The
   calibration harness now defaults to jerk `10.0` and the characterized
   acceleration profile; verify those values on the next physical trial and
   refine them if traction testing requires it.

4. **Calibration enablement remains a hardware-validation gate.** The measured
   values live in `src/config/drivetrain/move_calibration_config.c` and the
   advanced-motion path is wired through
   `drivetrain_set_advanced_body_velocity()`, but `.enabled` remains false
   until after-calibration validation succeeds. See
   `docs/TUNING_ROADMAP.md`'s “Advanced motion API caller guide” for the exact
   control-loop contract and scope.

## 7. Quick reference: build/test commands

```
pio test -e native                          # 97 unit tests, no hardware
pio run -e calibration                       # build the calibration harness
pio run -e calibration -t upload             # flash it
pio device monitor -e calibration            # serial console
```

For a browser-based calibration workflow, open
`tools/calibration_dashboard.html` in desktop Chrome or Edge, connect to the
calibration board at 115200 baud, and use its guarded `move`, `rotate`, `arc`,
`movel`, and `movep` controls. It records operator-entered physical measurements and exports
CSV; use `tools/calibration_helper.html` to compute and export the final
calibration curves.

Current native test count: 97 (all passing as of this handoff). Git branch:
`tuning`, pushed to `origin/tuning` as of commit `7be4688` — the fixes in §5b
and §5c above are **uncommitted, local-only** as of this handoff (see
`git status`); commit them before considering this state "saved."
