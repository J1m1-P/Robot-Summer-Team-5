/* MoveS/RotS/MoveC calibration harness -- see platformio.ini's [env:calibration]. */
#include <Arduino.h>

#include <math.h>
#include <esp_timer.h>

#include "config/drivetrain/drivetrain_config.h"
#include "control/drivetrain/drivetrain.h"
#include "control/drivetrain/drivetrain_odometry_source.h"
#include "control/drivetrain/motion_estimate_adapter.h"
#include "control/drivetrain/move_c.h"
#include "control/drivetrain/move_l.h"
#include "control/drivetrain/move_p.h"
#include "control/drivetrain/move_r.h"
#include "control/drivetrain/move_s.h"
#include "control/drivetrain/rot_s.h"

// -----------------------------------------------------------------------------
// Real-time tuning only
// This entire file is an isolated hardware harness for running MoveS/RotS on
// the real robot and reading back odometry, feeding the calibration paper's
// (docs/TUNING_ROADMAP.md §3/§5d) F_lon/F_lat/F_ang procedure. Not the
// production entry point; kept entirely separate from main.cpp.
// -----------------------------------------------------------------------------
//
// Wires MoveS/RotS (unit-tested but never run on hardware before this) into
// the production Drivetrain facade -- same drivetrain_set_body_velocity()/
// drivetrain_update() cycle main.cpp would use -- plus
// drivetrain_odometry_source, which continuously turns live encoder counts
// into a DrivetrainPose via drivetrain_odometry_update(). MoveS/RotS
// themselves are genuinely open-loop -- they never read this pose. Progress
// is tracked purely by integrating each module's own commanded output over
// time (see move_s.h/rot_s.h), so nothing about a move's duration or
// deceleration point depends on real position/heading feedback -- odometry
// only runs here for this harness's own reporting, captured separately at
// "move"/"rotate" start (see move_start_pose/rotate_start_heading_rad
// below) purely so the completion summary has something to compare against.
//
// IMPORTANT -- what this harness can and can't tell you:
//   The completion summary reports the odometry's own estimate of how far
//   the robot moved/turned, compared to what was commanded. Because MoveS/
//   RotS no longer consult odometry to decide when to stop, this is no
//   longer a near-zero-by-construction number -- real mechanical error
//   (wheel slip, radius mismatch, per-wheel response differences) now shows
//   up here too. It is still NOT a full substitute for the paper's
//   x_e/y_e/theta_e, though: odometry only measures wheel *rotation*, not
//   whether the wheel actually gripped the floor, so wheel slip specifically
//   is invisible to it either way. Getting real x_e/y_e/theta_e requires
//   physically measuring where the robot actually ended up and comparing
//   that against the commanded distance/angle printed at "move"/"rotate"
//   start:
//     - MoveS trial (derives F_lon, F_lat -- paper eq. 18-20): BEFORE
//       running "move", mark both the robot's start position and its
//       heading (a line off the front). After it completes, tape-measure
//       x_e (longitudinal) and y_e (lateral) endpoint error against
//       commanded_mm, AND separately protractor-measure theta_e -- the
//       robot's own heading drift from the marked starting heading. theta_e
//       is a direct reading, not derived from x_e/y_e.
//     - RotS trial (derives F_ang -- paper eq. 21): a SEPARATE experiment
//       from the above, not reusing its data. Protractor-measure the actual
//       rotation against commanded_deg.
//   Compute F_lon/F_lat/F_ang by hand per the paper from those measurements.
//   Note F_ang is architecturally different from F_lon/F_lat: the paper
//   bakes F_lon/F_lat into the wheel Jacobian (eq. 22-23), but adds F_ang
//   separately as a scalar on the *commanded target angle* (not on angular
//   speed, not via the Jacobian) because F_lat alone didn't fully correct
//   heading error -- see rot_s.h's header comment for where that hooks in.
//
// Serial commands (newline-terminated), typeable any time:
//   show                        -- print current MoveS/RotS config and
//                                whether the drivetrain is enabled.
//   tol <meters>                -- MoveSConfig.distance_tolerance_m.
//   angtol <deg>                -- RotSConfig.angle_tolerance_rad.
//   accel <mps2>                -- MoveSConfig.max_accel_mps2.
//   alpha <deg_s2>              -- RotSConfig.max_alpha_rad_s2.
//   jerk <value>                -- shared max_jerk_mps3, applied to both
//                                MoveS's and RotS's speed profiles (the
//                                module's math is unit-agnostic, same as
//                                RotS reusing it directly for angular ramps).
//   move <distance_m> <heading_deg> <speed_mps>
//                                -- starts a MoveS run. heading_deg is in
//                                the body frame (0 = forward, positive =
//                                toward +vy/strafe-left), not a world
//                                heading -- matches move_s_start()'s
//                                heading_rad convention.
//   rotate <angle_deg> <max_omega_deg_s>
//                                -- starts a RotS run. angle_deg is signed
//                                (positive = counterclockwise).
//   stop                        -- aborts an active move/rotation immediately
//                                via drivetrain_stop() (controlled zero
//                                target, not a hardware brake).
//   brake                       -- engages the hardware brake and disables
//                                the drivetrain; use "enable" to resume.
//   enable                      -- re-enables the drivetrain after a brake.
//   zero                        -- re-anchors the world-frame pose to the
//                                robot's current physical position (e.g. a
//                                known tape datum): 0,0,0. Rejected while a
//                                motion is active. Every subsequent movep/
//                                rotl target is then expressed relative to
//                                this point, not whatever the old origin was.
//
// All of the above live only in RAM here -- they reset to this file's
// compiled-in defaults on reboot. jerk/accel/alpha/tol here mirror the
// values locked in via tuning_main.cpp's own jerk/accel fields where
// applicable (see docs/TUNING_ROADMAP.md §5c-note) -- MoveSConfig/RotSConfig
// don't have a permanent home in src/config/ yet since nothing outside this
// harness consumes them.
//
// Streams one CSV line at kTelemetryPeriodMs while a move/rotation is
// active -- "millis,mode,pose_x_mm,pose_y_mm,heading_deg,remaining,vx,vy,omega"
// (mode is "move" or "rotate"; remaining is meters for move, degrees for
// rotate).

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

constexpr float kDefaultDistanceToleranceM = 0.01f;
constexpr float kDefaultAngleToleranceDeg = 2.0f;
// Shared calibration-profile default.  The previous 1.0 value took a full
// 1.5 s to reverse RotS's configured 1.5 rad/s² acceleration, making a
// short rotation unable to brake cleanly.  MoveS and RotS both use the
// one-way, jerk-aware braking scheme, so their calibration trials should
// use the same practical profile default.
constexpr float kDefaultMaxJerkMps3 = 10.0f;
constexpr float kDefaultMaxAccelMps2 = 2.5f;
// Not yet calibrated -- matches TAPE_FOLLOWER_CONFIG's heading config,
// same placeholder RotSConfig's own header comment documents.
constexpr float kDefaultMaxAlphaDegS2 = 1.5f * kRadToDeg;

constexpr unsigned long kTelemetryPeriodMs = 50; // 20 Hz

enum class CalMode { kIdle, kMove, kRotate, kArc, kMoveL, kMoveP, kMoveR };

Drivetrain drivetrain = {};
bool drivetrain_ready = false;

DrivetrainOdometrySourceConfig odom_source_config = {};
DrivetrainOdometrySource odom_source = {};
DrivetrainOdometry odometry = {};

MoveSConfig move_s_config = {
    .speed_profile = {.max_jerk_mps3 = kDefaultMaxJerkMps3},
    .distance_tolerance_m = kDefaultDistanceToleranceM,
    .max_accel_mps2 = kDefaultMaxAccelMps2,
};
RotSConfig rot_s_config = {
    .speed_profile = {.max_jerk_mps3 = kDefaultMaxJerkMps3},
    .angle_tolerance_rad = kDefaultAngleToleranceDeg * kDegToRad,
    .max_alpha_rad_s2 = kDefaultMaxAlphaDegS2 * kDegToRad,
};

MoveCConfig move_c_config = {
    .off_tape_motion = {
        .controller = {.proportional_gain = 1.0f, .integral_gain = 0.0f,
                       .derivative_gain = 0.0f, .integral_limit = 0.2f,
                       .correction_min = -0.25f, .correction_max = 0.25f},
        .controller_dt_max_s = 0.05f},
    .speed_profile = {.max_jerk_mps3 = kDefaultMaxJerkMps3},
    .heading_controller = {.proportional_gain = 1.0f, .integral_gain = 0.0f,
                            .derivative_gain = 0.0f, .integral_limit = 0.5f,
                            .correction_min = -1.0f, .correction_max = 1.0f},
    .arc_length_tolerance_m = 0.01f,
    .radial_tolerance_m = 0.01f,
    .heading_tolerance_rad = 2.0f * kDegToRad,
    .max_accel_mps2 = kDefaultMaxAccelMps2,
    .max_alpha_rad_s2 = kDefaultMaxAlphaDegS2 * kDegToRad,
    .max_omega_rad_s = 1.0f,
};

MoveLConfig move_l_config = {
    .off_tape_motion = {
        .controller = {.proportional_gain = 1.0f, .integral_gain = 0.0f,
                       .derivative_gain = 0.0f, .integral_limit = 0.2f,
                       .correction_min = -0.25f, .correction_max = 0.25f},
        .controller_dt_max_s = 0.05f},
    .speed_profile = {.max_jerk_mps3 = kDefaultMaxJerkMps3},
    .distance_tolerance_m = 0.01f,
    .max_accel_mps2 = kDefaultMaxAccelMps2,
};

MovePConfig move_p_config = {
    .off_tape_motion = move_l_config.off_tape_motion,
    .speed_profile = {.max_jerk_mps3 = kDefaultMaxJerkMps3},
    .heading_controller = {.proportional_gain = 1.0f, .integral_gain = 0.0f,
                            .derivative_gain = 0.0f, .integral_limit = 0.5f,
                            .correction_min = -1.0f, .correction_max = 1.0f},
    .distance_tolerance_m = 0.01f,
    .heading_tolerance_rad = 2.0f * kDegToRad,
    .max_accel_mps2 = kDefaultMaxAccelMps2,
    .max_alpha_rad_s2 = kDefaultMaxAlphaDegS2 * kDegToRad,
    .max_omega_rad_s = 1.0f,
};

MoveRConfig move_r_config = {
    .heading_controller = {.proportional_gain = 2.0f, .integral_gain = 0.0f,
                            .derivative_gain = 0.05f, .integral_limit = 0.5f,
                            .correction_min = -1.0f, .correction_max = 1.0f},
    .speed_profile = {.max_jerk_mps3 = kDefaultMaxJerkMps3},
    .heading_tolerance_rad = 2.0f * kDegToRad,
    .max_alpha_rad_s2 = kDefaultMaxAlphaDegS2 * kDegToRad,
    .max_omega_rad_s = 1.0f,
    .controller_dt_max_s = 0.05f,
};

MoveS move_s_state = {};
RotS rot_s_state = {};
MoveC move_c_state = {};
MoveL move_l_state = {};
MoveP move_p_state = {};
MoveR move_r_state = {};
MotionEstimate move_l_start_estimate = {};
float move_l_direction_x = 0.0f;
float move_l_direction_y = 0.0f;
CalMode cal_mode = CalMode::kIdle;

// MoveS/RotS no longer track a start pose/heading themselves (open-loop by
// design -- see move_s.h/rot_s.h). Captured here purely for this harness's
// own odometry-only sanity-check prints, never fed back into the primitives.
DrivetrainPose move_start_pose = {};
float move_world_direction_x = 0.0f;
float move_world_direction_y = 0.0f;
float rotate_start_heading_rad = 0.0f;

int64_t last_update_us = 0;
unsigned long last_print_ms = 0;

void print_usage() {
    Serial.println("# usage: show | tol <m> | angtol <deg> | accel <mps2> | alpha <deg_s2> | jerk <value> | move <distance_m> <heading_deg> <speed_mps> | rotate <angle_deg> <max_omega_deg_s> | rotl <target_heading_deg> | arc <radius_m> <angle_deg> <speed_mps> | movel <distance_m> <heading_deg> <speed_mps> | movep <target_x_m> <target_y_m> <final_heading_deg> <speed_mps> | stop | brake | enable | zero");
}

void print_config() {
    Serial.print("# move: tol_m=");
    Serial.print(move_s_config.distance_tolerance_m, 4);
    Serial.print(" accel_mps2=");
    Serial.print(move_s_config.max_accel_mps2, 4);
    Serial.print(" jerk_mps3=");
    Serial.println(move_s_config.speed_profile.max_jerk_mps3, 4);

    Serial.print("# rotate: angtol_deg=");
    Serial.print(rot_s_config.angle_tolerance_rad * kRadToDeg, 4);
    Serial.print(" alpha_deg_s2=");
    Serial.print(rot_s_config.max_alpha_rad_s2 * kRadToDeg, 4);
    Serial.print(" jerk_mps3=");
    Serial.println(rot_s_config.speed_profile.max_jerk_mps3, 4);

    Serial.print("# drivetrain enabled=");
    Serial.println(drivetrain_ready ? "1" : "0");
}

// Commands a controlled zero-velocity target and returns to idle -- used by
// both "stop" and internally once a move/rotation completes.
void stop_motion() {
    cal_mode = CalMode::kIdle;
    if (drivetrain_ready) {
        const esp_err_t error = drivetrain_stop(&drivetrain);
        if (error != ESP_OK) {
            Serial.print("# stop failed; braking: ");
            Serial.println(esp_err_to_name(error));
            drivetrain_brake(&drivetrain);
            drivetrain_ready = false;
        }
    }
    Serial.println("# stopped");
}

bool set_velocity_or_brake(const DrivetrainBodyVelocity &velocity) {
    const esp_err_t error = drivetrain_set_body_velocity(
        &drivetrain, velocity.vx, velocity.vy, velocity.omega);
    if (error == ESP_OK) return true;
    Serial.print("# drivetrain command rejected; braking: ");
    Serial.println(esp_err_to_name(error));
    drivetrain_brake(&drivetrain);
    drivetrain_ready = false;
    cal_mode = CalMode::kIdle;
    return false;
}

void brake_drivetrain() {
    cal_mode = CalMode::kIdle;
    const esp_err_t error = drivetrain_brake(&drivetrain);
    drivetrain_ready = false;
    if (error != ESP_OK) {
        Serial.print("# brake failed: ");
        Serial.println(esp_err_to_name(error));
        return;
    }
    Serial.println("# hardware brake engaged; use 'enable' to resume");
}

void enable_drivetrain() {
    if (drivetrain_ready) {
        Serial.println("# already enabled");
        return;
    }
    const esp_err_t error = drivetrain_enable(&drivetrain);
    if (error != ESP_OK) {
        Serial.print("# enable failed: ");
        Serial.println(esp_err_to_name(error));
        return;
    }
    drivetrain_ready = true;
    Serial.println("# drivetrain enabled");
}

bool motion_active() {
    return cal_mode != CalMode::kIdle;
}

// Re-anchors the world-frame pose to wherever the robot physically is right
// now -- e.g. at a known tape datum -- so every subsequent absolute-target
// call (movep/rotl) is expressed in a small local map relative to this point
// instead of one long-lived global map accumulating drift across the whole
// course. Both resets are required together: drivetrain_odometry_reset()
// zeros the pose itself, while drivetrain_odometry_source_reset() clears the
// previous-count baseline the source diffs against -- without it, the next
// update would compute a delta against stale pre-reset counts and instantly
// reintroduce whatever position the robot was actually at.
void zero_pose() {
    if (motion_active()) {
        Serial.println("# zero rejected: another motion is active; use 'stop' first");
        return;
    }
    drivetrain_odometry_source_reset(&odom_source);
    drivetrain_odometry_reset(&odometry);
    Serial.println("# ZERO pose reset to origin (0, 0, 0)");
}

void start_move(float distance_m, float heading_deg, float speed_mps) {
    if (!drivetrain_ready) {
        Serial.println("# move: drivetrain is not enabled");
        return;
    }
    if (motion_active()) {
        Serial.println("# move rejected: another motion is active; use 'stop' first");
        return;
    }
    const esp_err_t error = move_s_start(
        &move_s_state, &move_s_config, distance_m, heading_deg * kDegToRad, speed_mps);
    if (error != ESP_OK) {
        Serial.print("# move start failed: ");
        Serial.println(esp_err_to_name(error));
        return;
    }
    // MoveS itself no longer tracks a start pose/world direction (open-loop
    // by design -- see move_s.h). Captured here instead, purely for the
    // odometry-only sanity check print_move_complete() prints afterward.
    move_start_pose = odometry.pose;
    // MoveS heading is expressed in the body frame: positive points toward
    // body +y (left), which is a CCW turn in the world
    // heading.  Convert it with addition when projecting telemetry into
    // the world frame.
    const float world_heading_rad = odometry.pose.heading_rad + heading_deg * kDegToRad;
    move_world_direction_x = cosf(world_heading_rad);
    move_world_direction_y = sinf(world_heading_rad);
    cal_mode = CalMode::kMove;
    Serial.print("# MOVE START commanded_distance_m=");
    Serial.print(distance_m, 4);
    Serial.print(" heading_deg=");
    Serial.print(heading_deg, 2);
    Serial.print(" speed_mps=");
    Serial.println(speed_mps, 4);
}

void start_rotate(float angle_deg, float max_omega_deg_s) {
    if (!drivetrain_ready) {
        Serial.println("# rotate: drivetrain is not enabled");
        return;
    }
    if (motion_active()) {
        Serial.println("# rotate rejected: another motion is active; use 'stop' first");
        return;
    }
    const esp_err_t error = rot_s_start(
        &rot_s_state, &rot_s_config, angle_deg * kDegToRad, max_omega_deg_s * kDegToRad);
    if (error != ESP_OK) {
        Serial.print("# rotate start failed: ");
        Serial.println(esp_err_to_name(error));
        return;
    }
    // RotS itself no longer tracks a start heading (open-loop by design --
    // see rot_s.h). Captured here instead, purely for print_rotate_complete().
    rotate_start_heading_rad = odometry.pose.heading_rad;
    cal_mode = CalMode::kRotate;
    Serial.print("# ROTATE START commanded_angle_deg=");
    Serial.print(angle_deg, 2);
    Serial.print(" max_omega_deg_s=");
    Serial.println(max_omega_deg_s, 2);
}

void start_arc(float radius_m, float angle_deg, float speed_mps) {
    if (!drivetrain_ready) {
        Serial.println("# arc: drivetrain is not enabled");
        return;
    }
    if (motion_active()) {
        Serial.println("# arc rejected: another motion is active; use 'stop' first");
        return;
    }
    MotionEstimate start = {};
    if (motion_estimate_from_drivetrain_pose(&odometry.pose, true, &start) != ESP_OK) {
        Serial.println("# arc start failed: invalid odometry estimate");
        return;
    }
    const esp_err_t error = move_c_start(&move_c_state, &move_c_config, &start,
                                         radius_m, angle_deg * kDegToRad, speed_mps);
    if (error != ESP_OK) {
        Serial.print("# arc start failed: ");
        Serial.println(esp_err_to_name(error));
        return;
    }
    cal_mode = CalMode::kArc;
    Serial.print("# ARC START radius_m=");
    Serial.print(radius_m, 4);
    Serial.print(" angle_deg=");
    Serial.print(angle_deg, 2);
    Serial.print(" speed_mps=");
    Serial.println(speed_mps, 4);
}

bool current_motion_estimate(MotionEstimate *estimate) {
    return motion_estimate_from_drivetrain_pose(&odometry.pose, true, estimate) == ESP_OK;
}

void start_movel(float distance_m, float heading_deg, float speed_mps) {
    if (!drivetrain_ready) { Serial.println("# movel: drivetrain is not enabled"); return; }
    if (motion_active()) {
        Serial.println("# movel rejected: another motion is active; use 'stop' first");
        return;
    }
    const esp_err_t error = move_l_start(&move_l_state, &move_l_config,
                                         distance_m, heading_deg * kDegToRad, speed_mps);
    if (error != ESP_OK) { Serial.print("# movel start failed: "); Serial.println(esp_err_to_name(error)); return; }
    MotionEstimate start = {};
    if (!current_motion_estimate(&start)) { Serial.println("# movel start failed: invalid estimate"); return; }
    // MoveL uses the same body-frame heading convention as MoveS: positive
    // angles point toward body +y/left, so they add to CCW world yaw.
    const float world_heading = start.heading_rad + heading_deg * kDegToRad;
    move_l_start_estimate = start;
    move_l_direction_x = cosf(world_heading);
    move_l_direction_y = sinf(world_heading);
    cal_mode = CalMode::kMoveL;
    Serial.print("# MOVEL START distance_m="); Serial.print(distance_m, 4);
    Serial.print(" heading_deg="); Serial.print(heading_deg, 2);
    Serial.print(" speed_mps="); Serial.println(speed_mps, 4);
}

void start_movep(float target_x_m, float target_y_m, float final_heading_deg, float speed_mps) {
    if (!drivetrain_ready) { Serial.println("# movep: drivetrain is not enabled"); return; }
    if (cal_mode != CalMode::kIdle) {
        Serial.println("# movep rejected: another motion is active; use 'stop' first");
        return;
    }
    MotionEstimate start = {};
    if (!current_motion_estimate(&start)) { Serial.println("# movep start failed: invalid estimate"); return; }
    if (hypotf(target_x_m - start.x_m, target_y_m - start.y_m) <=
        move_p_config.distance_tolerance_m) {
        Serial.println("# movep start failed: target is already within tolerance");
        return;
    }
    const esp_err_t error = move_p_start(&move_p_state, &move_p_config, &start,
                                         target_x_m, target_y_m,
                                         final_heading_deg * kDegToRad, speed_mps);
    if (error != ESP_OK) { Serial.print("# movep start failed: "); Serial.println(esp_err_to_name(error)); return; }
    cal_mode = CalMode::kMoveP;
    Serial.print("# MOVEP START target_x_m="); Serial.print(target_x_m, 4);
    Serial.print(" target_y_m="); Serial.print(target_y_m, 4);
    Serial.print(" final_heading_deg="); Serial.print(final_heading_deg, 2);
    Serial.print(" speed_mps="); Serial.println(speed_mps, 4);
}

void start_mover(float target_heading_deg) {
    if (!drivetrain_ready) { Serial.println("# rotl: drivetrain is not enabled"); return; }
    if (cal_mode != CalMode::kIdle) {
        Serial.println("# rotl rejected: another motion is active; use 'stop' first");
        return;
    }
    MotionEstimate start = {};
    if (!current_motion_estimate(&start)) {
        Serial.println("# rotl start failed: invalid estimate");
        return;
    }
    const esp_err_t error = move_r_start(
        &move_r_state, &move_r_config, &start, target_heading_deg * kDegToRad);
    if (error != ESP_OK) {
        Serial.print("# rotl start failed: ");
        Serial.println(esp_err_to_name(error));
        return;
    }
    cal_mode = CalMode::kMoveR;
    Serial.print("# ROTL START target_heading_deg=");
    Serial.println(target_heading_deg, 2);
}

// Prints the odometry-only sanity check described in the file header --
// NOT the paper's x_e/y_e/theta_e. Uses the pose/direction captured at
// start_move() above, since MoveS itself no longer tracks either.
void print_move_complete() {
    const float total_dx_mm = odometry.pose.x_mm - move_start_pose.x_mm;
    const float total_dy_mm = odometry.pose.y_mm - move_start_pose.y_mm;
    const float longitudinal_mm =
        total_dx_mm * move_world_direction_x +
        total_dy_mm * move_world_direction_y;
    const float lateral_mm =
        -total_dx_mm * move_world_direction_y +
        total_dy_mm * move_world_direction_x;

    Serial.print("# MOVE COMPLETE commanded_mm=");
    Serial.print(move_s_state.distance_m * 1000.0f, 1);
    Serial.print(" odom_longitudinal_mm=");
    Serial.print(longitudinal_mm, 1);
    Serial.print(" odom_lateral_mm=");
    Serial.println(lateral_mm, 1);
    Serial.println("# odometry-only sanity check, not x_e/y_e/theta_e -- "
                    "physically measure the real endpoint now: tape-measure "
                    "x_e (longitudinal) and y_e (lateral) against "
                    "commanded_mm for F_lon, AND protractor-measure theta_e "
                    "(heading drift vs. the heading you marked before "
                    "'move') for F_lat -- theta_e is a direct reading, not "
                    "derived from x_e/y_e.");
}

// Uses the heading captured at start_rotate() above, since RotS itself no
// longer tracks it.
void print_rotate_complete() {
    const float odom_delta_deg =
        (odometry.pose.heading_rad - rotate_start_heading_rad) * kRadToDeg;

    Serial.print("# ROTATE COMPLETE commanded_deg=");
    Serial.print(rot_s_state.angle_rad * kRadToDeg, 2);
    Serial.print(" odom_delta_deg=");
    Serial.println(odom_delta_deg, 2);
    Serial.println("# odometry-only sanity check, not theta_e -- physically "
                    "measure the real rotation now (protractor) and use "
                    "that against commanded_deg for the paper's F_ang.");
}

void handle_serial_command() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line == "show") { print_config(); return; }
    if (line == "stop") { stop_motion(); return; }
    if (line == "brake") { brake_drivetrain(); return; }
    if (line == "enable") { enable_drivetrain(); return; }
    if (line == "zero") { zero_pose(); return; }

    const int firstSpace = line.indexOf(' ');
    if (firstSpace < 0) { print_usage(); return; }

    const String key = line.substring(0, firstSpace);
    String rest = line.substring(firstSpace + 1);
    rest.trim();

    if (key == "move") {
        String tokens[3];
        int count = 0;
        String remaining = rest;
        while (count < 3 && remaining.length() > 0) {
            const int space = remaining.indexOf(' ');
            if (space < 0) {
                tokens[count++] = remaining;
                remaining = "";
            } else {
                tokens[count++] = remaining.substring(0, space);
                remaining = remaining.substring(space + 1);
                remaining.trim();
            }
        }
        if (count < 3) {
            Serial.println("# usage: move <distance_m> <heading_deg> <speed_mps>");
            return;
        }
        start_move(tokens[0].toFloat(), tokens[1].toFloat(), tokens[2].toFloat());
        return;
    }

    if (key == "rotate") {
        const int secondSpace = rest.indexOf(' ');
        if (secondSpace < 0) {
            Serial.println("# usage: rotate <angle_deg> <max_omega_deg_s>");
            return;
        }
        const float angle_deg = rest.substring(0, secondSpace).toFloat();
        const float max_omega_deg_s = rest.substring(secondSpace + 1).toFloat();
        start_rotate(angle_deg, max_omega_deg_s);
        return;
    }

    if (key == "rotl") {
        start_mover(rest.toFloat());
        return;
    }

    if (key == "arc") {
        String tokens[3];
        int count = 0;
        String remaining = rest;
        while (count < 3 && remaining.length() > 0) {
            const int space = remaining.indexOf(' ');
            if (space < 0) {
                tokens[count++] = remaining;
                remaining = "";
            } else {
                tokens[count++] = remaining.substring(0, space);
                remaining = remaining.substring(space + 1);
                remaining.trim();
            }
        }
        if (count < 3) {
            Serial.println("# usage: arc <radius_m> <angle_deg> <speed_mps>");
            return;
        }
        start_arc(tokens[0].toFloat(), tokens[1].toFloat(), tokens[2].toFloat());
        return;
    }

    if (key == "movel" || key == "movep") {
        String tokens[4];
        int count = 0;
        String remaining = rest;
        while (count < 4 && remaining.length() > 0) {
            const int space = remaining.indexOf(' ');
            if (space < 0) { tokens[count++] = remaining; remaining = ""; }
            else { tokens[count++] = remaining.substring(0, space); remaining = remaining.substring(space + 1); remaining.trim(); }
        }
        if ((key == "movel" && count < 3) || (key == "movep" && count < 4)) {
            Serial.println(key == "movel"
                ? "# usage: movel <distance_m> <heading_deg> <speed_mps>"
                : "# usage: movep <target_x_m> <target_y_m> <final_heading_deg> <speed_mps>");
            return;
        }
        if (key == "movel") start_movel(tokens[0].toFloat(), tokens[1].toFloat(), tokens[2].toFloat());
        else start_movep(tokens[0].toFloat(), tokens[1].toFloat(), tokens[2].toFloat(), tokens[3].toFloat());
        return;
    }

    const float value = rest.toFloat();

    if (key == "tol") {
        move_s_config.distance_tolerance_m = value;
        Serial.print("# distance_tolerance_m = ");
        Serial.println(move_s_config.distance_tolerance_m, 4);
    } else if (key == "angtol") {
        rot_s_config.angle_tolerance_rad = value * kDegToRad;
        Serial.print("# angle_tolerance_deg = ");
        Serial.println(value, 4);
    } else if (key == "accel") {
        move_s_config.max_accel_mps2 = value;
        Serial.print("# max_accel_mps2 = ");
        Serial.println(move_s_config.max_accel_mps2, 4);
    } else if (key == "alpha") {
        rot_s_config.max_alpha_rad_s2 = value * kDegToRad;
        Serial.print("# max_alpha_deg_s2 = ");
        Serial.println(value, 4);
    } else if (key == "jerk") {
        move_s_config.speed_profile.max_jerk_mps3 = value;
        rot_s_config.speed_profile.max_jerk_mps3 = value;
        Serial.print("# max_jerk_mps3 = ");
        Serial.println(value, 4);
    } else {
        print_usage();
    }
}

void print_telemetry(unsigned long now_ms, float remaining, DrivetrainBodyVelocity velocity) {
    Serial.print(now_ms);
    Serial.print(',');
    const char *mode_name = cal_mode == CalMode::kMove ? "move" :
        cal_mode == CalMode::kRotate ? "rotate" :
        cal_mode == CalMode::kArc ? "arc" :
        cal_mode == CalMode::kMoveL ? "movel" :
        cal_mode == CalMode::kMoveP ? "movep" : "rotl";
    Serial.print(mode_name);
    Serial.print(',');
    Serial.print(odometry.pose.x_mm, 2);
    Serial.print(',');
    Serial.print(odometry.pose.y_mm, 2);
    Serial.print(',');
    Serial.print(odometry.pose.heading_rad * kRadToDeg, 2);
    Serial.print(',');
    Serial.print(remaining, 4);
    Serial.print(',');
    Serial.print(velocity.vx, 4);
    Serial.print(',');
    Serial.print(velocity.vy, 4);
    Serial.print(',');
    Serial.println(velocity.omega, 4);
}

void service_move(unsigned long now_ms, float dt_s, bool should_print) {
    MoveSOutput output = {};
    const esp_err_t error = move_s_update(&move_s_state, dt_s, &output);
    if (error != ESP_OK || !output.motion_valid) {
        Serial.println("# move update failed; stopping");
        stop_motion();
        return;
    }

    if (!set_velocity_or_brake(output.requested_velocity)) return;

    if (should_print) print_telemetry(now_ms, output.remaining_distance_m, output.requested_velocity);

    if (output.status == MOVE_S_COMPLETE) {
        print_move_complete();
        stop_motion();
    }
}

void service_rotate(unsigned long now_ms, float dt_s, bool should_print) {
    RotSOutput output = {};
    const esp_err_t error = rot_s_update(&rot_s_state, dt_s, &output);
    if (error != ESP_OK || !output.motion_valid) {
        Serial.println("# rotate update failed; stopping");
        stop_motion();
        return;
    }

    if (!set_velocity_or_brake(output.requested_velocity)) return;

    if (should_print) {
        print_telemetry(now_ms, output.remaining_angle_rad * kRadToDeg, output.requested_velocity);
    }

    if (output.status == ROT_S_COMPLETE) {
        print_rotate_complete();
        stop_motion();
    }
}

void service_arc(unsigned long now_ms, float dt_s, bool should_print) {
    MotionEstimate estimate = {};
    if (motion_estimate_from_drivetrain_pose(&odometry.pose, true, &estimate) != ESP_OK) {
        Serial.println("# arc estimate invalid; stopping");
        stop_motion();
        return;
    }
    MoveCOutput output = {};
    const esp_err_t error = move_c_update(&move_c_state, &estimate, dt_s, &output);
    if (error != ESP_OK || !output.motion_valid) {
        Serial.println("# arc update failed; stopping");
        stop_motion();
        return;
    }
    const esp_err_t command_error = drivetrain_set_advanced_body_velocity(
        &drivetrain, output.requested_velocity.vx, output.requested_velocity.vy,
        output.requested_velocity.omega);
    if (command_error != ESP_OK) {
        Serial.print("# advanced arc command rejected; braking: ");
        Serial.println(esp_err_to_name(command_error));
        drivetrain_brake(&drivetrain);
        drivetrain_ready = false;
        cal_mode = CalMode::kIdle;
        return;
    }
    if (should_print) print_telemetry(now_ms, output.remaining_arc_m, output.requested_velocity);
    if (output.status == MOVE_C_COMPLETE) {
        Serial.println("# ARC COMPLETE");
        stop_motion();
    }
}

bool apply_advanced_output(const DrivetrainBodyVelocity &velocity) {
    const esp_err_t error = drivetrain_set_advanced_body_velocity(
        &drivetrain, velocity.vx, velocity.vy, velocity.omega);
    if (error == ESP_OK) return true;
    Serial.print("# advanced command rejected; braking: ");
    Serial.println(esp_err_to_name(error));
    drivetrain_brake(&drivetrain);
    drivetrain_ready = false;
    cal_mode = CalMode::kIdle;
    return false;
}

void service_movel(unsigned long now_ms, float dt_s, bool should_print) {
    MotionEstimate estimate = {};
    if (!current_motion_estimate(&estimate)) { Serial.println("# movel estimate invalid; stopping"); stop_motion(); return; }
    const float dx = estimate.x_m - move_l_start_estimate.x_m;
    const float dy = estimate.y_m - move_l_start_estimate.y_m;
    MoveLInput input = {
        .along_track_progress_m = dx * move_l_direction_x + dy * move_l_direction_y,
        // Match path_planner_line_feedback(): world path-left is
        // (direction_y, -direction_x), and the controller receives the
        // negated right displacement so its signed lateral command points
        // back toward the path under the drivetrain's +body-y/left
        // convention.
        .cross_track_error_m = dx * move_l_direction_y -
                               dy * move_l_direction_x,
        .valid = true,
    };
    MoveLOutput output = {};
    const esp_err_t error = move_l_update(&move_l_state, &input, dt_s, &output);
    if (error != ESP_OK || !output.motion_valid) { Serial.println("# movel update failed; stopping"); stop_motion(); return; }
    if (!apply_advanced_output(output.requested_velocity)) return;
    if (should_print) print_telemetry(now_ms, output.remaining_distance_m, output.requested_velocity);
    if (output.status == MOVE_L_COMPLETE) { Serial.println("# MOVEL COMPLETE"); stop_motion(); }
}

void service_movep(unsigned long now_ms, float dt_s, bool should_print) {
    MotionEstimate estimate = {};
    if (!current_motion_estimate(&estimate)) { Serial.println("# movep estimate invalid; stopping"); stop_motion(); return; }
    MovePOutput output = {};
    const esp_err_t error = move_p_update(&move_p_state, &estimate, dt_s, &output);
    if (error != ESP_OK || !output.motion_valid) { Serial.println("# movep update failed; stopping"); stop_motion(); return; }
    if (!apply_advanced_output(output.requested_velocity)) return;
    if (should_print) print_telemetry(now_ms, output.distance_to_goal_m, output.requested_velocity);
    if (output.status == MOVE_P_COMPLETE) { Serial.println("# MOVEP COMPLETE"); stop_motion(); }
}

void service_mover(unsigned long now_ms, float dt_s, bool should_print) {
    MotionEstimate estimate = {};
    if (!current_motion_estimate(&estimate)) {
        Serial.println("# rotl estimate invalid; stopping");
        stop_motion();
        return;
    }
    MoveROutput output = {};
    const esp_err_t error = move_r_update(&move_r_state, &estimate, dt_s, &output);
    if (error != ESP_OK || !output.motion_valid) {
        Serial.println("# rotl update failed; stopping");
        stop_motion();
        return;
    }
    if (!apply_advanced_output(output.requested_velocity)) return;
    if (should_print) {
        print_telemetry(now_ms, output.heading_error_rad * kRadToDeg,
                        output.requested_velocity);
    }
    if (output.status == MOVE_R_COMPLETE) {
        Serial.println("# ROTL COMPLETE");
        stop_motion();
    }
}

}  // namespace

// Initializes serial telemetry, the drivetrain facade, and odometry source.
void setup() {
    Serial.begin(115200);
    delay(1000);

    odom_source_config.x_drive_kinematics = DRIVETRAIN_CONFIG.x_drive_kinematics;
    odom_source_config.counts_per_revolution_fl =
        DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_FL]->counts_per_revolution;
    odom_source_config.counts_per_revolution_fr =
        DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_FR]->counts_per_revolution;
    odom_source_config.counts_per_revolution_bl =
        DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_BL]->counts_per_revolution;
    odom_source_config.counts_per_revolution_br =
        DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_BR]->counts_per_revolution;

    esp_err_t error = drivetrain_init(&drivetrain, &DRIVETRAIN_CONFIG);
    if (error == ESP_OK) error = drivetrain_enable(&drivetrain);
    drivetrain_ready = (error == ESP_OK);
    if (!drivetrain_ready) {
        Serial.print("# drivetrain init/enable failed: ");
        Serial.println(esp_err_to_name(error));
    }

    print_usage();
    print_config();
    Serial.println("millis,mode,pose_x_mm,pose_y_mm,heading_deg,remaining,vx,vy,omega");

    last_update_us = esp_timer_get_time();
}

// Services commands, keeps odometry current every cycle, and advances
// whichever move/rotation (if any) is active.
void loop() {
    handle_serial_command();

    const int64_t loop_now_us = esp_timer_get_time();
    float dt_s = (float)(loop_now_us - last_update_us) / 1.0e6f;
    if (dt_s <= 0.0f) return;
    last_update_us = loop_now_us;

    // MoveS/RotS are self-integrated (planned_progress_m/rad += commanded *
    // dt_s each cycle, see move_s.h/rot_s.h) with no external signal to
    // correct a bad step -- unlike drivetrain_update()'s own internal dt,
    // which already coasts rather than integrate an oversized one
    // (DrivetrainConfig.max_control_dt_s), this loop's own dt_s had no such
    // guard. A single anomalously large dt_s (e.g. a blocking Serial.print()
    // of the telemetry line stalling a cycle) gets baked permanently into
    // the plan, observed on hardware as commanded omega briefly exceeding
    // its own configured max_omega_rad_s ceiling -- structurally impossible
    // from rot_s.c's math alone, only reachable via a corrupted dt_s.
    // Clamping (not skipping the cycle) keeps motion continuous while
    // bounding the worst-case single-step integration error.
    if (dt_s > DRIVETRAIN_CONFIG.max_control_dt_s) {
        dt_s = DRIVETRAIN_CONFIG.max_control_dt_s;
    }

    if (cal_mode == CalMode::kIdle && drivetrain_ready) {
        // Keep the watchdog serviced (and encoders updating) while idle too,
        // via the same controlled-stop path "stop" uses -- otherwise the
        // command timeout coasts the drivetrain and encoder_driver_update()
        // stops being called, leaving odometry stale between moves.
        if (!set_velocity_or_brake({})) return;
    }
    if (drivetrain_ready) {
        // set_velocity_or_brake() above refreshes the drivetrain command
        // timestamp. Use a fresh timestamp here; passing the timestamp from
        // before that command makes now_us < last_command_us and produces
        // ESP_ERR_INVALID_ARG on the first idle cycle after boot/enable.
        const int64_t update_now_us = esp_timer_get_time();
        const esp_err_t error = drivetrain_update(&drivetrain, update_now_us);
        if (error != ESP_OK) {
            Serial.print("# drivetrain update failed; braking: ");
            Serial.println(esp_err_to_name(error));
            drivetrain_brake(&drivetrain);
            drivetrain_ready = false;
            cal_mode = CalMode::kIdle;
            return;
        }
    }

    const DrivetrainWheelCounts counts = {
        .fl = drivetrain_get_encoder_accumulated_count(&drivetrain, DRIVETRAIN_MOTOR_FL),
        .fr = drivetrain_get_encoder_accumulated_count(&drivetrain, DRIVETRAIN_MOTOR_FR),
        .bl = drivetrain_get_encoder_accumulated_count(&drivetrain, DRIVETRAIN_MOTOR_BL),
        .br = drivetrain_get_encoder_accumulated_count(&drivetrain, DRIVETRAIN_MOTOR_BR),
    };
    drivetrain_odometry_source_update(&odom_source, &odom_source_config, &counts, &odometry);

    const unsigned long now_ms = millis();
    const bool should_print = (now_ms - last_print_ms >= kTelemetryPeriodMs);
    if (should_print) last_print_ms = now_ms;

    if (cal_mode == CalMode::kMove) {
        service_move(now_ms, dt_s, should_print);
    } else if (cal_mode == CalMode::kRotate) {
        service_rotate(now_ms, dt_s, should_print);
    } else if (cal_mode == CalMode::kArc) {
        service_arc(now_ms, dt_s, should_print);
    } else if (cal_mode == CalMode::kMoveL) {
        service_movel(now_ms, dt_s, should_print);
    } else if (cal_mode == CalMode::kMoveP) {
        service_movep(now_ms, dt_s, should_print);
    } else if (cal_mode == CalMode::kMoveR) {
        service_mover(now_ms, dt_s, should_print);
    }
}
