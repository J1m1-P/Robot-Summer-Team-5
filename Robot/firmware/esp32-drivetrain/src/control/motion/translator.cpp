#include "control/motion/translator.hpp"

#include <cmath>
#include <cstdio>
#include <Arduino.h>
#include "esp_timer.h"
#include "control/pid/bounded_pid.h"
#include <robot_common/fixed_rate_gate.h>
#include <robot_common/math_utils.h>

#ifndef ROBOT_MOTION_DIAGNOSTICS
#define ROBOT_MOTION_DIAGNOSTICS 0
#endif

namespace {

constexpr int64_t kCtrlPeriodUs = 5000;   // 200 Hz
constexpr float kPosTol = 0.005f;         // m: final position tolerance
constexpr float kHeadTol = 0.0349066f;    // rad: final heading tolerance (2°)
constexpr float kMotionOmegaRadS = 0.6f;
constexpr float kApproachRampDistanceM = 0.03f;
constexpr float kMinRampSpeedMps = 0.1f;
constexpr float kMinPrecisionTranslateSpeedMps = 0.05f;
constexpr float kShortMoveNudgeS = 0.05f;
constexpr float kSettleS = 0.15f;
// Endpoint correction uses a bounded pulse above the wheel-controller
// deadband, followed by a pause for fresh odometry. Continuous low-speed
// correction can command PWM without producing useful wheel motion.
// Use continuous closed-loop travel until the final 10 mm, then apply the
// bounded nudge correction for settling without making small moves pulse.
constexpr float kEndpointNudgeBandM = 0.010f;
constexpr float kEndpointNudgeSpeedMps = 0.10f;
constexpr float kEndpointNudgePulseS = 0.05f;
constexpr float kEndpointNudgeRampS = 0.005f;
constexpr float kEndpointNudgePauseS = 0.02f;
constexpr float kPositionGain = 6.0f;
constexpr float kHeadingGain = 3.0f;
constexpr float kMaxDt = 0.02f;           // s: reject delayed cycles

#if ROBOT_MOTION_DIAGNOSTICS
struct MotionDiagnosticSnapshot {
    bool valid = false;
    int64_t timestamp_us = 0;
    float distance_error_m = 0.0f;
    float step_distance_m = 0.0f;
    float cumulative_distance_m = 0.0f;
    float command_vx = 0.0f;
    float command_vy = 0.0f;
    float command_omega = 0.0f;
    float nudge_phase_s = 0.0f;
    bool nudge_active = false;
    Pose pose = {};
    DrivetrainWheelCounts counts = {};
    float target_mps[DRIVETRAIN_MOTOR_MAX] = {};
    float velocity_mps[DRIVETRAIN_MOTOR_MAX] = {};
    float duty[DRIVETRAIN_MOTOR_MAX] = {};
};

constexpr DrivetrainMotorId kDiagnosticMotors[DRIVETRAIN_MOTOR_MAX] = {
    DRIVETRAIN_MOTOR_FL,
    DRIVETRAIN_MOTOR_FR,
    DRIVETRAIN_MOTOR_BL,
    DRIVETRAIN_MOTOR_BR,
};

void print_motion_diagnostic_snapshot(
    const MotionDiagnosticSnapshot &snapshot,
    const char *reason,
    int64_t t0,
    int error_code) {
    std::printf(
        "# motion snapshot: reason=%s error=%d sample_t=%lu elapsed_ms=%lu "
        "valid=%u pose=(%.4f,%.4f,%.4f) err=%.4f step=%.4f total=%.4f "
        "cmd=(%.3f,%.3f,%.3f) nudge=(%u,%.3f)\n",
        reason,
        error_code,
        (unsigned long)(snapshot.timestamp_us / 1000),
        snapshot.timestamp_us >= t0
            ? (unsigned long)((snapshot.timestamp_us - t0) / 1000)
            : 0UL,
        snapshot.valid ? 1U : 0U,
        snapshot.pose.x_m,
        snapshot.pose.y_m,
        snapshot.pose.heading_rad,
        snapshot.distance_error_m,
        snapshot.step_distance_m,
        snapshot.cumulative_distance_m,
        snapshot.command_vx,
        snapshot.command_vy,
        snapshot.command_omega,
        snapshot.nudge_active ? 1U : 0U,
        snapshot.nudge_phase_s);
    std::printf(
        "# motion wheels: counts=(%ld,%ld,%ld,%ld) "
        "FL=(target=%.3f vel=%.3f duty=%.3f) "
        "FR=(target=%.3f vel=%.3f duty=%.3f) "
        "BL=(target=%.3f vel=%.3f duty=%.3f) "
        "BR=(target=%.3f vel=%.3f duty=%.3f)\n",
        (long)snapshot.counts.fl,
        (long)snapshot.counts.fr,
        (long)snapshot.counts.bl,
        (long)snapshot.counts.br,
        snapshot.target_mps[0], snapshot.velocity_mps[0], snapshot.duty[0],
        snapshot.target_mps[1], snapshot.velocity_mps[1], snapshot.duty[1],
        snapshot.target_mps[2], snapshot.velocity_mps[2], snapshot.duty[2],
        snapshot.target_mps[3], snapshot.velocity_mps[3], snapshot.duty[3]);
}
#endif

float wrap(float a) {
    while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

// weights for the same four channels used by the line follower's steering
constexpr float kAlignWeights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};
constexpr float kAlignErrorTol = 0.3f;  // channels considered centered below this
constexpr float kAlignSettleS = 0.15f;  // must stay centered this long to finish
// Filters the discrete, channel-quantized error before it's differentiated
// (avoids derivative kick on channel-bit steps) and smooths the commanded
// omega, matching the line follower's EMA treatment of the same sensors.
constexpr float kAlignEmaAlpha = 0.4f;
constexpr float kAlignOmegaEmaAlpha = 0.15f;
constexpr BoundedPidConfig kAlignPidConfig = {
    .proportional_gain = 0.8f,
    .integral_gain = 0.0f,
    .derivative_gain = 0.05f,
    .integral_limit = 0.0f,
    .correction_min = -1.0f,
    .correction_max = 1.0f,
};

int AlignSensorIndex(Direction dir) {
    switch (dir) {
        case Direction::PX: return 0;
        case Direction::MX: return 1;
        case Direction::PY: return 2;
    }
    return 0;
}

// Approximates the guide sensor's location as the centerpoint of the axle
// between the two wheels on that side, from the drivetrain's own geometry.
void AlignPivotOffset(const XDriveKinematicsConfig &geom, Direction dir,
                      float *pivot_x_m, float *pivot_y_m) {
    *pivot_x_m = 0.0f;
    *pivot_y_m = 0.0f;
    switch (dir) {
        case Direction::PX: *pivot_x_m = geom.chassis_half_length_m; break;
        case Direction::MX: *pivot_x_m = -geom.chassis_half_length_m; break;
        case Direction::PY: *pivot_y_m = geom.chassis_half_width_m; break;
    }
}

// weighted centroid of the channels matching the target feature (tape when
// !on_gap, the gap between two tape edges when on_gap), in [-3, 3]; false if
// no channel matches
bool ComputeAlignError(const TapeSensor *sensor, bool on_gap, float *error_out) {
    const bool channel[4] = {
        sensor->channel_0, sensor->channel_1, sensor->channel_2, sensor->channel_3};
    float sum = 0.0f;
    int active = 0;
    for (int i = 0; i < 4; ++i) {
        if (channel[i] != on_gap) { sum += kAlignWeights[i]; active++; }
    }
    if (active == 0) return false;
    *error_out = sum / static_cast<float>(active);
    return true;
}

}  // namespace

esp_err_t precision_move(
    const PrecisionMoveContext *context,
    const PrecisionMoveTarget *target,
    float timeout_s) {
    if (context == nullptr || context->drivetrain == nullptr ||
        context->sequence_controller == nullptr ||
        context->sequence_controller->pose_tracker == nullptr ||
        target == nullptr || !std::isfinite(target->dx_body_m) ||
        !std::isfinite(target->dy_body_m) || !std::isfinite(target->delta_heading_rad) ||
        !std::isfinite(target->body_velocity.vx) ||
        !std::isfinite(target->body_velocity.vy) ||
        !std::isfinite(target->body_velocity.omega) ||
        !std::isfinite(timeout_s) || timeout_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (target->tape_stop_enabled &&
        (context->sensors[0] == nullptr || context->sensors[1] == nullptr ||
         context->sensors[2] == nullptr ||
         !tape_stop_spec_is_valid(&target->tape_stop_spec))) {
        return ESP_ERR_INVALID_ARG;
    }
    PoseTracker *pose_tracker =
        context->sequence_controller->pose_tracker;

    const Pose start = pose_tracker_get_pose(pose_tracker);
    if (!std::isfinite(start.x_m) || !std::isfinite(start.y_m) ||
        !std::isfinite(start.heading_rad)) {
        return ESP_ERR_INVALID_STATE;
    }

    Pose goal = {};
    float dx = 0.0f;
    float dy = 0.0f;
    if (target->world_goal_enabled) {
        goal = target->world_goal;
        dx = goal.x_m - start.x_m;
        dy = goal.y_m - start.y_m;
    } else {
        const float c0 = std::cos(start.heading_rad), s0 = std::sin(start.heading_rad);
        dx = c0 * target->dx_body_m - s0 * target->dy_body_m;
        dy = s0 * target->dx_body_m + c0 * target->dy_body_m;
        goal = {
            start.x_m + dx,
            start.y_m + dy,
            wrap(start.heading_rad + target->delta_heading_rad),
        };
    }
    // Translation and rotation are executed as separate phases. Hold the
    // heading that was active at translation start; any requested heading
    // change is applied afterward by FinalRotate.
    const float translation_heading = wrap(
        goal.heading_rad - target->delta_heading_rad);
    const float cruise_speed = std::hypot(
        target->body_velocity.vx, target->body_velocity.vy);
    const float translation_distance = std::hypot(dx, dy);

    enum class State { Translate, FinalRotate, Settle };
    State state = translation_distance > 1.0e-6f
        ? State::Translate : State::FinalRotate;
 #if ROBOT_MOTION_DIAGNOSTICS
    // Print before starting the timed control operation so USB logging cannot
    // consume the first control interval and trip the dt safety limit.
    const int64_t log_start = esp_timer_get_time();
    std::printf(
        "# motion start: t=%lu timeout_s=%.2f dx=%.3f dy=%.3f dtheta=%.3f "
        "cruise_vx=%.3f cruise_vy=%.3f\n",
        (unsigned long)(log_start / 1000),
        timeout_s,
        target->dx_body_m,
        target->dy_body_m,
        target->delta_heading_rad,
        target->body_velocity.vx,
        target->body_velocity.vy);
 #endif

    const int64_t t0 = esp_timer_get_time();
    FixedRateGate gate = {kCtrlPeriodUs, t0};
    Pose prev = start;                           // previous estimate
    float cumulative_distance_m = 0.0f;
    TapeStopCondition tape_stop;
    float translate_elapsed_s = 0.0f;
    float settled_s = 0.0f;
    bool endpoint_correction_applied = false;
    float endpoint_nudge_phase_s = 0.0f;
#if ROBOT_MOTION_DIAGNOSTICS
    MotionDiagnosticSnapshot diagnostic = {};
#endif
    auto stop = [&]() {
        drivetrain_stop(context->drivetrain);
    };

    while (true) {
        const int64_t now = esp_timer_get_time();
        if (static_cast<float>(now - t0) / 1.0e6f >= timeout_s) {
            stop();
#if ROBOT_MOTION_DIAGNOSTICS
            std::printf(
                "# motion exit: reason=timeout t=%lu elapsed_ms=%lu\n",
                (unsigned long)(now / 1000),
                (unsigned long)((now - t0) / 1000));
            print_motion_diagnostic_snapshot(
                diagnostic, "timeout", t0, ESP_ERR_TIMEOUT);
#endif
            return ESP_ERR_TIMEOUT;
        }

        int64_t dt_us = 0;
        if (!gate.Ready(now, &dt_us)) {
            delay(1);
            continue;
        }
        const float dt = static_cast<float>(dt_us) / 1.0e6f;
        if (!std::isfinite(dt) || dt <= 0.0f || dt > kMaxDt) {
            stop();
#if ROBOT_MOTION_DIAGNOSTICS
            std::printf(
                "# motion exit: reason=invalid_dt t=%lu dt_s=%.6f\n",
                (unsigned long)(now / 1000),
                dt);
            print_motion_diagnostic_snapshot(
                diagnostic, "invalid_dt", t0, ESP_ERR_INVALID_STATE);
#endif
            return ESP_ERR_INVALID_STATE;
        }
        const esp_err_t pose_err = robot_sequence_controller_update(
            context->sequence_controller, static_cast<uint32_t>(now / 1000));
        if (pose_err != ESP_OK || !context->sequence_controller->running) {
            stop();
#if ROBOT_MOTION_DIAGNOSTICS
            std::printf(
                "# motion exit: reason=sequence_state t=%lu pose_err=%d "
                "running=%u updating_movement=%u current_step=%u\n",
                (unsigned long)(now / 1000),
                (int)pose_err,
                context->sequence_controller->running ? 1U : 0U,
                context->sequence_controller->updating_movement ? 1U : 0U,
                (unsigned)context->sequence_controller->current_step);
            print_motion_diagnostic_snapshot(
                diagnostic, "sequence_state", t0,
                pose_err != ESP_OK ? (int)pose_err : (int)ESP_ERR_INVALID_STATE);
#endif
            return pose_err != ESP_OK ? pose_err : ESP_ERR_INVALID_STATE;
        }
        if (target->external_stop_requested != nullptr &&
            *target->external_stop_requested) {
            stop();
            return ESP_OK;
        }

        const Pose cur = pose_tracker_get_pose(pose_tracker);
        if (!std::isfinite(cur.x_m) || !std::isfinite(cur.y_m) ||
            !std::isfinite(cur.heading_rad)) {
            stop();
#if ROBOT_MOTION_DIAGNOSTICS
            std::printf(
                "# motion exit: reason=invalid_pose t=%lu x=%.4f y=%.4f "
                "heading=%.4f\n",
                (unsigned long)(now / 1000),
                cur.x_m,
                cur.y_m,
                cur.heading_rad);
            print_motion_diagnostic_snapshot(
                diagnostic, "invalid_pose", t0, ESP_ERR_INVALID_STATE);
#endif
            return ESP_ERR_INVALID_STATE;
        }

        const float step_distance_m = std::hypot(cur.x_m - prev.x_m,
                                                 cur.y_m - prev.y_m);
        cumulative_distance_m += step_distance_m;
        prev = cur;

        if (target->tape_stop_enabled) {
            const esp_err_t sensor_error = tape_sensor_driver_read_all(
                context->sensors);
            if (sensor_error != ESP_OK) {
                stop();
                return sensor_error;
            }
            const bool tape_stop_reached = tape_stop_condition_update(
                &tape_stop, &target->tape_stop_spec, context->sensors,
                cumulative_distance_m);
            if (tape_stop_reached) {
                stop();
                return ESP_OK;
            }
        }

        const float error_world_x = goal.x_m - cur.x_m;
        const float error_world_y = goal.y_m - cur.y_m;
        const float distance_error = std::hypot(error_world_x, error_world_y);
        const float c = std::cos(cur.heading_rad), s = std::sin(cur.heading_rad);
        const float error_body_x = c * error_world_x + s * error_world_y;
        const float error_body_y = -s * error_world_x + c * error_world_y;
        DrivetrainBodyVelocity cmd = {};

        switch (state) {
        case State::Translate: {
            const bool short_translation =
                translation_distance <= kPosTol;
            const bool short_move_nudge_active =
                short_translation && translate_elapsed_s < kShortMoveNudgeS;
            if (distance_error <= kPosTol) {
                if (!short_move_nudge_active &&
                    std::fabs(target->delta_heading_rad) > kHeadTol) {
                    state = State::FinalRotate;
                    break;
                }
                if (!short_move_nudge_active &&
                    target->tape_stop_enabled &&
                    !tape_stop_condition_triggered(&tape_stop)) {
                    stop();
                    return ESP_ERR_TIMEOUT;
                }
                if (!short_move_nudge_active) {
                    state = State::FinalRotate;
                    break;
                }
            }

            float command_speed = cruise_speed;
            if (distance_error < kApproachRampDistanceM) {
                const float floor_speed = std::fmin(
                    kMinRampSpeedMps, cruise_speed);
                const float t = clamp(
                    distance_error / kApproachRampDistanceM, 0.0f, 1.0f);
                command_speed = floor_speed +
                    t * (cruise_speed - floor_speed);
            }
            // Control each body axis independently so a small cross-track
            // error receives enough velocity to overcome drivetrain deadband.
            const float position_gain = std::fmax(
                kPositionGain, command_speed / kApproachRampDistanceM);
            cmd.vx = clamp(error_body_x * position_gain,
                           -command_speed, command_speed);
            cmd.vy = clamp(error_body_y * position_gain,
                           -command_speed, command_speed);
            const float command_norm = std::hypot(cmd.vx, cmd.vy);
            if (command_norm > command_speed) {
                const float scale = command_speed / command_norm;
                cmd.vx *= scale;
                cmd.vy *= scale;
            }
            if (command_norm > 0.0f &&
                (distance_error > kPosTol || short_move_nudge_active) &&
                command_norm < kMinPrecisionTranslateSpeedMps) {
                const float min_speed = std::fmin(
                    kMinPrecisionTranslateSpeedMps, command_speed);
                const float scale = min_speed / command_norm;
                cmd.vx *= scale;
                cmd.vy *= scale;
            }
            const float heading_error = wrap(
                translation_heading - cur.heading_rad);
            cmd.omega = clamp(
                heading_error * kHeadingGain,
                -kMotionOmegaRadS,
                kMotionOmegaRadS);
            translate_elapsed_s += dt;

            break;
        }
        case State::FinalRotate: {
            const float heading_error = wrap(goal.heading_rad -
                                             cur.heading_rad);
            if (std::fabs(heading_error) > kHeadTol) {
                // Rotate alone. Mixing translation correction into this
                // command can saturate the wheel targets.
                const float requested_omega =
                    std::fabs(target->body_velocity.omega);
                const float rotate_limit = requested_omega > 0.0f
                    ? requested_omega : kMotionOmegaRadS;
                cmd.omega = clamp(
                    heading_error * kHeadingGain,
                    -rotate_limit, rotate_limit);
                break;
            }

            if (target->tape_stop_enabled &&
                !tape_stop_condition_triggered(&tape_stop)) {
                stop();
                return ESP_ERR_TIMEOUT;
            }
            if (target->external_stop_requested != nullptr &&
                !*target->external_stop_requested) {
                stop();
                return ESP_ERR_TIMEOUT;
            }

            if (distance_error > kPosTol) {
                // Once heading is settled, correct any position movement
                // caused by the turn before accepting the final pose.
                const float direction_scale = kMinRampSpeedMps / distance_error;
                cmd.vx = error_body_x * direction_scale;
                cmd.vy = error_body_y * direction_scale;
                break;
            }

            // Observe coast-induced drift before accepting the endpoint.
            stop();
            settled_s = endpoint_correction_applied ? kSettleS : 0.0f;
            state = State::Settle;
            continue;
        }
        case State::Settle: {
            // Let passive motion finish before deciding whether correction
            // is needed. Chasing drift immediately causes repeated reversals.
            if (settled_s < kSettleS) {
                settled_s += dt;
                continue;
            }

            const float heading_error = wrap(
                goal.heading_rad - cur.heading_rad);
            if (!endpoint_correction_applied &&
                (distance_error > kPosTol ||
                 std::fabs(heading_error) > kHeadTol)) {
                endpoint_correction_applied = true;
                state = State::FinalRotate;
                continue;
            }

            // Commit the nominal endpoint only after the measured pose has
            // settled and any required correction has been applied once.
            const Pose corrected_goal = {
                goal.x_m, goal.y_m, goal.heading_rad};
            const esp_err_t snap_error = pose_tracker_set_pose(
                pose_tracker, &corrected_goal);
            if (snap_error != ESP_OK) {
                stop();
                return snap_error;
            }
            stop();
            return ESP_OK;
        }
        }

        const bool endpoint_nudge_band =
            distance_error > kPosTol &&
            distance_error <= kEndpointNudgeBandM;
        if (!endpoint_nudge_band) {
            endpoint_nudge_phase_s = 0.0f;
        } else {
            endpoint_nudge_phase_s += dt;
            if (endpoint_nudge_phase_s >=
                kEndpointNudgePulseS + kEndpointNudgePauseS) {
                endpoint_nudge_phase_s = 0.0f;
            }

            const bool pulse_active =
                endpoint_nudge_phase_s < kEndpointNudgePulseS;
            const float error_norm = std::hypot(error_body_x, error_body_y);
            if (pulse_active && error_norm > 1.0e-6f) {
                const float ramp_in = endpoint_nudge_phase_s /
                    kEndpointNudgeRampS;
                const float ramp_out =
                    (kEndpointNudgePulseS - endpoint_nudge_phase_s) /
                    kEndpointNudgeRampS;
                const float pulse_scale = clamp(
                    std::fmin(ramp_in, ramp_out), 0.0f, 1.0f);
                const float nudge_speed =
                    kEndpointNudgeSpeedMps * pulse_scale;
                cmd.vx = error_body_x / error_norm * nudge_speed;
                cmd.vy = error_body_y / error_norm * nudge_speed;
            } else {
                // Pause with zero translation so the next odometry sample is
                // measured after the pulse rather than during it.
                cmd.vx = 0.0f;
                cmd.vy = 0.0f;
            }
        }

        const esp_err_t command_err = drivetrain_set_advanced_body_velocity(
            context->drivetrain, cmd.vx, cmd.vy, cmd.omega);
        const esp_err_t update_err = command_err == ESP_OK
            // set_* refreshes last_command_us, so sample time again before
            // updating; the earlier loop timestamp may now be stale.
            ? drivetrain_update(context->drivetrain, esp_timer_get_time())
            : command_err;
#if ROBOT_MOTION_DIAGNOSTICS
        diagnostic.valid = true;
        diagnostic.timestamp_us = now;
        diagnostic.distance_error_m = distance_error;
        diagnostic.step_distance_m = step_distance_m;
        diagnostic.cumulative_distance_m = cumulative_distance_m;
        diagnostic.command_vx = cmd.vx;
        diagnostic.command_vy = cmd.vy;
        diagnostic.command_omega = cmd.omega;
        diagnostic.nudge_phase_s = endpoint_nudge_phase_s;
        diagnostic.nudge_active = endpoint_nudge_band;
        diagnostic.pose = cur;
        diagnostic.counts = drivetrain_get_wheel_counts(context->drivetrain);
        for (int motor = 0; motor < DRIVETRAIN_MOTOR_MAX; ++motor) {
            diagnostic.target_mps[motor] =
                drivetrain_get_target_velocity_mps(
                    context->drivetrain, kDiagnosticMotors[motor]);
            diagnostic.velocity_mps[motor] =
                drivetrain_get_encoder_velocity_mps(
                    context->drivetrain, kDiagnosticMotors[motor]);
            diagnostic.duty[motor] = drivetrain_get_applied_duty(
                context->drivetrain, kDiagnosticMotors[motor]);
        }
#endif
        if (update_err != ESP_OK) {
            stop();
#if ROBOT_MOTION_DIAGNOSTICS
            std::printf(
                "# motion update error: t=%lu elapsed_ms=%lu command_err=%d "
                "update_err=%d\n",
                (unsigned long)(now / 1000),
                (unsigned long)((now - t0) / 1000),
                (int)command_err,
                (int)update_err);
            print_motion_diagnostic_snapshot(
                diagnostic, "update_error", t0, (int)update_err);
#endif
            return update_err;
        }
    }
}

esp_err_t timed_move(
    const PrecisionMoveContext *context,
    float vx_mps,
    float vy_mps,
    float duration_s) {
    if (context == nullptr || context->drivetrain == nullptr ||
        context->sequence_controller == nullptr ||
        context->sequence_controller->pose_tracker == nullptr ||
        !std::isfinite(vx_mps) || !std::isfinite(vy_mps) ||
        !std::isfinite(duration_s) || duration_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    PoseTracker *pose_tracker = context->sequence_controller->pose_tracker;
    const Pose start = pose_tracker_get_pose(pose_tracker);
    if (!std::isfinite(start.x_m) || !std::isfinite(start.y_m) ||
        !std::isfinite(start.heading_rad)) {
        return ESP_ERR_INVALID_STATE;
    }

    // Travel is body-relative, so the commanded direction is fixed in the
    // start heading's frame. Correct drift perpendicular to that direction
    // (cross-track) and hold the start heading, but never touch the
    // along-track component -- that stays open-loop speed*time by design.
    const float command_speed = std::hypot(vx_mps, vy_mps);
    const bool has_direction = command_speed > 1.0e-6f;
    const float along_x = has_direction ? vx_mps / command_speed : 0.0f;
    const float along_y = has_direction ? vy_mps / command_speed : 0.0f;
    const float perp_x = -along_y;
    const float perp_y = along_x;

    const int64_t t0 = esp_timer_get_time();
    FixedRateGate gate = {kCtrlPeriodUs, t0};
    auto stop = [&]() { drivetrain_stop(context->drivetrain); };

    while (true) {
        const int64_t now = esp_timer_get_time();
        if (static_cast<float>(now - t0) / 1.0e6f >= duration_s) {
            stop();
            return ESP_OK;
        }

        int64_t dt_us = 0;
        if (!gate.Ready(now, &dt_us)) {
            delay(1);
            continue;
        }
        const float dt = static_cast<float>(dt_us) / 1.0e6f;
        if (!std::isfinite(dt) || dt <= 0.0f || dt > kMaxDt) {
            stop();
            return ESP_ERR_INVALID_STATE;
        }

        const esp_err_t pose_err = robot_sequence_controller_update(
            context->sequence_controller, static_cast<uint32_t>(now / 1000));
        if (pose_err != ESP_OK || !context->sequence_controller->running) {
            stop();
            return pose_err != ESP_OK ? pose_err : ESP_ERR_INVALID_STATE;
        }

        const Pose cur = pose_tracker_get_pose(pose_tracker);
        if (!std::isfinite(cur.x_m) || !std::isfinite(cur.y_m) ||
            !std::isfinite(cur.heading_rad)) {
            stop();
            return ESP_ERR_INVALID_STATE;
        }

        float cmd_vx = vx_mps;
        float cmd_vy = vy_mps;
        if (has_direction) {
            // Displacement since start, resolved into the start heading's
            // body frame so it can be split into along-track/cross-track.
            const float c0 = std::cos(start.heading_rad);
            const float s0 = std::sin(start.heading_rad);
            const float world_dx = cur.x_m - start.x_m;
            const float world_dy = cur.y_m - start.y_m;
            const float disp_body_x = c0 * world_dx + s0 * world_dy;
            const float disp_body_y = -s0 * world_dx + c0 * world_dy;
            const float cross_track_error =
                disp_body_x * perp_x + disp_body_y * perp_y;
            const float correction = clamp(
                -cross_track_error * kPositionGain,
                -kMinRampSpeedMps, kMinRampSpeedMps);
            cmd_vx += correction * perp_x;
            cmd_vy += correction * perp_y;
        }
        const float heading_error = wrap(start.heading_rad - cur.heading_rad);
        const float cmd_omega = clamp(
            heading_error * kHeadingGain, -kMotionOmegaRadS, kMotionOmegaRadS);

        const esp_err_t command_err = drivetrain_set_advanced_body_velocity(
            context->drivetrain, cmd_vx, cmd_vy, cmd_omega);
        const esp_err_t update_err = command_err == ESP_OK
            ? drivetrain_update(context->drivetrain, esp_timer_get_time())
            : command_err;
        if (update_err != ESP_OK) {
            stop();
            return update_err;
        }
    }
}

esp_err_t align_on_tape(const TapeAlignContext *context, Direction travel_dir,
                        Direction feedback_dir, bool on_gap, float timeout_s) {
    if (context == nullptr || context->drivetrain == nullptr ||
        context->sequence_controller == nullptr ||
        context->sequence_controller->pose_tracker == nullptr ||
        context->sensors[0] == nullptr || context->sensors[1] == nullptr ||
        context->sensors[2] == nullptr ||
        !std::isfinite(timeout_s) || timeout_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    float pivot_x_m = 0.0f, pivot_y_m = 0.0f;
    AlignPivotOffset(context->drivetrain->config->x_drive_kinematics,
                     travel_dir, &pivot_x_m, &pivot_y_m);
    const TapeSensor *sensor = context->sensors[AlignSensorIndex(feedback_dir)];

    const int64_t t0 = esp_timer_get_time();
    FixedRateGate gate = {kCtrlPeriodUs, t0};
    BoundedPidState pid_state = {};
    float settled_s = 0.0f;
    float filtered_error = 0.0f;
    float smoothed_omega = 0.0f;

    auto stop = [&]() { drivetrain_stop(context->drivetrain); };

    while (true) {
        const int64_t now = esp_timer_get_time();
        if (static_cast<float>(now - t0) / 1.0e6f >= timeout_s) {
            stop();
            return ESP_ERR_TIMEOUT;
        }

        int64_t dt_us = 0;
        if (!gate.Ready(now, &dt_us)) {
            delay(1);
            continue;
        }
        const float dt = static_cast<float>(dt_us) / 1.0e6f;
        if (!std::isfinite(dt) || dt <= 0.0f || dt > kMaxDt) {
            stop();
            return ESP_ERR_INVALID_STATE;
        }

        const esp_err_t pose_err = robot_sequence_controller_update(
            context->sequence_controller, static_cast<uint32_t>(now / 1000));
        if (pose_err != ESP_OK || !context->sequence_controller->running) {
            stop();
            return pose_err != ESP_OK ? pose_err : ESP_ERR_INVALID_STATE;
        }
        if (tape_sensor_driver_read_all(context->sensors) != ESP_OK) {
            stop();
            return ESP_FAIL;
        }

        float error = 0.0f;
        if (!ComputeAlignError(sensor, on_gap, &error)) {
            stop();
            return ESP_FAIL;
        }
        filtered_error = kAlignEmaAlpha * error + (1.0f - kAlignEmaAlpha) * filtered_error;

        if (std::fabs(filtered_error) < kAlignErrorTol) {
            settled_s += dt;
            if (settled_s >= kAlignSettleS) {
                stop();
                return ESP_OK;
            }
        } else {
            settled_s = 0.0f;
        }

        const float omega_raw = bounded_pid_update(
            &pid_state, &kAlignPidConfig, filtered_error, dt);
        smoothed_omega = kAlignOmegaEmaAlpha * omega_raw +
            (1.0f - kAlignOmegaEmaAlpha) * smoothed_omega;
        const float omega = smoothed_omega;
        const float vx = omega * pivot_y_m;
        const float vy = -omega * pivot_x_m;

        const esp_err_t command_err = drivetrain_set_advanced_body_velocity(
            context->drivetrain, vx, vy, omega);
        const esp_err_t update_err = command_err == ESP_OK
            ? drivetrain_update(context->drivetrain, esp_timer_get_time())
            : command_err;
        if (update_err != ESP_OK) {
            stop();
            return update_err;
        }
    }
}
