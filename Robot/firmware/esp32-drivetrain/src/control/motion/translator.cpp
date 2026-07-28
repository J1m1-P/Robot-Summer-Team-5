#include "control/motion/translator.hpp"

#include <cmath>
#include <Arduino.h>
#include "esp_timer.h"
#include <robot_common/fixed_rate_gate.h>
#include <robot_common/math_utils.h>

namespace {

constexpr int64_t kCtrlPeriodUs = 5000;   // 200 Hz
constexpr float kPosTol = 0.003f;         // m: final position tolerance
constexpr float kHeadTol = 0.0349066f;    // rad: final heading tolerance (2°)
constexpr float kMotionOmegaRadS = 0.6f;
constexpr float kApproachRampDistanceM = 0.03f;
constexpr float kMinRampSpeedMps = 0.1f;
constexpr float kPositionGain = 6.0f;
constexpr float kHeadingGain = 3.0f;
constexpr float kMaxDt = 0.02f;           // s: reject delayed cycles

float wrap(float a) {
    while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

}  // namespace

esp_err_t precision_move(
    const PrecisionMoveContext *context,
    const PrecisionMoveTarget *target,
    float timeout_s) {
    if (context == nullptr || context->drivetrain == nullptr ||
        context->pose_service == nullptr || context->pose_service->pose_tracker == nullptr ||
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

    const Pose start = pose_tracker_get_pose(context->pose_service->pose_tracker);
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
    const float cruise_speed = std::hypot(
        target->body_velocity.vx, target->body_velocity.vy);
    const float translation_distance = std::hypot(dx, dy);

    enum class State { Translate, FinalRotate };
    State state = translation_distance > 1.0e-4f
        ? State::Translate : State::FinalRotate;
    const int64_t t0 = esp_timer_get_time();
    FixedRateGate gate = {kCtrlPeriodUs, t0};
    Pose prev = start;                           // previous estimate
    float cumulative_distance_m = 0.0f;
    TapeStopCondition tape_stop;

    auto stop = [&]() {
        drivetrain_stop(context->drivetrain);
    };

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
        const esp_err_t pose_err = pose_service_update(
            context->pose_service, static_cast<uint32_t>(now / 1000));
        if (pose_err != ESP_OK) {
            stop();
            return pose_err;
        }

        const Pose cur = pose_tracker_get_pose(context->pose_service->pose_tracker);
        if (!std::isfinite(cur.x_m) || !std::isfinite(cur.y_m) ||
            !std::isfinite(cur.heading_rad)) {
            stop();
            return ESP_ERR_INVALID_STATE;
        }

        cumulative_distance_m += std::hypot(cur.x_m - prev.x_m,
                                            cur.y_m - prev.y_m);
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
            if (distance_error <= kPosTol) {
                if (target->tape_stop_enabled &&
                    !tape_stop_condition_triggered(&tape_stop)) {
                    stop();
                    return ESP_ERR_TIMEOUT;
                }
                state = State::FinalRotate;
                break;
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

            break;
        }
        case State::FinalRotate: {
            const float heading_error = wrap(goal.heading_rad -
                                             cur.heading_rad);
            if (std::fabs(heading_error) > kHeadTol) {
                // Rotate alone. Mixing translation correction into this
                // command can saturate the wheel targets.
                cmd.omega = clamp(
                    heading_error * kHeadingGain,
                    -kMotionOmegaRadS, kMotionOmegaRadS);
                break;
            }

            if (target->tape_stop_enabled &&
                !tape_stop_condition_triggered(&tape_stop)) {
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

            const Pose corrected_goal = {
                goal.x_m, goal.y_m, goal.heading_rad};
            const esp_err_t snap_error = pose_tracker_set_pose(
                context->pose_service->pose_tracker, &corrected_goal);
            if (snap_error != ESP_OK) {
                stop();
                return snap_error;
            }
            stop();
            return ESP_OK;
        }
        }

        const esp_err_t command_err = drivetrain_set_advanced_body_velocity(
            context->drivetrain, cmd.vx, cmd.vy, cmd.omega);
        const esp_err_t update_err = command_err == ESP_OK
            // set_* refreshes last_command_us, so sample time again before
            // updating; the earlier loop timestamp may now be stale.
            ? drivetrain_update(context->drivetrain, esp_timer_get_time())
            : command_err;
        if (update_err != ESP_OK) {
            stop();
            return update_err;
        }
    }
}
