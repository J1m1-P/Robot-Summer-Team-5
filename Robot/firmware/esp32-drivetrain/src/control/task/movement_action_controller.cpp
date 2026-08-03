/* Implements tape-guided and ordinary drivetrain actions. */
#include "control/task/movement_action_controller.h"

#ifndef ROBOT_MOTION_DIAGNOSTICS
#define ROBOT_MOTION_DIAGNOSTICS 0
#endif

#include <cmath>
#include <cstdio>
#include <stddef.h>

#include "control/line_following/line_follower.hpp"
#include <Arduino.h>
#include "control/motion/translator.hpp"

namespace {

constexpr float kTapeFollowSpeedMps = 0.35f;
constexpr float kSideTowerFollowSpeedMps = 0.15f;
constexpr float kTapeFollowTimeoutS = 30.0f;
constexpr float kLocatorApproachSpeedMps = 0.10f;
constexpr float kPrecisionVxMps = 0.2f;
constexpr float kPrecisionVyMps = 0.15f;
constexpr float kPrecisionOmegaRadS = 1.0f;
constexpr float kPrecisionTimeoutS = 15.0f;
constexpr float kTapeSeekMaxDistanceM = 1.5f;
// Stay just inside the +/-pi wrap boundary so the safety-bound turn is
// unambiguously clockwise.
constexpr float kTapeSeekMaxRotationRad =
    static_cast<float>(M_PI) - 0.01f;
constexpr TapeStopSpec kSideTapeStopSpec = {
    .sensor_mask = 1U << 2,  // side sensor
    .required_sensor_count = 1,
    .channel_mask = 1U << TAPE_SENSOR_CHANNEL_0,
};
constexpr TapeStopSpec kFrontRightTapeStopSpec = {
    .sensor_mask = 1U << 0,  // front/PX sensor
    .required_sensor_count = 1,
    // Front channels run from absolute right (0) to absolute left (3).
    // During a CW sweep, the right detector reaches the tape first.
    .channel_mask = 1U << TAPE_SENSOR_CHANNEL_0,
};
constexpr TapeStopSpec kFrontLeftTapeStopSpec = {
    .sensor_mask = 1U << 0,  // front/PX sensor
    .required_sensor_count = 1,
    // Front channels run from absolute right (0) to absolute left (3).
    // During a CCW sweep, the left detector reaches the tape first.
    .channel_mask = 1U << TAPE_SENSOR_CHANNEL_3,
};

LineFollowerContext *g_line_follower_ctx = nullptr;
PrecisionMoveContext *g_precision_move_ctx = nullptr;
Pose g_planned_pose = {};
bool g_planned_pose_valid = false;

float wrap_angle(float angle) {
    while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle <= -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

void sync_planned_pose() {
    if (g_precision_move_ctx == nullptr ||
        g_precision_move_ctx->sequence_controller == nullptr ||
        g_precision_move_ctx->sequence_controller->pose_tracker == nullptr) {
        return;
    }
    const Pose pose = pose_tracker_get_pose(
        g_precision_move_ctx->sequence_controller->pose_tracker);
    if (!std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) ||
        !std::isfinite(pose.heading_rad)) {
        g_planned_pose_valid = false;
        return;
    }
    g_planned_pose = pose;
    g_planned_pose_valid = true;
}

bool follow_tape_action(
    LineFollowerContext *context,
    Direction direction,
    float speed_mps,
    StopCondition stop_condition,
    float distance_m,
    float timeout_s,
    TapeMarkerSensor marker_sensor = TapeMarkerSensor::AUTO) {
    const TapeFollowMode follow_mode =
        direction == Direction::PX || direction == Direction::MX
            ? TapeFollowMode::SINGLE_SENSOR
            : TapeFollowMode::SINGLE_SENSOR;
    const bool success = follow_tape(
        context, direction, speed_mps, stop_condition, distance_m, timeout_s,
        follow_mode, marker_sensor);
    if (success) sync_planned_pose();
    return success;
}

bool action_requires_nonnegative_distance(MovementAction action) {
    switch (action) {
        case MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE:
        case MOVEMENT_ACTION_MX_TAPE_FOLLOW_DISTANCE:
        case MOVEMENT_ACTION_PY_TAPE_FOLLOW_DISTANCE:
        case MOVEMENT_ACTION_GO_PX_UNTIL_SIDE_TAPE:
        case MOVEMENT_ACTION_GO_MX_UNTIL_SIDE_TAPE:
        case MOVEMENT_ACTION_GO_PY_UNTIL_FRONT_TAPE:
        case MOVEMENT_ACTION_GO_MX_UNTIL_LOCATOR:
        case MOVEMENT_ACTION_GO_PY_UNTIL_SOLAR_PANEL:
            return true;
        default:
            return false;
    }
}

bool action_requires_positive_sweep(MovementAction action) {
    return action == MOVEMENT_ACTION_ROTATE_CW_UNTIL_SIDE_TAPE ||
           action == MOVEMENT_ACTION_ROTATE_CW_UNTIL_FRONT_TAPE ||
           action == MOVEMENT_ACTION_ROTATE_CCW_UNTIL_FRONT_TAPE;
}

}  // namespace

extern "C" void movement_action_controller_set_line_follower_context(
    LineFollowerContext *ctx) {
    g_line_follower_ctx = ctx;
}

extern "C" void movement_action_controller_set_precision_move_context(
    PrecisionMoveContext *ctx) {
    g_precision_move_ctx = ctx;
}

extern "C" void movement_action_controller_notify_locator_contact(
    MovementActionController *controller) {
    if (controller != nullptr &&
        controller->action == MOVEMENT_ACTION_GO_MX_UNTIL_LOCATOR) {
        controller->locator_contact_detected = true;
    }
}

extern "C" void movement_action_controller_notify_solar_panel_contact(
    MovementActionController *controller) {
    if (controller != nullptr &&
        controller->action == MOVEMENT_ACTION_GO_PY_UNTIL_SOLAR_PANEL) {
        controller->solar_panel_contact_detected = true;
    }
}

extern "C" void movement_action_controller_begin_sequence(void) {
    g_planned_pose_valid = false;
    if (g_precision_move_ctx != nullptr &&
        g_precision_move_ctx->sequence_controller != nullptr &&
        g_precision_move_ctx->sequence_controller->pose_tracker != nullptr) {
        sync_planned_pose();
    }
}

float speed_or_default(float speed, float default_speed) {
    return speed > 0.0f ? speed : default_speed;
}

bool precision_action(
    float dx_body,
    float dy_body,
    float dhead_rad,
    const TapeStopSpec *tape_stop_spec = nullptr,
    float speed_mps = 0.0f,
    float omega_rad_s = 0.0f,
    const bool *external_stop_requested = nullptr) {
    if (g_precision_move_ctx == nullptr ||
        g_precision_move_ctx->sequence_controller == nullptr) {
#if ROBOT_MOTION_DIAGNOSTICS
        std::printf(
            "# precision action rejected: missing precision context\n");
#endif
        return false;
    }
    const float vx_mps = speed_mps > 0.0f ? speed_mps : kPrecisionVxMps;
    const float vy_mps = speed_mps > 0.0f ? speed_mps : kPrecisionVyMps;
    const DrivetrainBodyVelocity body_velocity = {
        .vx = dx_body == 0.0f ? 0.0f : std::copysign(vx_mps, dx_body),
        .vy = dy_body == 0.0f ? 0.0f : std::copysign(vy_mps, dy_body),
        .omega = dhead_rad == 0.0f
            ? 0.0f : std::copysign(
                speed_or_default(omega_rad_s, kPrecisionOmegaRadS),
                dhead_rad),
    };
    PrecisionMoveTarget target = {
        .dx_body_m = dx_body,
        .dy_body_m = dy_body,
        .delta_heading_rad = dhead_rad,
        .body_velocity = body_velocity,
    };
    if (tape_stop_spec != nullptr) {
        target.tape_stop_enabled = true;
        target.tape_stop_spec = *tape_stop_spec;
    }
    target.external_stop_requested = external_stop_requested;
    Pose planned_goal = {};
    // Pure rotations do not need a world-position goal. Reusing the planned
    // position after a preceding translation can introduce a small position
    // error and incorrectly send the rotation through the translation phase.
    if (g_planned_pose_valid &&
        (std::fabs(dx_body) > 1.0e-6f || std::fabs(dy_body) > 1.0e-6f)) {
        const float c = std::cos(g_planned_pose.heading_rad);
        const float s = std::sin(g_planned_pose.heading_rad);
        planned_goal = {
            g_planned_pose.x_m + c * dx_body - s * dy_body,
            g_planned_pose.y_m + s * dx_body + c * dy_body,
            wrap_angle(g_planned_pose.heading_rad + dhead_rad),
        };
        target.world_goal_enabled = true;
        target.world_goal = planned_goal;
    }
    const esp_err_t move_error = precision_move(
        g_precision_move_ctx, &target, kPrecisionTimeoutS);
    const bool success = move_error == ESP_OK;
#if ROBOT_MOTION_DIAGNOSTICS
    std::printf(
        "# precision action result: success=%u error=%d dx=%.3f dy=%.3f "
        "dtheta=%.3f\n",
        success ? 1U : 0U,
        (int)move_error,
        dx_body,
        dy_body,
        dhead_rad);
#endif
    if (success &&
        (tape_stop_spec != nullptr || external_stop_requested != nullptr)) {
        sync_planned_pose();
    } else if (success && g_planned_pose_valid) {
        if (std::fabs(dx_body) > 1.0e-6f ||
            std::fabs(dy_body) > 1.0e-6f) {
            g_planned_pose = planned_goal;
        } else {
            g_planned_pose.heading_rad = wrap_angle(
                g_planned_pose.heading_rad + dhead_rad);
        }
    }
    return success;
}

extern "C" esp_err_t movement_action_controller_init(
    MovementActionController *controller,
    MovementAction action,
    float action_value) {
    return movement_action_controller_init_with_speed(
        controller, action, action_value, 0.0f);
}

extern "C" esp_err_t movement_action_controller_init_with_speed(
    MovementActionController *controller,
    MovementAction action,
    float action_value,
    float speed) {
    if (controller == NULL ||
        (unsigned int)action >= (unsigned int)MOVEMENT_ACTION_MAX ||
        !std::isfinite(action_value) ||
        !std::isfinite(speed) || speed < 0.0f ||
        (action_requires_nonnegative_distance(action) && action_value < 0.0f) ||
        (action_requires_positive_sweep(action) && action_value <= 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Controller init
    *controller = MovementActionController{};
    controller->action = action;
    controller->action_value = action_value;
    controller->speed = speed;
    return ESP_OK;
}

extern "C" esp_err_t movement_action_controller_init_general_motion(
    MovementActionController *controller,
    float dx_body_m,
    float dy_body_m,
    float delta_heading_rad) {
    if (controller == nullptr || !std::isfinite(dx_body_m) ||
        !std::isfinite(dy_body_m) || !std::isfinite(delta_heading_rad)) {
        return ESP_ERR_INVALID_ARG;
    }
    *controller = MovementActionController{};
    controller->action = MOVEMENT_ACTION_GENERAL_MOTION;
    controller->dx_body_m = dx_body_m;
    controller->dy_body_m = dy_body_m;
    controller->delta_heading_rad = delta_heading_rad;
    return ESP_OK;
}

extern "C" bool movement_action_controller_update(
    MovementActionController *controller) {
    if (controller == NULL) return false;

    // Movement actions use three controller parameters:
    // action selects the maneuver; action_value is its distance in metres or
    // rotation in degrees (zero when sensor feedback determines the stop).
    // speed is optional in m/s, or rad/s for rotation; zero uses the default.
    switch (controller->action) {
        case MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx,
                    Direction::PX,
                    speed_or_default(controller->speed, kTapeFollowSpeedMps),
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_MX_TAPE_FOLLOW_DISTANCE:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx,
                    Direction::MX,
                    speed_or_default(controller->speed, kTapeFollowSpeedMps),
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_PY_TAPE_FOLLOW_DISTANCE:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx,
                    Direction::PY,
                    speed_or_default(controller->speed, kTapeFollowSpeedMps),
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            return false;
        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx,
                    Direction::PY,
                    speed_or_default(controller->speed, kSideTowerFollowSpeedMps),
                    StopCondition::RISE_ONE, 0.0f,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER_FRONT:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx,
                    Direction::PY,
                    speed_or_default(controller->speed, kSideTowerFollowSpeedMps),
                    StopCondition::RISE_ONE, 0.0f,
                    kTapeFollowTimeoutS, TapeMarkerSensor::FRONT);
            }
            return false;

        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_GAP:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx,
                    Direction::PY,
                    speed_or_default(controller->speed, kTapeFollowSpeedMps),
                    StopCondition::RISE_TWO, 0.0f,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_PX_TAPE_FOLLOW_UNTIL_GAP:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx,
                    Direction::PX,
                    speed_or_default(controller->speed, kTapeFollowSpeedMps),
                    StopCondition::RISE_TWO, 0.0f,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_PX_TAPE_FOLLOW_UNTIL_ALL_CHANNELS_ON:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx,
                    Direction::PX,
                    speed_or_default(controller->speed, kTapeFollowSpeedMps),
                    StopCondition::ALL_CHANNELS_ON, 0.0f,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_HABITAT_FRONT:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx,
                    Direction::PY,
                    speed_or_default(controller->speed, kTapeFollowSpeedMps),
                    StopCondition::RISE_TWO, 0.0f, kTapeFollowTimeoutS,
                    TapeMarkerSensor::FRONT);
            }
            return false;

        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_HABITAT_BACK:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx,
                    Direction::PY,
                    speed_or_default(controller->speed, kTapeFollowSpeedMps),
                    StopCondition::RISE_TWO, 0.0f, kTapeFollowTimeoutS,
                    TapeMarkerSensor::BACK);
            }
            return false;
            
        // use action_value to determine whether to use lateral one or lateral two stop condition
        case MOVEMENT_ACTION_MX_TAPE_STRAFE_ALIGN:
            return false;

        // positive action value is forward, negative is backward
        case MOVEMENT_ACTION_GO_X_DISTANCE:
            return precision_action(
                controller->action_value, 0.0f, 0.0f, nullptr,
                speed_or_default(controller->speed, kPrecisionVxMps));

        // Positive action value is body +y (left); negative is body -y
        // (right), matching DrivetrainBodyVelocity's documented frame.
        case MOVEMENT_ACTION_GO_Y_DISTANCE:
            return precision_action(
                0.0f, controller->action_value, 0.0f, nullptr,
                speed_or_default(controller->speed, kPrecisionVyMps));

        // Positive action value is body -y (right); negative is body +y
        // (left).

        case MOVEMENT_ACTION_GO_MY_DISTANCE:
            return precision_action(
                0.0f, -controller->action_value, 0.0f, nullptr,
                speed_or_default(controller->speed, kPrecisionVyMps));

        case MOVEMENT_ACTION_GO_PX_DISTANCE:
            return precision_action(
                controller->action_value, 0.0f, 0.0f, nullptr,
                speed_or_default(controller->speed, kPrecisionVxMps));

        case MOVEMENT_ACTION_GO_PY_DISTANCE:
            return precision_action(
                0.0f, controller->action_value, 0.0f, nullptr,
                speed_or_default(controller->speed, kPrecisionVyMps));

        case MOVEMENT_ACTION_ROTATE:
            return precision_action(0.0f, 0.0f,
                                    controller->action_value * static_cast<float>(M_PI) / 180.0f,
                                    nullptr, 0.0f,
                                    speed_or_default(controller->speed,
                                                     kPrecisionOmegaRadS));

        case MOVEMENT_ACTION_GENERAL_MOTION:
            return precision_action(
                controller->dx_body_m, controller->dy_body_m,
                controller->delta_heading_rad, nullptr,
                speed_or_default(controller->speed, kPrecisionVxMps));

        // action_value is the cruise speed in m/s; 0 keeps the default speed.
        case MOVEMENT_ACTION_GO_PX_UNTIL_SIDE_TAPE:
            return precision_action(
                kTapeSeekMaxDistanceM, 0.0f, 0.0f, &kSideTapeStopSpec,
                speed_or_default(
                    controller->speed,
                    controller->action_value > 0.0f
                        ? controller->action_value : kPrecisionVxMps));

        case MOVEMENT_ACTION_GO_MX_UNTIL_SIDE_TAPE:
            return precision_action(
                -kTapeSeekMaxDistanceM, 0.0f, 0.0f, &kSideTapeStopSpec,
                speed_or_default(
                    controller->speed,
                    controller->action_value > 0.0f
                        ? controller->action_value : kPrecisionVxMps));

        case MOVEMENT_ACTION_GO_PY_UNTIL_FRONT_TAPE:
            return precision_action(
                0.0f, kTapeSeekMaxDistanceM, 0.0f,
                &kFrontLeftTapeStopSpec,
                speed_or_default(
                    controller->speed,
                    controller->action_value > 0.0f
                        ? controller->action_value : kPrecisionVyMps));

        // action_value is the CW sweep bound in degrees (clamped to the
        // wrap-safe maximum).
        case MOVEMENT_ACTION_ROTATE_CW_UNTIL_SIDE_TAPE:
            return precision_action(
                0.0f, 0.0f,
                -std::fmin(controller->action_value * static_cast<float>(M_PI) / 180.0f,
                           kTapeSeekMaxRotationRad),
                &kSideTapeStopSpec, 0.0f,
                speed_or_default(controller->speed, kPrecisionOmegaRadS));

        // action_value is the CW sweep bound in degrees (clamped to the
        // wrap-safe maximum). The front sensor's right detector leads CW.
        case MOVEMENT_ACTION_ROTATE_CW_UNTIL_FRONT_TAPE:
            return precision_action(
                0.0f, 0.0f,
                -std::fmin(controller->action_value * static_cast<float>(M_PI) / 180.0f,
                           kTapeSeekMaxRotationRad),
                &kFrontRightTapeStopSpec, 0.0f,
                speed_or_default(controller->speed, kPrecisionOmegaRadS));

        // action_value is the CCW sweep bound in degrees. The precision
        // motion layer clamps it just inside +/-180 degrees.
        case MOVEMENT_ACTION_ROTATE_CCW_UNTIL_FRONT_TAPE:
            return precision_action(
                0.0f, 0.0f,
                std::fmin(controller->action_value * static_cast<float>(M_PI) / 180.0f,
                          kTapeSeekMaxRotationRad),
                &kFrontLeftTapeStopSpec, 0.0f,
                speed_or_default(controller->speed, kPrecisionOmegaRadS));

        case MOVEMENT_ACTION_GO_MX_UNTIL_LOCATOR:
            return precision_action(
                -controller->action_value, 0.0f, 0.0f, nullptr,
                speed_or_default(controller->speed, kLocatorApproachSpeedMps),
                0.0f, &controller->locator_contact_detected);

        case MOVEMENT_ACTION_GO_PY_UNTIL_SOLAR_PANEL:
            return precision_action(
                0.0f, controller->action_value, 0.0f, nullptr,
                speed_or_default(controller->speed, kLocatorApproachSpeedMps),
                0.0f, &controller->solar_panel_contact_detected);

        default:
            return false;
    }

    return false;
}
