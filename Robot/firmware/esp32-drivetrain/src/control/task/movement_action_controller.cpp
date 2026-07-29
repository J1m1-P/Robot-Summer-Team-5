/* Implements tape-guided and ordinary drivetrain actions. */
#include "control/task/movement_action_controller.h"

#include <cmath>
#include <stddef.h>

#include "control/line_following/line_follower.hpp"
#ifdef ARDUINO
#include <Arduino.h>
#include "esp_timer.h"
#include "control/motion/translator.hpp"
#include <robot_common/fixed_rate_gate.h>
#endif

namespace {

constexpr float kTapeFollowSpeedMps = 0.5f;
constexpr float kTapeFollowTimeoutS = 30.0f;
#ifdef ARDUINO
constexpr float kLocatorApproachSpeedMps = -0.10f;
constexpr float kLocatorApproachTimeoutS = 15.0f;
constexpr int64_t kLocatorControlPeriodUs = 5000;
#endif
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

LineFollowerContext *g_line_follower_ctx = nullptr;
#ifdef ARDUINO
PrecisionMoveContext *g_precision_move_ctx = nullptr;
Pose g_planned_pose = {};
bool g_planned_pose_valid = false;
#endif

float wrap_angle(float angle) {
    while (angle > static_cast<float>(M_PI)) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle <= -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

#ifdef ARDUINO
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
#else
void sync_planned_pose() {}
#endif

bool follow_tape_action(
    LineFollowerContext *context,
    Direction direction,
    float speed_mps,
    StopCondition stop_condition,
    float distance_m,
    float timeout_s) {
    const bool success = follow_tape(
        context, direction, speed_mps, stop_condition, distance_m, timeout_s);
    if (success) sync_planned_pose();
    return success;
}

bool action_requires_nonnegative_distance(MovementAction action) {
    switch (action) {
        case MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE:
        case MOVEMENT_ACTION_BACK_TAPE_FOLLOW_DISTANCE:
        case MOVEMENT_ACTION_LEFT_TAPE_FOLLOW_DISTANCE:
        case MOVEMENT_ACTION_GO_FORWARD_UNTIL_SIDE_TAPE:
        case MOVEMENT_ACTION_ROTATE_CW_UNTIL_SIDE_TAPE:
            return true;
        default:
            return false;
    }
}

}  // namespace

extern "C" void movement_action_controller_set_line_follower_context(
    LineFollowerContext *ctx) {
    g_line_follower_ctx = ctx;
}

extern "C" void movement_action_controller_set_precision_move_context(
    PrecisionMoveContext *ctx) {
#ifdef ARDUINO
    g_precision_move_ctx = ctx;
#else
    (void)ctx;
#endif
}

extern "C" void movement_action_controller_notify_locator_contact(
    MovementActionController *controller) {
    if (controller != nullptr &&
        controller->action == MOVEMENT_ACTION_GO_BACKWARD_UNTIL_LOCATOR) {
        controller->locator_contact_detected = true;
    }
}

extern "C" void movement_action_controller_begin_sequence(void) {
#ifdef ARDUINO
    g_planned_pose_valid = false;
    if (g_precision_move_ctx != nullptr &&
        g_precision_move_ctx->sequence_controller != nullptr &&
        g_precision_move_ctx->sequence_controller->pose_tracker != nullptr) {
        sync_planned_pose();
    }
#endif
}

#ifdef ARDUINO
bool precision_action(
    float dx_body,
    float dy_body,
    float dhead_rad,
    const TapeStopSpec *tape_stop_spec = nullptr,
    float speed_mps = 0.0f) {
    if (g_precision_move_ctx == nullptr ||
        g_precision_move_ctx->sequence_controller == nullptr) {
        return false;
    }
    const float vx_mps = speed_mps > 0.0f ? speed_mps : kPrecisionVxMps;
    const float vy_mps = speed_mps > 0.0f ? speed_mps : kPrecisionVyMps;
    const DrivetrainBodyVelocity body_velocity = {
        .vx = dx_body == 0.0f ? 0.0f : std::copysign(vx_mps, dx_body),
        .vy = dy_body == 0.0f ? 0.0f : std::copysign(vy_mps, dy_body),
        .omega = dhead_rad == 0.0f
            ? 0.0f : std::copysign(kPrecisionOmegaRadS, dhead_rad),
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
    Pose planned_goal = {};
    if (g_planned_pose_valid) {
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
    const bool success =
        precision_move(g_precision_move_ctx, &target, kPrecisionTimeoutS) ==
        ESP_OK;
    if (success && tape_stop_spec != nullptr) {
        sync_planned_pose();
    } else if (success && g_planned_pose_valid) {
        g_planned_pose = planned_goal;
    }
    return success;
}

bool drive_backward_until_locator(MovementActionController *controller) {
    if (g_precision_move_ctx == nullptr ||
        g_precision_move_ctx->drivetrain == nullptr ||
        g_precision_move_ctx->sequence_controller == nullptr) {
        return false;
    }

    const int64_t start_us = esp_timer_get_time();
    FixedRateGate gate = {kLocatorControlPeriodUs, start_us};
    auto stop = [controller]() {
        drivetrain_stop(g_precision_move_ctx->drivetrain);
        return controller->locator_contact_detected;
    };

    while (true) {
        const int64_t now_us = esp_timer_get_time();
        if (controller->locator_contact_detected) return stop();
        if (static_cast<float>(now_us - start_us) / 1.0e6f >=
            kLocatorApproachTimeoutS) {
            return stop();
        }

        int64_t unused_dt_us = 0;
        if (!gate.Ready(now_us, &unused_dt_us)) {
            delay(1);
            continue;
        }

        // Keep communication and pose live during this blocking action.
        if (robot_sequence_controller_update(
                g_precision_move_ctx->sequence_controller,
                static_cast<uint32_t>(now_us / 1000)) != ESP_OK ||
            controller->locator_contact_detected) {
            return stop();
        }

        esp_err_t error = drivetrain_set_advanced_body_velocity(
            g_precision_move_ctx->drivetrain,
            kLocatorApproachSpeedMps,
            0.0f,
            0.0f);
        if (error == ESP_OK) {
            error = drivetrain_update(
                g_precision_move_ctx->drivetrain,
                now_us);
        }
        if (error != ESP_OK) return stop();
    }
}
#endif

extern "C" esp_err_t movement_action_controller_init(
    MovementActionController *controller,
    MovementAction action,
    float action_value) {
    if (controller == NULL ||
        (unsigned int)action >= (unsigned int)MOVEMENT_ACTION_MAX ||
        !std::isfinite(action_value) ||
        (action_requires_nonnegative_distance(action) && action_value < 0.0f)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Controller init
    *controller = MovementActionController{};
    controller->action = action;
    controller->action_value = action_value;
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

    // Decide what to do for this action
    switch (controller->action) {
        case MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx, Direction::PX, kTapeFollowSpeedMps,
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_BACK_TAPE_FOLLOW_DISTANCE:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx, Direction::MX, kTapeFollowSpeedMps,
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_LEFT_TAPE_FOLLOW_DISTANCE:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx, Direction::PY, kTapeFollowSpeedMps,
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            return false;
        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx, Direction::PY, 0.15,
                    StopCondition::LATERAL_ONE, 0.0f,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_GAP:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx, Direction::PY, kTapeFollowSpeedMps,
                    StopCondition::LATERAL_TWO, 0.0f,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_UNTIL_GAP:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx, Direction::PX, kTapeFollowSpeedMps,
                    StopCondition::LATERAL_TWO, 0.0f,
                    kTapeFollowTimeoutS);
            }
            return false;

        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_HABITAT:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape_action(
                    g_line_follower_ctx, Direction::PY, kTapeFollowSpeedMps,
                    StopCondition::LATERAL_TWO, 0.0f, kTapeFollowTimeoutS);
            }
            return false;
            
        // use action_value to determine whether to use lateral one or lateral two stop condition
        case MOVEMENT_ACTION_BACK_TAPE_STRAFE_ALIGN:
            return false;

        // positive action value is forward, negative is backward
        case MOVEMENT_ACTION_GO_X_DISTANCE:
#ifdef ARDUINO
            return precision_action(controller->action_value, 0.0f, 0.0f);
#else
            return false;
#endif
            break;

        // positive action value is right, negative is left
        case MOVEMENT_ACTION_GO_Y_DISTANCE:
#ifdef ARDUINO
            return precision_action(0.0f, controller->action_value, 0.0f);
#else
            return false;
#endif
            break;

        // positive action is counterclockwise, negative is clockwise

        case MOVEMENT_ACTION_GO_RIGHT_DISTANCE:
#ifdef ARDUINO
            return precision_action(0.0f, -controller->action_value, 0.0f);
#else
            return false;
#endif
            break;

        case MOVEMENT_ACTION_GO_FORWARD:
#ifdef ARDUINO
            return precision_action(controller->action_value, 0.0f, 0.0f);
#else
            return false;
#endif
            break;

        case MOVEMENT_ACTION_GO_LEFT_DISTANCE:
#ifdef ARDUINO
            return precision_action(0.0f, controller->action_value, 0.0f);
#else
            return false;
#endif
            break;
        case MOVEMENT_ACTION_ROTATE:
#ifdef ARDUINO
            return precision_action(0.0f, 0.0f,
                                    controller->action_value * static_cast<float>(M_PI) / 180.0f);
#else
            return false;
#endif
            break;

        case MOVEMENT_ACTION_GENERAL_MOTION:
#ifdef ARDUINO
            return precision_action(
                controller->dx_body_m, controller->dy_body_m,
                controller->delta_heading_rad);
#else
            return false;
#endif
            break;

        // action_value is the cruise speed in m/s; 0 keeps the default speed.
        case MOVEMENT_ACTION_GO_FORWARD_UNTIL_SIDE_TAPE:
#ifdef ARDUINO
            return precision_action(
                kTapeSeekMaxDistanceM, 0.0f, 0.0f, &kSideTapeStopSpec,
                controller->action_value);
#else
            return false;
#endif

        // action_value is the CW sweep bound in degrees (clamped to the
        // wrap-safe maximum).
        case MOVEMENT_ACTION_ROTATE_CW_UNTIL_SIDE_TAPE:
#ifdef ARDUINO
            return precision_action(
                0.0f, 0.0f,
                -std::fmin(controller->action_value * static_cast<float>(M_PI) / 180.0f,
                           kTapeSeekMaxRotationRad),
                &kSideTapeStopSpec);
#else
            return false;
#endif

        case MOVEMENT_ACTION_GO_BACKWARD_UNTIL_LOCATOR:
#ifdef ARDUINO
            return drive_backward_until_locator(controller);
#else
            return false;
#endif
            break;

        default:
            return false;
    }

    return false;
}
