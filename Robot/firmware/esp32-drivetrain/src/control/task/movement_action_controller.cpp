/* Implements tape-guided and ordinary drivetrain actions. */
#include "control/task/movement_action_controller.h"

#include <cmath>
#include <stddef.h>
#include <stdio.h>

#include "control/line_following/line_follower.hpp"
#ifdef ARDUINO
#include <Arduino.h>
#include "esp_timer.h"
#include "control/motion/translator.hpp"
#include <robot_common/fixed_rate_gate.h>
#endif

namespace {

constexpr float kTapeFollowSpeedMps = 0.25f;
constexpr float kTapeFollowTimeoutS = 30.0f;
#ifdef ARDUINO
constexpr float kLocatorApproachSpeedMps = -0.10f;
constexpr float kLocatorApproachTimeoutS = 15.0f;
constexpr int64_t kLocatorControlPeriodUs = 5000;
#endif

LineFollowerContext *g_line_follower_ctx = nullptr;
#ifdef ARDUINO
PrecisionMoveContext *g_precision_move_ctx = nullptr;
#endif

bool action_requires_nonnegative_distance(MovementAction action) {
    switch (action) {
        case MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE:
        case MOVEMENT_ACTION_BACK_TAPE_FOLLOW_DISTANCE:
        case MOVEMENT_ACTION_LEFT_TAPE_FOLLOW_DISTANCE:
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

#ifdef ARDUINO
bool precision_action(float dx_body, float dy_body, float dhead_rad) {
    if (g_precision_move_ctx == nullptr ||
        g_precision_move_ctx->pose_service == nullptr) {
        return false;
    }
    const PrecisionMoveTarget target = {
        .dx_body_m = dx_body,
        .dy_body_m = dy_body,
        .delta_heading_rad = dhead_rad,
        .body_velocity = {.vx = 0.15f, .vy = 0.0f, .omega = 0.0f},
    };
    return precision_move(g_precision_move_ctx, &target, 15.0f) == ESP_OK;
}

bool drive_backward_until_locator(MovementActionController *controller) {
    if (g_precision_move_ctx == nullptr ||
        g_precision_move_ctx->drivetrain == nullptr ||
        g_precision_move_ctx->pose_service == nullptr) {
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

        // PoseService drains arm UART and delivers the microswitch event.
        if (pose_service_update(
                g_precision_move_ctx->pose_service,
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

extern "C" bool movement_action_controller_update(
    MovementActionController *controller) {
    if (controller == NULL) return false;

    // Decide what to do for this action
    switch (controller->action) {
        case MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape(
                    g_line_follower_ctx, Direction::PX, kTapeFollowSpeedMps,
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            printf(
                "# PLACEHOLDER: Front tape following for %.1f m\n",
                controller->action_value);
            break;

        case MOVEMENT_ACTION_BACK_TAPE_FOLLOW_DISTANCE:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape(
                    g_line_follower_ctx, Direction::MX, kTapeFollowSpeedMps,
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            printf(
                "# PLACEHOLDER: Back tape following for %.1f m\n",
                controller->action_value);
            break;

        case MOVEMENT_ACTION_LEFT_TAPE_FOLLOW_DISTANCE:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape(
                    g_line_follower_ctx, Direction::PY, kTapeFollowSpeedMps,
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            printf(
                "# PLACEHOLDER: Left tape following for %.1f m\n",
                controller->action_value);
            break;
            
        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape(
                    g_line_follower_ctx, Direction::PY, kTapeFollowSpeedMps,
                    StopCondition::LATERAL_ONE, 0.0f, kTapeFollowTimeoutS);
            }
            break;

        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_HABITAT:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape(
                    g_line_follower_ctx, Direction::PY, kTapeFollowSpeedMps,
                    StopCondition::LATERAL_TWO, 0.0f, kTapeFollowTimeoutS);
            }
            break;
            
        // use action_value to determine whether to use lateral one or lateral two stop condition
        case MOVEMENT_ACTION_BACK_TAPE_STRAFE_ALIGN:
            if (g_line_follower_ctx != nullptr) {
                printf("# PLACEHOLDER: Back tape strafe align\n");
            }
            break;

        // positive action value is forward, negative is backward
        case MOVEMENT_ACTION_GO_X_DISTANCE:
#ifdef ARDUINO
            return precision_action(controller->action_value, 0.0f, 0.0f);
#else
            printf("# PLACEHOLDER: Go forward %.1f m\n", controller->action_value);
#endif
            break;

        // positive action value is right, negative is left
        case MOVEMENT_ACTION_GO_Y_DISTANCE:
#ifdef ARDUINO
            return precision_action(0.0f, controller->action_value, 0.0f);
#else
            printf("# PLACEHOLDER: Go left %.1f m\n", controller->action_value);
#endif
            break;

        // positive action is counterclockwise, negative is clockwise
        case MOVEMENT_ACTION_ROTATE:
#ifdef ARDUINO
            return precision_action(0.0f, 0.0f,
                                    controller->action_value * static_cast<float>(M_PI) / 180.0f);
#else
            printf("# PLACEHOLDER: Rotate %.0f degrees\n", controller->action_value);
#endif
            break;
            
        case MOVEMENT_ACTION_GO_FORWARD_UNTIL_SIDE_TAPE:
#ifdef ARDUINO
            printf("placeholder");
#else
            printf("# PLACEHOLDER: Go forward until side tape\n");
#endif
            break;

        case MOVEMENT_ACTION_ROTATE_CW_UNTIL_TAPE:
            if (g_line_follower_ctx != nullptr) {
                printf("# PLACEHOLDER: Rotate CW until tape aligned\n");
            }
            break;

        case MOVEMENT_ACTION_GO_BACKWARD_UNTIL_LOCATOR:
#ifdef ARDUINO
            return drive_backward_until_locator(controller);
#else
            printf("# PLACEHOLDER: Go backward until locator\n");
#endif
            break;

        default:
            return false;
    }

    // Replace this immediate completion when the motion is implemented
    return true;
}
