/* Implements tape-guided and ordinary drivetrain actions. */
#include "control/task/movement_action_controller.h"

#include <stddef.h>
#include <stdio.h>

#include "control/line_following/line_follower.hpp"
#ifdef ARDUINO
#include <cmath>
#include "control/motion/translator.hpp"
#endif

namespace {

constexpr float kTapeFollowSpeedMps = 0.25f;
constexpr float kTapeFollowTimeoutS = 12.0f;

LineFollowerContext *g_line_follower_ctx = nullptr;
#ifdef ARDUINO
PrecisionMoveContext *g_precision_move_ctx = nullptr;
#endif

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
#endif

extern "C" esp_err_t movement_action_controller_init(
    MovementActionController *controller,
    MovementAction action,
    float action_value) {
    if (controller == NULL ||
        (unsigned int)action >= (unsigned int)MOVEMENT_ACTION_MAX) {
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
                    g_line_follower_ctx, Direction::PY, kTapeFollowSpeedMps,
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            printf(
                "# PLACEHOLDER: Tape following for %.1f m\n",
                controller->action_value);
            break;

        case MOVEMENT_ACTION_ROTATE_CW_UNTIL_TAPE_ALIGNED:
            printf("# PLACEHOLDER: Rotate CW until aligned with tape again\n");
            break;

        case MOVEMENT_ACTION_GO_FORWARD:
#ifdef ARDUINO
            return precision_action(controller->action_value, 0.0f, 0.0f);
#else
            printf("# PLACEHOLDER: Go forward %.1f m\n", controller->action_value);
#endif
            break;

        case MOVEMENT_ACTION_GO_LEFT_DISTANCE:
#ifdef ARDUINO
            return precision_action(0.0f, controller->action_value, 0.0f);
#else
            printf("# PLACEHOLDER: Go left %.1f m\n", controller->action_value);
#endif
            break;

        case MOVEMENT_ACTION_GO_RIGHT_DISTANCE:
#ifdef ARDUINO
            return precision_action(0.0f, -controller->action_value, 0.0f);
#else
            printf("# PLACEHOLDER: Go right %.1f m\n", controller->action_value);
#endif
            break;

        case MOVEMENT_ACTION_ROTATE:
#ifdef ARDUINO
            return precision_action(0.0f, 0.0f,
                                    controller->action_value * static_cast<float>(M_PI) / 180.0f);
#else
            printf("# PLACEHOLDER: Rotate %.0f degrees\n", controller->action_value);
#endif
            break;

        default:
            return false;
    }

    // Replace this immediate completion when the motion is implemented
    return true;
}
