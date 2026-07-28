/* Implements tape-guided and ordinary drivetrain actions. Only
 * MOVEMENT_ACTION_TAPE_FOLLOW_DISTANCE is real; the rest are still
 * placeholders. */
#include "control/task/movement_action_controller.h"

#include <stddef.h>
#include <stdio.h>

#include "control/line_following/line_follower.hpp"

namespace {

constexpr float kTapeFollowSpeedMps = 0.25f;
constexpr float kTapeFollowTimeoutS = 30.0f;

LineFollowerContext *g_line_follower_ctx = nullptr;

}  // namespace

extern "C" void movement_action_controller_set_line_follower_context(
    LineFollowerContext *ctx) {
    g_line_follower_ctx = ctx;
}

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
                    g_line_follower_ctx, Direction::PX, kTapeFollowSpeedMps,
                    StopCondition::DISTANCE, controller->action_value,
                    kTapeFollowTimeoutS);
            }
            printf(
                "# PLACEHOLDER: Tape following for %.1f m\n",
                controller->action_value);
            break;

        case MOVEMENT_ACTION_ROTATE:

            printf("# PLACEHOLDER: Rotate CW until aligned with tape again\n");
            break;

        case MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER:
            if (g_line_follower_ctx != nullptr) {
                return follow_tape(
                    g_line_follower_ctx, Direction::PY, kTapeFollowSpeedMps,
                    StopCondition::LATERAL_ONE, 0.0f, kTapeFollowTimeoutS);
            }
            break;

        case MOVEMENT_ACTION_GO_FORWARD:
            printf(
                "# PLACEHOLDER: Go forward %.1f m\n",
                controller->action_value);
            break;

        // case MOVEMENT_ACTION_ROTATE_CW_UNTIL_TAPE_ALIGNED:
        //     printf(
        //         "# PLACEHOLDER: Rotate %.0f degrees\n",
        //         controller->action_value);
        //     break;

        default:
            return false;
    }

    // Replace this immediate completion when the motion is implemented
    return true;
}
