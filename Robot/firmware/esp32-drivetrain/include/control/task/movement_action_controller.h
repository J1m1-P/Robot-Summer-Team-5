/* Declares reusable tape-guided and ordinary drivetrain actions. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

// Opaque here -- only line_following code (a C++ translation unit) knows
// its contents. Lets this header, and everything that includes it
// (robot_sequence_controller, all plain C), stay C.
typedef struct LineFollowerContext LineFollowerContext;

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE = 0,
    MOVEMENT_ACTION_ROTATE_CW_UNTIL_TAPE_ALIGNED,
    MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER,
    MOVEMENT_ACTION_GO_LEFT_DISTANCE,
    MOVEMENT_ACTION_GO_RIGHT_DISTANCE,
    MOVEMENT_ACTION_GO_FORWARD,
    MOVEMENT_ACTION_ROTATE,
    MOVEMENT_ACTION_MAX,
} MovementAction;

typedef struct {
    MovementAction action;
    float action_value;
} MovementActionController;

// Prepares one action. The motion implementations are intentionally placeholders.
esp_err_t movement_action_controller_init(
    MovementActionController *controller,
    MovementAction action,
    float action_value);

// Updates the active action and returns true when it is complete.
bool movement_action_controller_update(
    MovementActionController *controller);

// Injects the hardware/pose access MOVEMENT_ACTION_TAPE_FOLLOW_DISTANCE
// needs. Call once during setup, after pose_service is ready. Leaving this
// unset (or passing NULL) keeps that action on its placeholder
// immediate-complete behavior -- what the native robot-sequence tests rely
// on, since they have no drivetrain/sensors/pose_service to give it.
void movement_action_controller_set_line_follower_context(
    LineFollowerContext *ctx);

#ifdef __cplusplus
}
#endif
