/* Declares reusable tape-guided and ordinary drivetrain actions. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

// Opaque here -- only line_following code (a C++ translation unit) knows
// its contents. Lets this header, and everything that includes it
// (robot_sequence_controller, all plain C), stay C.
typedef struct LineFollowerContext LineFollowerContext;
typedef struct PrecisionMoveContext PrecisionMoveContext;

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE = 0,
    MOVEMENT_ACTION_BACK_TAPE_FOLLOW_DISTANCE,
    MOVEMENT_ACTION_LEFT_TAPE_FOLLOW_DISTANCE,
    MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER,
    MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_HABITAT,
    MOVEMENT_ACTION_BACK_TAPE_STRAFE_ALIGN,
    MOVEMENT_ACTION_GO_X_DISTANCE,
    MOVEMENT_ACTION_GO_Y_DISTANCE,
    MOVEMENT_ACTION_ROTATE,
    MOVEMENT_ACTION_GO_FORWARD_UNTIL_SIDE_TAPE,
    MOVEMENT_ACTION_ROTATE_CW_UNTIL_TAPE,
    MOVEMENT_ACTION_GO_BACKWARD_UNTIL_LOCATOR,
    MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_GAP,
    MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_UNTIL_GAP,
    MOVEMENT_ACTION_GO_LEFT_DISTANCE,
    MOVEMENT_ACTION_GO_RIGHT_DISTANCE,
    MOVEMENT_ACTION_GO_FORWARD,
    MOVEMENT_ACTION_GENERAL_MOTION,
    MOVEMENT_ACTION_MAX,
} MovementAction;

typedef struct {
    MovementAction action;
    float action_value;
    bool locator_contact_detected;
    float dx_body_m;
    float dy_body_m;
    float delta_heading_rad;
} MovementActionController;

// Prepares one action.
esp_err_t movement_action_controller_init(
    MovementActionController *controller,
    MovementAction action,
    float action_value);

// Prepares a body-relative translation and rotation action.
esp_err_t movement_action_controller_init_general_motion(
    MovementActionController *controller,
    float dx_body_m,
    float dy_body_m,
    float delta_heading_rad);

// Updates the active action. Every action here blocks until it's fully
// resolved, so the result is always final: true means it succeeded, false
// means it failed (timeout, lost tape, error)
// Must not call again after failure
bool movement_action_controller_update(
    MovementActionController *controller);

// Injects the hardware/pose access the tape-follow-distance actions need.
// Call once during setup, after pose_service is ready. Leaving this unset
// (or passing NULL) makes tape-follow actions fail cleanly.
void movement_action_controller_set_line_follower_context(
    LineFollowerContext *ctx);

// Borrows the single global drivetrain/pose-service context.
void movement_action_controller_set_precision_move_context(
    PrecisionMoveContext *ctx);

// Called when the arm ESP reports that the locator microswitch was pressed.
void movement_action_controller_notify_locator_contact(
    MovementActionController *controller);

// Anchors subsequent precision actions to one world-frame sequence origin.
void movement_action_controller_begin_sequence(void);

#ifdef __cplusplus
}
#endif
