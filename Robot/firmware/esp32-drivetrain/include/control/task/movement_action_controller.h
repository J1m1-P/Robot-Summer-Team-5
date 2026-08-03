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
    MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE = 0,
    MOVEMENT_ACTION_MX_TAPE_FOLLOW_DISTANCE,
    MOVEMENT_ACTION_PY_TAPE_FOLLOW_DISTANCE,
    MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER,
    MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER_FRONT,
    MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_HABITAT_FRONT,
    MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_HABITAT_BACK,
    MOVEMENT_ACTION_MX_TAPE_STRAFE_ALIGN,
    MOVEMENT_ACTION_GO_X_DISTANCE,
    MOVEMENT_ACTION_GO_Y_DISTANCE,
    MOVEMENT_ACTION_ROTATE,
    MOVEMENT_ACTION_GO_PX_UNTIL_SIDE_TAPE,
    MOVEMENT_ACTION_GO_MX_UNTIL_SIDE_TAPE,
    MOVEMENT_ACTION_GO_PY_UNTIL_FRONT_TAPE,
    // Require a positive maximum sweep in degrees.
    MOVEMENT_ACTION_ROTATE_CW_UNTIL_SIDE_TAPE,
    MOVEMENT_ACTION_ROTATE_CW_UNTIL_FRONT_TAPE,
    MOVEMENT_ACTION_ROTATE_CCW_UNTIL_FRONT_TAPE,
    MOVEMENT_ACTION_GO_MX_UNTIL_LOCATOR,
    MOVEMENT_ACTION_GO_PY_UNTIL_SOLAR_PANEL,
    MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_GAP,
    MOVEMENT_ACTION_PX_TAPE_FOLLOW_UNTIL_GAP,
    MOVEMENT_ACTION_GO_PY_DISTANCE,
    MOVEMENT_ACTION_GO_MY_DISTANCE,
    MOVEMENT_ACTION_GO_PX_DISTANCE,
    MOVEMENT_ACTION_GENERAL_MOTION,
    MOVEMENT_ACTION_PX_TAPE_FOLLOW_UNTIL_ALL_CHANNELS_ON,
    MOVEMENT_ACTION_MAX,
} MovementAction;

typedef struct {
    MovementAction action;
    float action_value;
    // Translation speed in m/s, or angular speed in rad/s for rotation.
    float speed;
    bool locator_contact_detected;
    bool solar_panel_contact_detected;
    float dx_body_m;
    float dy_body_m;
    float delta_heading_rad;
} MovementActionController;

// Prepares one action.
esp_err_t movement_action_controller_init(
    MovementActionController *controller,
    MovementAction action,
    float action_value);

// Prepares an action with an optional movement speed. Translation and tape
// actions use m/s; rotation actions use rad/s. Zero selects the default.
esp_err_t movement_action_controller_init_with_speed(
    MovementActionController *controller,
    MovementAction action,
    float action_value,
    float speed_mps);

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
// Call once during setup, after the sequence controller is ready. Leaving this
// (or passing NULL) makes tape-follow actions fail cleanly.
void movement_action_controller_set_line_follower_context(
    LineFollowerContext *ctx);

// Borrows the single global drivetrain/sequence-controller context.
void movement_action_controller_set_precision_move_context(
    PrecisionMoveContext *ctx);

// Called when the arm ESP reports that the locator microswitch was pressed.
void movement_action_controller_notify_locator_contact(
    MovementActionController *controller);

// Called when the arm ESP reports that the solar-panel microswitch was pressed.
void movement_action_controller_notify_solar_panel_contact(
    MovementActionController *controller);

// Anchors subsequent precision actions to one world-frame sequence origin.
void movement_action_controller_begin_sequence(void);

#ifdef __cplusplus
}
#endif
