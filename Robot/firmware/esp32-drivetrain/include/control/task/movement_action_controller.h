/* Declares reusable tape-guided and ordinary drivetrain actions. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOVEMENT_ACTION_TAPE_FOLLOW_DISTANCE = 0,
    MOVEMENT_ACTION_ROTATE_CW_UNTIL_TAPE_ALIGNED,
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

#ifdef __cplusplus
}
#endif
