/* Implements the shared off-tape (open-field) motion PID behavior. */
#include "control/drivetrain/off_tape_motion.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

bool off_tape_motion_config_is_valid(const OffTapeMotionConfig *config)
{
    if (config == NULL) {
        return false;
    }

    return bounded_pid_config_is_valid(&config->controller) &&
           isfinite(config->controller_dt_max_s) &&
           config->controller_dt_max_s > 0.0f;
}

esp_err_t off_tape_motion_init(OffTapeMotion *motion,
                               const OffTapeMotionConfig *config)
{
    if (motion == NULL || !off_tape_motion_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (motion->config != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(motion, 0, sizeof(*motion));
    motion->config = config;
    return ESP_OK;
}

esp_err_t off_tape_motion_reset(OffTapeMotion *motion)
{
    if (motion == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (motion->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    bounded_pid_reset(&motion->controller_state);
    return ESP_OK;
}

esp_err_t off_tape_motion_update(OffTapeMotion *motion,
                                 const OffTapeMotionInput *input,
                                 float dt_s,
                                 OffTapeMotionOutput *output)
{
    if (motion == NULL || input == NULL || output == NULL ||
        !isfinite(input->error) || !isfinite(input->travel_velocity_mps) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (motion->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(output, 0, sizeof(*output));

    /* Cap controller dt so a scheduling stall cannot create a large integral
     * or derivative impulse on the next valid measurement. */
    float controller_dt_s = dt_s;
    if (controller_dt_s > motion->config->controller_dt_max_s) {
        controller_dt_s = motion->config->controller_dt_max_s;
        bounded_pid_reset(&motion->controller_state);
    }

    output->requested_velocity.vx = input->travel_velocity_mps;
    output->requested_velocity.vy = bounded_pid_update(
        &motion->controller_state, &motion->config->controller,
        input->error, controller_dt_s);
    output->requested_velocity.omega = 0.0f;
    output->motion_valid = true;
    return ESP_OK;
}
