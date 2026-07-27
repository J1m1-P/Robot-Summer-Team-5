/* Implements bounded tape alignment without continuous target hunting. */
#include "control/tape_following/tape_alignment.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool estimate(TapeAlignment *alignment,
                     TapeFollowerSensor sensor,
                     const TapeSensor *sample,
                     float *error)
{
    if (sample == NULL || alignment->config->estimators[sensor] == NULL) {
        return false;
    }
    return tape_line_estimator_compute_error(
        sample, alignment->config->estimators[sensor],
            &alignment->estimator_states[sensor], error);
}

bool tape_alignment_config_is_valid(const TapeAlignmentConfig *config)
{
    if (config == NULL || config->mode >= TAPE_ALIGNMENT_MODE_COUNT ||
        !isfinite(config->correction_speed_mps) ||
        config->correction_speed_mps <= 0.0f ||
        !isfinite(config->error_tolerance) ||
        config->error_tolerance < 0.0f || !isfinite(config->timeout_s) ||
        config->timeout_s <= 0.0f || config->settle_samples == 0) {
        return false;
    }
    const bool longitudinal =
        config->estimators[TAPE_FOLLOWER_FRONT] != NULL &&
        config->estimators[TAPE_FOLLOWER_BACK] != NULL;
    if (!longitudinal) return false;
    if (config->mode == TAPE_ALIGNMENT_L_ALIGN_PY_MX &&
        config->estimators[TAPE_FOLLOWER_SIDE] == NULL) return false;
    return tape_line_estimator_config_is_valid(
               config->estimators[TAPE_FOLLOWER_FRONT]) &&
           tape_line_estimator_config_is_valid(
               config->estimators[TAPE_FOLLOWER_BACK]) &&
           (config->estimators[TAPE_FOLLOWER_SIDE] == NULL ||
            tape_line_estimator_config_is_valid(
                config->estimators[TAPE_FOLLOWER_SIDE]));
}

esp_err_t tape_alignment_init(TapeAlignment *alignment,
                              const TapeAlignmentConfig *config)
{
    if (alignment == NULL || !tape_alignment_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (alignment->config != NULL) return ESP_ERR_INVALID_STATE;
    memset(alignment, 0, sizeof(*alignment));
    alignment->config = config;
    alignment->status = TAPE_ALIGNMENT_IDLE;
    return ESP_OK;
}

esp_err_t tape_alignment_start(TapeAlignment *alignment)
{
    if (alignment == NULL) return ESP_ERR_INVALID_ARG;
    if (alignment->config == NULL || alignment->started) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(alignment->estimator_states, 0, sizeof(alignment->estimator_states));
    alignment->status = TAPE_ALIGNMENT_RUNNING;
    alignment->elapsed_s = 0.0f;
    alignment->settle_count = 0;
    alignment->started = true;
    return ESP_OK;
}

esp_err_t tape_alignment_reset(TapeAlignment *alignment)
{
    if (alignment == NULL) return ESP_ERR_INVALID_ARG;
    if (alignment->config == NULL) return ESP_ERR_INVALID_STATE;
    alignment->status = TAPE_ALIGNMENT_IDLE;
    alignment->elapsed_s = 0.0f;
    alignment->settle_count = 0;
    alignment->started = false;
    memset(alignment->estimator_states, 0, sizeof(alignment->estimator_states));
    return ESP_OK;
}

esp_err_t tape_alignment_update(TapeAlignment *alignment,
                                const TapeAlignmentInput *input,
                                TapeAlignmentOutput *output)
{
    if (alignment == NULL || input == NULL || output == NULL ||
        !isfinite(input->dt_s) || input->dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (alignment->config == NULL || !alignment->started) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(output, 0, sizeof(*output));
    output->front_error = NAN;
    output->back_error = NAN;
    output->side_error = NAN;
    alignment->elapsed_s += input->dt_s;

    const bool front_seen = estimate(alignment, TAPE_FOLLOWER_FRONT,
                                     input->sensors[TAPE_FOLLOWER_FRONT],
                                     &output->front_error);
    const bool back_seen = estimate(alignment, TAPE_FOLLOWER_BACK,
                                    input->sensors[TAPE_FOLLOWER_BACK],
                                    &output->back_error);
    const bool side_seen = estimate(alignment, TAPE_FOLLOWER_SIDE,
                                    input->sensors[TAPE_FOLLOWER_SIDE],
                                    &output->side_error);

    bool close = false;
    if (alignment->config->mode ==
        TAPE_ALIGNMENT_I_ALIGN_LONGITUDINAL) {
        close = front_seen && back_seen &&
                fabsf(output->front_error) <= alignment->config->error_tolerance &&
                fabsf(output->back_error) <= alignment->config->error_tolerance;
        if (!close && front_seen && back_seen) {
            const float error =
                0.5f * (output->front_error + output->back_error);
            output->requested_velocity.vy = copysignf(
                alignment->config->correction_speed_mps, error);
            output->motion_valid = true;
        }
    } else {
        /* In an L alignment the longitudinal leg is already mostly acquired.
         * The side sensor supplies the remaining lateral correction; the back
         * sensor only has to confirm the perpendicular leg is present. */
        close = side_seen && back_seen &&
                fabsf(output->side_error) <= alignment->config->error_tolerance;
        if (!close && side_seen && back_seen) {
            output->requested_velocity.vx = -copysignf(
                alignment->config->correction_speed_mps, output->side_error);
            output->motion_valid = true;
        }
    }

    if (close) {
        alignment->settle_count++;
        output->aligned = true;
        if (alignment->settle_count >= alignment->config->settle_samples) {
            alignment->status = TAPE_ALIGNMENT_COMPLETE;
        }
    } else {
        alignment->settle_count = 0;
    }
    if (alignment->status == TAPE_ALIGNMENT_RUNNING &&
        alignment->elapsed_s >= alignment->config->timeout_s) {
        alignment->status = TAPE_ALIGNMENT_TIMEOUT;
    }
    output->status = alignment->status;
    if (alignment->status != TAPE_ALIGNMENT_RUNNING) {
        memset(&output->requested_velocity, 0,
               sizeof(output->requested_velocity));
        output->motion_valid = false;
    }
    return ESP_OK;
}
