/* Implements debounced task detection using the robot's left tape module. */
#include "sensing/tape_following/tape_task_detection.h"

#include <stddef.h>
#include <string.h>

/* Counts asserted channels in one already-sampled tape module. */
static uint8_t count_active_channels(const TapeSensor *sensor)
{
    return (uint8_t)sensor->channel_0 + (uint8_t)sensor->channel_1 +
           (uint8_t)sensor->channel_2 + (uint8_t)sensor->channel_3;
}

/* Increments a debounce counter without allowing uint8_t rollover. */
static void increment_saturated(uint8_t *counter)
{
    if (*counter < UINT8_MAX) {
        (*counter)++;
    }
}

esp_err_t tape_task_detector_init(TapeTaskDetector *detector,
                                  const TapeTaskDetectorConfig *config)
{
    if (detector == NULL || config == NULL ||
        config->minimum_active_channels == 0 ||
        config->minimum_active_channels > TAPE_SENSOR_CHANNEL_COUNT ||
        config->confirmation_samples == 0 || config->release_samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (detector->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(detector, 0, sizeof(*detector));
    detector->config = config;
    detector->initialized = true;
    return ESP_OK;
}

esp_err_t tape_task_detector_reset(TapeTaskDetector *detector)
{
    if (detector == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!detector->initialized || detector->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    detector->active_sample_count = 0;
    detector->inactive_sample_count = 0;
    detector->detected = false;
    return ESP_OK;
}

esp_err_t tape_task_detector_update(TapeTaskDetector *detector,
                                    const TapeSensor *left_sensor,
                                    TapeTaskDetectionOutput *output)
{
    if (detector == NULL || left_sensor == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!detector->initialized || detector->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(output, 0, sizeof(*output));
    output->active_channel_count = count_active_channels(left_sensor);
    bool task_observed = output->active_channel_count >=
                         detector->config->minimum_active_channels;

    if (!detector->detected) {
        detector->inactive_sample_count = 0;
        if (task_observed) {
            increment_saturated(&detector->active_sample_count);
            if (detector->active_sample_count >=
                detector->config->confirmation_samples) {
                detector->detected = true;
                detector->active_sample_count = 0;
                output->detection_started = true;
            }
        } else {
            detector->active_sample_count = 0;
        }
    } else {
        detector->active_sample_count = 0;
        if (!task_observed) {
            increment_saturated(&detector->inactive_sample_count);
            if (detector->inactive_sample_count >=
                detector->config->release_samples) {
                detector->detected = false;
                detector->inactive_sample_count = 0;
                output->detection_ended = true;
            }
        } else {
            detector->inactive_sample_count = 0;
        }
    }

    output->detected = detector->detected;
    return ESP_OK;
}

