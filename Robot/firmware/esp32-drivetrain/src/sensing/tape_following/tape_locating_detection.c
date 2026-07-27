/* Implements one-piece/two-piece locating-marker detection along tape. */
#include "sensing/tape_following/tape_locating_detection.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static unsigned char count_active_channels(const TapeSensor *sensor)
{
    return (unsigned char)sensor->channel_0 +
           (unsigned char)sensor->channel_1 +
           (unsigned char)sensor->channel_2 +
           (unsigned char)sensor->channel_3;
}

static void increment_saturated(unsigned char *counter)
{
    if (*counter < 255U) (*counter)++;
}

bool tape_locating_detector_config_is_valid(
    const TapeLocatingDetectorConfig *config)
{
    return config != NULL &&
           config->locating_side < TAPE_LOCATING_SIDE_COUNT &&
           (config->expected_marker == TAPE_LOCATING_MARKER_SINGLE ||
            config->expected_marker == TAPE_LOCATING_MARKER_DOUBLE) &&
           config->minimum_active_channels > 0U &&
           config->minimum_active_channels <= TAPE_SENSOR_CHANNEL_COUNT &&
           config->confirmation_samples > 0U &&
           config->release_samples > 0U &&
           isfinite(config->tape_width_m) && config->tape_width_m > 0.0f &&
           isfinite(config->double_center_distance_m) &&
           config->double_center_distance_m > config->tape_width_m &&
           isfinite(config->spacing_tolerance_m) &&
           config->spacing_tolerance_m >= 0.0f &&
           config->spacing_tolerance_m < config->double_center_distance_m;
}

esp_err_t tape_locating_detector_init(
    TapeLocatingDetector *detector,
    const TapeLocatingDetectorConfig *config)
{
    if (detector == NULL || !tape_locating_detector_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (detector->config != NULL) return ESP_ERR_INVALID_STATE;

    memset(detector, 0, sizeof(*detector));
    detector->config = config;
    return ESP_OK;
}

esp_err_t tape_locating_detector_reset(TapeLocatingDetector *detector)
{
    if (detector == NULL) return ESP_ERR_INVALID_ARG;
    if (detector->config == NULL) return ESP_ERR_INVALID_STATE;

    const TapeLocatingDetectorConfig *config = detector->config;
    memset(detector, 0, sizeof(*detector));
    detector->config = config;
    return ESP_OK;
}

static void emit_marker(TapeLocatingDetector *detector,
                        TapeLocatingMarker marker,
                        float center_progress_m,
                        TapeLocatingDetectionOutput *output)
{
    if (detector->event_latched) return;
    detector->event_latched = true;
    output->marker = marker;
    output->event = true;
    output->marker_center_progress_m = center_progress_m;
}

static void finish_piece(TapeLocatingDetector *detector,
                         TapeLocatingDetectionOutput *output)
{
    const float center_progress_m =
        0.5f * (detector->current_piece_start_m + detector->progress_m);

    if (!detector->waiting_for_second_piece) {
        if (detector->config->expected_marker ==
            TAPE_LOCATING_MARKER_SINGLE) {
            emit_marker(detector, TAPE_LOCATING_MARKER_SINGLE,
                        center_progress_m, output);
            return;
        }
        detector->first_piece_center_m = center_progress_m;
        detector->waiting_for_second_piece = true;
        return;
    }

    const float center_spacing_m =
        center_progress_m - detector->first_piece_center_m;
    const float spacing_error_m = fabsf(
        center_spacing_m - detector->config->double_center_distance_m);
    if (spacing_error_m <= detector->config->spacing_tolerance_m) {
        emit_marker(detector, TAPE_LOCATING_MARKER_DOUBLE,
                    0.5f * (detector->first_piece_center_m + center_progress_m),
                    output);
        return;
    }

    /* The previous candidate was not a valid pair. Treat this piece as the
     * beginning of a new candidate so a later pair can still be recognized. */
    detector->first_piece_center_m = center_progress_m;
}

esp_err_t tape_locating_detector_update(
    TapeLocatingDetector *detector,
    const TapeSensor *sensor,
    float along_tape_delta_m,
    TapeLocatingDetectionOutput *output)
{
    if (detector == NULL || sensor == NULL || output == NULL ||
        !isfinite(along_tape_delta_m)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (detector->config == NULL) return ESP_ERR_INVALID_STATE;

    memset(output, 0, sizeof(*output));
    detector->progress_m += along_tape_delta_m;
    output->active_channel_count = count_active_channels(sensor);

    const bool observed = output->active_channel_count >=
                          detector->config->minimum_active_channels;
    if (!detector->piece_active) {
        detector->inactive_sample_count = 0;
        if (observed) {
            increment_saturated(&detector->active_sample_count);
            if (detector->active_sample_count >=
                detector->config->confirmation_samples) {
                detector->piece_active = true;
                detector->active_sample_count = 0;
                detector->current_piece_start_m = detector->progress_m;
            }
        } else {
            detector->active_sample_count = 0;
        }

        if (detector->config->expected_marker == TAPE_LOCATING_MARKER_DOUBLE &&
            detector->waiting_for_second_piece &&
            detector->progress_m - detector->first_piece_center_m >
                detector->config->double_center_distance_m +
                    detector->config->spacing_tolerance_m) {
            emit_marker(detector, TAPE_LOCATING_MARKER_SINGLE,
                        detector->first_piece_center_m, output);
        }
    } else {
        detector->active_sample_count = 0;
        if (!observed) {
            increment_saturated(&detector->inactive_sample_count);
            if (detector->inactive_sample_count >=
                detector->config->release_samples) {
                detector->piece_active = false;
                detector->inactive_sample_count = 0;
                finish_piece(detector, output);
            }
        } else {
            detector->inactive_sample_count = 0;
        }
    }

    return ESP_OK;
}
