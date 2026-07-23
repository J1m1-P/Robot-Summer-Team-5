/* Declares one-piece/two-piece locating-marker detection along tape. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "drivers/tape_sensor/tape_sensor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TAPE_LOCATING_CW = 0,
    TAPE_LOCATING_CCW,
    TAPE_LOCATING_SIDE_COUNT,
} TapeLocatingSide;

typedef enum {
    TAPE_LOCATING_MARKER_NONE = 0,
    TAPE_LOCATING_MARKER_SINGLE,
    TAPE_LOCATING_MARKER_DOUBLE,
} TapeLocatingMarker;

typedef struct {
    /* Required session choice; retained here so the event is self-describing. */
    TapeLocatingSide locating_side;
    TapeLocatingMarker expected_marker;
    unsigned char minimum_active_channels;
    unsigned char confirmation_samples;
    unsigned char release_samples;
    float tape_width_m;
    float double_center_distance_m;
    float spacing_tolerance_m;
} TapeLocatingDetectorConfig;

typedef struct {
    const TapeLocatingDetectorConfig *config;
    unsigned char active_sample_count;
    unsigned char inactive_sample_count;
    bool piece_active;
    bool waiting_for_second_piece;
    bool event_latched;
    float progress_m;
    float first_piece_center_m;
    float current_piece_start_m;
} TapeLocatingDetector;

typedef struct {
    unsigned char active_channel_count;
    TapeLocatingMarker marker;
    bool event;
    float marker_center_progress_m;
} TapeLocatingDetectionOutput;

bool tape_locating_detector_config_is_valid(
    const TapeLocatingDetectorConfig *config);

esp_err_t tape_locating_detector_init(
    TapeLocatingDetector *detector,
    const TapeLocatingDetectorConfig *config);

esp_err_t tape_locating_detector_reset(TapeLocatingDetector *detector);

/* Processes one locating-sensor sample and the net progress since the prior
 * update. The progress delta is projected onto the session tape axis. */
esp_err_t tape_locating_detector_update(
    TapeLocatingDetector *detector,
    const TapeSensor *sensor,
    float along_tape_delta_m,
    TapeLocatingDetectionOutput *output);

#ifdef __cplusplus
}
#endif
