/* Declares debounced task detection using the robot's left tape module. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "drivers/tape_sensor/tape_sensor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Defines how broad and persistent a left-sensor observation must be to count
 * as a task marker. Release debounce prevents repeated events on one marker. */
typedef struct {
    uint8_t minimum_active_channels;
    uint8_t confirmation_samples;
    uint8_t release_samples;
} TapeTaskDetectorConfig;

/* Retains debounce counters and the stable task-detection level. */
typedef struct {
    const TapeTaskDetectorConfig *config;
    uint8_t active_sample_count;
    uint8_t inactive_sample_count;
    bool detected;
} TapeTaskDetector;

/* Reports the stable level plus one-update transition events for RobotManager. */
typedef struct {
    uint8_t active_channel_count;
    bool detected;
    bool detection_started;
    bool detection_ended;
} TapeTaskDetectionOutput;

/* Validates configuration and prepares a cleared detector.
 * Zero-initialize the runtime object before its first init call. */
esp_err_t tape_task_detector_init(TapeTaskDetector *detector,
                                  const TapeTaskDetectorConfig *config);

/* Clears accumulated samples and the stable detection level. */
esp_err_t tape_task_detector_reset(TapeTaskDetector *detector);

/* Processes one current reading from the left tape module. */
esp_err_t tape_task_detector_update(TapeTaskDetector *detector,
                                    const TapeSensor *left_sensor,
                                    TapeTaskDetectionOutput *output);

#ifdef __cplusplus
}
#endif

