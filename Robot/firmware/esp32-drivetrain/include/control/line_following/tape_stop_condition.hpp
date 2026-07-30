#ifndef TAPE_STOP_CONDITION_HPP
#define TAPE_STOP_CONDITION_HPP

#include <stdint.h>

#include "drivers/tape_sensor/tape_sensor_driver.h"

// Sensor-module bits used by TapeStopSpec::sensor_mask:
//   bit 0 = PX/front, bit 1 = MX/back, bit 2 = PY/side.
// TapeStopSpec::channel_mask is a separate mask over channels 0..3 within
// each selected module; it does not select a sensor module.
constexpr uint8_t kTapeStopAllSensors =
    (1U << TAPE_SENSOR_MODULE_COUNT) - 1U;

struct TapeStopSpec {
    uint8_t sensor_mask = 0;  // which sensor modules to inspect
    uint8_t required_sensor_count = 0;
    uint8_t channel_mask = 1U << TAPE_SENSOR_CHANNEL_0;  // channels to inspect
    bool stop_on_gap = false;
    uint8_t gap_edge_channel_mask = 1U << TAPE_SENSOR_CHANNEL_1;
    float max_gap_distance_m = 0.08f;
};

struct TapeStopCondition {
    bool previous_active = false;
    bool tape_seen = false;
    bool gap_seen = false;
    bool triggered = false;
    float first_edge_distance_m = 0.0f;
    float target_distance_m = 0.0f;
};

bool tape_stop_spec_is_valid(const TapeStopSpec *spec);

bool tape_stop_condition_update(
    TapeStopCondition *condition,
    const TapeStopSpec *spec,
    TapeSensor *const sensors[TAPE_SENSOR_MODULE_COUNT],
    float cumulative_distance_m);

// True while a selected marker sensor sees tape on any channel.
bool tape_stop_condition_candidate_active(
    const TapeStopCondition *condition,
    const TapeStopSpec *spec,
    TapeSensor *const sensors[TAPE_SENSOR_MODULE_COUNT]);

bool tape_stop_condition_triggered(const TapeStopCondition *condition);
float tape_stop_condition_target_distance_m(
    const TapeStopCondition *condition);

#endif
