#include "control/line_following/tape_stop_condition.hpp"

#include <cmath>

namespace {

bool sensor_channel_active(const TapeSensor *sensor, uint8_t channel_mask,
                           uint8_t required_channel_count) {
    if (sensor == nullptr) return false;
    const bool channels[TAPE_SENSOR_CHANNEL_COUNT] = {
        sensor->channel_0, sensor->channel_1,
        sensor->channel_2, sensor->channel_3,
    };
    int active_count = 0;
    for (int channel = 0; channel < TAPE_SENSOR_CHANNEL_COUNT; ++channel) {
        if ((channel_mask & (1U << channel)) != 0U && channels[channel]) {
            ++active_count;
        }
    }
    return active_count >= required_channel_count;
}

bool sensor_mask_active(
    const TapeStopSpec *spec,
    TapeSensor *const sensors[TAPE_SENSOR_MODULE_COUNT],
    uint8_t channel_mask) {
    int active_count = 0;
    for (int index = 0; index < TAPE_SENSOR_MODULE_COUNT; ++index) {
        if ((spec->sensor_mask & (1U << index)) != 0U &&
            sensor_channel_active(sensors[index], channel_mask,
                                  spec->required_channel_count)) {
            ++active_count;
        }
    }
    return active_count >= spec->required_sensor_count;
}

}  // namespace

bool tape_stop_spec_is_valid(const TapeStopSpec *spec) {
    if (spec == nullptr || spec->sensor_mask == 0U ||
        (spec->sensor_mask & ~kTapeStopAllSensors) != 0U ||
        spec->required_sensor_count == 0U ||
        spec->required_sensor_count > TAPE_SENSOR_MODULE_COUNT ||
        spec->required_sensor_count >
            __builtin_popcount(static_cast<unsigned>(spec->sensor_mask)) ||
        spec->channel_mask == 0U ||
        (spec->channel_mask & ~((1U << TAPE_SENSOR_CHANNEL_COUNT) - 1U)) != 0U ||
        spec->required_channel_count == 0U ||
        spec->required_channel_count >
            __builtin_popcount(static_cast<unsigned>(spec->channel_mask)) ||
        spec->gap_edge_channel_mask == 0U ||
        (spec->gap_edge_channel_mask &
            ~((1U << TAPE_SENSOR_CHANNEL_COUNT) - 1U)) != 0U ||
        !std::isfinite(spec->max_gap_distance_m) ||
        spec->max_gap_distance_m <= 0.0f) {
        return false;
    }
    return true;
}

bool tape_stop_condition_update(
    TapeStopCondition *condition,
    const TapeStopSpec *spec,
    TapeSensor *const sensors[TAPE_SENSOR_MODULE_COUNT],
    float cumulative_distance_m) {
    if (condition == nullptr || sensors == nullptr ||
        !tape_stop_spec_is_valid(spec) ||
        !std::isfinite(cumulative_distance_m)) {
        return false;
    }

    const bool active = sensor_mask_active(
        spec, sensors, spec->stop_on_gap
            ? spec->gap_edge_channel_mask : spec->channel_mask);

    if (!condition->triggered) {
        if (spec->stop_on_gap) {
            if (condition->tape_seen && cumulative_distance_m -
                    condition->first_edge_distance_m > spec->max_gap_distance_m) {
                condition->tape_seen = false;
                condition->gap_seen = false;
            }
            const bool rising = active && !condition->previous_active;
            if (!condition->tape_seen && rising) {
                condition->tape_seen = true;
                condition->first_edge_distance_m = cumulative_distance_m;
            } else if (condition->tape_seen && !condition->gap_seen && !active) {
                condition->gap_seen = true;
            } else if (condition->tape_seen && condition->gap_seen && rising) {
                const float spacing = cumulative_distance_m -
                    condition->first_edge_distance_m;
                if (spacing <= spec->max_gap_distance_m) {
                    condition->triggered = true;
                    condition->target_distance_m = cumulative_distance_m;
                    condition->previous_active = active;
                    return true;
                }
                condition->first_edge_distance_m = cumulative_distance_m;
                condition->gap_seen = false;
            }
        } else if (active && !condition->previous_active) {
            condition->triggered = true;
            condition->target_distance_m = cumulative_distance_m;
            condition->previous_active = active;
            return true;
        }
        condition->previous_active = active;
        return false;
    }

    return cumulative_distance_m >= condition->target_distance_m;
}

bool tape_stop_condition_candidate_active(
    const TapeStopCondition *condition,
    const TapeStopSpec *spec,
    TapeSensor *const sensors[TAPE_SENSOR_MODULE_COUNT]) {
    if (condition == nullptr || spec == nullptr || sensors == nullptr ||
        !tape_stop_spec_is_valid(spec)) return false;
    if (spec->stop_on_gap) return condition->tape_seen;
    return sensor_mask_active(spec, sensors, spec->channel_mask);
}

bool tape_stop_condition_triggered(const TapeStopCondition *condition) {
    return condition != nullptr && condition->triggered;
}

float tape_stop_condition_target_distance_m(
    const TapeStopCondition *condition) {
    return condition != nullptr ? condition->target_distance_m : 0.0f;
}
