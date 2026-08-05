// Line following: own weighted-channel line error, EMA filter, PD steering.
#include "control/line_following/line_follower.hpp"

#include <Arduino.h>
#include <cmath>

#include "esp_timer.h"
#include "control/motion/stall_escalation.h"
#include "control/pid/bounded_pid.h"
#include <robot_common/fixed_rate_gate.h>
#include <robot_common/math_utils.h>

namespace {

constexpr int64_t kControlPeriodUs = 5000;  // 200 Hz
constexpr int kFrontSensorIndex = 0;
constexpr int kBackSensorIndex = 1;
constexpr int kSideSensorIndex = 2;

constexpr float kFrontWeights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};
constexpr float kBackWeights[4] = {-3.0f, -1.0f, 1.0f, 3.0f};  // MX is mounted mirrored

constexpr float kP = 0.48f;
constexpr float kD = 0.15f;
constexpr float kEmaAlpha = 0.4f;
constexpr float kMaxOmegaRadS = 2.0;

// Two-sensor mode uses the existing tape error units. These limits keep the
// new lateral correction from taking over the requested travel speed.
constexpr float kTwoSensorLateralGain = 0.25f;
constexpr float kMaxLateralCorrectionMps = 0.25f;

// Deadband due to tape width and sensor pitch
constexpr float kErrorDeadband = 1.5f;

// EMA for omega adjustment
constexpr float kOmegaEmaAlpha = 0.15f;

// No integral term
constexpr BoundedPidConfig kSteeringPidConfig = {
    .proportional_gain = kP,
    .integral_gain = 0.0f,
    .derivative_gain = kD,
    .integral_limit = 0.0f,
    .correction_min = -kMaxOmegaRadS,
    .correction_max = kMaxOmegaRadS,
};

constexpr float kSearchOmegaRadS = 2.0f;  // spin rate while hunting for lost tape
// Sweep toward the last known side by this much; if still not found, reverse
// and sweep the same amount past the start heading on the other side before
// giving up.
constexpr float kSearchSweepRad = 45.0f * static_cast<float>(M_PI) / 180.0f;

float wrap(float a) {
    while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

// Ramps speed down over the last stretch before a known DISTANCE stop
// point instead of driving at full speed right up to the abrupt stop.
constexpr float kApproachRampDistanceM = 0.03f;

constexpr float kMinRampSpeedMps = 0.1f;
constexpr float kHeadingWindowStartFromEndM = 0.13f;
constexpr float kHeadingWindowEndFromEndM = 0.03f;
constexpr size_t kHeadingWindowCapacity = 512;

struct HeadingSample {
    float distance_m;
    float heading_rad;
};

struct DirectionInfo {
    int sensor_index;
    const float *weights;
};

DirectionInfo GetDirectionInfo(Direction dir) {
    switch (dir) {
        case Direction::PX: return {kFrontSensorIndex, kFrontWeights};
        case Direction::MX: return {1, kBackWeights};
        case Direction::PY: return {kSideSensorIndex, kFrontWeights};
    }
    return {0, kFrontWeights};
}

TapeStopSpec GetMarkerStopSpec(
    Direction dir,
    StopCondition stop_type,
    TapeMarkerSensor marker_sensor) {
    uint8_t sensor_mask = 0U;
    switch (marker_sensor) {
        case TapeMarkerSensor::FRONT:
            sensor_mask = 1U << kFrontSensorIndex;
            break;
        case TapeMarkerSensor::BACK:
            sensor_mask = 1U << kBackSensorIndex;
            break;
        case TapeMarkerSensor::SIDE:
            sensor_mask = 1U << kSideSensorIndex;
            break;
        case TapeMarkerSensor::AUTO:
            switch (dir) {
                case Direction::PX: sensor_mask = 1U << kSideSensorIndex; break;
                case Direction::PY: sensor_mask = 1U << kBackSensorIndex; break;
                case Direction::MX: break;  // No -y sensor is installed.
            }
            break;
    }
    return {
        .sensor_mask = sensor_mask,
        .required_sensor_count = static_cast<uint8_t>(
            sensor_mask == 0U ? 0U : 1U),
        .channel_mask = 1U << TAPE_SENSOR_CHANNEL_0,
        .stop_on_gap = stop_type == StopCondition::RISE_TWO,
        .gap_edge_channel_mask = 1U << TAPE_SENSOR_CHANNEL_1,
        .max_gap_distance_m = 0.08f,
    };
}

// weighted centroid of active channels, in [-3, 3]; false if line is lost
bool ComputeLineError(const TapeSensor *s, const float w[4], float *error_out) {
    float sum = 0.0f;
    int active = 0;
    if (s->channel_0) { sum += w[0]; active++; }
    if (s->channel_1) { sum += w[1]; active++; }
    if (s->channel_2) { sum += w[2]; active++; }
    if (s->channel_3) { sum += w[3]; active++; }
    if (active == 0) return false;
    *error_out = sum / static_cast<float>(active);
    return true;
}

bool AllChannelsOn(const TapeSensor *sensor) {
    return sensor != nullptr && sensor->channel_0 && sensor->channel_1 &&
           sensor->channel_2 && sensor->channel_3;
}

bool CircularMeanHeading(const HeadingSample *samples, size_t sample_count,
                         float endpoint_distance_m, float *heading_out) {
    if (samples == nullptr || sample_count == 0 || heading_out == nullptr) {
        return false;
    }

    float sine_sum = 0.0f;
    float cosine_sum = 0.0f;
    size_t selected = 0;
    const float window_min = endpoint_distance_m -
        kHeadingWindowStartFromEndM;
    const float window_max = endpoint_distance_m -
        kHeadingWindowEndFromEndM;
    for (size_t i = 0; i < sample_count; ++i) {
        if (samples[i].distance_m < window_min ||
            samples[i].distance_m > window_max) {
            continue;
        }
        sine_sum += std::sin(samples[i].heading_rad);
        cosine_sum += std::cos(samples[i].heading_rad);
        ++selected;
    }
    if (selected == 0 ||
        (std::fabs(sine_sum) < 1.0e-6f &&
         std::fabs(cosine_sum) < 1.0e-6f)) {
        return false;
    }
    *heading_out = std::atan2(sine_sum, cosine_sum);
    return std::isfinite(*heading_out);
}

}  // namespace

static bool follow_tape_impl(
    LineFollowerContext *ctx, Direction dir, float speed_mps,
    StopCondition stop_type, float stop_value, float timeout_s,
    TapeFollowMode mode, TapeMarkerSensor marker_sensor,
    float *average_heading_rad_out) {
    if (ctx == nullptr || ctx->drivetrain == nullptr ||
        ctx->sequence_controller == nullptr ||
        ctx->sequence_controller->pose_tracker == nullptr ||
        ctx->sensors[0] == nullptr || ctx->sensors[1] == nullptr ||
        ctx->sensors[2] == nullptr) {
        return false;
    }
    RobotSequenceController *controller = ctx->sequence_controller;

    auto Abort = [ctx](bool result) {
        const esp_err_t stop_error = drivetrain_stop(ctx->drivetrain);
        return result && stop_error == ESP_OK;
    };

    const DirectionInfo steer = GetDirectionInfo(dir);
    const int64_t start_us = esp_timer_get_time();
    FixedRateGate gate = {kControlPeriodUs, start_us};
    TapeStopCondition tape_stop;
    const bool marker_stop_requested =
        stop_type == StopCondition::RISE_ONE ||
        stop_type == StopCondition::RISE_TWO;
    const TapeStopSpec marker_stop_spec =
        GetMarkerStopSpec(dir, stop_type, marker_sensor);
    float cumulative_distance_m = 0.0f;
    float filtered_error = 0.0f;
    BoundedPidState steering_pid_state = {};
    float last_error_sign = 1.0f;
    bool searching = false;
    bool search_reversed = false;
    float search_start_heading_rad = 0.0f;
    float smoothed_omega = 0.0f;
    Pose previous_pose = pose_tracker_get_pose(controller->pose_tracker);
    HeadingSample heading_samples[kHeadingWindowCapacity] = {};
    size_t heading_sample_count = 0;
    StallEscalation stall = {};

    while (true) {
        const int64_t now_us = esp_timer_get_time();
        const float elapsed_s = static_cast<float>(now_us - start_us) / 1e6f;
        if (elapsed_s >= timeout_s) return Abort(false);

        int64_t dt_us = 0;
        if (!gate.Ready(now_us, &dt_us)) {
            delay(1);
            continue;
        }
        const float dt_s = static_cast<float>(dt_us) / 1e6f;

        if (tape_sensor_driver_read_all(ctx->sensors) != ESP_OK) return Abort(false);
        if (robot_sequence_controller_update(
                ctx->sequence_controller,
                static_cast<uint32_t>(now_us / 1000)) != ESP_OK ||
            !ctx->sequence_controller->running) {
            return Abort(false);
        }
        const Pose current_pose =
            pose_tracker_get_pose(controller->pose_tracker);
        const float step_distance_m = std::hypot(
            current_pose.x_m - previous_pose.x_m,
            current_pose.y_m - previous_pose.y_m);
        cumulative_distance_m += step_distance_m;
        previous_pose = current_pose;
        if (average_heading_rad_out != nullptr) {
            if (heading_sample_count == kHeadingWindowCapacity) {
                for (size_t i = 1; i < heading_sample_count; ++i) {
                    heading_samples[i - 1] = heading_samples[i];
                }
                --heading_sample_count;
            }
            heading_samples[heading_sample_count++] = {
                cumulative_distance_m, current_pose.heading_rad};
        }

        // This stop condition applies to the sensor module used to follow
        // the current direction: front for PX, back for MX, and side for PY.
        // Check it before issuing another drive command so the stop is
        // immediate on the first sample where all four channels are on tape.
        if (stop_type == StopCondition::ALL_CHANNELS_ON &&
            AllChannelsOn(ctx->sensors[steer.sensor_index])) {
            if (average_heading_rad_out != nullptr &&
                !CircularMeanHeading(heading_samples, heading_sample_count,
                                     cumulative_distance_m,
                                     average_heading_rad_out)) {
                return Abort(false);
            }
            return Abort(true);
        }

        const bool marker_stop_done = marker_stop_requested &&
            tape_stop_condition_update(&tape_stop, &marker_stop_spec,
                                       ctx->sensors, cumulative_distance_m);

        // Ramp speed down over the last kApproachRampDistanceM before stop point
        float ramp_target_speed_mps = speed_mps;
        const float ramp_floor_mps = fminf(kMinRampSpeedMps, speed_mps);
        if (stop_type == StopCondition::RISE_TWO &&
            tape_stop_condition_candidate_active(
                &tape_stop, &marker_stop_spec, ctx->sensors)) {
            ramp_target_speed_mps = ramp_floor_mps;
        }
        const bool has_distance_target = stop_type == StopCondition::DISTANCE ||
            (marker_stop_requested &&
             tape_stop_condition_triggered(&tape_stop));
        if (has_distance_target) {
            const float target_m =
                stop_type == StopCondition::DISTANCE ? stop_value :
                tape_stop_condition_target_distance_m(&tape_stop);
            const float remaining_m = target_m - cumulative_distance_m;
            if (remaining_m < kApproachRampDistanceM) {
                const float t = clamp(remaining_m / kApproachRampDistanceM, 0.0f, 1.0f);
                ramp_target_speed_mps = ramp_floor_mps + t * (speed_mps - ramp_floor_mps);
            }
        }

        float raw_error = 0.0f;
        bool on_tape = ComputeLineError(
            ctx->sensors[steer.sensor_index], steer.weights, &raw_error);
        float lateral_error = 0.0f;
        bool aligned_tape = false;
        if (mode == TapeFollowMode::FRONT_BACK_ALIGNED &&
            (dir == Direction::PX || dir == Direction::MX)) {
            float front_error = 0.0f;
            float back_error = 0.0f;
            const bool front_on_tape = ComputeLineError(
                ctx->sensors[kFrontSensorIndex], kFrontWeights, &front_error);
            const bool back_on_tape = ComputeLineError(
                ctx->sensors[kBackSensorIndex], kBackWeights, &back_error);
            aligned_tape = front_on_tape && back_on_tape;
            if (aligned_tape) {
                // The average measures sideways displacement. The difference
                // measures whether the body is skewed across the tape.
                lateral_error = (front_error + back_error) * 0.5f;
                raw_error = (front_error - back_error) * 0.5f;
                on_tape = true;
            }
        }

        float omega;
        float vx = 0.0f, vy = 0.0f;
        if (on_tape) {
            last_error_sign = raw_error >= 0.0f ? 1.0f : -1.0f;
            searching = false;
            search_reversed = false;
            filtered_error = kEmaAlpha * raw_error + (1.0f - kEmaAlpha) * filtered_error;
            // Soft threshold with deadband
            const float error_magnitude = std::fabs(filtered_error);
            const float deadbanded_error = error_magnitude < kErrorDeadband
                ? 0.0f
                : std::copysign(error_magnitude - kErrorDeadband, filtered_error);
            omega = bounded_pid_update(
                &steering_pid_state, &kSteeringPidConfig,
                deadbanded_error, dt_s);
            if (aligned_tape) {
                // Keep the existing angular polarity/gains. The new term is
                // only the sideways correction needed to center both sensors.
                vy = clamp(kTwoSensorLateralGain * lateral_error,
                           -kMaxLateralCorrectionMps,
                           kMaxLateralCorrectionMps);
            }
            switch (dir) {
                case Direction::PX: vx = ramp_target_speed_mps; break;
                case Direction::MX: vx = -ramp_target_speed_mps; break;
                case Direction::PY: vy = ramp_target_speed_mps; break;
            }
        } else {
            if (!searching) {
                searching = true;
                search_start_heading_rad = current_pose.heading_rad;
            }
            // Progress toward last_error_sign's side, normalized so it always
            // grows from 0 while sweeping that way, regardless of the sign.
            const float progress_rad =
                wrap(current_pose.heading_rad - search_start_heading_rad) *
                last_error_sign;
            if (!search_reversed) {
                if (progress_rad >= kSearchSweepRad) search_reversed = true;
            } else if (progress_rad <= -kSearchSweepRad) {
                return Abort(false);
            }
            // spin toward last known side, then reverse past the start heading
            omega = (search_reversed ? -last_error_sign : last_error_sign) *
                kSearchOmegaRadS;
        }
        smoothed_omega = kOmegaEmaAlpha * omega + (1.0f - kOmegaEmaAlpha) * smoothed_omega;
        omega = smoothed_omega;

        const float scale = stall_escalation_update(
            &stall, std::hypot(vx, vy), step_distance_m, dt_s);
        const DrivetrainConfig *drivetrain_config = ctx->drivetrain->config;
        stall_escalation_apply_scale(&vx, &vy, scale,
                                     drivetrain_config->max_vx_mps,
                                     drivetrain_config->max_vy_mps);

        // use corrected velocity with calibration
        const esp_err_t command_error = drivetrain_set_advanced_body_velocity(
            ctx->drivetrain, vx, vy, omega);
        if (command_error != ESP_OK) return Abort(false);

        // Fresh timestamp
        const esp_err_t update_error = drivetrain_update(
            ctx->drivetrain, esp_timer_get_time());
        if (update_error != ESP_OK) return Abort(false);

        bool stop_reached = false;
        switch (stop_type) {
            case StopCondition::TIME_ONLY: stop_reached = elapsed_s >= stop_value; break;
            case StopCondition::DISTANCE: stop_reached = cumulative_distance_m >= stop_value; break;
            case StopCondition::RISE_ONE:
            case StopCondition::RISE_TWO: stop_reached = marker_stop_done; break;
            case StopCondition::ALL_CHANNELS_ON: break;
        }
        if (!stop_reached) continue;

        const bool result = Abort(true);
        return result;
    }
}

bool follow_tape(LineFollowerContext *ctx, Direction dir, float speed_mps,
                 StopCondition stop_type, float stop_value, float timeout_s,
                 TapeFollowMode mode, TapeMarkerSensor marker_sensor) {
    return follow_tape_impl(ctx, dir, speed_mps, stop_type, stop_value,
                            timeout_s, mode, marker_sensor, nullptr);
}

bool follow_tape_until_all_channels_average_heading(
    LineFollowerContext *ctx, float speed_mps, float timeout_s,
    float *average_heading_rad_out) {
    if (average_heading_rad_out == nullptr) return false;
    return follow_tape_impl(
        ctx, Direction::PX, speed_mps, StopCondition::ALL_CHANNELS_ON, 0.0f,
        timeout_s, TapeFollowMode::SINGLE_SENSOR,
        TapeMarkerSensor::AUTO, average_heading_rad_out);
}
