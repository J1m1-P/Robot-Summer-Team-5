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

constexpr int kFrontSensorIndex = 0;
constexpr int kBackSensorIndex = 1;
constexpr int kSideSensorIndex = 2;

struct LineFollowerTuning {
    int64_t control_period_us;
    float front_weights[4];
    float back_weights[4];
    BoundedPidConfig steering_pid;
    float error_ema_alpha;
    float error_deadband;
    float omega_ema_alpha;
    float two_sensor_lateral_gain;
    float max_lateral_correction_mps;
    float search_omega_rad_s;
    float search_sweep_rad;
    float approach_ramp_distance_m;
    float min_ramp_speed_mps;
};

// Values preserved from origin/main. Every tape-following command uses this
// profile except MOVEMENT_ACTION_PX_TAPE_FOLLOW_UNTIL_ALL_CHANNELS_ON.
constexpr LineFollowerTuning kStandardTuning = {
    .control_period_us = 5000,  // 200 Hz
    .front_weights = {-3.0f, -1.0f, 1.0f, 3.0f},
    .back_weights = {-3.0f, -1.0f, 1.0f, 3.0f},
    .steering_pid = {
        .proportional_gain = 0.48f,
        .integral_gain = 0.0f,
        .derivative_gain = 0.15f,
        .integral_limit = 0.0f,
        .correction_min = -2.0f,
        .correction_max = 2.0f,
    },
    .error_ema_alpha = 0.4f,
    .error_deadband = 1.5f,
    .omega_ema_alpha = 0.15f,
    .two_sensor_lateral_gain = 0.25f,
    .max_lateral_correction_mps = 0.25f,
    .search_omega_rad_s = 2.0f,
    .search_sweep_rad = 45.0f * static_cast<float>(M_PI) / 180.0f,
    .approach_ramp_distance_m = 0.03f,
    .min_ramp_speed_mps = 0.1f,
};

// Values preserved from the current new-habitat working copy.
constexpr LineFollowerTuning kHabitatApproachTuning = {
    .control_period_us = 5000,  // 200 Hz
    .front_weights = {-3.0f, -1.0f, 1.0f, 3.0f},
    .back_weights = {-3.0f, -1.0f, 1.0f, 3.0f},
    .steering_pid = {
        .proportional_gain = 5.0f,
        .integral_gain = 0.0f,
        .derivative_gain = 0.15f,
        .integral_limit = 0.0f,
        .correction_min = -2.0f,
        // Positive omega is CCW/left. Keep full right authority while
        // limiting left steering during the habitat approach.
        .correction_max = 0.8f,
    },
    .error_ema_alpha = 1.0f,
    .error_deadband = 0.25f,
    .omega_ema_alpha = 1.0f,
    .two_sensor_lateral_gain = 0.25f,
    .max_lateral_correction_mps = 0.25f,
    .search_omega_rad_s = 2.0f,
    .search_sweep_rad = 45.0f * static_cast<float>(M_PI) / 180.0f,
    .approach_ramp_distance_m = 0.03f,
    .min_ramp_speed_mps = 0.1f,
};

const LineFollowerTuning &GetTuning(TapeFollowTuningProfile profile) {
    return profile == TapeFollowTuningProfile::HABITAT_APPROACH
        ? kHabitatApproachTuning
        : kStandardTuning;
}

float wrap(float a) {
    while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

constexpr float kHeadingWindowStartFromEndM = 0.13f;
constexpr float kHeadingWindowEndFromEndM = 0.03f;
constexpr size_t kHeadingWindowCapacity = 512;
constexpr size_t kHeadingRegressionPointCount = 5;

struct HeadingSample {
    float distance_m;
    float x_m;
    float y_m;
};

struct DirectionInfo {
    int sensor_index;
    const float *weights;
};

DirectionInfo GetDirectionInfo(Direction dir,
                               const LineFollowerTuning &tuning) {
    switch (dir) {
        case Direction::PX:
            return {kFrontSensorIndex, tuning.front_weights};
        case Direction::MX:
            return {kBackSensorIndex, tuning.back_weights};
        case Direction::PY:
            return {kSideSensorIndex, tuning.front_weights};
    }
    return {kFrontSensorIndex, tuning.front_weights};
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

bool RegressionHeading(const HeadingSample *samples, size_t sample_count,
                       float endpoint_distance_m, float *heading_out) {
    if (samples == nullptr || sample_count == 0 || heading_out == nullptr) {
        return false;
    }

    const float window_min = endpoint_distance_m -
        kHeadingWindowStartFromEndM;
    const float window_max = endpoint_distance_m -
        kHeadingWindowEndFromEndM;

    // Pick a few discrete points at even distances through the window. The
    // control loop samples much more densely, but fitting every sample would
    // overweight any period where the loop was delayed or the robot moved
    // slowly.
    HeadingSample points[kHeadingRegressionPointCount] = {};
    for (size_t point = 0; point < kHeadingRegressionPointCount; ++point) {
        const float target_distance = window_min +
            (window_max - window_min) *
            static_cast<float>(point) /
            static_cast<float>(kHeadingRegressionPointCount - 1);
        size_t best_index = sample_count;
        float best_error = INFINITY;
        for (size_t i = 0; i < sample_count; ++i) {
            if (samples[i].distance_m < window_min ||
                samples[i].distance_m > window_max) {
                continue;
            }
            const float error = std::fabs(samples[i].distance_m -
                                          target_distance);
            if (error < best_error) {
                best_error = error;
                best_index = i;
            }
        }
        if (best_index == sample_count) return false;
        points[point] = samples[best_index];
    }

    float mean_x = 0.0f;
    float mean_y = 0.0f;
    for (const HeadingSample &point : points) {
        mean_x += point.x_m;
        mean_y += point.y_m;
    }
    mean_x /= static_cast<float>(kHeadingRegressionPointCount);
    mean_y /= static_cast<float>(kHeadingRegressionPointCount);

    // Principal-axis linear regression: unlike y = mx + b, this also works
    // when the world-frame path is close to vertical.
    float covariance_xx = 0.0f;
    float covariance_xy = 0.0f;
    float covariance_yy = 0.0f;
    for (const HeadingSample &point : points) {
        const float dx = point.x_m - mean_x;
        const float dy = point.y_m - mean_y;
        covariance_xx += dx * dx;
        covariance_xy += dx * dy;
        covariance_yy += dy * dy;
    }
    if (covariance_xx + covariance_yy < 1.0e-10f) {
        return false;
    }

    float heading = 0.5f * std::atan2(
        2.0f * covariance_xy, covariance_xx - covariance_yy);

    // A regression line has no direction. Choose the sign that points from
    // the earliest sampled point toward the latest one.
    const float travel_x = points[kHeadingRegressionPointCount - 1].x_m -
        points[0].x_m;
    const float travel_y = points[kHeadingRegressionPointCount - 1].y_m -
        points[0].y_m;
    if (std::cos(heading) * travel_x + std::sin(heading) * travel_y < 0.0f) {
        heading += static_cast<float>(M_PI);
    }
    *heading_out = wrap(heading);
    return std::isfinite(*heading_out);
}

}  // namespace

static bool follow_tape_impl(
    LineFollowerContext *ctx, Direction dir, float speed_mps,
    StopCondition stop_type, float stop_value, float timeout_s,
    TapeFollowMode mode, TapeMarkerSensor marker_sensor,
    TapeFollowTuningProfile tuning_profile,
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

    const LineFollowerTuning &tuning = GetTuning(tuning_profile);
    const DirectionInfo steer = GetDirectionInfo(dir, tuning);
    const int64_t start_us = esp_timer_get_time();
    FixedRateGate gate = {tuning.control_period_us, start_us};
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
                cumulative_distance_m, current_pose.x_m, current_pose.y_m};
        }

        // This stop condition applies to the sensor module used to follow
        // the current direction: front for PX, back for MX, and side for PY.
        // Check it before issuing another drive command so the stop is
        // immediate on the first sample where all four channels are on tape.
        if (stop_type == StopCondition::ALL_CHANNELS_ON &&
            AllChannelsOn(ctx->sensors[steer.sensor_index])) {
            if (average_heading_rad_out != nullptr &&
                !RegressionHeading(heading_samples, heading_sample_count,
                                   cumulative_distance_m,
                                   average_heading_rad_out)) {
                return Abort(false);
            }
            return Abort(true);
        }

        const bool marker_stop_done = marker_stop_requested &&
            tape_stop_condition_update(&tape_stop, &marker_stop_spec,
                                       ctx->sensors, cumulative_distance_m);

        // Ramp speed down over the configured approach distance before stop.
        float ramp_target_speed_mps = speed_mps;
        const float ramp_floor_mps = fminf(tuning.min_ramp_speed_mps,
                                           speed_mps);
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
            if (remaining_m < tuning.approach_ramp_distance_m) {
                const float t = clamp(
                    remaining_m / tuning.approach_ramp_distance_m,
                    0.0f, 1.0f);
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
                ctx->sensors[kFrontSensorIndex], tuning.front_weights,
                &front_error);
            const bool back_on_tape = ComputeLineError(
                ctx->sensors[kBackSensorIndex], tuning.back_weights,
                &back_error);
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
            filtered_error = tuning.error_ema_alpha * raw_error +
                (1.0f - tuning.error_ema_alpha) * filtered_error;
            // Soft threshold with deadband
            const float error_magnitude = std::fabs(filtered_error);
            const float deadbanded_error =
                error_magnitude < tuning.error_deadband
                ? 0.0f
                : std::copysign(error_magnitude - tuning.error_deadband,
                                filtered_error);
            omega = bounded_pid_update(
                &steering_pid_state, &tuning.steering_pid,
                deadbanded_error, dt_s);
            if (aligned_tape) {
                // Keep the existing angular polarity/gains. The new term is
                // only the sideways correction needed to center both sensors.
                vy = clamp(tuning.two_sensor_lateral_gain * lateral_error,
                           -tuning.max_lateral_correction_mps,
                           tuning.max_lateral_correction_mps);
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
                if (progress_rad >= tuning.search_sweep_rad) {
                    search_reversed = true;
                }
            } else if (progress_rad <= -tuning.search_sweep_rad) {
                return Abort(false);
            }
            // spin toward last known side, then reverse past the start heading
            omega = (search_reversed ? -last_error_sign : last_error_sign) *
                tuning.search_omega_rad_s;
        }
        smoothed_omega = tuning.omega_ema_alpha * omega +
            (1.0f - tuning.omega_ema_alpha) * smoothed_omega;
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
                 TapeFollowMode mode, TapeMarkerSensor marker_sensor,
                 TapeFollowTuningProfile tuning_profile) {
    return follow_tape_impl(ctx, dir, speed_mps, stop_type, stop_value,
                            timeout_s, mode, marker_sensor, tuning_profile,
                            nullptr);
}

bool follow_tape_until_all_channels_average_heading(
    LineFollowerContext *ctx, float speed_mps, float timeout_s,
    float *average_heading_rad_out) {
    if (average_heading_rad_out == nullptr) return false;
    return follow_tape_impl(
        ctx, Direction::PX, speed_mps, StopCondition::ALL_CHANNELS_ON, 0.0f,
        timeout_s, TapeFollowMode::SINGLE_SENSOR,
        TapeMarkerSensor::AUTO, TapeFollowTuningProfile::STANDARD,
        average_heading_rad_out);
}
