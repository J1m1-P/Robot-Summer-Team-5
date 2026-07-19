/* Defines all drivetrain tape-sensor and tape-behavior configuration. */
#include "config/tape_following/tape_following_config.h"

#include "config/pin_map.h"

/* Shared multiplexer and tape-module hardware assignments. */
const TapeSensorMuxConfig TAPE_SENSOR_MUX_CONFIG = {
    .channel_select_a_pin = PIN_TF_CHSEL1_PIN,
    .channel_select_b_pin = PIN_TF_CHSEL2_PIN,
};

const TapeSensorDriverConfig FRONT_TAPE_SENSOR_CONFIG = {
    .module_output_pin = PIN_TF_FRONT_INPUT,
};

const TapeSensorDriverConfig BACK_TAPE_SENSOR_CONFIG = {
    .module_output_pin = PIN_TF_BACK_INPUT,
};

const TapeSensorDriverConfig LEFT_TAPE_SENSOR_CONFIG = {
    .module_output_pin = PIN_TF_LEFT_INPUT,
};

/* Guidance-channel weights ordered from the robot's left to right. */
const TapeLineEstimatorConfig FRONT_TAPE_LINE_ESTIMATOR_CONFIG = {
    .channel_weights = {-3.0f, -1.0f, 1.0f, 3.0f},
};

const TapeLineEstimatorConfig BACK_TAPE_LINE_ESTIMATOR_CONFIG = {
    .channel_weights = {-3.0f, -1.0f, 1.0f, 3.0f},
};

/* Conservative initial values. Verify correction polarity and tune the
 * proportional gain at low speed on the assembled robot. */
const TapeFollowerConfig TAPE_FOLLOWER_CONFIG = {
    .front_estimator = &FRONT_TAPE_LINE_ESTIMATOR_CONFIG,
    .back_estimator = &BACK_TAPE_LINE_ESTIMATOR_CONFIG,
    .controller = {
        .proportional_gain = 0.10f,
        .integral_gain = 0.0f,
        .derivative_gain = 0.0f,
        .integral_limit = 1.0f,
        .correction_min = -0.30f,
        .correction_max = 0.30f,
    },
    .heading_gain_s_inv = 2.0f,
    .max_omega_rad_s = 0.80f,
    .max_angular_acceleration_rad_s2 = 1.50f,
    .search_velocity_mps = 0.15f,
    .lost_timeout_s = 0.50f,
    .controller_dt_max_s = 0.05f,
};

/* Require a short, stable left-module observation before reporting a task. */
const TapeTaskDetectorConfig TAPE_TASK_DETECTOR_CONFIG = {
    .minimum_active_channels = 2,
    .confirmation_samples = 3,
    .release_samples = 3,
};

