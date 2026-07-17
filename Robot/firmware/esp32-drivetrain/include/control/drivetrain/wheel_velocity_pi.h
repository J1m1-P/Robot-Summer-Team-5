/* Declares one wheel's feedforward-plus-PI velocity controller. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Defines feedforward, PI, output, integral, and slew limits for one wheel.
typedef struct {
    float kff;
    float kff_offset;
    float kp;
    float ki;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float duty_slew_per_s;
} WheelVelocityPiConfig;

// Stores integral history and the previous output for one wheel controller.
typedef struct {
    float integral;
    float last_duty;
} WheelVelocityPi;

// Checks gains and limits before they enter a running wheel controller.
bool wheel_velocity_pi_config_is_valid(const WheelVelocityPiConfig *config);

// Advances one wheel controller and returns a bounded signed duty output.
esp_err_t wheel_velocity_pi_update(
    WheelVelocityPi *pi,
    const WheelVelocityPiConfig *config,
    float target_mps,
    float measured_mps,
    float dt_s,
    float *duty_out
);

// Clears integral history and the previous slew-limited duty.
void wheel_velocity_pi_reset(WheelVelocityPi *pi);

#ifdef __cplusplus
}
#endif
