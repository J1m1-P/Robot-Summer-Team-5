#include "sensors/tape_following_PID.h"
#include <stddef.h>

void tape_pid_state_reset(TapePidState *state)
{
    if (state == NULL) {
        return;
    }
    /*
     * reset all fields to their initial state. 
     * This is important to call at startup and whenever switching which TapeSensor 
     * is driving steering, so that the PID controller does not carry over stale integral or derivative history from a previous sensor.
    */
    state->integral = 0.0f;
    state->prev_error = 0.0f;
    state->last_known_error = 0.0f;
    state->has_prev_error = false;
    state->line_was_present = false;

}

bool tape_pid_compute_error(const TapeSensor *sensor,
                             const TapePidSensorConfig *sensor_cfg,
                             TapePidState *state,
                             float *out_error)
{
    if (sensor == NULL || sensor_cfg == NULL || state == NULL || out_error == NULL) {
        return false;
    }

    /* 
     * Computes the weighted centroid of the tape sensor readings based on the provided weights.
     */
    float weighted_sum = 0.0f;
    int active_count = 0;
    if (sensor->sensor_0) { weighted_sum += sensor_cfg->weights[0]; active_count++; }
    if (sensor->sensor_1) { weighted_sum += sensor_cfg->weights[1]; active_count++; }
    if (sensor->sensor_2) { weighted_sum += sensor_cfg->weights[2]; active_count++; }
    if (sensor->sensor_3) { weighted_sum += sensor_cfg->weights[3]; active_count++; }

    /* 
     * If at least one sensor is active, compute the weighted average error.
     */

    if (active_count > 0) {
        *out_error = weighted_sum / (float)active_count;
        state->last_known_error = *out_error;
        state->line_was_present = true;
        return true;
    }

    /* 
     * If no sensors are active, the line is lost. Use the last known error to determine the fallback error.
     * If the last known error was positive, assume the line is to the right; if negative, assume it's to the left. 
     * This helps maintain a reasonable steering direction even when the line is lost.
     */

    *out_error = (state->last_known_error >= 0.0f)
                    ? sensor_cfg->weights[TAPE_PID_CHANNEL_COUNT - 1]
                    : sensor_cfg->weights[0];
    state->line_was_present = false;
    return false; 
}

float tape_pid_update(TapePidState *state,
                       const TapePidGains *gains,
                       float error,
                       float dt_s)
{
    if (state == NULL || gains == NULL || dt_s <= 0.0f) {
        return 0.0f;
    }

    /* proportional term */
    float p_term = gains->kp * error;

    /*
     * integral term with anti-windup: accumulate the integral of the error over time,
     * but clamp it to the specified integral_limit to prevent excessive accumulation (windup).
     */
    state->integral += error * dt_s;
    if (state->integral > gains->integral_limit) {
        state->integral = gains->integral_limit;
    }  
    if (state->integral < -gains->integral_limit) {
            state->integral = -gains->integral_limit;
        }
    float i_term = gains->ki * state->integral;
    /* 
     * derivative term: compute the rate of change of the error. 
     * If this is the first call after a reset, skip the derivative term to avoid a spike.
     */
    float d_term = 0.0f;
    if (state->has_prev_error) {
        d_term = gains->kd * (error - state->prev_error) / dt_s;
    }
    state->prev_error = error;
    state->has_prev_error = true;
    /* 
     * Combine the P, I, and D terms to compute the final correction value. 
     * Clamp this value to the specified output range to ensure it stays within acceptable bounds.
    */

    float correction = p_term + i_term + d_term;
    if (correction > gains->output_max) {
        correction = gains->output_max;
    }
    if (correction < gains->output_min) {
        correction = gains->output_min;
    }
    return correction; 
}