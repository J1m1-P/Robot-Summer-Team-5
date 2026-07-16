/* Declares multiplexed tape sensor initialization and channel sampling. */
#ifndef TAPE_FOLLOWING_H
#define TAPE_FOLLOWING_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Number of tape sensor modules sampled through the shared channel selector.
#define TAPE_MODULE_COUNT 3

/* 
 * The chsel are shared hardware, so they are not part of the config.
 * They are defined in pin_map.h and used directly in tape_following.c. 
*/

typedef struct {
    gpio_num_t module_out; /* TF1_OUT, TF2_OUT, or TF3_OUT */
} TapeSensorConfig;

typedef struct {
    const TapeSensorConfig *config;
    bool sensor_0;
    bool sensor_1;
    bool sensor_2;
    bool sensor_3;
} TapeSensor;

// Identifies the four multiplexer channels available on each tape module.
typedef enum {
    TAPE_CH_0 = 0,
    TAPE_CH_1 = 1,
    TAPE_CH_2 = 2,
    TAPE_CH_3 = 3
} TapeChannel;

/* Initialize one sensor module: stores its config, configures its
 * module_out pin as an input, and (once) configures the shared mux
 * select pins as outputs. Call once per module at startup. */
esp_err_t tape_sensor_init(TapeSensor *sensor, const TapeSensorConfig *config);

 /*Low-level function to check if a specific tape sensor module is currently detecting tape.
 * This function does not consider which channel is selected and operates on a single sensor module. 
 * It returns true if the module's output pin indicates tape detection, and false otherwise.
 */
bool pin_is_on_tape(TapeSensor *sensor);

/* Cycles the shared mux through all 4 channels once, and at each step
 * reads all 3 modules, updating each TapeSensor's sensor_0..sensor_3
 * fields. (Outer loop is channel, inner loop is module.)
 * Takes an array of pointers so front/back/left can stay
 * separate named instances while still being read together. */
esp_err_t tape_sensor_read_all_channels(TapeSensor *sensors[TAPE_MODULE_COUNT]);

/* Same as above, but also packs each module's 4 bits into the low
 * nibble of out_bits[m] (bit0 = TAPE_CH_0 ... bit3 = TAPE_CH_3).
 * out_bits[m] is an array of 3 bytes, one for each module. Each byte is the 4 bit module reading condensed into one number.
 */
esp_err_t tape_sensor_read_all_channels_raw(TapeSensor *sensors[TAPE_MODULE_COUNT],
                                             uint8_t out_bits[TAPE_MODULE_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* TAPE_FOLLOWING_H */
