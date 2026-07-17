/* Declares multiplexed tape sensor hardware initialization and sampling. */
#ifndef TAPE_SENSOR_DRIVER_H
#define TAPE_SENSOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Number of tape sensor modules sampled through the shared channel selector.
#define TAPE_SENSOR_MODULE_COUNT 3

// Identifies the four multiplexer channels available on each tape module.
typedef enum {
    TAPE_SENSOR_CHANNEL_0 = 0,
    TAPE_SENSOR_CHANNEL_1,
    TAPE_SENSOR_CHANNEL_2,
    TAPE_SENSOR_CHANNEL_3,
    TAPE_SENSOR_CHANNEL_COUNT
} TapeSensorChannel;

// Configures the shared multiplexer used by all tape sensor modules.
typedef struct {
    gpio_num_t channel_select_a_pin;
    gpio_num_t channel_select_b_pin;
} TapeSensorMuxConfig;

// Holds the shared multiplexer configuration and initialization state.
typedef struct {
    const TapeSensorMuxConfig *config;
    bool initialized;
} TapeSensorMux;

// Configures one tape sensor module output.
typedef struct {
    gpio_num_t module_output_pin;
} TapeSensorDriverConfig;

// Holds one module's configuration and latest four channel readings.
typedef struct {
    const TapeSensorDriverConfig *config;
    TapeSensorMux *mux;
    bool channel_0;
    bool channel_1;
    bool channel_2;
    bool channel_3;
} TapeSensor;

// Initializes the shared multiplexer pins once for this mux instance.
esp_err_t tape_sensor_mux_init(TapeSensorMux *mux,
                               const TapeSensorMuxConfig *config);

// Initializes one module input and associates it with the shared multiplexer.
esp_err_t tape_sensor_driver_init(TapeSensor *sensor,
                                  const TapeSensorDriverConfig *config,
                                  TapeSensorMux *mux);

// Reads the currently selected channel from one initialized tape module.
bool tape_sensor_driver_is_on_tape(const TapeSensor *sensor);

// Selects and samples every channel for all three tape sensor modules.
esp_err_t tape_sensor_driver_read_all(TapeSensor *sensors[TAPE_SENSOR_MODULE_COUNT]);

// Samples every channel and packs each module's readings into one nibble.
esp_err_t tape_sensor_driver_read_all_raw(
    TapeSensor *sensors[TAPE_SENSOR_MODULE_COUNT],
    uint8_t out_bits[TAPE_SENSOR_MODULE_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* TAPE_SENSOR_DRIVER_H */
