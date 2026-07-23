/* Defines the claw's left, middle, and right VL53L0X sensors. */
#include "config/tof_config.h"

#include "config/i2c_bus_config.h"
#include "config/pin_map.h"

#define TOF_DEFAULT_ADDRESS 0x29U
#define TOF_SETTINGS(address, pin) {               \
    .device = {                                    \
        .default_i2c_address = TOF_DEFAULT_ADDRESS,\
        .target_i2c_address = (address),            \
        .shutdown_pin = (pin),                      \
    },                                              \
    .profile = VL53L0X_PROFILE_DEFAULT,             \
    .timing_budget_us = 33000U,                     \
    .stop_timeout_ms = 100U,                        \
    .stale_after_ms = 150U,                         \
}

static const VL53L0XConfig sensor_configs[ARM_TOF_COUNT] = {
    [ARM_TOF_LEFT] = TOF_SETTINGS(0x30U, PIN_TOF_LEFT_XSHUT),
    /* The always-on sensor is assigned its address before the others boot. */
    [ARM_TOF_MID] = TOF_SETTINGS(0x31U, GPIO_NUM_NC),
    [ARM_TOF_RIGHT] = TOF_SETTINGS(0x32U, PIN_TOF_RIGHT_XSHUT),
};

_Static_assert(ARM_TOF_COUNT == TOF_SENSOR_COUNT,
               "ToF manager and arm topology must contain three sensors");

const TofManagerConfig ARM_TOF_CONFIG = {
    .bus_config = &SENSOR_I2C_BUS_CONFIG,
    .sensor_configs = sensor_configs,
};
