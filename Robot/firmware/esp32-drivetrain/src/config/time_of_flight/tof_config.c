/* Defines the drivetrain's two side sensors and center grid sensor. */
#include "config/time_of_flight/tof_config.h"

#include "config/communication/i2c_bus_config.h"
#include "config/pin_map.h"

#define TOF_DEFAULT_ADDRESS 0x29U

static const VL53L0XConfig vl53l0x_configs[DRIVETRAIN_VL53L0X_COUNT] = {
    [DRIVETRAIN_TOF_LEFT] = {
        .device = {
            .default_i2c_address = TOF_DEFAULT_ADDRESS,
            .target_i2c_address = 0x30U,
            .shutdown_pin = PIN_TOF_LEFT_XSHUT,
        },
        .profile = VL53L0X_PROFILE_DEFAULT,
        .timing_budget_us = 33000U,
        .stop_timeout_ms = 100U,
        .stale_after_ms = 150U,
    },
    /* Always-on device; the manager assigns this address first. */
    [DRIVETRAIN_TOF_RIGHT] = {
        .device = {
            .default_i2c_address = TOF_DEFAULT_ADDRESS,
            .target_i2c_address = 0x32U,
            .shutdown_pin = GPIO_NUM_NC,
        },
        .profile = VL53L0X_PROFILE_DEFAULT,
        .timing_budget_us = 33000U,
        .stop_timeout_ms = 100U,
        .stale_after_ms = 150U,
    },
};

static const VL53L5CXConfig vl53l5cx_configs[DRIVETRAIN_VL53L5CX_COUNT] = {
    [DRIVETRAIN_TOF_CENTER] = {
        .device = {
            .default_i2c_address = TOF_DEFAULT_ADDRESS,
            .target_i2c_address = 0x33U,
            .shutdown_pin = PIN_TOF_CENTER_LPN,
        },
        .resolution = VL53L5CX_CONFIG_RESOLUTION_4X4,
        .ranging_mode = VL53L5CX_CONFIG_RANGING_MODE_CONTINUOUS,
        .target_order = VL53L5CX_CONFIG_TARGET_CLOSEST,
        .ranging_frequency_hz = 10U,
        .sharpener_percent = 5U,
        .stale_after_ms = 300U,
    },
};

const TofManagerConfig DRIVETRAIN_TOF_CONFIG = {
    .bus_config = &SENSOR_I2C_BUS_CONFIG,
    .vl53l0x_configs = vl53l0x_configs,
    .vl53l0x_count = DRIVETRAIN_VL53L0X_COUNT,
    .vl53l5cx_configs = vl53l5cx_configs,
    .vl53l5cx_count = DRIVETRAIN_VL53L5CX_COUNT,
};
