#pragma once 

#include <stdint.h>

#include "driver/gpio.h"

#ifdef __cplusplus 
extern "C" {
#endif

typedef enum {
    VL53L0X_SENSOR_LEFT = 0, 
    VL53L0X_SENSOR_MID, 
    VL53L0X_SENSOR_RIGHT, 
    VL53L0X_SENSOR_COUNT
} VL53L0XSensorId;

typedef enum {
    VL53L0X_PROFILE_DEFAULT = 0,
    VL53L0X_PROFILE_HIGH_SPEED,
    VL53L0X_PROFILE_HIGH_ACCURACY,
    VL53L0X_PROFILE_LONG_RANGE
} VL53L0XProfile;

typedef struct {
    VL53L0XSensorId id;
    VL53L0XProfile profile;

    uint8_t defalut_i2c_address;
    uint8_t target_i2c_address;

    gpio_num_t xshut_pin;
    gpio_num_t intr_pin;
    
    uint32_t timing_budget_us;
    uint32_t timeout_ms;
} VL53L0XConfig;

extern const VL53L0XConfig LEFT_VL53L0X_CONFIG; 
extern const VL53L0XConfig MID_VL53L0X_CONFIG; 
extern const VL53L0XConfig RIGHT_VL53L0X_CONFIG; 

#ifdef __cplusplus
}
#endif