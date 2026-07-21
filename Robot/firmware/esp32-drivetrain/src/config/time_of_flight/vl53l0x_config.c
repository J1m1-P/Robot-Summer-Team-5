#include "config/pin_map.h"
#include "config/time_of_flight/vl53l0x_config.h"

#ifdef __cplusplus 
extern "C" {
#endif

#define VL53L0X_DEFAULT_I2C_ADDRESS     0x29U

#define LEFT_OFFSET                     0x01U
#define MID_OFFSET                      0x02U
#define RIGHT_OFFSET                    0x03U

#define DEFAULT_TIMING_BUDGET_US        33000U
#define DEFAULT_TIMEOUT_MS              100U

const VL53L0XConfig LEFT_VL53L0X_CONFIG = {
    .id = VL53L0X_SENSOR_LEFT, 
    .profile = VL53L0X_PROFILE_DEFAULT, 

    .default_i2c_address = VL53L0X_DEFAULT_I2C_ADDRESS, 
    .target_i2c_address = VL53L0X_DEFAULT_I2C_ADDRESS + LEFT_OFFSET, 
    
    .xshut_pin = PIN_TOF1_XSHUT, 
    .intr_pin = GPIO_NUM_NC, 

    .timing_budget_us = DEFAULT_TIMING_BUDGET_US, 
    .timeout_ms = DEFAULT_TIMEOUT_MS, 
};

const VL53L0XConfig MID_VL53L0X_CONFIG = {
    .id = VL53L0X_SENSOR_MID, 
    .profile = VL53L0X_PROFILE_DEFAULT, 

    .default_i2c_address = VL53L0X_DEFAULT_I2C_ADDRESS, 
    .target_i2c_address = VL53L0X_DEFAULT_I2C_ADDRESS + MID_OFFSET, 
    
    .xshut_pin = PIN_TOF2_XSHUT, 
    .intr_pin = GPIO_NUM_NC, 

    .timing_budget_us = DEFAULT_TIMING_BUDGET_US, 
    .timeout_ms = DEFAULT_TIMEOUT_MS, 
};

const VL53L0XConfig RIGHT_VL53L0X_CONFIG = {
    .id = VL53L0X_SENSOR_RIGHT, 
    .profile = VL53L0X_PROFILE_DEFAULT, 

    .default_i2c_address = VL53L0X_DEFAULT_I2C_ADDRESS, 
    .target_i2c_address = VL53L0X_DEFAULT_I2C_ADDRESS + RIGHT_OFFSET, 
    
    .xshut_pin = GPIO_NUM_NC, 
    .intr_pin = GPIO_NUM_NC, 

    .timing_budget_us = DEFAULT_TIMING_BUDGET_US, 
    .timeout_ms = DEFAULT_TIMEOUT_MS, 
};

#ifdef __cplusplus
}
#endif
