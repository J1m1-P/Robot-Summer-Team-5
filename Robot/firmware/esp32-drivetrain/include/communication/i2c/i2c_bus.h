#pragma once 

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_port_t port;

    gpio_num_t sda_pin;
    gpio_num_t scl_pin;

    uint32_t clock_speed_hz;
    uint32_t timeout_ms;

    bool enable_internal_pullups;
} I2cBusConfig;

typedef struct {
    const I2cBusConfig *config;
    bool initialized;
} I2cBus;

// This represents a device connected on the i2c bus
typedef struct {
    I2cBus *bus;
    uint8_t address;
    bool initialized;
} I2cDevice;

// Bus Management
// Zero-initialize the runtime object before its first init call:
// I2cBus bus = {0};
esp_err_t i2c_bus_init(I2cBus *bus, const I2cBusConfig *config); 

esp_err_t i2c_bus_deinit(I2cBus *bus);

bool i2c_bus_is_initialized(const I2cBus *bus);

// Checks whether a device acknowledge an address
esp_err_t i2c_bus_probe(const I2cBus *bus, uint8_t address);

// Device management
esp_err_t i2c_device_init(I2cDevice *device, I2cBus *bus, uint8_t address);

// Basic I2C transactions. 
esp_err_t i2c_device_write(const I2cDevice *device, const uint8_t *data, size_t data_size);

esp_err_t i2c_device_read(const I2cDevice *device, uint8_t *data, size_t data_size);

/*
 * Performs:
 *
 * START
 * address + write
 * write data
 * REPEATED START
 * address + read
 * read data
 * STOP
 *
 * This is the most common transaction for reading sensor registers.
 */
esp_err_t i2c_device_write_read(
    const I2cDevice *device,
    const uint8_t *write_data,
    size_t write_size,
    uint8_t *read_data,
    size_t read_size
);

#ifdef __cplusplus
}
#endif
