/*
 * Public interface for a shared ESP32 I2C master bus and its attached devices.
 * Each firmware target supplies its own pins, port, clock, and timeout settings.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Defines the hardware and timing settings used to initialize an I2C master bus.
typedef struct {
    i2c_port_t port;

    gpio_num_t sda_pin;
    gpio_num_t scl_pin;

    uint32_t clock_speed_hz;
    uint32_t timeout_ms;

    bool enable_internal_pullups;
} I2cBusConfig;

// Holds the configuration and initialization state of one I2C bus.
typedef struct {
    const I2cBusConfig *config;
    bool initialized;
} I2cBus;

// Represents one addressed device connected to an initialized I2C bus.
typedef struct {
    I2cBus *bus;
    uint8_t address;
    bool initialized;
} I2cDevice;

// Configures and installs an I2C master bus. The bus must be zero-initialized before its first call.
// Example: I2cBus bus = {0};
esp_err_t i2c_bus_init(I2cBus *bus, const I2cBusConfig *config);

// Removes the I2C driver and clears the bus's runtime state.
esp_err_t i2c_bus_deinit(I2cBus *bus);

// Reports whether the bus has a valid configuration and is initialized.
bool i2c_bus_is_initialized(const I2cBus *bus);

// Sends an address-only transaction to check whether a device acknowledges it.
esp_err_t i2c_bus_probe(const I2cBus *bus, uint8_t address);

// Associates a 7-bit device address with an initialized bus.
esp_err_t i2c_device_init(I2cDevice *device, I2cBus *bus, uint8_t address);

// Writes one or more bytes to an initialized I2C device.
esp_err_t i2c_device_write(const I2cDevice *device, const uint8_t *data, size_t data_size);

// Reads one or more bytes from an initialized I2C device.
esp_err_t i2c_device_read(const I2cDevice *device, uint8_t *data, size_t data_size);

/*
 * Writes a request and then reads a response using a repeated start condition.
 * This is the common transaction for reading sensor registers:
 *
 * START
 * address + write
 * write data
 * REPEATED START
 * address + read
 * read data
 * STOP
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
