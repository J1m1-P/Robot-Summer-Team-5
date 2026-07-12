#include "communication/i2c/i2c_bus.h"

#include "freertos/FreeRTOS.h"


// Helper Functions
static bool i2c_address_is_valid(uint8_t address) {
    // I2C uses a 7-bit address
    return address <= 0x7FU;
}

static bool i2c_bus_is_ready(const I2cBus *bus) {
    return  bus != NULL &&
            bus->initialized &&
            bus->config != NULL;
}

static bool i2c_device_is_ready(const I2cDevice *device) {
    return  device != NULL &&
            device->initialized && 
            device->bus != NULL && 
            i2c_bus_is_ready(device->bus);
}

static TickType_t i2c_bus_get_timeout_ticks(const I2cBus *bus) {
    return pdMS_TO_TICKS(bus->config->timeout_ms);
}


// Bus Management
esp_err_t i2c_bus_init(I2cBus *bus, const I2cBusConfig *config) {
    if (bus == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    if (bus->initialized) return ESP_ERR_INVALID_STATE;
    if (config->clock_speed_hz == 0U) return ESP_ERR_INVALID_ARG;

    i2c_config_t driver_config = {0};

    driver_config.mode = I2C_MODE_MASTER;

    driver_config.sda_io_num = (int)config->sda_pin;
    driver_config.scl_io_num = (int)config->scl_pin;

    gpio_pullup_t pullup_mode = config->enable_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    driver_config.sda_pullup_en = pullup_mode;
    driver_config.scl_pullup_en = pullup_mode;

    driver_config.master.clk_speed = config->clock_speed_hz;

    esp_err_t err = i2c_param_config(config->port, &driver_config);
    if (err != ESP_OK) return err; 

    // Master Mode does not require RX or TX software buffers. 
    err = i2c_driver_install(config->port, driver_config.mode, 0, 0, 0);
    if (err != ESP_OK) return err;

    bus->config = config;
    bus->initialized = true;

    return ESP_OK;
}

esp_err_t i2c_bus_deinit(I2cBus *bus) {
    if (bus == NULL) return ESP_ERR_INVALID_ARG;
    if (!i2c_bus_is_ready(bus)) return ESP_ERR_INVALID_STATE;

    esp_err_t err = i2c_driver_delete(bus->config->port);
    if (err != ESP_OK) return err;

    bus->config = NULL;
    bus->initialized = false;

    return ESP_OK;
}

bool i2c_bus_is_initialized(const I2cBus *bus) {
    return i2c_bus_is_ready(bus);
}

esp_err_t i2c_bus_probe(const I2cBus *bus, uint8_t address) {
    if (bus == NULL) return ESP_ERR_INVALID_ARG;
    if (!i2c_bus_is_ready(bus)) return ESP_ERR_INVALID_ARG;
    if (!i2c_address_is_valid(address)) return ESP_ERR_INVALID_ARG;

    /*
     * We manually construct an address-only transaction:
     *
     * START
     * address + write
     * STOP
     *
     * A device at this address should return ACK.
     */

    i2c_cmd_handle_t command = i2c_cmd_link_create();
    if (command == NULL) return ESP_ERR_NO_MEM;

    // Queue start bit
    esp_err_t err = i2c_master_start(command);

    // Queue the device address attched with a write bit, enable acknowledgement bit
    if (err == ESP_OK) {
        err = i2c_master_write_byte(
            command, 
            (uint8_t)((address << 1U) | I2C_MASTER_WRITE), 
            true
        );
    }

    // Queue stop bit
    if (err == ESP_OK) {
        err = i2c_master_stop(command);
    }

    // Flush the queue
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(
            bus->config->port, 
            command, 
            i2c_bus_get_timeout_ticks(bus)
        );
    }

    i2c_cmd_link_delete(command);

    return err;
}

// Device Management
esp_err_t i2c_device_init(I2cDevice *device, I2cBus *bus, uint8_t address) {
    if (device == NULL || bus == NULL) return ESP_ERR_INVALID_ARG;
    if (!i2c_bus_is_ready(bus)) return ESP_ERR_INVALID_STATE;
    if (!i2c_address_is_valid(address)) return ESP_ERR_INVALID_ARG;

    device->bus = bus;
    device->address = address;
    device->initialized = true;

    return ESP_OK;
}

esp_err_t i2c_device_write(const I2cDevice *device, const uint8_t *data, size_t data_size) {
    if (device == NULL) return ESP_ERR_INVALID_ARG;
    if (!i2c_device_is_ready(device)) return ESP_ERR_INVALID_STATE;
    if (data == NULL || data_size == 0U) return ESP_ERR_INVALID_ARG;
    
    return i2c_master_write_to_device(
        device->bus->config->port, 
        device->address, 
        data, 
        data_size, 
        i2c_bus_get_timeout_ticks(device->bus)
    );
}

esp_err_t i2c_device_read(const I2cDevice *device, uint8_t *data, size_t data_size) {
    if (device == NULL) return ESP_ERR_INVALID_ARG;
    if (!i2c_device_is_ready(device)) return ESP_ERR_INVALID_STATE;
    if (data == NULL || data_size == 0U) return ESP_ERR_INVALID_ARG;

    return i2c_master_read_from_device(
        device->bus->config->port, 
        device->address, 
        data, 
        data_size, 
        i2c_bus_get_timeout_ticks(device->bus)
    );
}

esp_err_t i2c_device_write_read(
    const I2cDevice *device, 
    const uint8_t *write_buffer, 
    size_t write_size, 
    uint8_t *read_buffer, 
    size_t read_size
) 
{
    if (device == NULL) return ESP_ERR_INVALID_ARG;
    if (!i2c_device_is_ready(device)) return ESP_ERR_INVALID_STATE;
    if (write_buffer == NULL || write_size == 0U) return ESP_ERR_INVALID_ARG;
    if (read_buffer == NULL || read_size == 0U) return ESP_ERR_INVALID_ARG;

    return i2c_master_write_read_device(
        device->bus->config->port, 
        device->address, 
        write_buffer, 
        write_size, 
        read_buffer, 
        read_size, 
        i2c_bus_get_timeout_ticks(device->bus)
    );
}


