/* Manually stepped Web Serial diagnostic for the arm ToF sensors. */
#include <Arduino.h>
#include <cstring>

#include "config/tof_config.h"
#include "config/pin_map.h"
#include "driver/gpio.h"
#include "drivers/time_of_flight/vl53l0x_driver.h"
#include "esp_err.h"
#include <robot_common/i2c_bus.h>

static I2cBus bus = {};
static VL53L0X sensors[ARM_TOF_COUNT] = {};
static uint8_t next_step = 0U;
static bool ranging = false;
static uint32_t last_report_ms = 0U;
static String command;

static const char *sensor_name(size_t index)
{
    static const char *names[ARM_TOF_COUNT] = {"left", "middle", "right"};
    return index < ARM_TOF_COUNT ? names[index] : "unknown";
}

static const char *name_for_config(const VL53L0XConfig *config)
{
    for (size_t index = 0U; index < ARM_TOF_COUNT; ++index) {
        if (config == &ARM_TOF_CONFIG.sensor_configs[index]) return sensor_name(index);
    }
    return "unknown";
}

// TEST ONLY: receives lifecycle events compiled out of production firmware.
extern "C" void vl53l0x_test_diagnostic_event(
    const VL53L0XConfig *config, const char *state, uint8_t address,
    esp_err_t error)
{
    Serial.printf("TOF,SENSOR,%s,%s,0x%02X,%ld\n", name_for_config(config),
                  state, address, static_cast<long>(error));
}

static esp_err_t hold_controlled_sensors()
{
    for (size_t index = 0U; index < ARM_TOF_COUNT; ++index) {
        const gpio_num_t pin = ARM_TOF_CONFIG.sensor_configs[index].device.shutdown_pin;
        if (pin == GPIO_NUM_NC) continue;
        const gpio_config_t config = {
            .pin_bit_mask = UINT64_C(1) << static_cast<uint32_t>(pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t error = gpio_config(&config);
        if (error == ESP_OK) error = gpio_set_level(pin, 0U);
        if (error != ESP_OK) return error;
    }
    delay(5);
    return ESP_OK;
}

static void reset_test()
{
    ranging = false;
    for (size_t index = ARM_TOF_COUNT; index > 0U; --index) {
        VL53L0X *sensor = &sensors[index - 1U];
        if (sensor->ranging) vl53l0x_driver_stop(sensor);
    }
    /* Power down controlled devices before restoring the always-on device. */
    const size_t order[ARM_TOF_COUNT] = {ARM_TOF_RIGHT, ARM_TOF_LEFT, ARM_TOF_MID};
    for (size_t position = 0U; position < ARM_TOF_COUNT; ++position) {
        VL53L0X *sensor = &sensors[order[position]];
        if (sensor->initialized) vl53l0x_driver_deinit(sensor);
    }
    if (bus.initialized) i2c_bus_deinit(&bus);
    memset(&bus, 0, sizeof(bus));
    memset(sensors, 0, sizeof(sensors));
    next_step = 0U;
    Serial.println("TOF,RESET,arm");
    for (size_t index = 0U; index < ARM_TOF_COUNT; ++index) {
        Serial.printf("TOF,SENSOR,%s,reset,0x29,0\n", sensor_name(index));
    }
}

static void report_error(const char *stage, esp_err_t error)
{
    Serial.printf("TOF,ERROR,%s,%ld,%s\n", stage, static_cast<long>(error),
                  esp_err_to_name(error));
}

static esp_err_t assign_sensor(size_t index)
{
    const VL53L0XConfig *config = &ARM_TOF_CONFIG.sensor_configs[index];
    Serial.printf("TOF,EVENT,address_begin,vl53l0x,%s,0x%02X,0x%02X,0\n",
                  sensor_name(index), config->device.default_i2c_address,
                  config->device.target_i2c_address);
    esp_err_t error = vl53l0x_driver_init(&sensors[index], &bus, config);
    if (error == ESP_OK) {
        error = i2c_bus_probe(&bus, config->device.target_i2c_address);
    }
    Serial.printf("TOF,EVENT,%s,vl53l0x,%s,0x%02X,0x%02X,%ld\n",
                  error == ESP_OK ? "address_ok" : "address_error",
                  sensor_name(index), config->device.default_i2c_address,
                  config->device.target_i2c_address, static_cast<long>(error));
    if (error == ESP_OK) {
        Serial.printf("TOF,VERIFY,%s,0x%02X,1\n", sensor_name(index),
                      config->device.target_i2c_address);
    }
    return error;
}

static void run_next_step()
{
    esp_err_t error = ESP_OK;
    switch (next_step) {
        case 0:
            Serial.println("TOF,STEP_BEGIN,shutdown");
            error = hold_controlled_sensors();
            if (error == ESP_OK) Serial.println("TOF,EVENT,shutdown_ok,bus,shared,0x00,0x00,0");
            break;
        case 1: {
            Serial.println("TOF,STEP_BEGIN,bus");
            error = i2c_bus_init(&bus, ARM_TOF_CONFIG.bus_config);
            if (error == ESP_OK) {
                Serial.printf("TOF,I2C_LINES,SDA,%d,SCL,%d\n",
                              gpio_get_level(static_cast<gpio_num_t>(PIN_I2C_SDA)),
                              gpio_get_level(static_cast<gpio_num_t>(PIN_I2C_SCL)));
            }
            uint8_t found_address = 0x29U;
            if (error == ESP_OK) {
                error = i2c_bus_probe(&bus, found_address);
                if (error != ESP_OK) {
                    found_address = ARM_TOF_CONFIG.sensor_configs[ARM_TOF_MID]
                                        .device.target_i2c_address;
                    error = i2c_bus_probe(&bus, found_address);
                }
            }
            if (error != ESP_OK && bus.initialized) {
                size_t found_count = 0U;
                for (uint8_t address = 0x08U; address <= 0x77U; ++address) {
                    if (i2c_bus_probe(&bus, address) == ESP_OK) {
                        Serial.printf("TOF,SCAN,0x%02X\n", address);
                        ++found_count;
                    }
                }
                Serial.printf("TOF,SCAN_DONE,%u\n",
                              static_cast<unsigned>(found_count));
            }
            if (error == ESP_OK) {
                Serial.printf("TOF,EVENT,bus_ok,bus,shared,0x%02X,0x%02X,0\n",
                              found_address, found_address);
                Serial.printf("TOF,VERIFY,middle,0x%02X,1\n", found_address);
            }
            break;
        }
        case 2: error = assign_sensor(ARM_TOF_MID); break;
        case 3: error = assign_sensor(ARM_TOF_LEFT); break;
        case 4: error = assign_sensor(ARM_TOF_RIGHT); break;
        case 5:
            Serial.println("TOF,STEP_BEGIN,ranging");
            for (size_t index = 0U; index < ARM_TOF_COUNT; ++index) {
                error = vl53l0x_driver_start(&sensors[index]);
                if (error != ESP_OK) break;
            }
            if (error == ESP_OK) {
                ranging = true;
                Serial.println("TOF,RANGING,arm");
            }
            break;
        default:
            Serial.println("TOF,COMPLETE,arm");
            return;
    }
    if (error != ESP_OK) {
        report_error(next_step == 0U ? "shutdown" : next_step == 1U ? "bus" :
                     next_step == 5U ? "ranging" : "address", error);
        return;
    }
    ++next_step;
    Serial.printf("TOF,NEXT,%u\n", next_step);
}

static void report_distances()
{
    for (size_t index = 0U; index < ARM_TOF_COUNT; ++index) {
        VL53L0XSample sample = {};
        if (vl53l0x_driver_get_sample(&sensors[index], &sample) != ESP_OK) continue;
        const uint8_t address = ARM_TOF_CONFIG.sensor_configs[index].device.target_i2c_address;
        Serial.printf("TOF,DIST,%s,0x%02X,%u,%u,%u\n", sensor_name(index),
                      address, sample.distance_mm, sample.valid ? 1U : 0U,
                      sample.range_status);
    }
}

static void stop_ranging()
{
    esp_err_t first_error = ESP_OK;
    for (size_t index = ARM_TOF_COUNT; index > 0U; --index) {
        VL53L0X *sensor = &sensors[index - 1U];
        if (!sensor->ranging) continue;
        const esp_err_t error = vl53l0x_driver_stop(sensor);
        if (first_error == ESP_OK && error != ESP_OK) first_error = error;
    }
    ranging = false;
    Serial.printf("TOF,STOPPED,arm,%ld\n", static_cast<long>(first_error));
}

static void handle_command(const String &line)
{
    if (line.startsWith("step ")) {
        const long requested_step = line.substring(5).toInt();
        if (requested_step < 0L || requested_step != next_step) {
            Serial.printf("TOF,ERROR,state,%u,expected_step_%u\n",
                          next_step, next_step);
        } else {
            run_next_step();
        }
    } else if (line == "reset" || line == "restart") reset_test();
    else if (line == "stop") stop_ranging();
    else if (line == "identify") {
        Serial.println("TOF,READY,arm");
        Serial.printf("TOF,STATE,arm,%u\n", next_step);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("TOF,READY,arm");
}

void loop()
{
    while (Serial.available()) {
        const char value = static_cast<char>(Serial.read());
        if (value == '\n' || value == '\r') {
            command.trim();
            if (!command.isEmpty()) handle_command(command);
            command = "";
        } else if (command.length() < 32U) command += value;
    }
    if (!ranging) { delay(2); return; }
    for (size_t index = 0U; index < ARM_TOF_COUNT; ++index) {
        const esp_err_t error = vl53l0x_driver_read(&sensors[index]);
        if (error != ESP_OK && error != ESP_ERR_NOT_FINISHED) report_error("poll", error);
    }
    if (millis() - last_report_ms >= 100U) {
        last_report_ms = millis();
        report_distances();
    }
    delay(1);
}
