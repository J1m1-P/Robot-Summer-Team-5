/* Manually stepped Web Serial diagnostic for the drivetrain ToF sensors. */
#include <Arduino.h>
#include <cstring>

#include "config/time_of_flight/tof_config.h"
#include "driver/gpio.h"
#include "drivers/time_of_flight/vl53l0x_driver.h"
#include "drivers/time_of_flight/vl53l5cx_driver.h"
#include "esp_err.h"
#include <robot_common/i2c_bus.h>

static I2cBus bus = {};
static VL53L0X side_sensors[DRIVETRAIN_VL53L0X_COUNT] = {};
static VL53L5CX center_sensor = {};
static uint8_t next_step = 0U;
static bool ranging = false;
static uint32_t last_report_ms = 0U;
static String command;

static const char *side_name(size_t index)
{
    static const char *names[DRIVETRAIN_VL53L0X_COUNT] = {"left", "right"};
    return index < DRIVETRAIN_VL53L0X_COUNT ? names[index] : "unknown";
}

static const char *side_name_for_config(const VL53L0XConfig *config)
{
    for (size_t index = 0U; index < DRIVETRAIN_VL53L0X_COUNT; ++index) {
        if (config == &DRIVETRAIN_TOF_CONFIG.vl53l0x_configs[index]) {
            return side_name(index);
        }
    }
    return "unknown";
}

// TEST ONLY: receives lifecycle events compiled out of production firmware.
extern "C" void vl53l0x_test_diagnostic_event(
    const VL53L0XConfig *config, const char *state, uint8_t address,
    esp_err_t error)
{
    Serial.printf("TOF,SENSOR,%s,%s,0x%02X,%ld\n",
                  side_name_for_config(config), state, address,
                  static_cast<long>(error));
}

// TEST ONLY: receives lifecycle events compiled out of production firmware.
extern "C" void vl53l5cx_test_diagnostic_event(
    const VL53L5CXConfig *, const char *state, uint8_t address,
    esp_err_t error)
{
    Serial.printf("TOF,SENSOR,center,%s,0x%02X,%ld\n", state, address,
                  static_cast<long>(error));
}

static esp_err_t hold_pin(gpio_num_t pin)
{
    const gpio_config_t config = {
        .pin_bit_mask = UINT64_C(1) << static_cast<uint32_t>(pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&config);
    return error == ESP_OK ? gpio_set_level(pin, 0U) : error;
}

static esp_err_t hold_controlled_sensors()
{
    esp_err_t error = hold_pin(
        DRIVETRAIN_TOF_CONFIG.vl53l0x_configs[DRIVETRAIN_TOF_LEFT]
            .device.shutdown_pin);
    if (error == ESP_OK) {
        error = hold_pin(DRIVETRAIN_TOF_CONFIG.vl53l5cx_configs[0]
                             .device.shutdown_pin);
    }
    delay(5);
    return error;
}

static void reset_test()
{
    ranging = false;
    if (center_sensor.ranging) vl53l5cx_driver_stop(&center_sensor);
    for (size_t index = 0U; index < DRIVETRAIN_VL53L0X_COUNT; ++index) {
        if (side_sensors[index].ranging) vl53l0x_driver_stop(&side_sensors[index]);
    }
    /* Power down controlled devices before restoring the always-on right sensor. */
    if (center_sensor.initialized) vl53l5cx_driver_deinit(&center_sensor);
    if (side_sensors[DRIVETRAIN_TOF_LEFT].initialized) {
        vl53l0x_driver_deinit(&side_sensors[DRIVETRAIN_TOF_LEFT]);
    }
    if (side_sensors[DRIVETRAIN_TOF_RIGHT].initialized) {
        vl53l0x_driver_deinit(&side_sensors[DRIVETRAIN_TOF_RIGHT]);
    }
    if (bus.initialized) i2c_bus_deinit(&bus);
    memset(&bus, 0, sizeof(bus));
    memset(side_sensors, 0, sizeof(side_sensors));
    memset(&center_sensor, 0, sizeof(center_sensor));
    next_step = 0U;
    Serial.println("TOF,RESET,drivetrain");
    Serial.println("TOF,SENSOR,left,reset,0x29,0");
    Serial.println("TOF,SENSOR,right,reset,0x29,0");
    Serial.println("TOF,SENSOR,center,reset,0x29,0");
}

static void report_error(const char *stage, esp_err_t error)
{
    Serial.printf("TOF,ERROR,%s,%ld,%s\n", stage, static_cast<long>(error),
                  esp_err_to_name(error));
}

static esp_err_t assign_side(size_t index)
{
    const VL53L0XConfig *config = &DRIVETRAIN_TOF_CONFIG.vl53l0x_configs[index];
    Serial.printf("TOF,EVENT,address_begin,vl53l0x,%s,0x%02X,0x%02X,0\n",
                  side_name(index), config->device.default_i2c_address,
                  config->device.target_i2c_address);
    esp_err_t error = vl53l0x_driver_init(&side_sensors[index], &bus, config);
    if (error == ESP_OK) {
        error = i2c_bus_probe(&bus, config->device.target_i2c_address);
    }
    Serial.printf("TOF,EVENT,%s,vl53l0x,%s,0x%02X,0x%02X,%ld\n",
                  error == ESP_OK ? "address_ok" : "address_error", side_name(index),
                  config->device.default_i2c_address,
                  config->device.target_i2c_address, static_cast<long>(error));
    if (error == ESP_OK) {
        Serial.printf("TOF,VERIFY,%s,0x%02X,1\n", side_name(index),
                      config->device.target_i2c_address);
    }
    return error;
}

static esp_err_t assign_center()
{
    const VL53L5CXConfig *config = &DRIVETRAIN_TOF_CONFIG.vl53l5cx_configs[0];
    Serial.printf("TOF,EVENT,address_begin,vl53l5cx,center,0x%02X,0x%02X,0\n",
                  config->device.default_i2c_address,
                  config->device.target_i2c_address);
    esp_err_t error = vl53l5cx_driver_init(&center_sensor, &bus, config);
    if (error == ESP_OK) {
        error = i2c_bus_probe(&bus, config->device.target_i2c_address);
    }
    Serial.printf("TOF,EVENT,%s,vl53l5cx,center,0x%02X,0x%02X,%ld\n",
                  error == ESP_OK ? "address_ok" : "address_error",
                  config->device.default_i2c_address,
                  config->device.target_i2c_address, static_cast<long>(error));
    if (error == ESP_OK) {
        Serial.printf("TOF,VERIFY,center,0x%02X,1\n",
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
            error = i2c_bus_init(&bus, DRIVETRAIN_TOF_CONFIG.bus_config);
            uint8_t found_address = 0x29U;
            if (error == ESP_OK) {
                error = i2c_bus_probe(&bus, found_address);
                if (error != ESP_OK) {
                    found_address = DRIVETRAIN_TOF_CONFIG
                                        .vl53l0x_configs[DRIVETRAIN_TOF_RIGHT]
                                        .device.target_i2c_address;
                    error = i2c_bus_probe(&bus, found_address);
                }
            }
            if (error == ESP_OK) {
                Serial.printf("TOF,EVENT,bus_ok,bus,shared,0x%02X,0x%02X,0\n",
                              found_address, found_address);
                Serial.printf("TOF,VERIFY,right,0x%02X,1\n", found_address);
            }
            break;
        }
        case 2: error = assign_side(DRIVETRAIN_TOF_RIGHT); break;
        case 3: error = assign_side(DRIVETRAIN_TOF_LEFT); break;
        case 4: error = assign_center(); break;
        case 5:
            Serial.println("TOF,STEP_BEGIN,ranging");
            for (size_t index = 0U; index < DRIVETRAIN_VL53L0X_COUNT; ++index) {
                error = vl53l0x_driver_start(&side_sensors[index]);
                if (error != ESP_OK) break;
            }
            if (error == ESP_OK) error = vl53l5cx_driver_start(&center_sensor);
            if (error == ESP_OK) {
                ranging = true;
                Serial.println("TOF,RANGING,drivetrain");
            }
            break;
        default:
            Serial.println("TOF,COMPLETE,drivetrain");
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
    for (size_t index = 0U; index < DRIVETRAIN_VL53L0X_COUNT; ++index) {
        VL53L0XSample sample = {};
        if (vl53l0x_driver_get_sample(&side_sensors[index], &sample) != ESP_OK) continue;
        const uint8_t address = DRIVETRAIN_TOF_CONFIG.vl53l0x_configs[index]
                                    .device.target_i2c_address;
        Serial.printf("TOF,DIST,%s,0x%02X,%u,%u,%u\n", side_name(index), address,
                      sample.distance_mm, sample.valid ? 1U : 0U,
                      sample.range_status);
    }
    if (!center_sensor.has_data) return;
    const uint8_t address = center_sensor.config->device.target_i2c_address;
    Serial.printf("TOF,GRID,center,0x%02X,%u", address,
                  static_cast<unsigned>(center_sensor.config->resolution));
    for (uint8_t zone = 0U; zone < center_sensor.config->resolution; ++zone) {
        int16_t distance = -1;
        vl53l5cx_driver_get_distance_mm(&center_sensor, zone, &distance);
        Serial.printf(",%d", distance);
    }
    Serial.println();
}

static void stop_ranging()
{
    esp_err_t first_error = ESP_OK;
    if (center_sensor.ranging) {
        const esp_err_t error = vl53l5cx_driver_stop(&center_sensor);
        if (error != ESP_OK) first_error = error;
    }
    for (size_t index = DRIVETRAIN_VL53L0X_COUNT; index > 0U; --index) {
        VL53L0X *sensor = &side_sensors[index - 1U];
        if (!sensor->ranging) continue;
        const esp_err_t error = vl53l0x_driver_stop(sensor);
        if (first_error == ESP_OK && error != ESP_OK) first_error = error;
    }
    ranging = false;
    Serial.printf("TOF,STOPPED,drivetrain,%ld\n",
                  static_cast<long>(first_error));
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
        Serial.println("TOF,READY,drivetrain");
        Serial.printf("TOF,STATE,drivetrain,%u\n", next_step);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("TOF,READY,drivetrain");
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
    for (size_t index = 0U; index < DRIVETRAIN_VL53L0X_COUNT; ++index) {
        const esp_err_t error = vl53l0x_driver_read(&side_sensors[index]);
        if (error != ESP_OK && error != ESP_ERR_NOT_FINISHED) report_error("poll", error);
    }
    const esp_err_t center_error = vl53l5cx_driver_read(&center_sensor);
    if (center_error != ESP_OK && center_error != ESP_ERR_NOT_FINISHED) {
        report_error("poll", center_error);
    }
    if (millis() - last_report_ms >= 100U) {
        last_report_ms = millis();
        report_distances();
    }
    delay(1);
}
