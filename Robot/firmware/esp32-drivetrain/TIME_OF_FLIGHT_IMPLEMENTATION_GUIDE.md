# Time-of-Flight Integration Guide

## Purpose of this guide

This document is a teaching scaffold for integrating one VL53L0X and one
VL53L5CX into the drivetrain firmware. It shows what each project file can look
like, why the file exists, and how data should move through the system.

The snippets are deliberately kept in this Markdown file. They are examples to
study and adapt, not code that has silently been added to the production build.
The ST sensor packages and their platform-layer function signatures vary by
version, so every `TODO(ST API)` marker must be matched to the exact vendor
package selected for this project.

The most important design rule is:

> ToF drivers measure the world. A higher-level policy decides what the robot
> should do about those measurements.

Consequently, I2C reads do not belong inside `drivetrain_update()`. A slow or
failed sensor transaction must not delay encoder sampling, wheel PI control, or
the drivetrain command watchdog.

## What already exists

The repository already provides:

- `SENSOR_I2C_BUS_CONFIG`, which selects I2C port 0, SDA pin 8, and SCL pin 9.
- `I2cBus` and `I2cDevice` in the sibling `robot-common` library.
- `PIN_TOF1_XSHUT` and `PIN_TOF2_XSHUT` in `pin_map.h`.
- Separate `include/`, `src/`, `config/`, `drivers/`, `control/`, and `harnesses/`
  layers.

Both sensors normally appear at the same default 7-bit address, `0x29`. They
cannot both be active at that address. The coordinator must hold one inactive
while it assigns the other a unique address.

ST documentation sometimes writes the default address as `0x52`. That is the
8-bit address-byte representation of the same 7-bit address (`0x29 << 1`). The
project's `I2cDevice` stores 7-bit addresses. Check which representation the
selected ST package expects and perform the conversion in exactly one place.

## Recommended finished structure

```text
include/
|-- config/
|   `-- time_of_flight/
|       `-- tof_config.h
|-- drivers/
|   `-- time_of_flight/
|       |-- vl53l0x_driver.h
|       `-- vl53l5cx_driver.h
`-- control/
    `-- time_of_flight/
        |-- tof_manager.h
        `-- obstacle_guard.h             # Optional second phase

src/
|-- config/
|   `-- time_of_flight/
|       `-- tof_config.c
|-- drivers/
|   `-- time_of_flight/
|       |-- vl53l0x_driver.c
|       |-- vl53l5cx_driver.c
|       `-- vl53l5cx_platform.c          # If required by the chosen ST ULD
|-- control/
|   `-- time_of_flight/
|       |-- tof_manager.c
|       `-- obstacle_guard.c             # Optional second phase
`-- harnesses/
    `-- tof_test_main.cpp

lib/
|-- st-vl53l0x-api/                      # Exact version pinned in source control
`-- st-vl53l5cx-uld/

test/
`-- test_obstacle_guard/
    `-- test_obstacle_guard.cpp
```

The vendor packages belong under `lib/`; robot-specific wrappers belong under
`drivers/time_of_flight/`. This stops ST types and version changes from leaking
into the rest of the firmware.

---

## 1. Board pin names

The VL53L0X signal is called `XSHUT`. The similar VL53L5CX signal is normally
called `LPn`. `LPn` disables communication and enters low-power idle, but is not
equivalent to a guaranteed full power reset.

Suggested `pin_map.h` excerpt:

```c
// Time-of-Flight sensor control pins.
#define PIN_VL53L0X_XSHUT       21
#define PIN_VL53L5CX_LPN        40

// Add these only if the PCB actually routes them to the ESP32.
// #define PIN_VL53L5CX_PWREN    ...
// #define PIN_VL53L5CX_I2C_RST  ...
// #define PIN_VL53L5CX_INT      ...
```

Why model-specific names are useful:

- A maintainer immediately knows which electrical behavior applies.
- The code cannot accidentally treat VL53L5CX `LPn` as a full reset.
- Physical sensor roles can later change without names such as `TOF1` becoming
  misleading.

Before renaming the existing macros, verify that GPIO 21 really connects to the
VL53L0X and GPIO 40 really connects to VL53L5CX LPn.

---

## 2. Sensor configuration

Configuration describes fixed choices: wiring, addresses, measurement timing,
and mounting orientation. Runtime samples and error counters do not belong in
configuration objects.

### `include/config/time_of_flight/tof_config.h`

```c
#pragma once

#include <stdint.h>

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// All addresses in project-owned structures are unshifted 7-bit addresses.
#define TOF_DEFAULT_I2C_ADDRESS 0x29U

typedef enum {
    TOF_SENSOR_MODEL_VL53L0X = 0,
    TOF_SENSOR_MODEL_VL53L5CX,
} TofSensorModel;

typedef enum {
    TOF_GRID_ROTATION_0 = 0,
    TOF_GRID_ROTATION_90,
    TOF_GRID_ROTATION_180,
    TOF_GRID_ROTATION_270,
} TofGridRotation;

typedef struct {
    TofSensorModel model;
    gpio_num_t xshut_pin;
    uint8_t default_address;
    uint8_t assigned_address;
    uint32_t timing_budget_us;
    uint32_t inter_measurement_ms;
    uint32_t maximum_sample_age_ms;
} Vl53l0xConfig;

typedef struct {
    TofSensorModel model;
    gpio_num_t lpn_pin;
    uint8_t default_address;
    uint8_t assigned_address;
    uint8_t resolution;             // Use 16 for 4x4 or 64 for 8x8.
    uint8_t ranging_frequency_hz;
    uint32_t maximum_sample_age_ms;
    TofGridRotation mounting_rotation;
} Vl53l5cxConfig;

extern const Vl53l0xConfig REAR_POINT_TOF_CONFIG;
extern const Vl53l5cxConfig FRONT_GRID_TOF_CONFIG;

#ifdef __cplusplus
}
#endif
```

Why use two configuration types instead of one large `TofConfig`:

- The VL53L0X has a scalar timing budget; the VL53L5CX has a grid resolution
  and frame frequency.
- Separate types prevent nonsensical configurations such as assigning a grid
  rotation to the point sensor.
- The compiler helps catch accidental sensor/type mix-ups.

### `src/config/time_of_flight/tof_config.c`

```c
#include "config/time_of_flight/tof_config.h"

#include "config/pin_map.h"

const Vl53l0xConfig REAR_POINT_TOF_CONFIG = {
    .model = TOF_SENSOR_MODEL_VL53L0X,
    .xshut_pin = PIN_VL53L0X_XSHUT,
    .default_address = TOF_DEFAULT_I2C_ADDRESS,
    .assigned_address = 0x31U,
    .timing_budget_us = 33000U,
    .inter_measurement_ms = 50U,
    .maximum_sample_age_ms = 150U,
};

const Vl53l5cxConfig FRONT_GRID_TOF_CONFIG = {
    .model = TOF_SENSOR_MODEL_VL53L5CX,
    .lpn_pin = PIN_VL53L5CX_LPN,
    .default_address = TOF_DEFAULT_I2C_ADDRESS,
    .assigned_address = 0x32U,
    .resolution = 16U,
    .ranging_frequency_hz = 10U,
    .maximum_sample_age_ms = 250U,
    .mounting_rotation = TOF_GRID_ROTATION_0,
};
```

These are conservative bring-up settings. Start with a 4x4 VL53L5CX frame and
a modest frequency. Move to 8x8 only after initialization, bus traffic, frame
orientation, and RAM usage are understood.

---

## 3. VL53L0X project-facing driver

The wrapper owns one `I2cDevice`, lifecycle flags, and its last sample. It hides
the ST API from the manager and application.

### `include/drivers/time_of_flight/vl53l0x_driver.h`

```c
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include <robot_common/i2c_bus.h>

#include "config/time_of_flight/tof_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t distance_mm;
    uint8_t range_status;
    bool valid;
    int64_t timestamp_us;
} Vl53l0xReading;

typedef struct {
    const Vl53l0xConfig *config;
    I2cDevice device;
    bool initialized;
    bool ranging;
    uint32_t consecutive_errors;
    Vl53l0xReading latest;

    // Add the exact ST VL53L0X device/context type here after selecting the
    // vendor package. Do not expose that type to higher layers.
} Vl53l0xDriver;

bool vl53l0x_driver_config_is_valid(const Vl53l0xConfig *config);

// Assumes the manager has activated only this sensor at its default address.
esp_err_t vl53l0x_driver_init(
    Vl53l0xDriver *driver,
    I2cBus *bus,
    const Vl53l0xConfig *config
);

esp_err_t vl53l0x_driver_start(Vl53l0xDriver *driver);
esp_err_t vl53l0x_driver_stop(Vl53l0xDriver *driver);
esp_err_t vl53l0x_driver_data_ready(Vl53l0xDriver *driver, bool *ready_out);
esp_err_t vl53l0x_driver_read(Vl53l0xDriver *driver, int64_t now_us);

esp_err_t vl53l0x_driver_get_latest(
    const Vl53l0xDriver *driver,
    Vl53l0xReading *reading_out
);

#ifdef __cplusplus
}
#endif
```

The distinction between `read()` and `get_latest()` is important:

- `read()` touches hardware and may take time.
- `get_latest()` only copies cached state and is safe for telemetry or policy
  code that should not unexpectedly perform I2C work.

### `src/drivers/time_of_flight/vl53l0x_driver.c`

```c
#include "drivers/time_of_flight/vl53l0x_driver.h"

#include <stddef.h>
#include <string.h>

static bool address_is_valid(uint8_t address)
{
    return address > 0U && address <= 0x7FU;
}

bool vl53l0x_driver_config_is_valid(const Vl53l0xConfig *config)
{
    return config != NULL &&
           config->model == TOF_SENSOR_MODEL_VL53L0X &&
           GPIO_IS_VALID_OUTPUT_GPIO(config->xshut_pin) &&
           address_is_valid(config->default_address) &&
           address_is_valid(config->assigned_address) &&
           config->default_address != config->assigned_address &&
           config->timing_budget_us > 0U &&
           config->inter_measurement_ms > 0U;
}

esp_err_t vl53l0x_driver_init(
    Vl53l0xDriver *driver,
    I2cBus *bus,
    const Vl53l0xConfig *config
)
{
    if (driver == NULL || bus == NULL ||
        !vl53l0x_driver_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (driver->initialized) return ESP_ERR_INVALID_STATE;

    memset(driver, 0, sizeof(*driver));
    driver->config = config;

    esp_err_t error = i2c_device_init(
        &driver->device, bus, config->default_address);
    if (error != ESP_OK) goto fail;

    error = i2c_bus_probe(bus, config->default_address);
    if (error != ESP_OK) goto fail;

    /*
     * TODO(ST API): connect the selected VL53L0X API context to
     * driver->device, then perform the vendor initialization sequence.
     * A typical ST sequence contains operations equivalent to:
     *
     *   DataInit
     *   StaticInit
     *   PerformRefCalibration
     *   PerformRefSpadManagement
     *   SetDeviceMode(continuous ranging)
     *   SetMeasurementTimingBudgetMicroSeconds
     *
     * Check every return value. Never combine statuses with bitwise OR,
     * because doing so loses the first useful failure location.
     */

    /*
     * TODO(ST API): program config->assigned_address into the sensor.
     * Convert between 7-bit and 8-bit forms here if this ST API requires it.
     */

    // Rebind the project-owned handle to the new 7-bit address.
    memset(&driver->device, 0, sizeof(driver->device));
    error = i2c_device_init(
        &driver->device, bus, config->assigned_address);
    if (error != ESP_OK) goto fail;

    error = i2c_bus_probe(bus, config->assigned_address);
    if (error != ESP_OK) goto fail;

    driver->initialized = true;
    return ESP_OK;

fail:
    memset(driver, 0, sizeof(*driver));
    return error;
}

esp_err_t vl53l0x_driver_start(Vl53l0xDriver *driver)
{
    if (driver == NULL) return ESP_ERR_INVALID_ARG;
    if (!driver->initialized || driver->ranging) return ESP_ERR_INVALID_STATE;

    // TODO(ST API): start continuous measurement and check its status.
    driver->ranging = true;
    return ESP_OK;
}

esp_err_t vl53l0x_driver_stop(Vl53l0xDriver *driver)
{
    if (driver == NULL) return ESP_ERR_INVALID_ARG;
    if (!driver->initialized) return ESP_ERR_INVALID_STATE;
    if (!driver->ranging) return ESP_OK;

    // TODO(ST API): stop continuous measurement and check its status.
    driver->ranging = false;
    return ESP_OK;
}

esp_err_t vl53l0x_driver_data_ready(Vl53l0xDriver *driver, bool *ready_out)
{
    if (driver == NULL || ready_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!driver->initialized || !driver->ranging) return ESP_ERR_INVALID_STATE;

    // TODO(ST API): query the measurement-ready flag without blocking.
    *ready_out = false;
    return ESP_OK;
}

esp_err_t vl53l0x_driver_read(Vl53l0xDriver *driver, int64_t now_us)
{
    if (driver == NULL || now_us < 0) return ESP_ERR_INVALID_ARG;
    if (!driver->initialized || !driver->ranging) return ESP_ERR_INVALID_STATE;

    /*
     * TODO(ST API): read the vendor measurement structure, copy distance and
     * range status, clear the sensor interrupt, and decide validity from the
     * documented status values. Do not assume every numeric distance is valid.
     */
    Vl53l0xReading next = {
        .distance_mm = 0U,
        .range_status = 0U,
        .valid = false,
        .timestamp_us = now_us,
    };

    driver->latest = next;
    driver->consecutive_errors = 0U;
    return ESP_OK;
}

esp_err_t vl53l0x_driver_get_latest(
    const Vl53l0xDriver *driver,
    Vl53l0xReading *reading_out
)
{
    if (driver == NULL || reading_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!driver->initialized) return ESP_ERR_INVALID_STATE;

    *reading_out = driver->latest;
    return ESP_OK;
}
```

This source is intentionally incomplete only where the selected ST package is
required. The argument validation, state transitions, address ownership, and
cached-data pattern are project architecture and can remain stable across
vendor updates.

---

## 4. VL53L5CX project-facing driver

The VL53L5CX returns a frame rather than one distance. Preserve per-zone status
instead of immediately reducing the frame to one number.

### `include/drivers/time_of_flight/vl53l5cx_driver.h`

```c
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include <robot_common/i2c_bus.h>

#include "config/time_of_flight/tof_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L5CX_MAX_ZONES 64U

typedef struct {
    uint16_t distance_mm[VL53L5CX_MAX_ZONES];
    uint8_t target_status[VL53L5CX_MAX_ZONES];
    uint64_t valid_zone_mask;
    uint8_t zone_count;
    uint8_t stream_count;
    int64_t timestamp_us;
} Vl53l5cxFrame;

typedef struct {
    const Vl53l5cxConfig *config;
    I2cDevice device;
    bool initialized;
    bool ranging;
    uint32_t consecutive_errors;
    Vl53l5cxFrame latest;

    // Add VL53L5CX_Configuration (or the equivalent exact vendor type) after
    // importing a pinned ULD version.
} Vl53l5cxDriver;

bool vl53l5cx_driver_config_is_valid(const Vl53l5cxConfig *config);

esp_err_t vl53l5cx_driver_init(
    Vl53l5cxDriver *driver,
    I2cBus *bus,
    const Vl53l5cxConfig *config
);

esp_err_t vl53l5cx_driver_start(Vl53l5cxDriver *driver);
esp_err_t vl53l5cx_driver_stop(Vl53l5cxDriver *driver);
esp_err_t vl53l5cx_driver_data_ready(Vl53l5cxDriver *driver, bool *ready_out);
esp_err_t vl53l5cx_driver_read(Vl53l5cxDriver *driver, int64_t now_us);

esp_err_t vl53l5cx_driver_get_latest(
    const Vl53l5cxDriver *driver,
    Vl53l5cxFrame *frame_out
);

#ifdef __cplusplus
}
#endif
```

A 64-bit mask is convenient because bit `n` describes zone `n`. It lets policy
code quickly skip zones whose target status is invalid. If multiple targets per
zone are enabled later, replace the simple arrays with dimensions for both zone
and target.

### Large-transfer helpers for `vl53l5cx_platform.c`

VL53L5CX initialization uploads roughly 84 KB of firmware. The ST platform
adapter must split large operations into transactions that the ESP32 stack and
available memory can handle.

The following helpers demonstrate the important mechanics using the existing
`robot-common` API:

```c
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include <robot_common/i2c_bus.h>

#define VL53L5CX_I2C_CHUNK_BYTES 128U

static esp_err_t vl53l5cx_write_register_block(
    const I2cDevice *device,
    uint16_t register_address,
    const uint8_t *data,
    size_t data_size
)
{
    if (device == NULL || data == NULL || data_size == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t transaction[2U + VL53L5CX_I2C_CHUNK_BYTES];
    size_t sent = 0U;

    while (sent < data_size) {
        size_t chunk = data_size - sent;
        if (chunk > VL53L5CX_I2C_CHUNK_BYTES) {
            chunk = VL53L5CX_I2C_CHUNK_BYTES;
        }

        const uint16_t current_register =
            (uint16_t)(register_address + sent);
        transaction[0] = (uint8_t)(current_register >> 8U);
        transaction[1] = (uint8_t)(current_register & 0xFFU);
        memcpy(&transaction[2], &data[sent], chunk);

        esp_err_t error = i2c_device_write(device, transaction, chunk + 2U);
        if (error != ESP_OK) return error;

        sent += chunk;
    }

    return ESP_OK;
}

static esp_err_t vl53l5cx_read_register_block(
    const I2cDevice *device,
    uint16_t register_address,
    uint8_t *data,
    size_t data_size
)
{
    if (device == NULL || data == NULL || data_size == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t received = 0U;
    while (received < data_size) {
        size_t chunk = data_size - received;
        if (chunk > VL53L5CX_I2C_CHUNK_BYTES) {
            chunk = VL53L5CX_I2C_CHUNK_BYTES;
        }

        const uint16_t current_register =
            (uint16_t)(register_address + received);
        const uint8_t request[2] = {
            (uint8_t)(current_register >> 8U),
            (uint8_t)(current_register & 0xFFU),
        };

        esp_err_t error = i2c_device_write_read(
            device, request, sizeof(request), &data[received], chunk);
        if (error != ESP_OK) return error;

        received += chunk;
    }

    return ESP_OK;
}
```

Why use a small fixed buffer:

- It avoids a large stack or heap allocation.
- Each transaction has a predictable upper bound.
- A failed chunk returns immediately with the original ESP error.
- Register address incrementing keeps consecutive chunks contiguous.

The exact chunk limit should be measured on the target. Do not assume the ULD's
largest requested transfer can be sent as one I2C transaction.

### `src/drivers/time_of_flight/vl53l5cx_driver.c` lifecycle outline

```c
#include "drivers/time_of_flight/vl53l5cx_driver.h"

#include <stddef.h>
#include <string.h>

bool vl53l5cx_driver_config_is_valid(const Vl53l5cxConfig *config)
{
    if (config == NULL || config->model != TOF_SENSOR_MODEL_VL53L5CX) {
        return false;
    }

    const bool resolution_valid =
        config->resolution == 16U || config->resolution == 64U;
    const uint8_t maximum_frequency =
        config->resolution == 16U ? 60U : 15U;

    return GPIO_IS_VALID_OUTPUT_GPIO(config->lpn_pin) &&
           config->default_address > 0U &&
           config->default_address <= 0x7FU &&
           config->assigned_address > 0U &&
           config->assigned_address <= 0x7FU &&
           config->default_address != config->assigned_address &&
           resolution_valid &&
           config->ranging_frequency_hz > 0U &&
           config->ranging_frequency_hz <= maximum_frequency;
}

esp_err_t vl53l5cx_driver_init(
    Vl53l5cxDriver *driver,
    I2cBus *bus,
    const Vl53l5cxConfig *config
)
{
    if (driver == NULL || bus == NULL ||
        !vl53l5cx_driver_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (driver->initialized) return ESP_ERR_INVALID_STATE;

    memset(driver, 0, sizeof(*driver));
    driver->config = config;

    esp_err_t error = i2c_device_init(
        &driver->device, bus, config->default_address);
    if (error != ESP_OK) goto fail;

    error = i2c_bus_probe(bus, config->default_address);
    if (error != ESP_OK) goto fail;

    /*
     * TODO(ST API): point the ULD platform context at driver->device.
     * Then call the operations equivalent to:
     *
     *   vl53l5cx_is_alive
     *   vl53l5cx_init                 (uploads sensor firmware)
     *   vl53l5cx_set_i2c_address
     *   vl53l5cx_set_resolution
     *   vl53l5cx_set_ranging_frequency_hz
     *
     * Translate the ULD status into esp_err_t at this boundary. Log the exact
     * failed stage because a generic "init failed" message is not enough to
     * distinguish wiring, firmware upload, byte order, and configuration.
     */

    memset(&driver->device, 0, sizeof(driver->device));
    error = i2c_device_init(
        &driver->device, bus, config->assigned_address);
    if (error != ESP_OK) goto fail;

    error = i2c_bus_probe(bus, config->assigned_address);
    if (error != ESP_OK) goto fail;

    driver->initialized = true;
    return ESP_OK;

fail:
    memset(driver, 0, sizeof(*driver));
    return error;
}

esp_err_t vl53l5cx_driver_start(Vl53l5cxDriver *driver)
{
    if (driver == NULL) return ESP_ERR_INVALID_ARG;
    if (!driver->initialized || driver->ranging) return ESP_ERR_INVALID_STATE;

    // TODO(ST API): vl53l5cx_start_ranging and status conversion.
    driver->ranging = true;
    return ESP_OK;
}

esp_err_t vl53l5cx_driver_stop(Vl53l5cxDriver *driver)
{
    if (driver == NULL) return ESP_ERR_INVALID_ARG;
    if (!driver->initialized) return ESP_ERR_INVALID_STATE;
    if (!driver->ranging) return ESP_OK;

    // TODO(ST API): vl53l5cx_stop_ranging and status conversion.
    driver->ranging = false;
    return ESP_OK;
}

esp_err_t vl53l5cx_driver_data_ready(Vl53l5cxDriver *driver, bool *ready_out)
{
    if (driver == NULL || ready_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!driver->initialized || !driver->ranging) return ESP_ERR_INVALID_STATE;

    // TODO(ST API): vl53l5cx_check_data_ready.
    *ready_out = false;
    return ESP_OK;
}

esp_err_t vl53l5cx_driver_read(Vl53l5cxDriver *driver, int64_t now_us)
{
    if (driver == NULL || now_us < 0) return ESP_ERR_INVALID_ARG;
    if (!driver->initialized || !driver->ranging) return ESP_ERR_INVALID_STATE;

    /*
     * TODO(ST API): call vl53l5cx_get_ranging_data into a private vendor
     * results object. Copy only the configured zones into this normalized
     * frame. Set a bit in valid_zone_mask only for documented valid target
     * statuses. Preserve raw target_status for diagnostics.
     */
    Vl53l5cxFrame next = {0};
    next.zone_count = driver->config->resolution;
    next.timestamp_us = now_us;

    driver->latest = next;
    driver->consecutive_errors = 0U;
    return ESP_OK;
}

esp_err_t vl53l5cx_driver_get_latest(
    const Vl53l5cxDriver *driver,
    Vl53l5cxFrame *frame_out
)
{
    if (driver == NULL || frame_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!driver->initialized) return ESP_ERR_INVALID_STATE;

    *frame_out = driver->latest;
    return ESP_OK;
}
```

Do not blindly copy target status rules from an online example. Use the status
definitions supplied with the exact ULD version and keep raw status values in
telemetry while bringing the sensor up.

---

## 5. ToF manager and address sequencing

The manager coordinates shared concerns. It borrows the one application-owned
`I2cBus`, owns both driver instances, and caches combined status.

### `include/control/time_of_flight/tof_manager.h`

```c
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include <robot_common/i2c_bus.h>

#include "drivers/time_of_flight/vl53l0x_driver.h"
#include "drivers/time_of_flight/vl53l5cx_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool initialized;
    bool ranging;
    bool point_sensor_healthy;
    bool grid_sensor_healthy;
    Vl53l0xReading point_reading;
    Vl53l5cxFrame grid_frame;
} TofManagerSnapshot;

typedef struct {
    I2cBus *bus;                    // Borrowed; the manager does not deinit it.
    Vl53l0xDriver point_sensor;
    Vl53l5cxDriver grid_sensor;
    bool initialized;
    bool ranging;
} TofManager;

esp_err_t tof_manager_init(
    TofManager *manager,
    I2cBus *bus,
    const Vl53l0xConfig *point_config,
    const Vl53l5cxConfig *grid_config
);

esp_err_t tof_manager_start(TofManager *manager);
esp_err_t tof_manager_stop(TofManager *manager);

// Polls readiness and reads only sensors that have a completed measurement.
esp_err_t tof_manager_update(TofManager *manager, int64_t now_us);

esp_err_t tof_manager_get_snapshot(
    const TofManager *manager,
    int64_t now_us,
    TofManagerSnapshot *snapshot_out
);

#ifdef __cplusplus
}
#endif
```

### `src/control/time_of_flight/tof_manager.c`

```c
#include "control/time_of_flight/tof_manager.h"

#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_err_t configure_control_pin_low(gpio_num_t pin)
{
    gpio_config_t config = {0};
    config.pin_bit_mask = 1ULL << pin;
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t error = gpio_config(&config);
    if (error != ESP_OK) return error;
    return gpio_set_level(pin, 0);
}

static bool timestamp_is_fresh(
    int64_t now_us,
    int64_t timestamp_us,
    uint32_t maximum_age_ms
)
{
    if (timestamp_us <= 0 || now_us < timestamp_us) return false;
    return now_us - timestamp_us <= (int64_t)maximum_age_ms * 1000LL;
}

esp_err_t tof_manager_init(
    TofManager *manager,
    I2cBus *bus,
    const Vl53l0xConfig *point_config,
    const Vl53l5cxConfig *grid_config
)
{
    if (manager == NULL || bus == NULL || point_config == NULL ||
        grid_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!i2c_bus_is_initialized(bus)) return ESP_ERR_INVALID_STATE;
    if (manager->initialized) return ESP_ERR_INVALID_STATE;

    memset(manager, 0, sizeof(*manager));
    manager->bus = bus;

    // Prevent the two default-address devices from answering together.
    esp_err_t error = configure_control_pin_low(point_config->xshut_pin);
    if (error != ESP_OK) goto fail;
    error = configure_control_pin_low(grid_config->lpn_pin);
    if (error != ESP_OK) goto fail;

    vTaskDelay(pdMS_TO_TICKS(10U));

    // Bring up and re-address the VL53L0X while VL53L5CX stays silent.
    error = gpio_set_level(point_config->xshut_pin, 1);
    if (error != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(5U));

    error = vl53l0x_driver_init(
        &manager->point_sensor, bus, point_config);
    if (error != ESP_OK) goto fail;

    // The point sensor now uses 0x31, so 0x29 is free for the grid sensor.
    error = gpio_set_level(grid_config->lpn_pin, 1);
    if (error != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(10U));

    error = vl53l5cx_driver_init(
        &manager->grid_sensor, bus, grid_config);
    if (error != ESP_OK) goto fail;

    manager->initialized = true;
    return ESP_OK;

fail:
    gpio_set_level(point_config->xshut_pin, 0);
    gpio_set_level(grid_config->lpn_pin, 0);
    memset(manager, 0, sizeof(*manager));
    return error;
}

esp_err_t tof_manager_start(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || manager->ranging) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t error = vl53l0x_driver_start(&manager->point_sensor);
    if (error != ESP_OK) return error;

    error = vl53l5cx_driver_start(&manager->grid_sensor);
    if (error != ESP_OK) {
        vl53l0x_driver_stop(&manager->point_sensor);
        return error;
    }

    manager->ranging = true;
    return ESP_OK;
}

esp_err_t tof_manager_stop(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = vl53l0x_driver_stop(&manager->point_sensor);
    esp_err_t grid_error = vl53l5cx_driver_stop(&manager->grid_sensor);
    manager->ranging = false;

    return first_error != ESP_OK ? first_error : grid_error;
}

esp_err_t tof_manager_update(TofManager *manager, int64_t now_us)
{
    if (manager == NULL || now_us < 0) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || !manager->ranging) {
        return ESP_ERR_INVALID_STATE;
    }

    bool ready = false;
    esp_err_t error = vl53l0x_driver_data_ready(
        &manager->point_sensor, &ready);
    if (error != ESP_OK) return error;
    if (ready) {
        error = vl53l0x_driver_read(&manager->point_sensor, now_us);
        if (error != ESP_OK) return error;
    }

    ready = false;
    error = vl53l5cx_driver_data_ready(&manager->grid_sensor, &ready);
    if (error != ESP_OK) return error;
    if (ready) {
        error = vl53l5cx_driver_read(&manager->grid_sensor, now_us);
        if (error != ESP_OK) return error;
    }

    return ESP_OK;
}

esp_err_t tof_manager_get_snapshot(
    const TofManager *manager,
    int64_t now_us,
    TofManagerSnapshot *snapshot_out
)
{
    if (manager == NULL || snapshot_out == NULL || now_us < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!manager->initialized) return ESP_ERR_INVALID_STATE;

    TofManagerSnapshot snapshot = {0};
    snapshot.initialized = manager->initialized;
    snapshot.ranging = manager->ranging;

    esp_err_t point_error = vl53l0x_driver_get_latest(
        &manager->point_sensor, &snapshot.point_reading);
    esp_err_t grid_error = vl53l5cx_driver_get_latest(
        &manager->grid_sensor, &snapshot.grid_frame);

    snapshot.point_sensor_healthy =
        point_error == ESP_OK &&
        snapshot.point_reading.valid &&
        timestamp_is_fresh(
            now_us,
            snapshot.point_reading.timestamp_us,
            manager->point_sensor.config->maximum_sample_age_ms);

    snapshot.grid_sensor_healthy =
        grid_error == ESP_OK &&
        snapshot.grid_frame.valid_zone_mask != 0U &&
        timestamp_is_fresh(
            now_us,
            snapshot.grid_frame.timestamp_us,
            manager->grid_sensor.config->maximum_sample_age_ms);

    *snapshot_out = snapshot;
    return ESP_OK;
}
```

The `goto fail` path is intentional. Initialization has many stages, and every
failed stage should return the hardware to a known inactive state. Centralized
cleanup is easier to audit than partially duplicated cleanup after every call.

### Warm-reboot warning

The sequence above describes a deterministic cold boot. VL53L0X XSHUT resets
the point sensor. VL53L5CX LPn can retain state, including its prior address,
while the ESP32 reboots.

Before treating this sample as production-ready, implement one of these:

1. Preferably, control VL53L5CX `PWREN` or its power rail and guarantee a full
   reset during manager initialization.
2. If the hardware cannot do that, hold VL53L0X inactive and probe both `0x29`
   and `0x32`. Recover based on the address that answers, using the exact ULD's
   documented reinitialization behavior.

The hardware test plan must include resetting only the ESP32 while leaving both
sensors powered.

---

## 6. Application composition

Only one application-level object should own and initialize the shared I2C bus.
The ToF manager borrows it. Future I2C sensors should borrow the same bus too.

Illustrative `setup()` and `loop()` integration:

```cpp
#include <Arduino.h>

#include "esp_timer.h"
#include <robot_common/i2c_bus.h>

#include "config/communication/i2c_bus_config.h"
#include "config/time_of_flight/tof_config.h"
#include "control/time_of_flight/tof_manager.h"

static I2cBus sensor_bus = {0};
static TofManager tof_manager = {0};
static bool tof_ready = false;

void setup()
{
    Serial.begin(115200);

    esp_err_t error = i2c_bus_init(
        &sensor_bus, &SENSOR_I2C_BUS_CONFIG);
    if (error != ESP_OK) {
        Serial.printf("I2C init failed: %s\n", esp_err_to_name(error));
        return;
    }

    error = tof_manager_init(
        &tof_manager,
        &sensor_bus,
        &REAR_POINT_TOF_CONFIG,
        &FRONT_GRID_TOF_CONFIG);
    if (error != ESP_OK) {
        Serial.printf("ToF init failed: %s\n", esp_err_to_name(error));
        return;
    }

    error = tof_manager_start(&tof_manager);
    if (error != ESP_OK) {
        Serial.printf("ToF start failed: %s\n", esp_err_to_name(error));
        return;
    }

    tof_ready = true;
}

void loop()
{
    const int64_t now_us = esp_timer_get_time();

    if (tof_ready) {
        esp_err_t error = tof_manager_update(&tof_manager, now_us);
        if (error != ESP_OK) {
            Serial.printf("ToF update failed: %s\n", esp_err_to_name(error));
            tof_ready = false;
        }
    }

    // The existing drivetrain update should run on its own required schedule.
    // Do not add a long delay here in production firmware.
}
```

This is composition code: it creates objects and connects modules. Sensor
register operations remain in drivers, address sequencing remains in the
manager, and motion control remains in the drivetrain.

---

## 7. Dedicated hardware-test harness

Do not first debug two sensors, address reassignment, motor noise, and obstacle
behavior simultaneously. A focused harness makes each failure visible.

### `src/harnesses/tof_test_main.cpp`

```cpp
#include <Arduino.h>

#include "esp_timer.h"
#include <robot_common/i2c_bus.h>

#include "config/communication/i2c_bus_config.h"
#include "config/time_of_flight/tof_config.h"
#include "control/time_of_flight/tof_manager.h"

static I2cBus bus = {0};
static TofManager manager = {0};
static bool ready = false;
static int64_t last_print_us = 0;

static void print_grid(const Vl53l5cxFrame &frame)
{
    const uint8_t width = frame.zone_count == 64U ? 8U : 4U;

    for (uint8_t row = 0; row < width; ++row) {
        for (uint8_t column = 0; column < width; ++column) {
            const uint8_t zone = (uint8_t)(row * width + column);
            const bool valid = (frame.valid_zone_mask & (1ULL << zone)) != 0U;

            if (valid) Serial.printf("%5u ", frame.distance_mm[zone]);
            else Serial.print("  --- ");
        }
        Serial.println();
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    esp_err_t error = i2c_bus_init(&bus, &SENSOR_I2C_BUS_CONFIG);
    if (error != ESP_OK) {
        Serial.printf("FAIL bus init: %s\n", esp_err_to_name(error));
        return;
    }

    error = tof_manager_init(
        &manager,
        &bus,
        &REAR_POINT_TOF_CONFIG,
        &FRONT_GRID_TOF_CONFIG);
    if (error != ESP_OK) {
        Serial.printf("FAIL manager init: %s\n", esp_err_to_name(error));
        return;
    }

    error = tof_manager_start(&manager);
    if (error != ESP_OK) {
        Serial.printf("FAIL ranging start: %s\n", esp_err_to_name(error));
        return;
    }

    ready = true;
    Serial.println("PASS ToF sensors started");
}

void loop()
{
    if (!ready) {
        delay(100);
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    esp_err_t error = tof_manager_update(&manager, now_us);
    if (error != ESP_OK) {
        Serial.printf("FAIL update: %s\n", esp_err_to_name(error));
        ready = false;
        return;
    }

    if (now_us - last_print_us >= 500000LL) {
        TofManagerSnapshot snapshot = {0};
        error = tof_manager_get_snapshot(&manager, now_us, &snapshot);
        if (error == ESP_OK) {
            Serial.printf(
                "VL53L0X: %s, distance=%u mm, status=%u\n",
                snapshot.point_sensor_healthy ? "valid" : "invalid/stale",
                snapshot.point_reading.distance_mm,
                snapshot.point_reading.range_status);

            Serial.printf(
                "VL53L5CX: %s, zones=%u, stream=%u\n",
                snapshot.grid_sensor_healthy ? "valid" : "invalid/stale",
                snapshot.grid_frame.zone_count,
                snapshot.grid_frame.stream_count);
            print_grid(snapshot.grid_frame);
        }
        last_print_us = now_us;
    }

    // Yield without imposing a large fixed application delay.
    delay(1);
}
```

### `platformio.ini` environment

```ini
[env:tof-test]
lib_extra_dirs = ../lib
platform = espressif32
board = esp32-s3-devkitm-1
framework = arduino

monitor_speed = 115200

build_unflags = -std=gnu++11
build_flags =
    -std=gnu++17
    -DARDUINO_USB_CDC_ON_BOOT=1

build_src_filter =
    -<*>
    +<harnesses/tof_test_main.cpp>
    +<drivers/time_of_flight/>
    +<control/time_of_flight/tof_manager.c>
    +<config/time_of_flight/tof_config.c>
    +<config/communication/i2c_bus_config.c>
```

Depending on how the ST packages declare their PlatformIO library metadata,
their sources may need explicit library dependencies or `lib_ldf_mode = deep+`.
Prefer correct `library.json` dependency declarations over permanently enabling
an overly broad dependency finder.

---

## 8. Optional obstacle policy

Add this only after the harness produces trustworthy measurements. It should be
pure logic with no GPIO, I2C, Arduino, or ST headers so native tests can exercise
it.

### `include/control/time_of_flight/obstacle_guard.h`

```c
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/time_of_flight/vl53l0x_driver.h"
#include "drivers/time_of_flight/vl53l5cx_driver.h"

typedef struct {
    uint16_t stop_distance_mm;
    uint16_t slowdown_distance_mm;
    float minimum_speed_scale;
    bool stop_when_front_sensor_stale;
} ObstacleGuardConfig;

typedef struct {
    float speed_scale;
    bool stop_requested;
    bool sensor_fault;
} ObstacleGuardDecision;

ObstacleGuardDecision obstacle_guard_evaluate_front(
    const ObstacleGuardConfig *config,
    const Vl53l5cxFrame *frame,
    bool frame_is_fresh
);
```

### `src/control/time_of_flight/obstacle_guard.c`

```c
#include "control/time_of_flight/obstacle_guard.h"

#include <stddef.h>

ObstacleGuardDecision obstacle_guard_evaluate_front(
    const ObstacleGuardConfig *config,
    const Vl53l5cxFrame *frame,
    bool frame_is_fresh
)
{
    ObstacleGuardDecision decision = {
        .speed_scale = 1.0f,
        .stop_requested = false,
        .sensor_fault = false,
    };

    if (config == NULL || frame == NULL || !frame_is_fresh) {
        decision.sensor_fault = true;
        decision.stop_requested =
            config == NULL || config->stop_when_front_sensor_stale;
        decision.speed_scale = decision.stop_requested ? 0.0f : 1.0f;
        return decision;
    }

    uint16_t nearest_mm = UINT16_MAX;
    for (uint8_t zone = 0U; zone < frame->zone_count; ++zone) {
        if ((frame->valid_zone_mask & (1ULL << zone)) == 0U) continue;
        if (frame->distance_mm[zone] < nearest_mm) {
            nearest_mm = frame->distance_mm[zone];
        }
    }

    if (nearest_mm == UINT16_MAX) {
        decision.sensor_fault = true;
        decision.stop_requested = config->stop_when_front_sensor_stale;
        decision.speed_scale = decision.stop_requested ? 0.0f : 1.0f;
        return decision;
    }

    if (nearest_mm <= config->stop_distance_mm) {
        decision.stop_requested = true;
        decision.speed_scale = 0.0f;
    } else if (nearest_mm < config->slowdown_distance_mm) {
        const float range = (float)(
            config->slowdown_distance_mm - config->stop_distance_mm);
        const float position = (float)(
            nearest_mm - config->stop_distance_mm) / range;
        decision.speed_scale = config->minimum_speed_scale +
            position * (1.0f - config->minimum_speed_scale);
    }

    return decision;
}
```

This first example uses every valid zone. A real robot will often select only
the zones aligned with its commanded motion. Otherwise an object safely beside
the robot may stop forward travel. Grid rotation from configuration must also
be applied before assigning zones to front/left/right sectors.

The policy should limit the command before calling
`drivetrain_set_body_velocity()`. It should not directly write motor duty.

---

## 9. Error-handling principles

### Keep the first meaningful error

Do not write this:

```c
status |= vendor_step_one();
status |= vendor_step_two();
status |= vendor_step_three();
```

It runs later steps after failure and destroys information about which call
failed. Prefer one checked call at a time and log the stage name.

### Separate invalid data from communication failure

- An I2C timeout is a communication error.
- A completed measurement with an invalid target status is a valid transaction
  containing an unusable range.
- An old cached sample is stale data.

These cases may lead to the same safety action, but they need different
diagnostic messages.

### Do not report zero as a real obstacle by default

Zero-initialized structures are useful for safe startup, but a zero distance is
not automatically a valid measurement. Always require a valid flag or valid
zone bit.

### Define failure policy above the driver

The driver should report errors. The application or obstacle guard decides
whether failure means continue, slow, controlled stop, or hardware brake.

---

## 10. Bring-up checklist

Implement and verify in this order:

1. Confirm sensor supply and I/O voltages with the exact breakout schematics.
2. Confirm external I2C pull-ups; internal ESP32 pull-ups are currently disabled.
3. Bring both control pins low and scan the bus: neither sensor should answer.
4. Activate only VL53L0X and verify `0x29`.
5. Change VL53L0X to `0x31` and verify it there.
6. Activate VL53L5CX and verify that it alone occupies `0x29`.
7. Validate short VL53L5CX register reads before attempting firmware upload.
8. Validate chunked firmware upload and record initialization duration.
9. Change VL53L5CX to `0x32`; verify both final addresses.
10. Start VL53L0X continuous ranging and inspect raw statuses.
11. Start VL53L5CX at 4x4/10 Hz and print the complete matrix.
12. Determine grid rotation using an object moved through known positions.
13. Reset only the ESP32 while sensor power stays on and verify recovery.
14. Run all four motors and watch for I2C errors or corrupted frames.
15. Only then introduce obstacle decisions into drivetrain command flow.

At 100 kHz, VL53L5CX firmware upload can be noticeably slow. Establish
correctness first, then test the shared bus at 400 kHz. Bus frequency is a board
configuration choice, not a magic performance fix; wiring capacitance and
pull-up strength still matter.

---

## 11. What should be tested without hardware

The native test environment cannot exercise ESP32 I2C, but it can test pure
decisions such as:

- Invalid or stale frames request the configured safe action.
- No valid zones never becomes a false zero-millimetre obstacle.
- Stop distance produces a zero speed scale.
- Slowdown distance interpolates within the expected bounds.
- Grid rotation maps physical left/right/front zones correctly.
- Only zones relevant to the commanded travel direction are considered.
- Integer timestamps do not overflow or accidentally accept future samples.

Vendor communication should be tested on hardware through `tof-test`; obstacle
math should be tested natively. This division makes failures much easier to
locate.

## Final design summary

- Configuration owns fixed addresses, pins, timing, and orientation.
- Each sensor driver owns its vendor API and converts results into project types.
- One ToF manager owns address sequencing and coordinated polling.
- One application-owned `I2cBus` is shared by all sensor drivers.
- Cached snapshots separate hardware access from consumers.
- Obstacle policy is pure control logic above the drivers.
- The existing drivetrain remains responsible only for deterministic motor and
  encoder control.

Following these boundaries costs a few extra files, but it prevents vendor API
details, I2C latency, and safety policy from becoming tangled in one module.
