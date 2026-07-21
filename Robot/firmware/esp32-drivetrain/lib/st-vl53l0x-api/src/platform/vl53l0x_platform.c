#include "vl53l0x_platform.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VL53L0X_MAX_TRANSFER_BYTES  64U

static bool vl53l0x_device_is_ready(VL53L0X_DEV device)
{
    return device != NULL &&
           device->i2c_device != NULL &&
           device->i2c_device->initialized &&
           device->i2c_device->bus != NULL;
}

static VL53L0X_Error vl53l0x_convert_i2c_error(esp_err_t error)
{
    return error == ESP_OK ? VL53L0X_ERROR_NONE
                           : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV device)
{
    return vl53l0x_device_is_ready(device)
        ? VL53L0X_ERROR_NONE
        : VL53L0X_ERROR_INVALID_PARAMS;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV device)
{
    return vl53l0x_device_is_ready(device)
        ? VL53L0X_ERROR_NONE
        : VL53L0X_ERROR_INVALID_PARAMS;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV device, uint8_t index,
                                 uint8_t *data, uint32_t count)
{
    if (!vl53l0x_device_is_ready(device) || data == NULL || count == 0U ||
        count > VL53L0X_MAX_TRANSFER_BYTES) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }

    uint8_t transaction[VL53L0X_MAX_TRANSFER_BYTES + 1U];
    transaction[0] = index;
    memcpy(&transaction[1], data, count);
    return vl53l0x_convert_i2c_error(i2c_device_write(
        device->i2c_device, transaction, count + 1U));
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV device, uint8_t index,
                                uint8_t *data, uint32_t count)
{
    if (!vl53l0x_device_is_ready(device) || data == NULL || count == 0U ||
        count > VL53L0X_MAX_TRANSFER_BYTES) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }

    return vl53l0x_convert_i2c_error(i2c_device_write_read(
        device->i2c_device, &index, 1U, data, count));
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV device, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(device, index, &data, 1U);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV device, uint8_t index, uint16_t data)
{
    uint8_t bytes[2] = {(uint8_t)(data >> 8U), (uint8_t)data};
    return VL53L0X_WriteMulti(device, index, bytes, sizeof(bytes));
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV device, uint8_t index,
                              uint32_t data)
{
    uint8_t bytes[4] = {
        (uint8_t)(data >> 24U), (uint8_t)(data >> 16U),
        (uint8_t)(data >> 8U), (uint8_t)data
    };
    return VL53L0X_WriteMulti(device, index, bytes, sizeof(bytes));
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV device, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(device, index, data, 1U);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV device, uint8_t index,
                             uint16_t *data)
{
    if (data == NULL) return VL53L0X_ERROR_INVALID_PARAMS;
    uint8_t bytes[2];
    VL53L0X_Error error = VL53L0X_ReadMulti(
        device, index, bytes, sizeof(bytes));
    if (error == VL53L0X_ERROR_NONE) {
        *data = (uint16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
    }
    return error;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV device, uint8_t index,
                              uint32_t *data)
{
    if (data == NULL) return VL53L0X_ERROR_INVALID_PARAMS;
    uint8_t bytes[4];
    VL53L0X_Error error = VL53L0X_ReadMulti(
        device, index, bytes, sizeof(bytes));
    if (error == VL53L0X_ERROR_NONE) {
        *data = ((uint32_t)bytes[0] << 24U) |
                ((uint32_t)bytes[1] << 16U) |
                ((uint32_t)bytes[2] << 8U) |
                bytes[3];
    }
    return error;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV device, uint8_t index,
                                 uint8_t and_data, uint8_t or_data)
{
    uint8_t data = 0U;
    VL53L0X_Error error = VL53L0X_RdByte(device, index, &data);
    if (error == VL53L0X_ERROR_NONE) {
        data = (uint8_t)((data & and_data) | or_data);
        error = VL53L0X_WrByte(device, index, data);
    }
    return error;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV device)
{
    if (!vl53l0x_device_is_ready(device)) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
    vTaskDelay(pdMS_TO_TICKS(1U));
    return VL53L0X_ERROR_NONE;
}
