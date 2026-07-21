#include "vl53l0x_platform.h"

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr uint32_t kMaxTransferBytes = 64U;

bool device_is_ready(VL53L0X_DEV dev) {
    return dev != nullptr && dev->i2c_device != nullptr &&
           dev->i2c_device->initialized && dev->i2c_device->bus != nullptr;
}

VL53L0X_Error convert_error(esp_err_t error) {
    return error == ESP_OK ? VL53L0X_ERROR_NONE
                           : VL53L0X_ERROR_CONTROL_INTERFACE;
}
}  // namespace

extern "C" {

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV dev) {
    return device_is_ready(dev) ? VL53L0X_ERROR_NONE
                                : VL53L0X_ERROR_INVALID_PARAMS;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV dev) {
    return device_is_ready(dev) ? VL53L0X_ERROR_NONE
                                : VL53L0X_ERROR_INVALID_PARAMS;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t index,
                                 uint8_t *data, uint32_t count) {
    if (!device_is_ready(dev) || data == nullptr || count == 0U ||
        count > kMaxTransferBytes) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }

    uint8_t transaction[kMaxTransferBytes + 1U];
    transaction[0] = index;
    std::memcpy(&transaction[1], data, count);
    return convert_error(
        i2c_device_write(dev->i2c_device, transaction, count + 1U));
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t index,
                                uint8_t *data, uint32_t count) {
    if (!device_is_ready(dev) || data == nullptr || count == 0U ||
        count > kMaxTransferBytes) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }

    return convert_error(i2c_device_write_read(
        dev->i2c_device, &index, 1U, data, count));
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t index, uint8_t data) {
    return VL53L0X_WriteMulti(dev, index, &data, 1U);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t index, uint16_t data) {
    uint8_t bytes[2] = {
        static_cast<uint8_t>(data >> 8U), static_cast<uint8_t>(data)};
    return VL53L0X_WriteMulti(dev, index, bytes, sizeof(bytes));
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t index, uint32_t data) {
    uint8_t bytes[4] = {
        static_cast<uint8_t>(data >> 24U), static_cast<uint8_t>(data >> 16U),
        static_cast<uint8_t>(data >> 8U), static_cast<uint8_t>(data)};
    return VL53L0X_WriteMulti(dev, index, bytes, sizeof(bytes));
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t index, uint8_t *data) {
    return VL53L0X_ReadMulti(dev, index, data, 1U);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t index, uint16_t *data) {
    if (data == nullptr) return VL53L0X_ERROR_INVALID_PARAMS;
    uint8_t bytes[2];
    VL53L0X_Error error = VL53L0X_ReadMulti(dev, index, bytes, sizeof(bytes));
    if (error == VL53L0X_ERROR_NONE) {
        *data = static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8U) |
                                      bytes[1]);
    }
    return error;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t index,
                              uint32_t *data) {
    if (data == nullptr) return VL53L0X_ERROR_INVALID_PARAMS;
    uint8_t bytes[4];
    VL53L0X_Error error = VL53L0X_ReadMulti(dev, index, bytes, sizeof(bytes));
    if (error == VL53L0X_ERROR_NONE) {
        *data = (static_cast<uint32_t>(bytes[0]) << 24U) |
                (static_cast<uint32_t>(bytes[1]) << 16U) |
                (static_cast<uint32_t>(bytes[2]) << 8U) | bytes[3];
    }
    return error;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t index,
                                 uint8_t and_data, uint8_t or_data) {
    uint8_t data = 0U;
    VL53L0X_Error error = VL53L0X_RdByte(dev, index, &data);
    if (error == VL53L0X_ERROR_NONE) {
        data = static_cast<uint8_t>((data & and_data) | or_data);
        error = VL53L0X_WrByte(dev, index, data);
    }
    return error;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev) {
    if (!device_is_ready(dev)) return VL53L0X_ERROR_INVALID_PARAMS;
    vTaskDelay(pdMS_TO_TICKS(1U));
    return VL53L0X_ERROR_NONE;
}

}  // extern "C"
