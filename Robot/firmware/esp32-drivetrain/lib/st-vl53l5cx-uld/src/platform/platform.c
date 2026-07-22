/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "platform.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VL53L5CX_I2C_CHUNK_BYTES 128U

static uint8_t vl53l5cx_platform_is_ready(
		const VL53L5CX_Platform *p_platform)
{
	return p_platform != NULL &&
		p_platform->i2c_device != NULL &&
		p_platform->i2c_device->initialized &&
		p_platform->i2c_device->bus != NULL &&
		p_platform->i2c_device->bus->initialized &&
		p_platform->address <= 0xFEU &&
		(p_platform->address & 0x01U) == 0U;
}

static void vl53l5cx_select_address(VL53L5CX_Platform *p_platform)
{
	/* robot-common uses 7-bit addresses; the ST ULD stores an 8-bit address. */
	p_platform->i2c_device->address = (uint8_t)(p_platform->address >> 1U);
}

uint8_t VL53L5CX_RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	return VL53L5CX_RdMulti(p_platform, RegisterAdress, p_value, 1U);
}

uint8_t VL53L5CX_WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	return VL53L5CX_WrMulti(p_platform, RegisterAdress, &value, 1U);
}

uint8_t VL53L5CX_WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	if (!vl53l5cx_platform_is_ready(p_platform) ||
		p_values == NULL || size == 0U) {
		return 1U;
	}

	vl53l5cx_select_address(p_platform);

	while (size > 0U) {
		uint32_t chunk = size > VL53L5CX_I2C_CHUNK_BYTES
			? VL53L5CX_I2C_CHUNK_BYTES : size;
		uint8_t transaction[VL53L5CX_I2C_CHUNK_BYTES + 2U];

		transaction[0] = (uint8_t)(RegisterAdress >> 8U);
		transaction[1] = (uint8_t)RegisterAdress;
		memcpy(&transaction[2], p_values, chunk);

		if (i2c_device_write(
				p_platform->i2c_device, transaction, chunk + 2U) != ESP_OK) {
			return 1U;
		}

		RegisterAdress = (uint16_t)(RegisterAdress + chunk);
		p_values += chunk;
		size -= chunk;
	}

	return 0U;
}

uint8_t VL53L5CX_RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	if (!vl53l5cx_platform_is_ready(p_platform) ||
		p_values == NULL || size == 0U) {
		return 1U;
	}

	vl53l5cx_select_address(p_platform);

	while (size > 0U) {
		uint32_t chunk = size > VL53L5CX_I2C_CHUNK_BYTES
			? VL53L5CX_I2C_CHUNK_BYTES : size;
		uint8_t register_address[2] = {
			(uint8_t)(RegisterAdress >> 8U),
			(uint8_t)RegisterAdress
		};

		if (i2c_device_write_read(
				p_platform->i2c_device,
				register_address,
				sizeof(register_address),
				p_values,
				chunk) != ESP_OK) {
			return 1U;
		}

		RegisterAdress = (uint16_t)(RegisterAdress + chunk);
		p_values += chunk;
		size -= chunk;
	}

	return 0U;
}

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform)
{
	/* Reset GPIO ownership stays in the project driver, not the vendor port. */
	(void)p_platform;
	return 1U;
}

void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
	uint32_t i;

	for (i = 0U; i < size; i += 4U) {
		uint8_t byte_0 = buffer[i];
		uint8_t byte_1 = buffer[i + 1U];
		buffer[i] = buffer[i + 3U];
		buffer[i + 1U] = buffer[i + 2U];
		buffer[i + 2U] = byte_1;
		buffer[i + 3U] = byte_0;
	}
}

uint8_t VL53L5CX_WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs)
{
	if (!vl53l5cx_platform_is_ready(p_platform)) {
		return 1U;
	}

	vTaskDelay(pdMS_TO_TICKS(TimeMs));
	return 0U;
}
