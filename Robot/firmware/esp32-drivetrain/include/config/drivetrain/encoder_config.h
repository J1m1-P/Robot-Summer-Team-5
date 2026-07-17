/* Exposes the four wheel-encoder configurations and their indexed lookup table. */
#pragma once

#include "drivers/encoder/encoder_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

// Individual wheel encoder configurations in physical wheel order.
extern const EncoderDriverConfig FL_ENCODER_CONFIG;  // Front Left Encoder
extern const EncoderDriverConfig FR_ENCODER_CONFIG;  // Front Right Encoder
extern const EncoderDriverConfig BL_ENCODER_CONFIG;  // Back Left Encoder
extern const EncoderDriverConfig BR_ENCODER_CONFIG;  // Back Right Encoder

// Indexed lookup matching EncoderId values.
extern const EncoderDriverConfig * const ENCODER_CONFIGS[ENCODER_ID_MAX];

#ifdef __cplusplus
}
#endif
