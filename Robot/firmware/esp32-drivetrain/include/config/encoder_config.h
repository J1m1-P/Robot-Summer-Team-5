#pragma once 

#include "drivers/encoder_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const EncoderDriverConfig FL_ENCODER_CONFIG;  // Front Left Encoder
extern const EncoderDriverConfig FR_ENCODER_CONFIG;  // Front Right Encoder
extern const EncoderDriverConfig BL_ENCODER_CONFIG;  // Back Left Encoder
extern const EncoderDriverConfig BR_ENCODER_CONFIG;  // Back Right Encoder

extern const EncoderDriverConfig * const ENCODER_CONFIGS[ENCODER_ID_MAX];  // Array of pointers to all encoder configurations

#ifdef __cplusplus
}
#endif
