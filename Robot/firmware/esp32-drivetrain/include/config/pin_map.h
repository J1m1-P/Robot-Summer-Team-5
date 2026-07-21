/* Defines the drivetrain ESP32's GPIO assignments for actuators and sensors. */
#pragma once

// Tape Following Pins
#define PIN_TF_FRONT_INPUT      1   // Pin for Tape Following output 1
#define PIN_TF_BACK_INPUT       2   // Pin for Tape Following output 2
#define PIN_TF_LEFT_INPUT       3   // Pin for Tape Following output 3
#define PIN_TF_CHSEL1_PIN       39  // Pin for Tape Following channel select 1
#define PIN_TF_CHSEL2_PIN       38  // Pin for Tape Following channel select 2

// Motor Control Pins
#define PIN_M1_PWM              4   // Pin for Motor 1 PWM control
#define PIN_M1_DIR              5   // Pin for Motor 1 direction
#define PIN_M2_PWM              6   // Pin for Motor 2 PWM control
#define PIN_M2_DIR              7   // Pin for Motor 2 direction
#define PIN_M3_PWM              10  // Pin for Motor 3 PWM control
#define PIN_M3_DIR              11  // Pin for Motor 3 direction
#define PIN_M4_PWM              12  // Pin for Motor 4 PWM control
#define PIN_M4_DIR              13  // Pin for Motor 4 direction
#define PIN_M_BRK               14  // Pin for Motor active braking control

// Encoder Pins
#define PIN_ENC1_A              46  // Pin for Encoder 1 channel A
#define PIN_ENC1_B              45  // Pin for Encoder 1 channel B
#define PIN_ENC2_A              42  // Pin for Encoder 2 channel A
#define PIN_ENC2_B              41  // Pin for Encoder 2 channel B
#define PIN_ENC3_A              15  // Pin for Encoder 3 channel A
#define PIN_ENC3_B              16  // Pin for Encoder 3 channel B
#define PIN_ENC4_A              17  // Pin for Encoder 4 channel A
#define PIN_ENC4_B              18  // Pin for Encoder 4 channel B

// Communication Pins
#define PIN_I2C_SDA             8   // Pin for I2C data
#define PIN_I2C_SCL             9   // Pin for I2C clock
#define PIN_TOP_ESP32_UART_RX   47  // Pin for UART receive to top ESP32
#define PIN_TOP_ESP32_UART_TX   48  // Pin for UART transmit to top ESP32

// Time of Flight (ToF) Sensor Pins
#define PIN_TOF1_XSHUT          21  // Pin for left ToF XSHUT (address changed to 0x30)
#define PIN_TOF2_XSHUT          40  // Pin for middle ToF XSHUT (address changed to 0x31)
