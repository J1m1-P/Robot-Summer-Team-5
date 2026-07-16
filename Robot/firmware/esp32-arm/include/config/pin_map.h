/* Defines the ESP32 arm board's GPIO assignments for motors, sensors, and links. */
#pragma once

// Servos
#define PIN_SERVO1_PWM      1
#define PIN_SERVO2_PWM      2
#define PIN_SERVO3_PWM      3
#define PIN_SERVO4_PWM      4
#define PIN_SERVO5_PWM      5
#define PIN_SERVO6_PWM      6

// Locating Motor
#define PIN_LOC_EN          13      // Power on the locating motor
#define PIN_LOC_SWITCH      14      // Microswtich on the locating motor

// Stepper Motor Driver
#define PIN_STEP1           42      // PWM (not really) for Stepper Motor1
#define PIN_STEP1_DIR       41      // Direction for Stepper Motor1
#define PIN_STEP2           15      // PWM (not really) for Stepper Motor2
#define PIN_STEP2_DIR       16      // Direction for Stepper Motor2
#define PIN_STEP3           17      // PWM (not really) for Stepper Motor3
#define PIN_STEP3_DIR       18      // Direction for Stepper Motor3
#define PIN_STEP4           21      // PWM (not really) for Stepper Motor4
#define PIN_STEP4_DIR       40      // Direction for Stepper Motor4

// Communication

// Time of Flight (I2C)
#define PIN_I2C_SDA         7
#define PIN_I2C_SCL         8

// Optical Sensors (SPI)
#define PIN_PMW_SDIO        9       // Shared bus -- both sensors tied together
#define PIN_PMW_SCLK        10      // Shared bus -- both sensors tied together
#define PIN_PMW_NCS_L       11      // Left sensor chip select
#define PIN_PMW_NCS_R       12      // Right sensor chip select

// UART to Drivetrain
#define PIN_DRIVETRAIN_UART_RX  48
#define PIN_DRIVETRAIN_UART_TX  47

// UART to PI
#define PIN_PI_UART_RX          39
#define PIN_PI_UART_TX          38
