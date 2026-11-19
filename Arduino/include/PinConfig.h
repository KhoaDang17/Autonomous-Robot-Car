#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <Arduino.h>

// ============================================================================
// 1. UART COMMUNICATION WITH ESP32-C6 MASTER (HARDWARE SERIAL: D0 RX / D1 TX)
// ============================================================================
#define UNO_BAUD            115200   // Must match UNO_BAUD on ESP32-C6
#define COMMAND_TIMEOUT_MS  500UL    // Watchdog: stop motors if no command arrives within 500 ms

// ============================================================================
// 2. STC8 MOTOR DRIVER SHIELD (SOFTWARE I2C) - KEYESTUDIO KS0560
// ============================================================================
#define STC8_SDA_PIN        3        // Software I2C Data pin (D3)
#define STC8_SCL_PIN        2        // Software I2C Clock pin (D2)
#define STC8_I2C_ADDR       0x30     // 7-bit I2C address of onboard STC8 motor chip

// Motor channel IDs mapped to KS0560 shield outputs
#define MOTOR_FRONT_RIGHT   1
#define MOTOR_FRONT_LEFT    2
#define MOTOR_REAR_RIGHT    3
#define MOTOR_REAR_LEFT     4

// Default PWM speed (0 - 255) for basic directional helper functions
#define DEFAULT_MOTOR_SPEED 100

#endif // PIN_CONFIG_H
