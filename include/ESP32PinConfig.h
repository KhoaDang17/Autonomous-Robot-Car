#ifndef ESP32_PIN_CONFIG_H
#define ESP32_PIN_CONFIG_H

#include <Arduino.h>

// ============================================================================
// 1. SERIAL & UART COMMUNICATION (DFROBOT FIREBEETLE 2 ESP32-C6)
// ============================================================================

// USB-C CDC Serial baud rate (PC Serial Monitor debugging)
#define DEBUG_SERIAL_BAUD 115200

// RPLIDAR C1 UART interface (Hardware UART1)
#define LIDAR_RX_PIN 17
#define LIDAR_TX_PIN 16
#define LIDAR_BAUD   460800

// Arduino Uno UART interface (Hardware UART0)
#define UNO_RX_PIN   4
#define UNO_TX_PIN   5
#define UNO_BAUD     115200

// ============================================================================
// 2. THREE FRONT ULTRASONIC SENSORS (TRAPEZOID ARRAY)
// ============================================================================
// Array index mapping: 0 = Front-Left, 1 = Front-Center, 2 = Front-Right
#define NUM_ULTRASONIC_SENSORS 3
#define US_LEFT_IDX   0
#define US_CENTER_IDX 1
#define US_RIGHT_IDX  2

const int TRIG_PINS[NUM_ULTRASONIC_SENSORS] = {18, 14, 3};
const int ECHO_PINS[NUM_ULTRASONIC_SENSORS] = {1, 2, 6};

// Pulse timeout (~25 ms corresponds to ~400 cm max range) and cross-talk pause
#define ULTRASONIC_TIMEOUT_US    25000UL
#define MAX_VALID_DISTANCE_CM    400.0f
#define ULTRASONIC_INTER_PING_MS 5

// ============================================================================
// 3. AUTONOMOUS NAVIGATION & COLLISION AVOIDANCE PARAMETERS
// ============================================================================

// Safety distance thresholds in centimeters (Milestones 3, 4, 5, and 6)
#define SAFE_FRONT_DISTANCE_CM      35.0f  // Trigger avoidance maneuver below this distance
#define EMERGENCY_STOP_DISTANCE_CM  18.0f  // Immediate reverse/escape below this distance
#define MIN_SIDE_CLEARANCE_CM       25.0f  // Minimum clearance required to turn into a side

// Mecanum velocity magnitudes (-255.0 to +255.0 PWM scale)
#define CRUISE_FORWARD_SPEED        110.0f
#define AVOIDANCE_TURN_SPEED        100.0f
#define AVOIDANCE_STRAFE_SPEED      100.0f
#define AVOIDANCE_BACKUP_SPEED      -80.0f

// Main control loop rate: 20 Hz (50 ms per cycle)
#define CONTROL_LOOP_INTERVAL_MS    50

#endif // ESP32_PIN_CONFIG_H
