#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>
#include "ESP32PinConfig.h"

// Distance measurements (cm) from the 3 front trapezoid HC-SR04 sensors
struct UltrasonicArrayData {
    float left_cm;    // Sensor 0: Front-Left angled
    float center_cm;  // Sensor 1: Front-Center straight ahead
    float right_cm;   // Sensor 2: Front-Right angled
};

// Configure TRIG_PINS as OUTPUT and ECHO_PINS as INPUT for all 3 sensors.
void initUltrasonicSensors();

// Trigger one sensor by index (US_LEFT_IDX, US_CENTER_IDX, US_RIGHT_IDX)
// and return distance in cm (returns MAX_VALID_DISTANCE_CM on timeout).
float readSingleUltrasonicCm(uint8_t sensor_index);

// Sequentially read Left, Center, and Right sensors with a short delay
// between pings to avoid acoustic cross-talk.
UltrasonicArrayData readAllUltrasonicSensors();

// Return true if distance_cm is valid (> 0) and closer than threshold_cm.
bool isObstacleDetected(float distance_cm, float threshold_cm = SAFE_FRONT_DISTANCE_CM);

#endif // ULTRASONIC_SENSOR_H
