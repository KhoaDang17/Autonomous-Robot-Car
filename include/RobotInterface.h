#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <stdint.h>

// ==========================================
// 1. COMMON DATA STRUCTURES
// ==========================================

// Sensor Data Packet (Input)
struct SensorData {
    float ultrasonic_front_cm;   // Distance from HC-SR04 ultrasonic sensor (cm)
    float lidar_front_cm;        // Clear space in front from LiDAR (cm)
    float lidar_left_cm;         // Clear space to the left from LiDAR (cm)
    float lidar_right_cm;        // Clear space to the right from LiDAR (cm)
    bool is_lidar_valid;         // LiDAR status (true if data is reliable)
    uint32_t timestamp_ms;       // Sampling time (used for velocity/time calculations)
};

// Motor Control Command (Output)
struct MotorCommand {
    float vx;     // Forward/Backward velocity (Positive = Forward, Negative = Backward)
    float vy;     // Strafing velocity (Positive = Right, Negative = Left)
    float omega;  // Angular velocity (Positive = Rotate Right/CW, Negative = Rotate Left/CCW)
};

// ==========================================
// 2. ALGORITHM LAYER (ONLINE TEAMMATE)
// To be implemented in: src/NavigationLogic.cpp
// ==========================================

// Determine movement decision based on environmental sensor data
MotorCommand computeAutonomousMove(const SensorData& sensor_in);

// ==========================================
// 3. HARDWARE LAYER (ON-SITE TEAMMATE)
// To be implemented in: src/HardwareDriver.cpp
// ==========================================

// Initialize GPIO pins, UART, I2C, and PCNT modules
void initHardware();

// Read and aggregate data from the ultrasonic sensor and LiDAR
SensorData readAllSensors();

// Convert velocity (vx, vy, omega) to PWM via Mecanum kinematics and send to Arduino
void executeMotorCommand(const MotorCommand& cmd);

#endif // ROBOT_INTERFACE_H