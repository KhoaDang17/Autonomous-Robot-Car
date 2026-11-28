#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <stdint.h>

// ============================================================================
// 1. COMMON DATA STRUCTURES
// Shared contract between the Hardware Layer and the Algorithm Layer
// ============================================================================

// Aggregated environmental distances (Input to NavigationLogic)
struct SensorData {
    float ultrasonic_left_cm;    // Front-Left angled ultrasonic sensor (cm)
    float ultrasonic_front_cm;   // Front-Center ultrasonic sensor (cm)
    float ultrasonic_right_cm;   // Front-Right angled ultrasonic sensor (cm)
    float lidar_front_cm;        // Clear space ahead from RPLIDAR C1 (cm)
    float lidar_left_cm;         // Clear space to the left from RPLIDAR C1 (cm)
    float lidar_right_cm;        // Clear space to the right from RPLIDAR C1 (cm)
    bool is_lidar_valid;         // True when fresh RPLIDAR C1 scan data is available
    uint32_t timestamp_ms;       // Sampling timestamp in milliseconds (millis())
};

// Mecanum body velocity command (Output from NavigationLogic)
struct MotorCommand {
    float vx;     // Forward/Backward velocity (Positive = Forward, Negative = Backward)
    float vy;     // Strafing velocity (Positive = Strafe Right, Negative = Strafe Left)
    float omega;  // Angular velocity (Positive = Rotate CW/Right, Negative = Rotate CCW/Left)
};

// ============================================================================
// 2. ALGORITHM LAYER (lib/NavigationLogic/NavigationLogic.cpp)
// ============================================================================

// Compute the next movement decision from fused Ultrasonic + 2D LiDAR readings.
MotorCommand computeAutonomousMove(const SensorData& sensor_in);

// ============================================================================
// 3. HARDWARE LAYER (lib/ArduinoLink/ArduinoLink.cpp,
//                    lib/UltrasonicSensor/UltrasonicSensor.cpp,
//                    lib/LidarSensor/LidarSensor.cpp)
// ============================================================================

// Initialize Ultrasonic GPIOs, RPLIDAR C1 UART1, and Arduino Uno UART0.
void initHardware();

// Poll all 3 trapezoid Ultrasonic sensors and the 2D LiDAR into one SensorData packet.
SensorData readAllSensors();

// Transmit the velocity command (vx, vy, omega) over UART0 to Arduino Uno.
void executeMotorCommand(const MotorCommand& cmd);

#endif // ROBOT_INTERFACE_H
