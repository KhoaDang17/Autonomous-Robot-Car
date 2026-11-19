#include "ArduinoLink.h"
#include "UltrasonicSensor.h"
#include "LidarSensor.h"
#include "PinConfig.h"

void initArduinoLink() {
    // Hardware UART0 dedicated to Arduino Uno communication (RX=4, TX=5, 115200 bps)
    Serial0.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);
}

void sendCommandToArduino(const MotorCommand& cmd) {
    // Protocol expected by Arduino Uno SerialComm: "CMD:<vx>,<vy>,<omega>\n"
    Serial0.print("CMD:");
    Serial0.print(cmd.vx, 1);
    Serial0.print(",");
    Serial0.print(cmd.vy, 1);
    Serial0.print(",");
    Serial0.println(cmd.omega, 1);
}

void sendStopToArduino() {
    Serial0.println("STOP");
}

// ============================================================================
// ROBOT INTERFACE HARDWARE LAYER IMPLEMENTATION
// ============================================================================

void initHardware() {
    initUltrasonicSensors();
    initLidarSensor();
    initArduinoLink();
}

SensorData readAllSensors() {
    SensorData data;

    // 1. Read the 3 front trapezoid HC-SR04 ultrasonic sensors
    UltrasonicArrayData us = readAllUltrasonicSensors();
    data.ultrasonic_left_cm  = us.left_cm;
    data.ultrasonic_front_cm = us.center_cm;
    data.ultrasonic_right_cm = us.right_cm;

    // 2. Read the 360-degree RPLIDAR C1 sector clearances
    LidarSectorSummary lidar = readLidarSectors();
    data.lidar_front_cm = lidar.front_cm;
    data.lidar_left_cm  = lidar.left_cm;
    data.lidar_right_cm = lidar.right_cm;
    data.is_lidar_valid = lidar.is_valid;

    data.timestamp_ms = millis();
    return data;
}

void executeMotorCommand(const MotorCommand& cmd) {
    sendCommandToArduino(cmd);
}
