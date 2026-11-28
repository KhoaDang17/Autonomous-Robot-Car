#include <Arduino.h>
#include "ESP32PinConfig.h"
#include "RobotInterface.h"
#include "ArduinoLink.h"
#include "NavigationLogic.h"

void setup() {
    Serial.begin(DEBUG_SERIAL_BAUD); // USB-C Serial Monitor for debugging
    initHardware();                  // Init 3x Ultrasonic, RPLIDAR C1, and Arduino UART0
    delay(2000);                     // Allow sensors and Arduino Uno to stabilize
}

void loop() {
    // 1. Collect sensor distances from 3 trapezoid Ultrasonics + RPLIDAR C1
    SensorData current_env = readAllSensors();

    // 2. Compute optimal Mecanum velocity (vx, vy, omega)
    MotorCommand action = computeAutonomousMove(current_env);

    // 3. Send motion command over UART0 to Arduino Uno
    executeMotorCommand(action);

    // 4. Maintain ~20 Hz control loop pacing
    delay(CONTROL_LOOP_INTERVAL_MS);
}
