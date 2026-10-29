#include <Arduino.h>
#include "RobotInterface.h"

void setup() {
    Serial.begin(115200);
    initHardware(); // Initialize hardware
    delay(2000);    // Wait for the system to stabilize
}

void loop() {
    // Step 1: Collect real-world sensor data
    SensorData current_env = readAllSensors(); 
    
    // Step 2: Make autonomous decisions based on the data
    MotorCommand action = computeAutonomousMove(current_env); 
    
    // Step 3: Convert the decision into motor commands (PWM)
    executeMotorCommand(action); 
    
    delay(50); // Run the loop at 20Hz (50ms delay)
}