#include <Arduino.h>
#include "MotorDriver.h"
#include "SerialComm.h"

void setup() {
    initSerialComm();  // Start Hardware Serial (D0 RX / D1 TX) at 115200 bps
    initMotorDriver(); // Initialize STC8 I2C motor controller and stop all motors
}

void loop() {
    // Continuously listen for UART velocity commands from ESP32-C6
    // and enforce the 500 ms safety watchdog timeout
    processIncomingCommands();
}
