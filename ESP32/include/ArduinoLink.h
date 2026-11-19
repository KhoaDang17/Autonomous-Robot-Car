#ifndef ARDUINO_LINK_H
#define ARDUINO_LINK_H

#include <Arduino.h>
#include "RobotInterface.h"

// Initialize Hardware UART0 (UNO_RX_PIN / UNO_TX_PIN) at UNO_BAUD (115200 bps).
void initArduinoLink();

// Transmit a velocity command packet "CMD:<vx>,<vy>,<omega>\n" to Arduino Uno.
void sendCommandToArduino(const MotorCommand& cmd);

// Transmit an immediate emergency brake packet "STOP\n" to Arduino Uno.
void sendStopToArduino();

#endif // ARDUINO_LINK_H
