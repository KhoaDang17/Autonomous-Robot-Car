#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <Arduino.h>

// Initialize Hardware Serial (D0 RX / D1 TX) at UNO_BAUD to receive ESP32-C6 commands.
void initSerialComm();

// Parse incoming UART packets ("CMD:<vx>,<vy>,<omega>\n" or "STOP\n")
// and enforce the COMMAND_TIMEOUT_MS safety watchdog.
void processIncomingCommands();

#endif // SERIAL_COMM_H
