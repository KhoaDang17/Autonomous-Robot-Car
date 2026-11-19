#include "SerialComm.h"
#include "PinConfig.h"
#include "MotorDriver.h"

// Watchdog state to track the most recent valid command from ESP32-C6
static unsigned long last_command_time_ms = 0;
static bool is_command_active = false;

void initSerialComm() {
    Serial.begin(UNO_BAUD);
    Serial.setTimeout(20); // Keep readStringUntil() responsive without long blocking
    last_command_time_ms = millis();
}

void processIncomingCommands() {
    // Read all complete newline-terminated packets currently in the UART buffer
    while (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();

        if (line.startsWith(F("CMD:"))) {
            // Expected payload format: "CMD:<vx>,<vy>,<omega>"
            String payload = line.substring(4);
            int first_comma = payload.indexOf(',');
            int second_comma = payload.indexOf(',', first_comma + 1);

            if (first_comma > 0 && second_comma > first_comma) {
                float vx = payload.substring(0, first_comma).toFloat();
                float vy = payload.substring(first_comma + 1, second_comma).toFloat();
                float omega = payload.substring(second_comma + 1).toFloat();

                driveMecanum(vx, vy, omega);
                last_command_time_ms = millis();
                is_command_active = true;
            }
        } else if (line.equalsIgnoreCase(F("STOP"))) {
            stopMotors();
            last_command_time_ms = millis();
            is_command_active = false;
        }
    }

    // Safety watchdog: stop all motors if ESP32-C6 link drops for > COMMAND_TIMEOUT_MS
    if (is_command_active && (millis() - last_command_time_ms > COMMAND_TIMEOUT_MS)) {
        stopMotors();
        is_command_active = false;
    }
}
