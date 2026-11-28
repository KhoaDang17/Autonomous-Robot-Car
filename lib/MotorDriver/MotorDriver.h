#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// ============================================================================
// LOW-LEVEL MOTOR INITIALIZATION & SINGLE-CHANNEL CONTROL
// ============================================================================

// Configure software I2C pins (SDA/SCL) and stop all four motors on startup.
void initMotorDriver();

// Drive one DC motor (id: 1..4) in a given direction at PWM speed (0..255).
void setSingleMotor(uint8_t motor_id, bool forward, uint8_t pwm);

// ============================================================================
// HIGH-LEVEL MECANUM MOVEMENT HELPERS (MILESTONE 2)
// ============================================================================

void moveForward(uint8_t speed);   // Move straight ahead
void moveBackward(uint8_t speed);  // Move straight backward
void turnLeft(uint8_t speed);      // Arc turn left while moving forward
void turnRight(uint8_t speed);     // Arc turn right while moving forward
void strafeLeft(uint8_t speed);    // Slide sideways left (Mecanum)
void strafeRight(uint8_t speed);   // Slide sideways right (Mecanum)
void rotateLeft(uint8_t speed);    // Spin counter-clockwise in place
void rotateRight(uint8_t speed);   // Spin clockwise in place
void stopMotors();                 // Brake/stop all four motors

// Convert body velocity (vx, vy, omega) into individual 4-wheel PWM commands.
void driveMecanum(float vx, float vy, float omega);

#endif // MOTOR_DRIVER_H
