#include "MotorDriver.h"
#include "PinConfig.h"
#include <math.h>

// ============================================================================
// INTERNAL SOFTWARE I2C PROTOCOL FOR STC8 MOTOR COPROCESSOR (0x30)
// ============================================================================

// Generate I2C START condition (SDA falls while SCL is HIGH).
static void i2cStart() {
    digitalWrite(STC8_SDA_PIN, HIGH); delayMicroseconds(1);
    digitalWrite(STC8_SCL_PIN, HIGH); delayMicroseconds(1);
    digitalWrite(STC8_SDA_PIN, LOW);  delayMicroseconds(1);
    digitalWrite(STC8_SCL_PIN, LOW);  delayMicroseconds(1);
}

// Generate I2C STOP condition (SDA rises while SCL is HIGH).
static void i2cStop() {
    digitalWrite(STC8_SCL_PIN, LOW);  delayMicroseconds(1);
    digitalWrite(STC8_SDA_PIN, LOW);  delayMicroseconds(1);
    digitalWrite(STC8_SCL_PIN, HIGH); delayMicroseconds(1);
    digitalWrite(STC8_SDA_PIN, HIGH); delayMicroseconds(1);
}

// Release SDA briefly on the 9th clock pulse to read the slave ACK bit.
static bool i2cReceiveAck() {
    digitalWrite(STC8_SCL_PIN, LOW);
    pinMode(STC8_SDA_PIN, INPUT);
    digitalWrite(STC8_SCL_PIN, HIGH); delayMicroseconds(1);
    bool ack = digitalRead(STC8_SDA_PIN);
    digitalWrite(STC8_SCL_PIN, LOW);
    pinMode(STC8_SDA_PIN, OUTPUT);
    return ack;
}

// Shift out 8 bits (MSB first) synchronized with the SCL clock line.
static void i2cSendByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(STC8_SCL_PIN, LOW); delayMicroseconds(1);
        digitalWrite(STC8_SDA_PIN, (data & 0x80) ? HIGH : LOW);
        data <<= 1;
        delayMicroseconds(1);
        digitalWrite(STC8_SCL_PIN, HIGH); delayMicroseconds(1);
    }
}

// Write one PWM register value to the STC8 chip over software I2C.
static void writeStc8Register(uint8_t reg_addr, uint8_t value) {
    i2cStart();
    i2cSendByte(STC8_I2C_ADDR << 1); // 7-bit address + Write bit (0)
    i2cReceiveAck();
    i2cSendByte(reg_addr);
    i2cReceiveAck();
    i2cSendByte(value);
    i2cReceiveAck();
    i2cStop();
}

// ============================================================================
// PUBLIC MOTOR DRIVER IMPLEMENTATION
// ============================================================================

void initMotorDriver() {
    pinMode(STC8_SDA_PIN, OUTPUT);
    pinMode(STC8_SCL_PIN, OUTPUT);
    stopMotors(); // Ensure motors remain stationary on power-up
}

void setSingleMotor(uint8_t motor_id, bool forward, uint8_t pwm) {
    uint8_t reg_in1 = 0;
    uint8_t reg_in2 = 0;

    // Map motor ID to its H-bridge register pair on the STC8 chip
    if (motor_id == MOTOR_FRONT_RIGHT)     { reg_in1 = 0x01; reg_in2 = 0x02; }
    else if (motor_id == MOTOR_FRONT_LEFT) { reg_in1 = 0x03; reg_in2 = 0x04; }
    else if (motor_id == MOTOR_REAR_RIGHT) { reg_in1 = 0x05; reg_in2 = 0x06; }
    else if (motor_id == MOTOR_REAR_LEFT)  { reg_in1 = 0x07; reg_in2 = 0x08; }
    else return;

    if (forward) {
        writeStc8Register(reg_in1, 0);
        writeStc8Register(reg_in2, pwm);
    } else {
        writeStc8Register(reg_in1, pwm);
        writeStc8Register(reg_in2, 0);
    }
}

void moveForward(uint8_t speed) {
    setSingleMotor(MOTOR_FRONT_RIGHT, true, speed);
    setSingleMotor(MOTOR_FRONT_LEFT,  true, speed);
    setSingleMotor(MOTOR_REAR_RIGHT,  true, speed);
    setSingleMotor(MOTOR_REAR_LEFT,   true, speed);
}

void moveBackward(uint8_t speed) {
    setSingleMotor(MOTOR_FRONT_RIGHT, false, speed);
    setSingleMotor(MOTOR_FRONT_LEFT,  false, speed);
    setSingleMotor(MOTOR_REAR_RIGHT,  false, speed);
    setSingleMotor(MOTOR_REAR_LEFT,   false, speed);
}

void turnLeft(uint8_t speed) {
    uint8_t inner_speed = speed / 3;
    setSingleMotor(MOTOR_FRONT_RIGHT, true, speed);
    setSingleMotor(MOTOR_FRONT_LEFT,  true, inner_speed);
    setSingleMotor(MOTOR_REAR_RIGHT,  true, speed);
    setSingleMotor(MOTOR_REAR_LEFT,   true, inner_speed);
}

void turnRight(uint8_t speed) {
    uint8_t inner_speed = speed / 3;
    setSingleMotor(MOTOR_FRONT_RIGHT, true, inner_speed);
    setSingleMotor(MOTOR_FRONT_LEFT,  true, speed);
    setSingleMotor(MOTOR_REAR_RIGHT,  true, inner_speed);
    setSingleMotor(MOTOR_REAR_LEFT,   true, speed);
}

void strafeLeft(uint8_t speed) {
    setSingleMotor(MOTOR_FRONT_RIGHT, true,  speed);
    setSingleMotor(MOTOR_FRONT_LEFT,  false, speed);
    setSingleMotor(MOTOR_REAR_RIGHT,  false, speed);
    setSingleMotor(MOTOR_REAR_LEFT,   true,  speed);
}

void strafeRight(uint8_t speed) {
    setSingleMotor(MOTOR_FRONT_RIGHT, false, speed);
    setSingleMotor(MOTOR_FRONT_LEFT,  true,  speed);
    setSingleMotor(MOTOR_REAR_RIGHT,  true,  speed);
    setSingleMotor(MOTOR_REAR_LEFT,   false, speed);
}

void rotateLeft(uint8_t speed) {
    setSingleMotor(MOTOR_FRONT_RIGHT, true,  speed);
    setSingleMotor(MOTOR_FRONT_LEFT,  false, speed);
    setSingleMotor(MOTOR_REAR_RIGHT,  true,  speed);
    setSingleMotor(MOTOR_REAR_LEFT,   false, speed);
}

void rotateRight(uint8_t speed) {
    setSingleMotor(MOTOR_FRONT_RIGHT, false, speed);
    setSingleMotor(MOTOR_FRONT_LEFT,  true,  speed);
    setSingleMotor(MOTOR_REAR_RIGHT,  false, speed);
    setSingleMotor(MOTOR_REAR_LEFT,   true,  speed);
}

void stopMotors() {
    for (uint8_t id = 1; id <= 4; id++) {
        setSingleMotor(id, true, 0);
    }
}

void driveMecanum(float vx, float vy, float omega) {
    // 4WD Mecanum inverse kinematics (vx: forward, vy: right, omega: CW)
    float fl = vx + vy + omega;
    float fr = vx - vy - omega;
    float rl = vx - vy + omega;
    float rr = vx + vy - omega;

    // Clamp each wheel speed to [0, 255] and extract rotation direction
    auto applyWheel = [](uint8_t motor_id, float signed_speed) {
        bool forward = (signed_speed >= 0.0f);
        float abs_val = fabs(signed_speed);
        if (abs_val > 255.0f) abs_val = 255.0f;
        setSingleMotor(motor_id, forward, static_cast<uint8_t>(abs_val));
    };

    applyWheel(MOTOR_FRONT_LEFT,  fl);
    applyWheel(MOTOR_FRONT_RIGHT, fr);
    applyWheel(MOTOR_REAR_LEFT,   rl);
    applyWheel(MOTOR_REAR_RIGHT,  rr);
}
