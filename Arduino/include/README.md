# Arduino Header Files (`Arduino/include/`)

This folder contains all `.h` header files that define hardware constants and public function interfaces for the **Arduino Uno Motor Controller Slave**.

---

## 1. [`PinConfig.h`](PinConfig.h) — Hardware Configuration & Constants

Centralizes every pin number, communication speed, and safety threshold so you never have to hunt through `.cpp` files to change a pin.

| Macro | Value | Purpose |
| :--- | :---: | :--- |
| `UNO_BAUD` | `115200` | Baud rate for Hardware `Serial` (`D0 RX` / `D1 TX`) linked to ESP32-C6 `UNO_TX_PIN` / `UNO_RX_PIN`. |
| `COMMAND_TIMEOUT_MS` | `500UL` | Safety watchdog timeout (in ms). If no new UART command arrives within `500 ms`, all motors stop automatically. |
| `STC8_SDA_PIN` | `3` | Arduino Digital Pin **D3** used as software I2C Data (`SDA`) to the KS0560 motor driver chip. |
| `STC8_SCL_PIN` | `2` | Arduino Digital Pin **D2** used as software I2C Clock (`SCL`) to the KS0560 motor driver chip. |
| `STC8_I2C_ADDR` | `0x30` | 7-bit I2C address of the onboard STC8 motor coprocessor. |
| `MOTOR_FRONT_RIGHT` | `1` | Channel ID `1` on STC8 shield (registers `0x01`, `0x02`). |
| `MOTOR_FRONT_LEFT` | `2` | Channel ID `2` on STC8 shield (registers `0x03`, `0x04`). |
| `MOTOR_REAR_RIGHT` | `3` | Channel ID `3` on STC8 shield (registers `0x05`, `0x06`). |
| `MOTOR_REAR_LEFT` | `4` | Channel ID `4` on STC8 shield (registers `0x07`, `0x08`). |
| `DEFAULT_MOTOR_SPEED` | `100` | Default PWM value (`0`–`255`) useful during Milestone 2 bench tests. |

---

## 2. [`MotorDriver.h`](MotorDriver.h) — STC8 Driver & Mecanum Motion API

Declares low-level motor initialization, single-motor PWM control, directional helpers (for Milestone 2), and full 4WD Mecanum kinematics.

### Function Reference & Usage Examples

#### `void initMotorDriver()`
- **Purpose:** Configures `STC8_SDA_PIN` (`D3`) and `STC8_SCL_PIN` (`D2`) as outputs and immediately sets all 4 motors to `0` PWM.
- **When to call:** Once inside `setup()`.
```cpp
void setup() {
    initMotorDriver();
}
```

#### `void setSingleMotor(uint8_t motor_id, bool forward, uint8_t pwm)`
- **Purpose:** Drives one individual motor channel. Useful for verifying wheel wiring and spinning direction during **Milestone 2**.
- **Parameters:**
  - `motor_id`: `MOTOR_FRONT_RIGHT` (`1`), `MOTOR_FRONT_LEFT` (`2`), `MOTOR_REAR_RIGHT` (`3`), or `MOTOR_REAR_LEFT` (`4`).
  - `forward`: `true` = spin forward, `false` = spin backward.
  - `pwm`: Speed from `0` (stopped) to `255` (full speed).
```cpp
// Spin only the Front-Left wheel forward at medium speed (120/255)
setSingleMotor(MOTOR_FRONT_LEFT, true, 120);
```

#### High-Level Directional Helpers (Milestone 2 Testing)
All helper functions accept `uint8_t speed` (`0`–`255`) and coordinate all 4 wheels automatically:

| Function | Movement Result | Example Call |
| :--- | :--- | :--- |
| `moveForward(speed)` | Drives straight forward | `moveForward(110);` |
| `moveBackward(speed)` | Drives straight backward | `moveBackward(100);` |
| `turnLeft(speed)` | Forward arc turn to the left (inner wheels at `speed / 3`) | `turnLeft(120);` |
| `turnRight(speed)` | Forward arc turn to the right (inner wheels at `speed / 3`) | `turnRight(120);` |
| `strafeLeft(speed)` | Pure sideways slide left using Mecanum rollers | `strafeLeft(110);` |
| `strafeRight(speed)` | Pure sideways slide right using Mecanum rollers | `strafeRight(110);` |
| `rotateLeft(speed)` | Spins counter-clockwise (CCW) in place | `rotateLeft(100);` |
| `rotateRight(speed)` | Spins clockwise (CW) in place | `rotateRight(100);` |
| `stopMotors()` | Sets PWM to `0` on all 4 channels | `stopMotors();` |

#### `void driveMecanum(float vx, float vy, float omega)`
- **Purpose:** The primary motor interface used by `SerialComm` when receiving commands from the ESP32-C6. Combines forward/backward motion, sideways strafing, and rotation simultaneously.
- **Parameters (`-255.0` to `+255.0`):**
  - `vx`: Forward (`> 0`) / Backward (`< 0`).
  - `vy`: Strafe Right (`> 0`) / Strafe Left (`< 0`).
  - `omega`: Rotate Clockwise (`> 0`) / Counter-Clockwise (`< 0`).
```cpp
// Example 1: Move forward at speed 110
driveMecanum(110.0f, 0.0f, 0.0f);

// Example 2: Move forward diagonally while rotating slightly right
driveMecanum(90.0f, 40.0f, 20.0f);

// Example 3: Stop all wheels
driveMecanum(0.0f, 0.0f, 0.0f);
```

---

## 3. [`SerialComm.h`](SerialComm.h) — UART Slave Communication API

Handles incoming motion packets from the FireBeetle 2 ESP32-C6 over Hardware `Serial`.

### Function Reference & Usage Examples

#### `void initSerialComm()`
- **Purpose:** Starts `Serial.begin(UNO_BAUD)` (`115200` bps), sets a short `20 ms` read timeout so `readStringUntil('\n')` never stalls the loop, and resets the watchdog timer.
- **When to call:** Once inside `setup()`.

#### `void processIncomingCommands()`
- **Purpose:** Reads any complete command line from `Serial`, executes `driveMecanum()` or `stopMotors()`, and automatically stops the motors if no packet has been received in the last `500 ms`.
- **When to call:** Continuously on every iteration of `loop()`.
```cpp
void setup() {
    initSerialComm();
    initMotorDriver();
}

void loop() {
    processIncomingCommands();
}
```
