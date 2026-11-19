# Arduino Source Files (`Arduino/src/`) — Code Walkthrough

This document explains how every `.cpp` file inside `Arduino/src/` works block by block, so any team member (including beginners) can understand and explain the low-level firmware during the final project viva.

---

## 1. [`main.cpp`](main.cpp) — Program Entry Point

`main.cpp` intentionally contains minimal code because all complex logic is separated into `MotorDriver.cpp` and `SerialComm.cpp`.

- **`setup()`**:
  1. Calls `initSerialComm()` to open the `115200` bps UART port (`D0 RX` / `D1 TX`).
  2. Calls `initMotorDriver()` to configure `D2`/`D3` as outputs and immediately command `0` PWM to all 4 motors so the car never jerks on boot.
- **`loop()`**:
  - Repeatedly calls `processIncomingCommands()` with zero artificial `delay()`. This ensures incoming `CMD:...` packets from the ESP32-C6 are parsed immediately without UART buffer overflow, and the `500 ms` safety watchdog triggers on time.

---

## 2. [`MotorDriver.cpp`](MotorDriver.cpp) — STC8 Software I2C & Mecanum Kinematics

This file has two distinct layers:

### Part A: Bit-Banged Software I2C Protocol (Lines 8–55)
The Keyestudio KS0560 motor shield routes motor PWM signals through an onboard **STC8 microcontroller** at I2C address `0x30`, wired to digital pins **D3 (SDA)** and **D2 (SCL)** rather than Arduino's hardware I2C pins (`A4`/`A5`). Because of this pin layout, the driver implements I2C in software ("bit-banging"):

1. **`i2cStart()` & `i2cStop()`**: Generate the electrical START condition (pulling `SDA` `LOW` while `SCL` is `HIGH`) and STOP condition (releasing `SDA` `HIGH` while `SCL` is `HIGH`).
2. **`i2cSendByte(uint8_t data)`**: Loops 8 times, placing each bit from MSB (`0x80`) to LSB onto `SDA` while pulsing `SCL` `HIGH` to clock it into the STC8 chip.
3. **`i2cReceiveAck()`**: Temporarily switches `SDA` to `INPUT` on the 9th clock cycle to read the acknowledge bit from the STC8 chip, then switches `SDA` back to `OUTPUT`.
4. **`writeStc8Register(uint8_t reg_addr, uint8_t value)`**: Sends a complete 3-byte I2C write transaction:
   - Byte 1: `(STC8_I2C_ADDR << 1)` = `0x60` (7-bit address `0x30` + Write bit `0`).
   - Byte 2: Target register address (`0x01` through `0x08`).
   - Byte 3: PWM duty cycle (`0`–`255`).

### Part B: Register Mapping & Single Motor Control (`setSingleMotor`)
Each motor H-bridge uses two STC8 registers (`reg_in1`, `reg_in2`):
- **Motor 1 (Front Right):** `0x01`, `0x02`
- **Motor 2 (Front Left):** `0x03`, `0x04`
- **Motor 3 (Rear Right):** `0x05`, `0x06`
- **Motor 4 (Rear Left):** `0x07`, `0x08`

Setting `reg_in1 = 0, reg_in2 = pwm` spins the wheel **forward**; setting `reg_in1 = pwm, reg_in2 = 0` spins it **backward**; setting both to `0` **stops** the wheel.

### Part C: 4WD Mecanum Inverse Kinematics (`driveMecanum`)
A Mecanum robot moves in any direction $(v_x, v_y, \omega)$ by mixing wheel velocities using these equations:

$$\begin{aligned}
\text{Front Left (FL)}  &= v_x + v_y + \omega \\
\text{Front Right (FR)} &= v_x - v_y - \omega \\
\text{Rear Left (RL)}   &= v_x - v_y + \omega \\
\text{Rear Right (RR)}  &= v_x + v_y - \omega
\end{aligned}$$

The internal `applyWheel` helper inspects the sign of each computed wheel speed (`>= 0` means forward, `< 0` means backward), takes `fabs()` clamped to `[0, 255]`, and passes it to `setSingleMotor()`.

---

## 3. [`SerialComm.cpp`](SerialComm.cpp) — UART Parser & Safety Watchdog

This file keeps track of two static state variables:
- `last_command_time_ms`: Timestamp (`millis()`) when the most recent valid command arrived.
- `is_command_active`: Tracks whether the motors are currently commanded to move (`true` after a `CMD:` packet, `false` after `STOP` or timeout).

### How `processIncomingCommands()` Works Step by Step:
1. **Read Available Lines (`while (Serial.available() > 0)`):**
   - Calls `Serial.readStringUntil('\n')` and trims whitespace/carriage returns (`\r`).
2. **Parse `CMD:<vx>,<vy>,<omega>`:**
   - Finds the two commas inside the payload string, converts the three substrings to `float` (`vx`, `vy`, `omega`), calls `driveMecanum(vx, vy, omega)`, resets `last_command_time_ms = millis()`, and sets `is_command_active = true`.
3. **Parse `STOP`:**
   - Calls `stopMotors()`, updates `last_command_time_ms`, and sets `is_command_active = false`.
4. **Watchdog Timeout Check:**
   - If `is_command_active` is `true` and `(millis() - last_command_time_ms > COMMAND_TIMEOUT_MS)` (`500 ms`), it automatically executes `stopMotors()` and clears `is_command_active = false`. This ensures the car never keeps driving blindly if the ESP32-C6 crashes or a UART wire comes loose.
