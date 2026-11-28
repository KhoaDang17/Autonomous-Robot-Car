# Firmware Libraries (`lib/`) — API Reference & Code Walkthrough

All modular firmware components are organized under `lib/` as private PlatformIO libraries. Each target environment in [`platformio.ini`](../platformio.ini) compiles only the modules it needs using `lib_ignore`.

---

## 1. ESP32-C6 Libraries (`env:firebeetle2_esp32c6`)

To let team members work on **navigation algorithms** and **hardware drivers** in parallel without merge conflicts, the ESP32-C6 codebase is split cleanly across [`RobotInterface.h`](RobotInterface/RobotInterface.h):

| Layer | Library | Files | Responsibility |
| :--- | :--- | :--- | :--- |
| **Interface Contract** | `RobotInterface` | [`RobotInterface/RobotInterface.h`](RobotInterface/RobotInterface.h) | Defines `SensorData` (inputs), `MotorCommand` (outputs), and layer functions. |
| **Algorithm Layer** | `NavigationLogic` | [`NavigationLogic/NavigationLogic.h`](NavigationLogic/NavigationLogic.h)<br>[`NavigationLogic/NavigationLogic.cpp`](NavigationLogic/NavigationLogic.cpp) | Pure decision-making: `computeAutonomousMove(const SensorData&)`. |
| **Hardware Layer** | `UltrasonicSensor` | [`UltrasonicSensor/UltrasonicSensor.h`](UltrasonicSensor/UltrasonicSensor.h)<br>[`UltrasonicSensor/UltrasonicSensor.cpp`](UltrasonicSensor/UltrasonicSensor.cpp) | Sequential 3-sensor trapezoid HC-SR04 polling & timeout safety. |
| **Hardware Layer** | `LidarSensor` | [`LidarSensor/LidarSensor.h`](LidarSensor/LidarSensor.h)<br>[`LidarSensor/LidarSensor.cpp`](LidarSensor/LidarSensor.cpp) | Slamtec RPLIDAR C1 `UART1` initialization & sector clearances. |
| **Hardware Layer** | `ArduinoLink` | [`ArduinoLink/ArduinoLink.h`](ArduinoLink/ArduinoLink.h)<br>[`ArduinoLink/ArduinoLink.cpp`](ArduinoLink/ArduinoLink.cpp) | Aggregates `readAllSensors()` and transmits `"CMD:..."` over `UART0`. |

---

### 1.1 [`RobotInterface/RobotInterface.h`](RobotInterface/RobotInterface.h) — Core Data Contracts

```cpp
struct SensorData {
    float ultrasonic_left_cm;    // Front-Left angled ultrasonic (cm)
    float ultrasonic_front_cm;   // Front-Center ultrasonic (cm)
    float ultrasonic_right_cm;   // Front-Right angled ultrasonic (cm)
    float lidar_front_cm;        // RPLIDAR C1 front sector minimum (cm)
    float lidar_left_cm;         // RPLIDAR C1 left sector minimum (cm)
    float lidar_right_cm;        // RPLIDAR C1 right sector minimum (cm)
    bool is_lidar_valid;         // True if LiDAR packets are actively arriving
    uint32_t timestamp_ms;       // Time sampled via millis()
};

struct MotorCommand {
    float vx;     // Forward (+) / Backward (-)
    float vy;     // Strafe Right (+) / Strafe Left (-)
    float omega;  // Rotate CW Right (+) / CCW Left (-)
};
```

```cpp
void setup() {
    initHardware(); // Sets up 3x Ultrasonic, RPLIDAR C1 UART1, and Arduino UART0
}

void loop() {
    SensorData env = readAllSensors();               // Hardware Layer
    MotorCommand cmd = computeAutonomousMove(env);   // Algorithm Layer
    executeMotorCommand(cmd);                        // Hardware Layer
    delay(CONTROL_LOOP_INTERVAL_MS);
}
```

---

### 1.2 [`UltrasonicSensor/`](UltrasonicSensor/) — 3-Sensor Trapezoid Array Driver

Manages the 3 front HC-SR04 sensors wired to `TRIG_PINS[3] = {18, 14, 3}` and `ECHO_PINS[3] = {1, 2, 6}`:

| Function / Struct | Description | Usage Example |
| :--- | :--- | :--- |
| `struct UltrasonicArrayData` | Holds `left_cm`, `center_cm`, `right_cm`. | `UltrasonicArrayData us = readAllUltrasonicSensors();` |
| `initUltrasonicSensors()` | Configures all `TRIG_PINS` (`OUTPUT`, `LOW`) and `ECHO_PINS` (`INPUT`). | `initUltrasonicSensors();` |
| `readSingleUltrasonicCm(idx)` | Reads one sensor (`US_LEFT_IDX`, `US_CENTER_IDX`, `US_RIGHT_IDX`). | `float front = readSingleUltrasonicCm(US_CENTER_IDX);` |
| `readAllUltrasonicSensors()` | Sequentially polls Left $\rightarrow$ Center $\rightarrow$ Right with `5 ms` cross-talk delay. | `UltrasonicArrayData d = readAllUltrasonicSensors();` |
| `isObstacleDetected(dist, thresh)` | Returns `true` if `0 < dist < thresh`. | `if (isObstacleDetected(front, 30.0f)) { ... }` |

**Internal Walkthrough (`UltrasonicSensor.cpp`):**
1. **`readSingleUltrasonicCm(uint8_t sensor_index)`** sends a $10\ \mu\text{s}$ `HIGH` trigger pulse and measures the echo width via `pulseIn(echo_pin, HIGH, ULTRASONIC_TIMEOUT_US)` (`25000 us` cap).
2. **Invalid/Timeout Handling (Milestone 5):** If `duration_us == 0` (no echo returned within $25\text{ ms}$) or the computed distance exceeds `400.0 cm`, it returns `MAX_VALID_DISTANCE_CM` (`400.0f`) instead of `0.0f`, preventing false emergency stops in wide rooms.
3. **Distance Formula:** $\text{distance (cm)} = \frac{\text{duration\_us} \times 0.0343}{2}$.
4. **`readAllUltrasonicSensors()`** waits `ULTRASONIC_INTER_PING_MS` (`5 ms`) between firing adjacent sensors so residual sound waves dissipate before the next receiver listens (preventing acoustic cross-talk).

---

### 1.3 [`LidarSensor/`](LidarSensor/) — Slamtec RPLIDAR C1 Driver

| Function / Struct | Description | Usage Example |
| :--- | :--- | :--- |
| `struct LidarSectorSummary` | Holds `front_cm`, `left_cm`, `right_cm`, and `is_valid`. | `LidarSectorSummary lidar = readLidarSectors();` |
| `initLidarSensor()` | Initializes `Serial1` on `GPIO 17/16` at `460800` bps (`LIDAR_BAUD`). | `initLidarSensor();` |
| `readLidarSectors()` | Parses scan frames and returns sector clearances. | `if (lidar.is_valid && lidar.left_cm > 50.0f) { ... }` |

**Internal Walkthrough (`LidarSensor.cpp`):**
- Holds a cached `LidarSectorSummary` (`front_cm`, `left_cm`, `right_cm`, `is_valid`).
- Structured as a Milestone 6 extension hook: once the RPLIDAR C1 scan packet parser decodes `(angle, distance_mm)` points from `Serial1`, it updates the minimum obstacle distance for:
  - **Front cone:** $330^\circ \dots 360^\circ$ and $0^\circ \dots 30^\circ$
  - **Right flank:** $30^\circ \dots 120^\circ$
  - **Left flank:** $240^\circ \dots 330^\circ$
- Sets `is_valid = true` so `NavigationLogic.cpp` activates full Ultrasonic + LiDAR fusion. While `is_valid == false`, the robot automatically falls back to the 3 trapezoid ultrasonic sensors without crashing.

---

### 1.4 [`ArduinoLink/`](ArduinoLink/) — Hardware Aggregator & UART0 Link

| Function | Description | Usage Example |
| :--- | :--- | :--- |
| `initArduinoLink()` | Initializes `Serial0` on `GPIO 4/5` at `115200` bps (`UNO_BAUD`). | `initArduinoLink();` |
| `sendCommandToArduino(cmd)` | Sends `"CMD:<vx>,<vy>,<omega>\n"` over `UART0`. | `MotorCommand c = {110, 0, 0}; sendCommandToArduino(c);` |
| `sendStopToArduino()` | Sends `"STOP\n"` for immediate emergency halt. | `sendStopToArduino();` |
| `initHardware()` | Initializes Ultrasonic GPIOs, RPLIDAR C1 `Serial1`, and Arduino `Serial0`. | `initHardware();` |
| `readAllSensors()` | Merges `readAllUltrasonicSensors()` + `readLidarSectors()` into `SensorData`. | `SensorData env = readAllSensors();` |
| `executeMotorCommand(cmd)` | Transmits `cmd` to Arduino Uno via `sendCommandToArduino(cmd)`. | `executeMotorCommand(cmd);` |

---

### 1.5 [`NavigationLogic/`](NavigationLogic/) — Multisensor Collision-Avoidance Engine

```cpp
// Example: Unit-testing NavigationLogic with synthetic SensorData (no hardware needed!)
SensorData mock_env = {
    45.0f, // ultrasonic_left_cm
    25.0f, // ultrasonic_front_cm (< 35 cm -> obstacle ahead!)
    80.0f, // ultrasonic_right_cm
    26.0f, // lidar_front_cm
    40.0f, // lidar_left_cm
    95.0f, // lidar_right_cm (right is wider -> should rotate right!)
    true,  // is_lidar_valid
    1000   // timestamp_ms
};

MotorCommand decision = computeAutonomousMove(mock_env);
// Result: decision.vx == 0.0f, decision.vy == 0.0f, decision.omega == +100.0f (Rotate Right)
```

**5-Stage Decision Cascade (`NavigationLogic.cpp`):**
1. **Stage 1 — Conservative Multisensor Fusion:** Takes `min(ultrasonic, lidar)` for the Front, Left, and Right sectors whenever `is_lidar_valid` is `true`. Ultrasonic detects **glass and black surfaces** that LiDAR lasers pass through or absorb, while 2D LiDAR detects **thin chair legs and wide-angle obstacles** outside ultrasonic cones.
2. **Stage 2 — Emergency Escape (`min_front_zone_cm < 18 cm`):** Immediately backs up (`vx = -80`) while rotating toward whichever side (`effective_left_cm` vs. `effective_right_cm`) has more open space.
3. **Stage 3 — Front-Center Blocked (`effective_front_cm < 35 cm`):** Stops forward travel (`vx = 0`) and rotates Left (`omega = -100`) if Left is wider and `> 25 cm`, Right (`omega = +100`) if Right `> 25 cm`, or spins clockwise in place in a dead-end.
4. **Stage 4 — Angled Side Avoidance (Left or Right Trapezoid Ultrasonic `< 35 cm`):** Slows forward speed (`vx = 55`), strafes sideways away from the obstacle (`vy = ±60`), and steers (`omega = ±50`) smoothly without stopping.
5. **Stage 5 — Cruise Forward:** When all front zones are clear (`>= 35 cm`), cruises straight ahead (`vx = 110, vy = 0, omega = 0`).

---

## 2. Arduino Uno Libraries (`env:uno`)

### 2.1 [`MotorDriver/`](MotorDriver/) — STC8 Software I2C & 4WD Mecanum Kinematics

#### Function Reference & Usage Examples

- **`void initMotorDriver()`**: Configures `STC8_SDA_PIN` (`D3`) and `STC8_SCL_PIN` (`D2`) as outputs and immediately sets all 4 motors to `0` PWM.
- **`void setSingleMotor(uint8_t motor_id, bool forward, uint8_t pwm)`**: Drives one individual motor channel (`MOTOR_FRONT_RIGHT = 1`, `MOTOR_FRONT_LEFT = 2`, `MOTOR_REAR_RIGHT = 3`, `MOTOR_REAR_LEFT = 4`) at `0..255` PWM.

| Helper Function | Movement Result | Example Call |
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
| `driveMecanum(vx, vy, omega)` | Mixes `(vx, vy, omega)` in `[-255, +255]` into all 4 wheels | `driveMecanum(90.0f, 40.0f, 20.0f);` |

**Internal Walkthrough (`MotorDriver.cpp`):**
1. **Bit-Banged Software I2C:** The Keyestudio KS0560 motor shield routes PWM signals through an onboard **STC8 microcontroller** at I2C address `0x30`, wired to digital pins **D3 (SDA)** and **D2 (SCL)** instead of Arduino's hardware I2C pins (`A4`/`A5`). `i2cStart()`, `i2cStop()`, `i2cSendByte()`, `i2cReceiveAck()`, and `writeStc8Register()` implement the 3-byte I2C register write (`0x60`, `reg_addr`, `pwm`).
2. **STC8 Register Mapping:** Motor 1 FR (`0x01`, `0x02`), Motor 2 FL (`0x03`, `0x04`), Motor 3 RR (`0x05`, `0x06`), Motor 4 RL (`0x07`, `0x08`).
3. **4WD Mecanum Inverse Kinematics (`driveMecanum`):**
   $$\begin{aligned}
   \text{Front Left (FL)}  &= v_x + v_y + \omega \\
   \text{Front Right (FR)} &= v_x - v_y - \omega \\
   \text{Rear Left (RL)}   &= v_x - v_y + \omega \\
   \text{Rear Right (RR)}  &= v_x + v_y - \omega
   \end{aligned}$$
   Each computed speed is clamped to `[0, 255]` with its sign mapped to forward (`>= 0`) or backward (`< 0`).

---

### 2.2 [`SerialComm/`](SerialComm/) — UART Packet Parser & Safety Watchdog

- **`void initSerialComm()`**: Starts `Serial.begin(UNO_BAUD)` (`115200` bps), sets a `20 ms` read timeout (`Serial.setTimeout(20)`), and initializes `last_command_time_ms = millis()`.
- **`void processIncomingCommands()`**: Reads complete newline-terminated lines from `Serial`, parses `CMD:<vx>,<vy>,<omega>` or `STOP`, and enforces the `500 ms` watchdog (`COMMAND_TIMEOUT_MS`): if `is_command_active` is `true` and `(millis() - last_command_time_ms > COMMAND_TIMEOUT_MS)`, `stopMotors()` is called automatically.

---

## 3. UART Command Protocol & Manual Motor Test (Arduino Uno)

Communication runs over Hardware UART (`115200` bps, 8N1, newline-terminated `\n`):

| Command Packet | Parameters | Meaning | Example |
| :--- | :--- | :--- | :--- |
| `CMD:<vx>,<vy>,<omega>\n` | `vx`: Forward (+) / Backward (-)<br>`vy`: Strafe Right (+) / Left (-)<br>`omega`: Rotate CW (+) / CCW (-)<br>*(Range: `-255.0` to `+255.0`)* | Drive all 4 Mecanum wheels simultaneously | `CMD:110.0,0.0,0.0\n` (Move forward)<br>`CMD:0.0,-100.0,0.0\n` (Strafe left)<br>`CMD:0.0,0.0,90.0\n` (Rotate clockwise) |
| `STOP\n` | None | Immediately stop all 4 motors | `STOP\n` |

> [!IMPORTANT]
> **Disconnect `D0 (RX)` and `D1 (TX)` wires before flashing the Arduino Uno!**
> The Arduino Uno shares Hardware `Serial` (`D0`/`D1`) with its USB-to-Serial programmer chip. Always unplug the jumper wires going to the ESP32-C6 before clicking **Upload**, then reconnect them afterward.

**Optional Manual Motor Test via USB Serial Monitor (`115200` baud, `Newline (\n)`):**
- Send `CMD:100,0,0` — all 4 wheels spin forward for `500 ms` and then stop automatically via the safety watchdog.
- Send `CMD:0,100,0` to test strafing right, `CMD:0,0,100` to test rotating clockwise, or `STOP` to halt immediately.
