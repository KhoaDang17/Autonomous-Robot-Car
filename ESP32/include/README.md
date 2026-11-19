# ESP32-C6 Header Files (`ESP32/include/`)

This folder contains all `.h` header files for the **DFRobot FireBeetle 2 ESP32-C6** firmware, including GPIO definitions, shared data contracts, and module APIs with usage examples.

---

## 1. [`PinConfig.h`](PinConfig.h) — Pin Mapping, Baud Rates & Thresholds

All hardware pins and tuning parameters are centralized here:

### UART Interfaces
| Macro | Value | Description |
| :--- | :---: | :--- |
| `DEBUG_SERIAL_BAUD` | `115200` | USB-C CDC Serial Monitor speed. |
| `LIDAR_RX_PIN` / `LIDAR_TX_PIN` | `17` / `16` | Hardware `Serial1` pins connected to **Slamtec RPLIDAR C1**. |
| `LIDAR_BAUD` | `460800` | Default UART baud rate for RPLIDAR C1. |
| `UNO_RX_PIN` / `UNO_TX_PIN` | `4` / `5` | Hardware `Serial0` pins connected to **Arduino Uno** (`D1 TX` / `D0 RX`). |
| `UNO_BAUD` | `115200` | Must match `UNO_BAUD` inside `Arduino/include/PinConfig.h`. |

### 3 Trapezoid Ultrasonic Sensors
| Array Index | Constant | `TRIG_PINS[i]` | `ECHO_PINS[i]` | Mounting Position |
| :---: | :--- | :---: | :---: | :--- |
| `0` | `US_LEFT_IDX` | `GPIO 18` | `GPIO 1` | Front-Left trapezoid angle |
| `1` | `US_CENTER_IDX` | `GPIO 14` | `GPIO 2` | Front-Center straight ahead |
| `2` | `US_RIGHT_IDX` | `GPIO 3` | `GPIO 6` | Front-Right trapezoid angle |

### Navigation & Safety Thresholds
- `ULTRASONIC_TIMEOUT_US` (`25000UL`): Caps `pulseIn()` wait time at $25\text{ ms}$ ($\approx 400\text{ cm}$ range).
- `MAX_VALID_DISTANCE_CM` (`400.0f`): Default clear-space distance returned when an ultrasonic echo times out.
- `ULTRASONIC_INTER_PING_MS` (`5`): $5\text{ ms}$ pause between firing adjacent sensors to eliminate acoustic cross-talk.
- `SAFE_FRONT_DISTANCE_CM` (`35.0f`): Triggers avoidance maneuver when an obstacle enters $35\text{ cm}$.
- `EMERGENCY_STOP_DISTANCE_CM` (`18.0f`): Triggers immediate backup + rotation when an obstacle is $< 18\text{ cm}$.
- `MIN_SIDE_CLEARANCE_CM` (`25.0f`): Minimum lateral space needed before committing to a left/right turn.
- `CRUISE_FORWARD_SPEED` (`110.0f`), `AVOIDANCE_TURN_SPEED` (`100.0f`), `AVOIDANCE_STRAFE_SPEED` (`100.0f`), `AVOIDANCE_BACKUP_SPEED` (`-80.0f`).
- `CONTROL_LOOP_INTERVAL_MS` (`50`): Runs the main loop at $20\text{ Hz}$.

---

## 2. [`RobotInterface.h`](RobotInterface.h) — Core Data Contracts & Layer API

Defines the shared structs connecting the Hardware Layer and the Algorithm Layer:

### `struct SensorData`
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
```

### `struct MotorCommand`
```cpp
struct MotorCommand {
    float vx;     // Forward (+) / Backward (-)
    float vy;     // Strafe Right (+) / Strafe Left (-)
    float omega;  // Rotate CW Right (+) / CCW Left (-)
};
```

### Layer Function Usage Example
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

## 3. [`UltrasonicSensor.h`](UltrasonicSensor.h) — 3-Sensor Trapezoid Array API

| Function / Struct | Description | Usage Example |
| :--- | :--- | :--- |
| `struct UltrasonicArrayData` | Holds `left_cm`, `center_cm`, `right_cm`. | `UltrasonicArrayData us = readAllUltrasonicSensors();` |
| `initUltrasonicSensors()` | Configures all `TRIG_PINS` (`OUTPUT`) and `ECHO_PINS` (`INPUT`). | `initUltrasonicSensors();` |
| `readSingleUltrasonicCm(idx)` | Reads one sensor (`US_LEFT_IDX`, `US_CENTER_IDX`, `US_RIGHT_IDX`). | `float front = readSingleUltrasonicCm(US_CENTER_IDX);` |
| `readAllUltrasonicSensors()` | Sequentially polls all 3 sensors with cross-talk delay. | `UltrasonicArrayData d = readAllUltrasonicSensors();` |
| `isObstacleDetected(dist, thresh)` | Returns `true` if `0 < dist < thresh`. | `if (isObstacleDetected(front, 30.0f)) { ... }` |

---

## 4. [`LidarSensor.h`](LidarSensor.h) — Slamtec RPLIDAR C1 API

| Function / Struct | Description | Usage Example |
| :--- | :--- | :--- |
| `struct LidarSectorSummary` | Holds `front_cm`, `left_cm`, `right_cm`, and `is_valid`. | `LidarSectorSummary lidar = readLidarSectors();` |
| `initLidarSensor()` | Initializes `Serial1` on `GPIO 17/16` at `460800` bps. | `initLidarSensor();` |
| `readLidarSectors()` | Parses scan frames and returns sector clearances. | `if (lidar.is_valid && lidar.left_cm > 50.0f) { ... }` |

---

## 5. [`ArduinoLink.h`](ArduinoLink.h) — UART0 Link to Arduino Uno

| Function | Description | Usage Example |
| :--- | :--- | :--- |
| `initArduinoLink()` | Initializes `Serial0` on `GPIO 4/5` at `115200` bps. | `initArduinoLink();` |
| `sendCommandToArduino(cmd)` | Sends `"CMD:<vx>,<vy>,<omega>\n"` over UART0. | `MotorCommand c = {110, 0, 0}; sendCommandToArduino(c);` |
| `sendStopToArduino()` | Sends `"STOP\n"` for immediate emergency halt. | `sendStopToArduino();` |

---

## 6. [`NavigationLogic.h`](NavigationLogic.h) — Decision Engine API

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
