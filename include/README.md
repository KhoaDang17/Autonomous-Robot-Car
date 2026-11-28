# Hardware Pinouts & Configuration Headers (`include/`)

This folder centralizes all hardware pin assignments, communication baud rates, and control/safety constants for both microcontrollers. Because `platformio.ini` sets `-Iinclude` in `[env]`, every library in `lib/` and entry point in `src/` can include these headers directly.

---

## 1. [`ESP32PinConfig.h`](ESP32PinConfig.h) — FireBeetle 2 ESP32-C6 Configuration

All hardware pins and tuning parameters for the **Main Brain (`env:firebeetle2_esp32c6`)** are defined here:

### UART Interfaces
| Macro | Value | Description |
| :--- | :---: | :--- |
| `DEBUG_SERIAL_BAUD` | `115200` | USB-C CDC Serial Monitor speed. |
| `LIDAR_RX_PIN` / `LIDAR_TX_PIN` | `17` / `16` | Hardware `Serial1` pins connected to **Slamtec RPLIDAR C1**. |
| `LIDAR_BAUD` | `460800` | Default UART baud rate for RPLIDAR C1. |
| `UNO_RX_PIN` / `UNO_TX_PIN` | `4` / `5` | Hardware `Serial0` pins connected to **Arduino Uno** (`D1 TX` / `D0 RX`). |
| `UNO_BAUD` | `115200` | Must match `UNO_BAUD` inside `ArduinoPinConfig.h`. |

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

> [!WARNING]
> **Voltage Level Note (`5V` Echo/UART $\rightarrow$ `3.3V` ESP32-C6):**
> The FireBeetle 2 ESP32-C6 GPIOs are **$3.3\text{V}$ logic**. Ensure the `ECHO` pins from the HC-SR04 sensors (`GPIO 1, 2, 6`) and the `TX` line coming from Arduino Uno (`GPIO 4`) are properly level-shifted or voltage-divided ($5\text{V} \rightarrow 3.3\text{V}$) if powered at $5\text{V}$.

---

## 2. [`ArduinoPinConfig.h`](ArduinoPinConfig.h) — Arduino Uno Configuration

All hardware pins, I2C addresses, and watchdog parameters for the **Motor Controller Slave (`env:uno`)** are defined here:

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
