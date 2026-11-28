# MCU Firmware Entry Points (`src/`)

This folder contains the `setup()` and `loop()` entry points for both microcontrollers in our Dual-MCU architecture. Each subdirectory corresponds to one environment in [`platformio.ini`](../platformio.ini), selected using `build_src_filter`:

| Folder | PlatformIO Environment | `build_src_filter` | Role |
| :--- | :--- | :--- | :--- |
| [`ESP32_Brain/`](ESP32_Brain/main.cpp) | `env:firebeetle2_esp32c6` | `-<*> +<ESP32_Brain/>` | **Main Brain (Master):** Reads sensors, runs collision-avoidance fusion, and sends UART commands. |
| [`Arduino_Driver/`](Arduino_Driver/main.cpp) | `env:uno` | `-<*> +<Arduino_Driver/>` | **Motor Slave:** Receives UART commands, computes Mecanum kinematics, and drives the STC8 chip. |

---

## 1. [`ESP32_Brain/main.cpp`](ESP32_Brain/main.cpp) — Main Autonomous Control Loop

Orchestrates the robot's **Sense $\rightarrow$ Think $\rightarrow$ Act** pipeline at **$20\text{ Hz}$**:

- **`setup()`**:
  1. Opens USB-C debugging serial (`Serial.begin(DEBUG_SERIAL_BAUD)` at `115200` bps).
  2. Calls `initHardware()` (implemented in [`lib/ArduinoLink/ArduinoLink.cpp`](../lib/ArduinoLink/ArduinoLink.cpp)) to configure the 3 ultrasonic sensor pins, start `Serial1` for the RPLIDAR C1, and start `Serial0` for the Arduino Uno.
  3. Waits `2000 ms` so the Arduino Uno finishes booting and the LiDAR motor spins up cleanly.
- **`loop()`**:
  1. **Sense:** `SensorData current_env = readAllSensors();` polls all 3 trapezoid ultrasonic sensors plus the 2D LiDAR sectors.
  2. **Think:** `MotorCommand action = computeAutonomousMove(current_env);` computes the safe `(vx, vy, omega)` velocity command.
  3. **Act:** `executeMotorCommand(action);` transmits `"CMD:<vx>,<vy>,<omega>\n"` over `UART0` to the Arduino Uno.
  4. **Pace:** `delay(CONTROL_LOOP_INTERVAL_MS);` (`50 ms`) maintains steady $20\text{ Hz}$ updates well under the Arduino's `500 ms` safety watchdog.

---

## 2. [`Arduino_Driver/main.cpp`](Arduino_Driver/main.cpp) — Low-Level Motor Slave Loop

Intentionally minimal because all I2C, kinematics, and UART logic is modularized in [`lib/MotorDriver/`](../lib/MotorDriver/) and [`lib/SerialComm/`](../lib/SerialComm/):

- **`setup()`**:
  1. Calls `initSerialComm()` to open the `115200` bps Hardware UART (`D0 RX` / `D1 TX`).
  2. Calls `initMotorDriver()` to configure `D2`/`D3` as outputs and immediately command `0` PWM to all 4 motors so the car never jerks on boot.
- **`loop()`**:
  - Repeatedly calls `processIncomingCommands()` with zero artificial `delay()` so incoming `CMD:...` packets from the ESP32-C6 are parsed immediately without UART buffer overflow and the `500 ms` safety watchdog triggers on time.
