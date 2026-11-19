# ESP32-C6 Source Files (`ESP32/src/`) — Code Walkthrough

This guide walks through every `.cpp` source file in `ESP32/src/`, explaining what each section of code does and why it is designed that way.

---

## 1. [`main.cpp`](main.cpp) — Main Autonomous Control Loop

`main.cpp` orchestrates the robot's Sense $\rightarrow$ Think $\rightarrow$ Act pipeline at **$20\text{ Hz}$**:

- **`setup()`**:
  1. Opens USB-C debugging serial (`Serial.begin(115200)`).
  2. Calls `initHardware()` (implemented in `ArduinoLink.cpp`) to configure the 3 ultrasonic sensor pins, start `Serial1` for the RPLIDAR C1, and start `Serial0` for the Arduino Uno.
  3. Waits `2000 ms` so the Arduino Uno finishes booting and the LiDAR motor spins up cleanly.
- **`loop()`**:
  1. **Sense:** `SensorData current_env = readAllSensors();` reads all 3 trapezoid ultrasonic sensors plus the 2D LiDAR sectors.
  2. **Think:** `MotorCommand action = computeAutonomousMove(current_env);` computes the safe `(vx, vy, omega)` velocity command.
  3. **Act:** `executeMotorCommand(action);` transmits `"CMD:<vx>,<vy>,<omega>\n"` over UART0 to the Arduino Uno.
  4. **Pace:** `delay(CONTROL_LOOP_INTERVAL_MS);` (`50 ms`) ensures steady command updates well under the Arduino's `500 ms` safety watchdog.

---

## 2. [`UltrasonicSensor.cpp`](UltrasonicSensor.cpp) — 3-Sensor Trapezoid Array Driver

Manages the 3 front HC-SR04 sensors wired to `TRIG_PINS[3] = {18, 14, 3}` and `ECHO_PINS[3] = {1, 2, 6}`:

1. **`initUltrasonicSensors()`**:
   - Loops `i = 0..2`, setting each `TRIG` pin to `OUTPUT` (`LOW`) and each `ECHO` pin to `INPUT`.
2. **`readSingleUltrasonicCm(uint8_t sensor_index)`**:
   - Guards against invalid array indices (`>= 3`).
   - Sends a standard $10\ \mu\text{s}$ `HIGH` pulse on `TRIG_PINS[sensor_index]`.
   - Calls `pulseIn(echo_pin, HIGH, ULTRASONIC_TIMEOUT_US)` (`25000 us` limit) so a missing echo never freezes the robot.
   - **Invalid/Timeout Handling (Milestone 5):** If `duration_us == 0` (no echo returned within $25\text{ ms}$) or the computed value exceeds `400.0 cm`, it returns `MAX_VALID_DISTANCE_CM` (`400.0f`) instead of `0.0f`, preventing false stops in wide rooms.
   - Converts round-trip microseconds to centimeters:
     $$\text{distance (cm)} = \frac{\text{duration\_us} \times 0.0343}{2}$$
3. **`readAllUltrasonicSensors()`**:
   - Polls **Left (`0`) $\rightarrow$ Center (`1`) $\rightarrow$ Right (`2`)** sequentially with a `5 ms` pause (`ULTRASONIC_INTER_PING_MS`) between pings. Firing them one at a time prevents ultrasonic waves from the Left sensor from bouncing back into the Center or Right receiver (acoustic cross-talk).

---

## 3. [`LidarSensor.cpp`](LidarSensor.cpp) — Slamtec RPLIDAR C1 Driver

1. **`initLidarSensor()`**:
   - Initializes **Hardware `Serial1`** at `460800` bps (`LIDAR_BAUD`) on `LIDAR_RX_PIN = 17` and `LIDAR_TX_PIN = 16`.
2. **`readLidarSectors()`**:
   - Holds a cached `LidarSectorSummary` (`front_cm`, `left_cm`, `right_cm`, `is_valid`).
   - Currently structured as a clean Milestone 6 extension hook: once the RPLIDAR C1 scan packet parser (or Slamtec SDK) decodes incoming `(angle, distance_mm)` points from `Serial1`, it updates the minimum obstacle distance for:
     - **Front cone:** $330^\circ \dots 360^\circ$ and $0^\circ \dots 30^\circ$
     - **Right flank:** $30^\circ \dots 120^\circ$
     - **Left flank:** $240^\circ \dots 330^\circ$
     - Sets `is_valid = true` so `NavigationLogic.cpp` activates full Ultrasonic + LiDAR fusion. While `is_valid == false`, the robot automatically falls back to the 3 trapezoid ultrasonic sensors without crashing!

---

## 4. [`ArduinoLink.cpp`](ArduinoLink.cpp) — Hardware Aggregator & UART0 Transmitter

This file bridges the Hardware Layer functions declared in `RobotInterface.h`:

1. **`initArduinoLink()` / `initHardware()`**:
   - Configures **Hardware `Serial0`** on `UNO_RX_PIN = 4` and `UNO_TX_PIN = 5` at `115200` bps (`UNO_BAUD`), and initializes the ultrasonic array and LiDAR UART1.
2. **`readAllSensors()`**:
   - Calls `readAllUltrasonicSensors()` and `readLidarSectors()`, merges all 6 distance fields plus `is_lidar_valid` and `millis()` into a single `SensorData` packet, and returns it to `main.cpp`.
3. **`sendCommandToArduino(const MotorCommand& cmd)` / `executeMotorCommand(cmd)`**:
   - Formats `vx`, `vy`, and `omega` to 1 decimal place and prints `"CMD:<vx>,<vy>,<omega>\n"` on `Serial0`.
4. **`sendStopToArduino()`**:
   - Sends `"STOP\n"` over `Serial0` when an immediate stop is required.

---

## 5. [`NavigationLogic.cpp`](NavigationLogic.cpp) — Multisensor Collision-Avoidance Engine

`computeAutonomousMove(const SensorData& sensor_in)` implements a deterministic **5-stage decision cascade**:

1. **Stage 1 — Conservative Multisensor Fusion:**
   - Takes `min(ultrasonic, lidar)` for the Front, Left, and Right sectors whenever `is_lidar_valid` is `true`.
   - Why `min()`? Ultrasonic detects **glass and black surfaces** that LiDAR lasers pass through or absorb, while 2D LiDAR detects **thin chair legs and wide-angle obstacles** that sit outside the ultrasonic cone. Taking the smaller (closer) reading guarantees maximum safety.
2. **Stage 2 — Emergency Escape (`min_front_zone_cm < 18 cm`):**
   - If any part of the front trapezoid detects an object closer than `18 cm`, the robot immediately backs up (`vx = -80`) while rotating toward whichever side (`effective_left_cm` vs. `effective_right_cm`) has more open space.
3. **Stage 3 — Front-Center Blocked (`effective_front_cm < 35 cm`):**
   - Stops forward travel (`vx = 0`) and compares `effective_left_cm` vs. `effective_right_cm`:
     - Turns Left (`omega = -100`) if the left side is wider and exceeds `25 cm`.
     - Turns Right (`omega = +100`) if the right side exceeds `25 cm`.
     - Rotates in place clockwise if both sides are narrow (escaping a dead-end).
4. **Stage 4 — Angled Side Avoidance (Left or Right Trapezoid Ultrasonic `< 35 cm`):**
   - When the center path is open (`>= 35 cm`) but either the Left or Right angled ultrasonic sensor detects an approaching wall or corner, the robot leverages its **Mecanum wheels** to **slow down (`vx = 55`), strafe sideways away from the obstacle (`vy = ±60`), and gently steer (`omega = ±50`)** smoothly without coming to a complete halt.
5. **Stage 5 — Cruise Forward:**
   - When all front zones are clear (`>= 35 cm`), cruises straight ahead (`vx = 110, vy = 0, omega = 0`).
