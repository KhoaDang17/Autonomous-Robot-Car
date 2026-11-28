# Testing, Calibration, and Evaluation Log

Use this document to record your experimental measurements and testing evidence for **Milestone 3**, **Milestone 5**, and **Milestone 6**, as well as the final technical report.

---

## 1. Milestone 3: Ultrasonic Sensor Distance Calibration (3-Sensor Trapezoid Array)

Place a flat obstacle at known distances in front of each HC-SR04 sensor (`Left = GPIO 18/1`, `Center = GPIO 14/2`, `Right = GPIO 3/6`) and record the measured distance printed by the ESP32-C6 (`115200` baud).

### 1.1 Actual vs. Measured Distance Table

| Trial | Actual Distance ($\text{cm}$) | Left Sensor ($\text{cm}$) | Center Sensor ($\text{cm}$) | Right Sensor ($\text{cm}$) | Absolute Error ($\text{cm}$) | Obstacle Detected (`< 35 cm`)? |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 1 | `10.0` | _TBD_ | _TBD_ | _TBD_ | _TBD_ | `Yes (Emergency < 18)` |
| 2 | `20.0` | _TBD_ | _TBD_ | _TBD_ | _TBD_ | `Yes` |
| 3 | `35.0` | _TBD_ | _TBD_ | _TBD_ | _TBD_ | `Threshold` |
| 4 | `50.0` | _TBD_ | _TBD_ | _TBD_ | _TBD_ | `No` |
| 5 | `100.0` | _TBD_ | _TBD_ | _TBD_ | _TBD_ | `No` |

### 1.2 Justification of Selected Safety Distance Thresholds
- **Emergency Escape Threshold (`18.0 cm`):** Accounts for braking inertia at cruise speed (`vx = 110`) and physical clearance of the trapezoid sensor mount.
- **Avoidance Trigger Threshold (`35.0 cm`):** Gives the 4WD Mecanum chassis sufficient room to strafe or rotate cleanly without clipping angled corners.

---

## 2. Milestone 5: Obstacle Avoidance Test Plan & Evaluation

### 2.1 Indoor Test Course Scenarios

| Scenario ID | Obstacle Arrangement | Expected Robot Behaviour | Observed Behaviour / Success Rate (e.g. `5/5`) | Notes / Identified Issues |
| :---: | :--- | :--- | :--- | :--- |
| **T1** | Flat wall straight ahead | Detect at $< 35\text{ cm}$, stop and rotate toward open space | _TBD_ | _TBD_ |
| **T2** | Angled obstacle entering Left or Right trapezoid zone | Center clear; strafe and steer smoothly away from side obstacle | _TBD_ | Uses Mecanum `vy` + `omega` in `lib/NavigationLogic/NavigationLogic.cpp` |
| **T3** | $90^\circ$ L-shaped corner | Detect blocked front and wall side; rotate toward open direction | _TBD_ | _TBD_ |
| **T4** | U-shaped dead-end trap | Back up if $< 18\text{ cm}$ and rotate in place until clear exit found | _TBD_ | _TBD_ |
| **T5** | Open room ($> 400\text{ cm}$, no echo) | Timeout returns `400.0 cm`; robot continues cruising smoothly | _TBD_ | Handled in `lib/UltrasonicSensor/UltrasonicSensor.cpp` |

### 2.2 Problems Encountered & Improvements Implemented

| Problem Identified | Root Cause | Solution Implemented in Code | Effect on Performance |
| :--- | :--- | :--- | :--- |
| Acoustic cross-talk between 3 front ultrasonic sensors | Adjacent HC-SR04 receivers picked up echoes from neighboring transmitters | Sequential polling with `5 ms` pause (`ULTRASONIC_INTER_PING_MS`) in `lib/UltrasonicSensor/UltrasonicSensor.cpp` | Eliminated false phantom readings between Left, Center, and Right channels |
| False `0 cm` readings in wide open areas | `pulseIn()` returns `0` when no echo returns before timeout | Mapped `duration_us == 0` to `MAX_VALID_DISTANCE_CM` (`400.0 cm`) | Eliminated sudden false emergency stops |
| Potential runaway motion if ESP32-C6 disconnects | Arduino Uno kept executing the last received PWM command | Added `500 ms` watchdog (`COMMAND_TIMEOUT_MS`) in `lib/SerialComm/SerialComm.cpp` | Robot automatically brakes within $0.5\text{ s}$ if UART link drops |

---

## 3. Milestone 6: Ultrasonic-Only vs. Multisensor (3x Ultrasonic + RPLIDAR C1) Comparison

Benchmark the **3-Ultrasonic-Only** mode (`is_lidar_valid = false`) against the **Combined 3x Ultrasonic + RPLIDAR C1** system (`is_lidar_valid = true`).

| Evaluation Metric | 3x Trapezoid Ultrasonic-Only | Multisensor (3x Ultrasonic + RPLIDAR C1) | Improvement Summary |
| :--- | :--- | :--- | :--- |
| **Field of View (FOV)** | Front trapezoid arc (~$90^\circ$–$120^\circ$ total) | Full $360^\circ$ planar laser scan + 3 front acoustic cones | Gains complete side and rear awareness when planning turn direction |
| **Thin / Low-Profile Obstacles** | Narrow chair/table legs may fall between sensor cones | High angular resolution from RPLIDAR C1 detects thin obstacles reliably | Fewer grazing collisions on chair legs |
| **Optically Challenging Surfaces** | Detects glass doors and matte black walls accurately | Laser beam can pass through glass or scatter on dark surfaces | `min(ultrasonic, lidar)` keeps full protection on glass/dark surfaces |
| **Collision Rate (across 10 runs)** | _TBD_ | _TBD_ | _TBD_ |
| **Average Course Completion Time** | _TBD_ | _TBD_ | _TBD_ |
