# Project Milestones & Deliverables Guide

This guide maps the **6 Course Project Milestones** directly to the files in this repository so every team member knows where each feature lives and what deliverables are required for **Grade 5 – Excellent**.

---

## Milestone Overview & Code Mapping

| Milestone | Title | Relevant Code & Docs in Repo | Target Grade |
| :---: | :--- | :--- | :---: |
| **1** | Robot Assembly & System Familiarisation | [`docs/system-architecture.md`](system-architecture.md)<br>[`docs/hardware-pinout.md`](hardware-pinout.md) | Grade 1 |
| **2** | Motor Control & Robot Movement | [`lib/MotorDriver/MotorDriver.h`](../lib/MotorDriver/MotorDriver.h)<br>[`lib/MotorDriver/MotorDriver.cpp`](../lib/MotorDriver/MotorDriver.cpp)<br>[`lib/SerialComm/SerialComm.cpp`](../lib/SerialComm/SerialComm.cpp) | Grade 1 |
| **3** | Ultrasonic Sensor Integration & Object Detection | [`lib/UltrasonicSensor/UltrasonicSensor.h`](../lib/UltrasonicSensor/UltrasonicSensor.h)<br>[`lib/UltrasonicSensor/UltrasonicSensor.cpp`](../lib/UltrasonicSensor/UltrasonicSensor.cpp)<br>[`docs/testing-and-evaluation.md`](testing-and-evaluation.md) | Grade 1 |
| **4** | Autonomous Collision Avoidance | [`lib/NavigationLogic/NavigationLogic.cpp`](../lib/NavigationLogic/NavigationLogic.cpp)<br>[`lib/ArduinoLink/ArduinoLink.cpp`](../lib/ArduinoLink/ArduinoLink.cpp) | Grade 2 |
| **5** | Testing, Evaluation, & Optimisation | [`docs/testing-and-evaluation.md`](testing-and-evaluation.md)<br>[`lib/UltrasonicSensor/UltrasonicSensor.cpp`](../lib/UltrasonicSensor/UltrasonicSensor.cpp)<br>[`lib/SerialComm/SerialComm.cpp`](../lib/SerialComm/SerialComm.cpp) | Grade 3 – 4 |
| **6** | Advanced Multisensor Object Detection & Collision Avoidance (3x Ultrasonic + 2D LiDAR) | [`lib/RobotInterface/RobotInterface.h`](../lib/RobotInterface/RobotInterface.h)<br>[`lib/LidarSensor/LidarSensor.cpp`](../lib/LidarSensor/LidarSensor.cpp)<br>[`lib/NavigationLogic/NavigationLogic.cpp`](../lib/NavigationLogic/NavigationLogic.cpp) | **Grade 5** |

---

## Detailed Checklist per Milestone

### Milestone 1: Robot Assembly and System Familiarisation
- [ ] Assemble the **Keyestudio KS0560 4WD Mecanum Robot Car** chassis, motors, Arduino Uno, battery pack, and 3D-printed mounts for the **3 trapezoid HC-SR04 sensors**, **FireBeetle 2 ESP32-C6**, and **RPLIDAR C1**.
- [ ] Verify PlatformIO builds and uploads firmware to both Arduino Uno (`env:uno`) and ESP32-C6 (`env:firebeetle2_esp32c6`) from the unified root [`platformio.ini`](../platformio.ini).
- **Deliverables:** Photograph of assembled robot, system block diagram & component descriptions ([`docs/system-architecture.md`](system-architecture.md)), and upload demonstration.

### Milestone 2: Motor Control and Robot Movement
- [ ] Verify the STC8 I2C motor functions in [`lib/MotorDriver/MotorDriver.cpp`](../lib/MotorDriver/MotorDriver.cpp) (`moveForward`, `moveBackward`, `turnLeft`, `turnRight`, `strafeLeft`, `strafeRight`, `rotateLeft`, `rotateRight`, `stopMotors`, `driveMecanum`).
- [ ] Test sending `CMD:<vx>,<vy>,<omega>` packets over UART from the Serial Monitor or ESP32-C6.
- **Deliverables:** C/C++ motor control code, movement demonstration, and STC8 PWM/direction explanation ([`lib/README.md`](../lib/README.md)).

### Milestone 3: Ultrasonic Sensor Integration and Object Detection
- [ ] Test all 3 front trapezoid HC-SR04 sensors (`TRIG_PINS = {18, 14, 3}`, `ECHO_PINS = {1, 2, 6}`) using [`lib/UltrasonicSensor/UltrasonicSensor.cpp`](../lib/UltrasonicSensor/UltrasonicSensor.cpp) (or [`test/ultrasonics_test.cpp`](../test/ultrasonics_test.cpp)).
- [ ] Measure known target distances with a ruler and record Serial Monitor readings in [`docs/testing-and-evaluation.md`](testing-and-evaluation.md).
- **Deliverables:** C/C++ ultrasonic sensing code, distance measurements demo, calibration table, and threshold justification (`SAFE_FRONT_DISTANCE_CM = 35.0 cm`, `EMERGENCY_STOP_DISTANCE_CM = 18.0 cm`).

### Milestone 4: Autonomous Collision Avoidance
- [ ] Run the full ESP32-C6 $\rightarrow$ UART0 $\rightarrow$ Arduino Uno control loop (`src/ESP32_Brain/main.cpp` + `src/Arduino_Driver/main.cpp`) using the 3 trapezoid ultrasonic sensors.
- **Deliverables:** Complete C/C++ avoidance code, decision flowchart ([`docs/system-architecture.md`](system-architecture.md)), live demonstration, and explanation of decision-making.

### Milestone 5: Testing, Evaluation, and Optimisation
- [ ] Evaluate the robot on an indoor test course (flat wall, corridor, L-corner, U-shaped dead-end, angled obstacles).
- [ ] Verify timeout handling (`ULTRASONIC_TIMEOUT_US` in `lib/UltrasonicSensor/UltrasonicSensor.cpp`), cross-talk delay (`ULTRASONIC_INTER_PING_MS`), and the Arduino UART watchdog (`COMMAND_TIMEOUT_MS = 500 ms` in `lib/SerialComm/SerialComm.cpp`).
- **Deliverables:** Updated code, test plan, recorded results, identified problems, and improvement summary in [`docs/testing-and-evaluation.md`](testing-and-evaluation.md).

### Milestone 6: Advanced Multisensor Object Detection and Collision Avoidance (Grade 5)
- [ ] Complete the **RPLIDAR C1** UART1 scan parser in [`lib/LidarSensor/LidarSensor.cpp`](../lib/LidarSensor/LidarSensor.cpp) (`RX = 17`, `TX = 16`, `460800` bps) to update `front_cm`, `left_cm`, `right_cm`, and `is_valid`.
- [ ] Tune multisensor fusion in [`lib/NavigationLogic/NavigationLogic.cpp`](../lib/NavigationLogic/NavigationLogic.cpp) combining all **3 trapezoid ultrasonic sensors** with **RPLIDAR C1** sector clearances.
- [ ] Benchmark **Ultrasonic-Only** (`is_lidar_valid = false`) vs. **Multisensor (3x Ultrasonic + RPLIDAR C1)** in [`docs/testing-and-evaluation.md`](testing-and-evaluation.md).
- **Deliverables:** Working multisensor robotic system, integrated C/C++ code, block diagram, sensor-combination explanation, and quantitative performance comparison.
