# System Architecture & Decision Flowcharts

This document provides the **System Block Diagrams** (required for **Milestone 1** and **Milestone 6**) and the **Autonomous Collision-Avoidance Flowchart** (required for **Milestone 4** and **Milestone 6**).

---

## 1. Hardware System Block Diagram (ESP32-C6 Main Brain + Arduino Motor Slave)

Our robot uses a **Master–Slave Dual-MCU architecture** over UART:
- The **DFRobot FireBeetle 2 ESP32-C6** reads all environmental sensors (**3 front trapezoid HC-SR04 Ultrasonic sensors** + top-mounted **Slamtec RPLIDAR C1**), executes multisensor collision avoidance, and sends motion commands over UART.
- The **Arduino Uno** receives UART motion commands (`115200` bps) on Hardware `Serial` (`D0`/`D1`), computes 4WD Mecanum wheel speeds, and drives the onboard **STC8 I2C motor chip** (`0x30`).

```mermaid
flowchart LR
    subgraph Sensors["Environmental Sensors (Connected to ESP32-C6)"]
        US_L["Ultrasonic Left (TRIG 18 / ECHO 1)"]
        US_C["Ultrasonic Center (TRIG 14 / ECHO 2)"]
        US_R["Ultrasonic Right (TRIG 3 / ECHO 6)"]
        LIDAR["Slamtec RPLIDAR C1 (Top 360° 2D LiDAR)"]
    end

    subgraph Brain["Main Brain / Master"]
        ESP32["DFRobot FireBeetle 2 ESP32-C6 (Sensor Fusion & Navigation)"]
    end

    subgraph Controller["Low-Level Motor Slave"]
        UNO["Arduino Uno (KS0560 Base Controller)"]
        STC8["STC8 I2C Motor Chip (Address 0x30)"]
    end

    subgraph Actuators["4WD Mecanum Drivetrain"]
        M1["Motor 1: Front Right"]
        M2["Motor 2: Front Left"]
        M3["Motor 3: Rear Right"]
        M4["Motor 4: Rear Left"]
    end

    US_L -->|"GPIO Echo Pulse"| ESP32
    US_C -->|"GPIO Echo Pulse"| ESP32
    US_R -->|"GPIO Echo Pulse"| ESP32
    LIDAR -->|"UART1 (460800 bps) RX 17 / TX 16"| ESP32

    ESP32 -->|"UART0 TX 5 -> Uno RX D0 (115200 bps) CMD:<vx>,<vy>,<omega>"| UNO
    UNO -->|"Software I2C (SDA D3 / SCL D2)"| STC8
    STC8 -->|"PWM"| M1
    STC8 -->|"PWM"| M2
    STC8 -->|"PWM"| M3
    STC8 -->|"PWM"| M4
```

### Purpose of Each Main Component (Milestone 1 Deliverable)

| Component | Role & Purpose |
| :--- | :--- |
| **DFRobot FireBeetle 2 ESP32-C6 (Main Brain)** | High-speed RISC-V microcontroller that polls all 3 ultrasonic sensors and the RPLIDAR C1, fuses their measurements, runs the autonomous navigation algorithm, and sends velocity commands to the Arduino Uno over UART0. |
| **3x Front HC-SR04 Ultrasonic Sensors (Trapezoid Layout)** | Mounted across the front bumper in a trapezoid (Left-angled, Center-forward, Right-angled) to cover forward and diagonal blind zones and reliably detect close-range or optically transparent obstacles (e.g., glass). |
| **Slamtec RPLIDAR C1 (2D LiDAR)** | Top-mounted $360^\circ$ laser scanner (`460800` bps UART1) providing wide-angle directional clearances (Front, Left, Right) and detecting thin obstacles outside ultrasonic beam cones. |
| **Arduino Uno (Motor Controller Slave)** | Receives ASCII velocity commands (`CMD:<vx>,<vy>,<omega>\n` or `STOP\n`) over Hardware UART (`D0`/`D1`), computes 4WD Mecanum inverse kinematics, enforces a `500 ms` safety watchdog, and drives the STC8 chip. |
| **STC8 Motor Driver Electronics (`0x30`)** | Onboard motor co-processor on the Keyestudio KS0560 expansion board generating PWM and H-bridge direction signals for all 4 DC motors via software I2C (`D3 SDA` / `D2 SCL`). |
| **Keyestudio KS0560 Chassis & 4WD Mecanum Wheels** | Omnidirectional drivetrain supporting forward, backward, turning, lateral strafing, and in-place rotation. |

---

## 2. Software Architecture & Layer Separation (ESP32-C6)

Inside [`lib/`](../lib/) and [`src/ESP32_Brain/`](../src/ESP32_Brain/), the ESP32-C6 firmware is decoupled via [`lib/RobotInterface/RobotInterface.h`](../lib/RobotInterface/RobotInterface.h):

```mermaid
flowchart TD
    subgraph HardwareLayer["Hardware Layer (UltrasonicSensor.cpp, LidarSensor.cpp, ArduinoLink.cpp)"]
        READ["readAllSensors()"]
        EXEC["executeMotorCommand(cmd)"]
    end

    subgraph Interface["Interface Contract (RobotInterface.h)"]
        SD["struct SensorData { ultrasonic_left_cm, ultrasonic_front_cm, ultrasonic_right_cm, lidar_front_cm, lidar_left_cm, lidar_right_cm, is_lidar_valid, timestamp_ms }"]
        MC["struct MotorCommand { vx, vy, omega }"]
    end

    subgraph AlgorithmLayer["Algorithm Layer (NavigationLogic.cpp)"]
        NAV["computeAutonomousMove(sensor_in)"]
    end

    READ --> SD
    SD --> NAV
    NAV --> MC
    MC --> EXEC
```

---

## 3. Inter-MCU UART Communication Protocol (`ESP32-C6 -> Arduino Uno`)

The **FireBeetle 2 ESP32-C6** (`UART0`, `TX = GPIO 5`, `RX = GPIO 4`) transmits newline-terminated ASCII packets at `115200` bps to the **Arduino Uno** (`Hardware Serial`, `D0 RX / D1 TX`):

| Packet Format | Example | Frequency | Description |
| :--- | :--- | :---: | :--- |
| `CMD:<vx>,<vy>,<omega>\n` | `CMD:110.0,0.0,0.0\n` | $20\text{ Hz}$ | Sets target Mecanum body velocity (`-255.0` to `+255.0`). |
| `STOP\n` | `STOP\n` | On demand | Immediate stop of all four DC motors. |

> **Safety Watchdog:** If the Arduino Uno receives no valid `CMD:` packet for more than `500 ms` (`COMMAND_TIMEOUT_MS`), it automatically stops all motors.

---

## 4. Autonomous Collision-Avoidance Flowchart (Milestones 4 & 6)

Below is the 5-stage decision cascade implemented in [`lib/NavigationLogic/NavigationLogic.cpp`](../lib/NavigationLogic/NavigationLogic.cpp):

```mermaid
flowchart TD
    START(["20 Hz Loop Start"]) --> READ["Read 3 Trapezoid Ultrasonics (L, C, R) & 2D LiDAR Sectors"]
    READ --> FUSE["Multisensor Fusion: effective = min(ultrasonic, lidar) per sector"]
    FUSE --> EMERG{"min_front_zone_cm < 18 cm?"}

    EMERG -- "Yes (Emergency)" --> ESCAPE["Reverse (vx = -80) & Rotate toward wider side (L vs R)"]
    EMERG -- "No" --> FRONT{"effective_front_cm < 35 cm?"}

    FRONT -- "Yes (Front Blocked)" --> COMPARE{"effective_left > effective_right AND > 25 cm?"}
    COMPARE -- "Yes" --> TURN_L["Stop vx, Rotate Left (omega = -100)"]
    COMPARE -- "No" --> RIGHT_OK{"effective_right > 25 cm?"}
    RIGHT_OK -- "Yes" --> TURN_R["Stop vx, Rotate Right (omega = +100)"]
    RIGHT_OK -- "No (Dead-End)" --> SPIN["Rotate Clockwise in Place to Escape"]

    FRONT -- "No (Center Clear)" --> SIDE_L{"ultrasonic_left < 35 cm AND <= ultrasonic_right?"}
    SIDE_L -- "Yes" --> STRAFE_R["Slow Forward (vx = 55), Strafe & Steer Right"]
    SIDE_L -- "No" --> SIDE_R{"ultrasonic_right < 35 cm?"}
    SIDE_R -- "Yes" --> STRAFE_L["Slow Forward (vx = 55), Strafe & Steer Left"]
    SIDE_R -- "No (All Clear)" --> CRUISE["Cruise Forward (vx = +110, vy = 0, omega = 0)"]

    ESCAPE --> SEND["Send CMD:<vx>,<vy>,<omega> via UART0 to Arduino Uno"]
    TURN_L --> SEND
    TURN_R --> SEND
    SPIN --> SEND
    STRAFE_R --> SEND
    STRAFE_L --> SEND
    CRUISE --> SEND
    SEND --> WAIT(["Wait 50 ms (Next Cycle)"])
```
