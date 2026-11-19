# Autonomous Robot Car: Object Detection & Collision Avoidance

An autonomous **4WD Mecanum Robot Car** capable of navigating indoor environments, detecting obstacles using a **3-sensor front trapezoid HC-SR04 Ultrasonic array** fused with a **Slamtec RPLIDAR C1 (360° 2D LiDAR)**, and executing real-time collision avoidance using a **UART Dual-MCU architecture**.

---

## 1. Hardware & Team Overview

### Hardware Components
- **High-Level Main Brain:** DFRobot FireBeetle 2 ESP32-C6
  - **3x Front Ultrasonic Sensors (Trapezoid Layout):** HC-SR04 Left, Center, Right (`TRIG: {18, 14, 3}`, `ECHO: {1, 2, 6}`)
  - **360° 2D Laser Scanner:** Slamtec RPLIDAR C1 (`UART1`, `RX = 17`, `TX = 16`, `460800` bps)
- **Robot Base Kit:** [Keyestudio KS0560 4WD Mecanum Robot Car](https://docs.keyestudio.com/projects/KS0560/en/latest/)
  - **Low-Level Motor Controller (Slave):** Arduino Uno (`Hardware Serial D0/D1`, `115200` bps)
  - **Motor Driver Electronics:** Onboard STC8 I2C Motor Driver Chip (`Address 0x30`, `SDA = D3`, `SCL = D2`) + 4x DC Motors with Mecanum Wheels
- **Custom Mounts:** 3D-printed mounting parts for the trapezoid ultrasonic array, RPLIDAR C1, and ESP32-C6

### Project Team
- **Team Members:** Khoa Dang, Mitul Ahammed
- **Supervisor:** Jawad Yasin

---

## 2. Repository Structure

Every firmware folder contains a dedicated `README.md`, plus detailed `include/README.md` (API & usage examples) and `src/README.md` (code walkthroughs) so new team members can learn and contribute immediately:

```text
Autonomous-Robot-Car/
├── Autonomous-Robot-Car.code-workspace   # Open this in VS Code to load Root, Arduino, and ESP32 together
├── README.md                             # Project overview, system architecture, & quick-start guide (this file)
│
├── ESP32/                                # [Main Brain] DFRobot FireBeetle 2 ESP32-C6 Firmware
│   ├── platformio.ini                    # PlatformIO configuration for FireBeetle 2 ESP32-C6 (USB CDC enabled)
│   ├── README.md                         # High-level overview, pinout summary, & build guide
│   ├── include/
│   │   ├── README.md                     # Detailed API reference & C++ usage examples for all ESP32 headers
│   │   ├── PinConfig.h                   # GPIO pins (LiDAR, Arduino UART0, 3x Ultrasonic), baud rates, & thresholds
│   │   ├── RobotInterface.h              # Shared SensorData & MotorCommand structs + layer contracts
│   │   ├── UltrasonicSensor.h            # 3-sensor trapezoid HC-SR04 array declarations
│   │   ├── LidarSensor.h                 # Slamtec RPLIDAR C1 UART1 acquisition declarations
│   │   ├── ArduinoLink.h                 # UART0 command transmitter to Arduino Uno declarations
│   │   └── NavigationLogic.h             # Multisensor collision-avoidance decision engine declarations
│   └── src/
│       ├── README.md                     # Step-by-step code walkthrough of every ESP32 .cpp file
│       ├── main.cpp                      # Main 20 Hz autonomous loop (Read Sensors -> Decide -> Send UART)
│       ├── UltrasonicSensor.cpp          # Sequential 3-sensor trigger/echo timing & timeout safety handling
│       ├── LidarSensor.cpp               # RPLIDAR C1 UART1 driver & Front/Left/Right sector aggregation
│       ├── ArduinoLink.cpp               # Hardware layer aggregator & UART0 "CMD:<vx>,<vy>,<omega>" sender
│       └── NavigationLogic.cpp           # 5-stage multisensor fusion & Mecanum avoidance algorithm
│
├── Arduino/                              # [Motor Slave] Arduino Uno Firmware (Keyestudio KS0560)
│   ├── platformio.ini                    # PlatformIO configuration for Arduino Uno (115200 bps)
│   ├── README.md                         # Overview, UART protocol, & manual Serial Monitor test guide
│   ├── include/
│   │   ├── README.md                     # Detailed API reference & C++ usage examples for all Arduino headers
│   │   ├── PinConfig.h                   # STC8 I2C pins, motor IDs, UART baud rate, & 500 ms watchdog timeout
│   │   ├── MotorDriver.h                 # STC8 I2C motor driver & 4WD Mecanum movement declarations
│   │   └── SerialComm.h                  # UART command receiver & watchdog declarations
│   └── src/
│       ├── README.md                     # Step-by-step code walkthrough of every Arduino .cpp file
│       ├── main.cpp                      # Dedicated UART-to-I2C motor slave loop
│       ├── MotorDriver.cpp               # Software I2C protocol for STC8 (0x30) + Mecanum kinematics
│       └── SerialComm.cpp                # Parses "CMD:<vx>,<vy>,<omega>" / "STOP" + 500 ms safety watchdog
│
└── docs/                                 # Course Deliverables, Wiring Diagrams, & Evaluation Templates
    ├── system-architecture.md            # Hardware block diagrams, UART protocol, & avoidance flowchart
    ├── hardware-pinout.md                # Complete GPIO pinout tables, STC8 registers, & Mecanum kinematics
    ├── milestones-guide.md               # Checklist & code mapping for Course Milestones 1 through 6
    └── testing-and-evaluation.md         # Calibration tables & test logs for Milestones 3, 5, & 6
```

---

## 3. How the System Works

1. **DFRobot FireBeetle 2 ESP32-C6 (`ESP32/`) — Main Brain:**
   - Polls the **3 front HC-SR04 ultrasonic sensors** (Left, Center, Right mounted in a trapezoid) sequentially to prevent acoustic cross-talk.
   - Reads $360^\circ$ laser scan data from the top-mounted **Slamtec RPLIDAR C1** (`UART1`, `460800` bps) and summarizes obstacle clearances ahead, left, and right.
   - Fuses all sensor readings into `SensorData` and calls `computeAutonomousMove()` to produce a `MotorCommand` (`vx`, `vy`, `omega`).
   - Sends `"CMD:<vx>,<vy>,<omega>\n"` over **Hardware UART0** (`GPIO 4 RX / GPIO 5 TX` at `115200` bps) to the Arduino Uno at $20\text{ Hz}$.
2. **Arduino Uno (`Arduino/`) — Dedicated Motor Controller Slave:**
   - Listens on **Hardware Serial** (`D0 RX / D1 TX` at `115200` bps) for `CMD:<vx>,<vy>,<omega>\n` or `STOP\n`.
   - Applies **4WD Mecanum inverse kinematics** (`driveMecanum()`) and writes individual wheel PWM speeds over software I2C (`D3 SDA / D2 SCL`) to the **STC8 chip** (`0x30`).
   - Enforces a `500 ms` safety watchdog (`COMMAND_TIMEOUT_MS`) so all motors stop automatically if UART communication drops.

---

## 4. Quick Start Guide for Team Members

### Prerequisites
1. Install **[Visual Studio Code](https://code.visualstudio.com/)**.
2. Install the **[PlatformIO IDE](https://platformio.org/install/ide?install=vscode)** extension inside VS Code.

### Step 1: Clone and Open the Workspace
```bash
git clone <your-github-repo-url>
cd Autonomous-Robot-Car
```
Open **`Autonomous-Robot-Car.code-workspace`** in VS Code (**File $\rightarrow$ Open Workspace from File...**). This loads Root, `Arduino`, and `ESP32` cleanly in one window.

### Step 2: Build & Upload the Arduino Uno Firmware (Motor Slave)
1. Switch the active PlatformIO environment to **`env:uno (Arduino)`** in the bottom status bar.
2. **Important:** Temporarily unplug the jumper wires connected to `D0 (RX)` and `D1 (TX)` before flashing over USB.
3. Click **Build ($\checkmark$)** or **Upload ($\rightarrow$)**.
4. Read [`Arduino/README.md`](Arduino/README.md) to test motor commands directly via Serial Monitor.

### Step 3: Build & Upload the FireBeetle 2 ESP32-C6 Firmware (Main Brain)
1. Switch the active PlatformIO environment to **`env:firebeetle2_esp32c6 (ESP32)`**.
2. Connect the **DFRobot FireBeetle 2 ESP32-C6** via USB-C and click **Build ($\checkmark$)** or **Upload ($\rightarrow$)**.
3. Read [`ESP32/README.md`](ESP32/README.md), [`ESP32/include/README.md`](ESP32/include/README.md), and [`ESP32/src/README.md`](ESP32/src/README.md) for API usage and line-by-line explanations.

---

## 5. Documentation & Milestone Links

- **[System Architecture & Flowcharts](docs/system-architecture.md)** — Hardware block diagram, UART protocol, and multisensor avoidance flowchart.
- **[Hardware Pinout & Wiring Guide](docs/hardware-pinout.md)** — Exact GPIO pins for ESP32-C6 (`LIDAR`, `UNO`, 3x Trapezoid Ultrasonics), Arduino Uno, STC8 registers, and Mecanum kinematics.
- **[Milestones 1–6 Guide & Checklist](docs/milestones-guide.md)** — Step-by-step deliverables checklist mapped to the project grading rubric (Grades 1–5).
- **[Testing, Calibration & Evaluation Log](docs/testing-and-evaluation.md)** — Ready-to-use tables for ultrasonic distance calibration, obstacle test runs, and Ultrasonic vs. Multisensor comparison.
