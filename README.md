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

Both the **ESP32-C6 Main Brain** and the **Arduino Uno Motor Slave** live in a single unified PlatformIO project at the repository root, using `build_src_filter` and `lib_ignore` in [`platformio.ini`](platformio.ini) to target each MCU cleanly. Dedicated `README.md` guides in [`include/README.md`](include/README.md), [`lib/README.md`](lib/README.md), and [`src/README.md`](src/README.md) explain every configuration constant, API, and implementation block:

```text
Autonomous-Robot-Car/
├── platformio.ini                        # Unified PlatformIO config (env:firebeetle2_esp32c6 & env:uno)
├── README.md                             # Project overview, system architecture, & quick-start guide (this file)
├── .gitignore                            # Ignores .pio/, .cache/, .vscode/, compile_commands.json, & OS temp files
│
├── include/                              # Shared hardware pin mappings & parameter configs (-Iinclude)
│   ├── README.md                         # Pinout tables, UART settings, & navigation threshold reference
│   ├── ESP32PinConfig.h                  # FireBeetle 2 ESP32-C6 GPIOs (LiDAR, Uno UART0, 3x Ultrasonic) & thresholds
│   └── ArduinoPinConfig.h                # Arduino Uno STC8 I2C pins, motor IDs, UART baud rate, & 500 ms watchdog
│
├── lib/                                  # Modular firmware libraries shared across targets
│   ├── README.md                         # Detailed API reference, usage examples, UART protocol, & code walkthroughs
│   ├── RobotInterface/
│   │   └── RobotInterface.h              # Shared SensorData & MotorCommand structs + layer contracts (ESP32)
│   ├── UltrasonicSensor/
│   │   ├── UltrasonicSensor.h            # 3-sensor trapezoid HC-SR04 array declarations (ESP32)
│   │   └── UltrasonicSensor.cpp          # Sequential 3-sensor trigger/echo timing & timeout handling (ESP32)
│   ├── LidarSensor/
│   │   ├── LidarSensor.h                 # Slamtec RPLIDAR C1 UART1 acquisition declarations (ESP32)
│   │   └── LidarSensor.cpp               # RPLIDAR C1 UART1 driver & Front/Left/Right sector aggregation (ESP32)
│   ├── ArduinoLink/
│   │   ├── ArduinoLink.h                 # UART0 command transmitter to Arduino Uno declarations (ESP32)
│   │   └── ArduinoLink.cpp               # Hardware layer aggregator & UART0 "CMD:<vx>,<vy>,<omega>" sender (ESP32)
│   ├── NavigationLogic/
│   │   ├── NavigationLogic.h             # Multisensor collision-avoidance decision engine declarations (ESP32)
│   │   └── NavigationLogic.cpp           # 5-stage multisensor fusion & Mecanum avoidance algorithm (ESP32)
│   ├── MotorDriver/
│   │   ├── MotorDriver.h                 # STC8 I2C motor driver & 4WD Mecanum movement declarations (Arduino)
│   │   └── MotorDriver.cpp               # Software I2C protocol for STC8 (0x30) + Mecanum kinematics (Arduino)
│   └── SerialComm/
│       ├── SerialComm.h                  # UART command receiver & watchdog declarations (Arduino)
│       └── SerialComm.cpp                # Parses "CMD:<vx>,<vy>,<omega>" / "STOP" + 500 ms safety watchdog (Arduino)
│
├── src/                                  # MCU entry points (selected via build_src_filter in platformio.ini)
│   ├── README.md                         # Entry-point walkthrough & PlatformIO multi-MCU build explanation
│   ├── ESP32_Brain/
│   │   └── main.cpp                      # [Main Brain] 20 Hz autonomous loop (Read Sensors -> Decide -> Send UART)
│   └── Arduino_Driver/
│       └── main.cpp                      # [Motor Slave] Dedicated UART-to-I2C motor slave loop
│
├── test/                                 # Standalone hardware tests & PlatformIO unit tests
│   ├── README                            # PlatformIO Test Runner notes
│   └── ultrasonics_test.cpp              # Standalone 3-sensor trapezoid HC-SR04 bench test sketch
│
└── docs/                                 # Course Deliverables, Wiring Diagrams, & Evaluation Templates
    ├── system-architecture.md            # Hardware block diagrams, UART protocol, & avoidance flowchart
    ├── hardware-pinout.md                # Complete GPIO pinout tables, STC8 registers, & Mecanum kinematics
    ├── milestones-guide.md               # Checklist & code mapping for Course Milestones 1 through 6
    └── testing-and-evaluation.md         # Calibration tables & test logs for Milestones 3, 5, & 6
```

---

## 3. How the System Works

1. **DFRobot FireBeetle 2 ESP32-C6 (`src/ESP32_Brain/` + `lib/` ESP32 modules) — Main Brain:**
   - Polls the **3 front HC-SR04 ultrasonic sensors** (Left, Center, Right mounted in a trapezoid) sequentially via `UltrasonicSensor` to prevent acoustic cross-talk.
   - Reads $360^\circ$ laser scan data from the top-mounted **Slamtec RPLIDAR C1** (`UART1`, `460800` bps) via `LidarSensor` and summarizes obstacle clearances ahead, left, and right.
   - Fuses all sensor readings into `SensorData` (`RobotInterface`) and calls `computeAutonomousMove()` (`NavigationLogic`) to produce a `MotorCommand` (`vx`, `vy`, `omega`).
   - Sends `"CMD:<vx>,<vy>,<omega>\n"` over **Hardware UART0** (`GPIO 4 RX / GPIO 5 TX` at `115200` bps) via `ArduinoLink` to the Arduino Uno at $20\text{ Hz}$.
2. **Arduino Uno (`src/Arduino_Driver/` + `lib/` Arduino modules) — Dedicated Motor Controller Slave:**
   - Listens on **Hardware Serial** (`D0 RX / D1 TX` at `115200` bps) via `SerialComm` for `CMD:<vx>,<vy>,<omega>\n` or `STOP\n`.
   - Applies **4WD Mecanum inverse kinematics** (`driveMecanum()` in `MotorDriver`) and writes individual wheel PWM speeds over software I2C (`D3 SDA / D2 SCL`) to the **STC8 chip** (`0x30`).
   - Enforces a `500 ms` safety watchdog (`COMMAND_TIMEOUT_MS`) so all motors stop automatically if UART communication drops.

---

## 4. Quick Start Guide for Team Members

### Prerequisites
1. Install **[Visual Studio Code](https://code.visualstudio.com/)**.
2. Install the **[PlatformIO IDE](https://platformio.org/install/ide?install=vscode)** extension inside VS Code.

### Step 1: Clone and Open the Project
```bash
git clone <your-github-repo-url>
cd Autonomous-Robot-Car
```
Open the root **`Autonomous-Robot-Car`** folder directly in VS Code (**File $\rightarrow$ Open Folder...**). PlatformIO will automatically detect [`platformio.ini`](platformio.ini) and load both MCU environments (`env:firebeetle2_esp32c6` and `env:uno`).

### Step 2: Build & Upload the Arduino Uno Firmware (Motor Slave)
1. Switch the active PlatformIO environment to **`env:uno`** in the bottom status bar (or run `pio run -e uno` from the terminal).
2. **Important:** Temporarily unplug the jumper wires connected to `D0 (RX)` and `D1 (TX)` before flashing over USB.
3. Click **Build ($\checkmark$)** or **Upload ($\rightarrow$)**.
4. Read [`lib/README.md`](lib/README.md#5-uart-command-protocol--manual-motor-test-arduino-uno) to test motor commands directly via Serial Monitor.

### Step 3: Build & Upload the FireBeetle 2 ESP32-C6 Firmware (Main Brain)
1. Switch the active PlatformIO environment to **`env:firebeetle2_esp32c6`** in the bottom status bar (or run `pio run -e firebeetle2_esp32c6`).
2. Connect the **DFRobot FireBeetle 2 ESP32-C6** via USB-C and click **Build ($\checkmark$)** or **Upload ($\rightarrow$)**.
3. Read [`include/README.md`](include/README.md), [`lib/README.md`](lib/README.md), and [`src/README.md`](src/README.md) for pin definitions, API usage, and line-by-line explanations.

---

## 5. Documentation & Milestone Links

- **[System Architecture & Flowcharts](docs/system-architecture.md)** — Hardware block diagram, UART protocol, and multisensor avoidance flowchart.
- **[Hardware Pinout & Wiring Guide](docs/hardware-pinout.md)** — Exact GPIO pins for ESP32-C6 (`LIDAR`, `UNO`, 3x Trapezoid Ultrasonics), Arduino Uno, STC8 registers, and Mecanum kinematics.
- **[Milestones 1–6 Guide & Checklist](docs/milestones-guide.md)** — Step-by-step deliverables checklist mapped to the project grading rubric (Grades 1–5).
- **[Testing, Calibration & Evaluation Log](docs/testing-and-evaluation.md)** — Ready-to-use tables for ultrasonic distance calibration, obstacle test runs, and Ultrasonic vs. Multisensor comparison.
