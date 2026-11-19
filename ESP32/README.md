# DFRobot FireBeetle 2 ESP32-C6 Firmware (Main Brain)

This PlatformIO project runs on the **DFRobot FireBeetle 2 ESP32-C6**, which serves as the **Main Brain (Master Controller)** of the Autonomous Robot Car.

---

## 1. Role in the System

The FireBeetle 2 ESP32-C6 performs all sensor acquisition, data fusion, and autonomous path planning:

1. **Front Trapezoid Ultrasonic Array (`UltrasonicSensor`)**: Triggers and reads **3 front HC-SR04 ultrasonic sensors** (Left, Center, Right) mounted in a trapezoid arrangement to cover the front and forward-diagonal blind spots.
2. **Top-Mounted 2D LiDAR (`LidarSensor`)**: Reads $360^\circ$ scan data from the **Slamtec RPLIDAR C1** over **Hardware UART1** (`460800` bps) and summarizes obstacle clearances in the Front, Left, and Right sectors.
3. **Multisensor Fusion & Decision Engine (`NavigationLogic`)**: Fuses the 3 trapezoid ultrasonic measurements with the 2D LiDAR sector data to compute the optimal Mecanum velocity vector `(vx, vy, omega)`.
4. **UART Motor Link (`ArduinoLink`)**: Transmits real-time velocity commands (`CMD:<vx>,<vy>,<omega>\n`) over **Hardware UART0** (`115200` bps) to the **Arduino Uno Motor Controller Slave**.

---

## 2. Pinout Summary (`include/PinConfig.h`)

| Peripheral | Interface | ESP32-C6 GPIO Pins | Settings / Notes |
| :--- | :--- | :--- | :--- |
| **Slamtec RPLIDAR C1** | Hardware `UART1` (`Serial1`) | `RX = GPIO 17`, `TX = GPIO 16` | `460800` bps (`LIDAR_BAUD`) |
| **Arduino Uno (Slave)** | Hardware `UART0` (`Serial0`) | `RX = GPIO 4`, `TX = GPIO 5` | `115200` bps (`UNO_BAUD`), cross-wired to Arduino `D1 TX` / `D0 RX` |
| **Ultrasonic 0 (Front-Left)** | Digital GPIO | `TRIG = GPIO 18`, `ECHO = GPIO 1` | Left-angled trapezoid wing |
| **Ultrasonic 1 (Front-Center)** | Digital GPIO | `TRIG = GPIO 14`, `ECHO = GPIO 2` | Center forward-facing sensor |
| **Ultrasonic 2 (Front-Right)** | Digital GPIO | `TRIG = GPIO 3`, `ECHO = GPIO 6` | Right-angled trapezoid wing |
| **PC Debug Monitor** | Native USB-C CDC (`Serial`) | USB-C Port | `115200` bps (`DEBUG_SERIAL_BAUD`) |

> [!WARNING]
> **Voltage Level Note (`5V` Echo/UART $\rightarrow$ `3.3V` ESP32-C6):**
> The FireBeetle 2 ESP32-C6 GPIOs are **$3.3\text{V}$ logic**. Ensure the `ECHO` pins from the HC-SR04 sensors (`GPIO 1, 2, 6`) and the `TX` line coming from Arduino Uno (`GPIO 4`) are properly level-shifted or voltage-divided ($5\text{V} \rightarrow 3.3\text{V}$) if powered at $5\text{V}$.

---

## 3. Decoupled Software Architecture (`RobotInterface.h`)

To let team members work on **navigation algorithms** and **hardware drivers** in parallel without merge conflicts, the codebase is split cleanly across [`include/RobotInterface.h`](include/RobotInterface.h):

| Layer | Files | Responsibility |
| :--- | :--- | :--- |
| **Interface Contract** | `include/RobotInterface.h` | Defines `SensorData` (inputs) and `MotorCommand` (outputs). |
| **Algorithm Layer** | `include/NavigationLogic.h`<br>`src/NavigationLogic.cpp` | Pure decision-making: `computeAutonomousMove(const SensorData&)`. Easy to tune or test with simulated `SensorData` values! |
| **Hardware Layer** | `include/UltrasonicSensor.h`, `src/UltrasonicSensor.cpp`<br>`include/LidarSensor.h`, `src/LidarSensor.cpp`<br>`include/ArduinoLink.h`, `src/ArduinoLink.cpp` | Implements `initHardware()`, `readAllSensors()`, and `executeMotorCommand()`. |

---

## 4. Folder Structure

```text
ESP32/
├── platformio.ini              # PlatformIO config for FireBeetle 2 ESP32-C6 (USB CDC enabled)
├── README.md                   # High-level overview, pinout summary, & build guide (this file)
├── include/                    # Header files (.h): pins, data contracts, & API declarations
│   ├── README.md               # Complete API reference & C++ usage examples for every module
│   ├── PinConfig.h             # Exact GPIO pins, UART baud rates, & safety distance thresholds
│   ├── RobotInterface.h        # Shared SensorData & MotorCommand structs + layer contracts
│   ├── UltrasonicSensor.h      # 3-sensor trapezoid HC-SR04 array declarations
│   ├── LidarSensor.h           # RPLIDAR C1 UART1 acquisition declarations
│   ├── ArduinoLink.h           # UART0 transmitter to Arduino Uno declarations
│   └── NavigationLogic.h       # Multisensor collision-avoidance algorithm declarations
└── src/                        # Implementation files (.cpp): sensor drivers & decision logic
    ├── README.md               # Detailed block-by-block code walkthrough for all .cpp files
    ├── main.cpp                # Main 20 Hz autonomous loop (Read -> Decide -> Transmit)
    ├── UltrasonicSensor.cpp    # Sequential 3-sensor trigger/echo timing & timeout handling
    ├── LidarSensor.cpp         # RPLIDAR C1 UART1 driver & sector clearance aggregation
    ├── ArduinoLink.cpp         # Hardware layer aggregator & UART0 packet transmitter
    └── NavigationLogic.cpp     # 5-stage multisensor fusion & collision-avoidance engine
```

> **Want to understand every file and function in detail?**
> - Read [`include/README.md`](include/README.md) for **structs, constants, and C++ usage examples**.
> - Read [`src/README.md`](src/README.md) for **step-by-step explanations of how every code block works**.

---

## 5. How to Build & Upload

1. Open `Autonomous-Robot-Car.code-workspace` in **VS Code** with **PlatformIO IDE** installed.
2. Select **`env:firebeetle2_esp32c6`** as the active PlatformIO environment in the bottom status bar.
3. Connect the **FireBeetle 2 ESP32-C6** via USB-C and click **Build ($\checkmark$)** or **Upload ($\rightarrow$)**.
4. Open the **Serial Monitor** at `115200` baud to view USB debug output.
