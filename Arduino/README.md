# Arduino Uno Firmware (Dedicated UART-to-I2C Motor Controller Slave)

This PlatformIO project runs on the **Arduino Uno** mounted on the **Keyestudio KS0560 4WD Mecanum Robot Car**.

---

## 1. Role in the System

In our Dual-MCU architecture, the **FireBeetle 2 ESP32-C6** is the main brain (reading all 3 ultrasonic sensors and the 2D LiDAR), while the **Arduino Uno** acts strictly as a dedicated **Low-Level Motor Controller Slave**:

1. **Receive UART Commands (`SerialComm`)**: Listens on **Hardware Serial** (`D0 RX` / `D1 TX` at `115200` bps) for motion packets sent by the ESP32-C6.
2. **Compute Wheel Speeds (`MotorDriver`)**: Applies **4WD Mecanum inverse kinematics** to convert body velocity `(vx, vy, omega)` into individual speeds and directions for all 4 wheels.
3. **Drive STC8 Motor Chip (`MotorDriver`)**: Sends PWM and direction values to the onboard **STC8 motor coprocessor** (`0x30`) over software I2C (`SDA = D3`, `SCL = D2`).
4. **Enforce Safety Watchdog (`SerialComm`)**: Automatically brakes all 4 motors if no valid command arrives for more than `500 ms` (`COMMAND_TIMEOUT_MS`), preventing runaway collisions if the UART wire disconnects or the ESP32 resets.

---

## 2. Folder Structure

```text
Arduino/
├── platformio.ini              # PlatformIO build & serial monitor settings for Arduino Uno
├── README.md                   # Overview, UART command protocol, & upload/testing guide (this file)
├── include/                    # Header files (.h): configurations & public function declarations
│   ├── README.md               # Detailed API reference & C++ usage examples for every function
│   ├── PinConfig.h             # Hardware pin assignments, baud rate, I2C address, & timeout constants
│   ├── MotorDriver.h           # STC8 motor driver & Mecanum movement function declarations
│   └── SerialComm.h            # UART command receiver & watchdog declarations
└── src/                        # Implementation files (.cpp): actual logic & hardware drivers
    ├── README.md               # Line-by-line code walkthrough of every .cpp module
    ├── main.cpp                # Minimal setup() & loop() running the UART motor slave
    ├── MotorDriver.cpp         # Software I2C protocol for STC8 + Mecanum kinematics math
    └── SerialComm.cpp          # UART packet parser ("CMD:..." / "STOP") + 500 ms watchdog
```

> **Want to understand the code in detail?**
> - Read [`include/README.md`](include/README.md) for **function signatures, parameter meanings, and usage examples**.
> - Read [`src/README.md`](src/README.md) for **how every section of the implementation works internally**.

---

## 3. UART Command Protocol (`ESP32-C6 -> Arduino Uno`)

Communication runs over Hardware UART (`115200` bps, 8N1, newline-terminated `\n`):

| Command Packet | Parameters | Meaning | Example |
| :--- | :--- | :--- | :--- |
| `CMD:<vx>,<vy>,<omega>\n` | `vx`: Forward (+) / Backward (-)<br>`vy`: Strafe Right (+) / Left (-)<br>`omega`: Rotate CW (+) / CCW (-)<br>*(Range: `-255.0` to `+255.0`)* | Drive all 4 Mecanum wheels simultaneously | `CMD:110.0,0.0,0.0\n` (Move forward)<br>`CMD:0.0,-100.0,0.0\n` (Strafe left)<br>`CMD:0.0,0.0,90.0\n` (Rotate clockwise) |
| `STOP\n` | None | Immediately stop all 4 motors | `STOP\n` |

---

## 4. How to Build, Upload, and Test Manually

> [!IMPORTANT]
> **Disconnect `D0 (RX)` and `D1 (TX)` wires before flashing!**
> The Arduino Uno shares Hardware `Serial` (`D0`/`D1`) with its USB-to-Serial programmer chip. Always unplug the jumper wires going to the ESP32-C6 before clicking **Upload**, then reconnect them afterward.

1. Open `Autonomous-Robot-Car.code-workspace` in **VS Code** with the **PlatformIO IDE** extension installed.
2. Select **`env:uno`** as the active PlatformIO environment in the bottom toolbar.
3. Connect the Arduino Uno via USB (with `D0`/`D1` external wires unplugged) and click **Upload ($\rightarrow$)**.
4. **Optional Manual Motor Test via USB Serial Monitor**:
   - Open the **Serial Monitor** (`115200` baud, line ending set to `Newline (\n)`).
   - Type `CMD:100,0,0` and press Enter — all 4 wheels will spin forward for `500 ms` and then automatically stop (thanks to the safety watchdog!).
   - Type `CMD:0,100,0` to test strafing right, or `CMD:0,0,100` to test rotating clockwise.
   - Type `STOP` to halt immediately.
