# Hardware Pinout & Wiring Guide

This document details all pin connections, UART links, STC8 I2C motor registers, and Mecanum kinematics for the **DFRobot FireBeetle 2 ESP32-C6 (Main Brain)** and **Arduino Uno / Keyestudio KS0560 (Motor Slave)**.

---

## 1. DFRobot FireBeetle 2 ESP32-C6 Pinout (Main Brain)

All ESP32-C6 pin assignments are defined in [`include/ESP32PinConfig.h`](../include/ESP32PinConfig.h).

### 1.1 Slamtec RPLIDAR C1 Connection (`UART1` / `Serial1`, `460800` baud)

| RPLIDAR C1 Signal | FireBeetle 2 ESP32-C6 Pin | Macro in `ESP32PinConfig.h` | Description |
| :--- | :--- | :--- | :--- |
| **VCC (5V)** | `5V / VUSB` | — | Stable 5V power input for LiDAR motor & laser |
| **LiDAR TX $\rightarrow$ ESP32 RX** | **GPIO 17** | `LIDAR_RX_PIN` | Receives $360^\circ$ scan stream (`460800` bps) |
| **LiDAR RX $\leftarrow$ ESP32 TX** | **GPIO 16** | `LIDAR_TX_PIN` | Sends scan control commands to RPLIDAR C1 |
| **GND** | `GND` | — | Common ground |

### 1.2 Arduino Uno UART Link (`UART0` / `Serial0`, `115200` baud)

| Signal | FireBeetle 2 ESP32-C6 Pin | Macro in `ESP32PinConfig.h` | Arduino Uno Pin | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **ESP32 RX $\leftarrow$ Arduino TX** | **GPIO 4** | `UNO_RX_PIN` | **D1 (`TX`)** | Use a $5\text{V} \rightarrow 3.3\text{V}$ voltage divider / level shifter! |
| **ESP32 TX $\rightarrow$ Arduino RX** | **GPIO 5** | `UNO_TX_PIN` | **D0 (`RX`)** | Sends `CMD:<vx>,<vy>,<omega>\n` and `STOP\n` |
| **Common Ground** | `GND` | — | `GND` | **Required:** Shared ground reference between both MCUs |

### 1.3 Three Front Ultrasonic Sensors (Trapezoid Array)

The 3 HC-SR04 sensors are mounted along the front trapezoid bumper (`TRIG_PINS[3] = {18, 14, 3}`, `ECHO_PINS[3] = {1, 2, 6}`):

| Index | Sensor Position | `TRIG` Pin (`TRIG_PINS`) | `ECHO` Pin (`ECHO_PINS`) | Mounting Orientation |
| :---: | :--- | :---: | :---: | :--- |
| `0` (`US_LEFT_IDX`) | **Front-Left** | **GPIO 18** | **GPIO 1** | Angled outward left on trapezoid wing |
| `1` (`US_CENTER_IDX`) | **Front-Center** | **GPIO 14** | **GPIO 2** | Straight ahead ($0^\circ$) |
| `2` (`US_RIGHT_IDX`) | **Front-Right** | **GPIO 3** | **GPIO 6** | Angled outward right on trapezoid wing |

> [!WARNING]
> **Voltage Level Safety ($5\text{V}$ vs. $3.3\text{V}$):**
> The FireBeetle 2 ESP32-C6 uses **$3.3\text{V}$ GPIO logic**, whereas standard HC-SR04 sensors and the Arduino Uno output **$5\text{V}$ signals**.
> - Place a resistor voltage divider (e.g., $1\text{ k}\Omega$ series + $2\text{ k}\Omega$ to `GND`) or bidirectional logic level shifter on every `ECHO` pin (`GPIO 1, 2, 6`) and on `UNO_RX_PIN` (`GPIO 4`) if powered from $5\text{V}$.
> - Disconnect Arduino `D0`/`D1` wires temporarily whenever flashing new firmware to the Arduino Uno over USB.

---

## 2. Arduino Uno (Keyestudio KS0560 Shield) Pinout (Motor Slave)

All Arduino Uno settings are defined in [`include/ArduinoPinConfig.h`](../include/ArduinoPinConfig.h).

### 2.1 UART & STC8 Motor Co-Processor Pins

| Function | Arduino Uno Pin | Macro in `ArduinoPinConfig.h` | Notes |
| :--- | :--- | :--- | :--- |
| **Hardware UART RX** | Digital Pin **D0 (`RX`)** | `UNO_BAUD` (`115200`) | Receives commands from ESP32-C6 `GPIO 5` (`UNO_TX_PIN`) |
| **Hardware UART TX** | Digital Pin **D1 (`TX`)** | `UNO_BAUD` (`115200`) | Connects through level shifter to ESP32-C6 `GPIO 4` (`UNO_RX_PIN`) |
| **Software I2C SDA** | Digital Pin **D3** | `STC8_SDA_PIN` | Data line to onboard STC8 motor chip (`0x30`) |
| **Software I2C SCL** | Digital Pin **D2** | `STC8_SCL_PIN` | Clock line to onboard STC8 motor chip (`0x30`) |

### 2.2 STC8 Motor PWM Registers (`Address 0x30`)

Each DC motor uses two registers (`reg_in1`, `reg_in2`) with PWM values `0`–`255`:
- **Forward:** `reg_in1 = 0`, `reg_in2 = pwm`
- **Backward:** `reg_in1 = pwm`, `reg_in2 = 0`
- **Stop:** `reg_in1 = 0`, `reg_in2 = 0`

| Motor ID | Wheel Position | `reg_in1` | `reg_in2` | Macro in `ArduinoPinConfig.h` |
| :---: | :--- | :---: | :---: | :--- |
| `1` | **Front Right (FR)** | `0x01` | `0x02` | `MOTOR_FRONT_RIGHT` |
| `2` | **Front Left (FL)** | `0x03` | `0x04` | `MOTOR_FRONT_LEFT` |
| `3` | **Rear Right (RR)** | `0x05` | `0x06` | `MOTOR_REAR_RIGHT` |
| `4` | **Rear Left (RL)** | `0x07` | `0x08` | `MOTOR_REAR_LEFT` |

---

## 3. Mecanum Wheel Kinematics Reference (Milestone 2)

In [`lib/MotorDriver/MotorDriver.cpp`](../lib/MotorDriver/MotorDriver.cpp), each wheel's signed speed is calculated from $(v_x, v_y, \omega)$:

$$\begin{aligned}
v_{\text{FL}} &= v_x + v_y + \omega \\
v_{\text{FR}} &= v_x - v_y - \omega \\
v_{\text{RL}} &= v_x - v_y + \omega \\
v_{\text{RR}} &= v_x + v_y - \omega
\end{aligned}$$

| Movement | $v_x$ | $v_y$ | $\omega$ | Front Left (M2) | Front Right (M1) | Rear Left (M4) | Rear Right (M3) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Move Forward** | $+S$ | $0$ | $0$ | Forward | Forward | Forward | Forward |
| **Move Backward** | $-S$ | $0$ | $0$ | Backward | Backward | Backward | Backward |
| **Strafe Right** | $0$ | $+S$ | $0$ | Forward | Backward | Backward | Forward |
| **Strafe Left** | $0$ | $-S$ | $0$ | Backward | Forward | Forward | Backward |
| **Rotate Right (CW)** | $0$ | $0$ | $+S$ | Forward | Backward | Forward | Backward |
| **Rotate Left (CCW)** | $0$ | $0$ | $-S$ | Backward | Forward | Backward | Forward |
| **Stop** | $0$ | $0$ | $0$ | Stop (`0`) | Stop (`0`) | Stop (`0`) | Stop (`0`) |
