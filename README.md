# 🏎️ ESP32 Advanced Line Follower (v5.2 "SLAM-Lite")

A high-performance, PID-controlled line-following robot with a real-time web dashboard for tuning, telemetry, and autonomous track mapping.

![Dashboard Preview](https://img.shields.io/badge/UI-Liquid_Glass-blue?style=for-the-badge)
![ESP32](https://img.shields.io/badge/Hardware-ESP32-green?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)

---

## 🌟 Key Features

### 🧠 Performance & Control
- **Dynamic PID Control:** Fine-tune Kp, Ki, and Kd on-the-fly via the web dashboard.
- **MPU6050 Assist:** Integrated Gyro/Accel for straight-line drift correction and 90° pivot turns.
- **Auto-PID Tuning:** Built-in Ziegler-Nichols tuning assistant.

### 🏎️ SLAM-Lite (Track Mapping)
- **Exploration Lap:** Robot records the track layout (straights vs. turns) and saves it to memory.
- **Speed Run Mode:** Using the recorded map, the robot performs **Predictive Braking** before corners and **Straight-Line Sprints** (up to 100% PWM) on recognized straights.
- **2D Visualizer:** Instantly render your recorded track path in the dashboard.

### 📊 Real-time Dashboard
- **Live Telemetry:** 50Hz sensor data, PID error graphs, and motor PWM gauges.
- **Battery Management:** Real-time voltage monitoring and low-battery alerts.
- **Settings Profiles:** Save up to 8 custom tuning presets to flash memory.
- **OTA Updates:** Flash firmware wirelessly via the browser — no USB required after first flash.

---

## 🔌 Hardware & Wiring

### Component List
- **Controller:** ESP32 (Dev Module)
- **Motor Driver:** DRV8833
- **Motors:** N20 1000 RPM (High Speed)
- **Sensors:** 8-Channel IR Array (Analog/Digital)
- **IMU:** MPU6050
- **Power:** 7.4V - 8.4V LiPo/Li-ion Battery

### Pin Configuration
| Component | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **IR2** | GPIO 36 | Leftmost |
| **IR3** | GPIO 39 | |
| **IR4** | GPIO 34 | Center-Left |
| **IR5** | GPIO 35 | Center-Right |
| **IR6** | GPIO 32 | |
| **IR7** | GPIO 33 | Rightmost |
| **Motor L1** | GPIO 23 | |
| **Motor L2** | GPIO 19 | |
| **Motor R1** | GPIO 13 | |
| **Motor R2** | GPIO 27 | |
| **MPU SDA** | GPIO 21 | I2C |
| **MPU SCL** | GPIO 22 | I2C |
| **Battery** | GPIO 15 | Via 1:1 Divider |

> [!IMPORTANT]
> GPIO 25 and 26 are reserved (used by WiFi). Avoid connecting IR sensors to these pins to prevent interference.

---

## 🚀 Getting Started

1. **Software:** Install [Arduino IDE](https://www.arduino.cc/en/software) and the [ESP32 Board Package](https://github.com/espressif/arduino-esp32).
2. **Libraries:** Install `ArduinoJson` (v7+) and `WebSockets` (by Markus Sattler) via Library Manager.
3. **Firmware:** Open `line_follower_esp32v5.1/line_follower_esp32v5.1.ino`.
4. **Flash:** Upload via USB. Open Serial Monitor (115200) to find the IP address.
5. **Dashboard:** Browse to `http://<IP_ADDRESS>/` to start tuning!

---

## 🛠️ How to use SLAM-Lite (Speed Run)

1. **Mapping:** Place the robot on the track and toggle **"Record Map"** on the dashboard. Complete one lap.
2. **Fetch:** Click **"Fetch Map"** to see the 2D path drawn on your dashboard.
3. **Speed Run:** Toggle **"Speed Run"** mode. Set a high `Speed Run Max` and a safe `Pre-Brake Speed`.
4. **Win:** Watch the robot blast through straights and brake perfectly for every corner!

---

## 📜 License
MIT License - Feel free to use and modify for your own robotics projects!

