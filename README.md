# README - Autonomous Robot Multi-Trip Watering Mission

## 📋 Overview

Sistem robot otonom dengan arsitektur **Master-Slave** untuk misi penyiraman tanaman. Robot akan mengambil 4 botol berwarna berbeda dari Water Station dan menempatkannya di Watering Zone yang sesuai dengan warna botol, kemudian kembali ke Home Base.

### 🎯 Mission Flow

```
START (Home Base) 
    ↓
┌───┴──────────────────────────────────────┐
│  Loop 4x (untuk setiap botol):          │
│    1. Navigate to Water Station          │
│    2. Detect & Grab Bottle (R/Y/G/B)    │
│    3. Find White Pot                     │
│    4. Navigate to Watering Zone          │
│    5. Release Bottle                     │
└───┬──────────────────────────────────────┘
    ↓
Return to Home Base
    ↓
DONE
```

---

## 🔧 Hardware

### Master (Raspberry Pi 5)
- **Processor:** Quad-core ARM Cortex-A76 @ 2.4GHz
- **RAM:** 4GB+ recommended
- **Camera:** Raspberry Pi Camera Module (CSI, 25° downward angle)
- **LiDAR:** YDLIDAR YDS2MD-1 (360°, 0.12-8m range)
- **OS:** Raspberry Pi OS (64-bit) Bookworm

### Slave (ESP32-S3)
- **Microcontroller:** ESP32-S3 (dual-core, 240MHz)
- **Motor Drivers:** 2x BTS7960 (untuk 4 roda differential drive)
- **Servos:** 2x MG90 (lift + gripper)
- **Communication:** USB Serial (115200 baud)

### Power System
- **12V Battery:** 3S LiPo (11.1V nominal) untuk motors
- **5V Buck Converter:** 5A rating untuk Raspberry Pi
- **Total Current:** ~5A @ 5V, 2-5A @ 12V

### Sensors & Targets
- **Navigation:** Vision (colored papers 50x50cm) + LiDAR (obstacle avoidance)
- **Target Colors:**
  - Pink/Magenta: Home Base
  - Dark Blue: Water Station
  - Red/Yellow/Green/Blue: Bottles & Watering Zones
  - White: Pot (circle detection)
  - Black: Physical obstacles

---

## 📁 Project Structure

```
Robotik/
├── esp32/
│   ├── main_autonomous/
│   │   └── main_autonomous.ino      # ESP32 serial-controlled firmware
│   └── mainServo_backup/
│       └── mainServo.ino             # Backup WiFi manual control
│
├── raspi/
│   ├── src/
│   │   ├── main.py                   # Multi-trip state machine
│   │   ├── config.py                 # Configuration & HSV ranges
│   │   ├── vision.py                 # OpenCV color + pot detection
│   │   ├── lidar_nav.py              # YDLIDAR obstacle avoidance
│   │   ├── serial_comm.py            # ESP32 communication
│   │   └── utils.py                  # PID, logging, helpers
│   │
│   ├── tools/
│   │   ├── color_calibration.py      # Interactive HSV tuning
│   │   ├── distance_calibration.py   # Distance estimation calibration
│   │   └── test_modules.py           # Component testing
│   │
│   ├── requirements.txt              # Python dependencies
│   └── launch.sh                     # Startup script
│
└── docs/
    ├── INSTALLATION.md               # Setup guide
    ├── HARDWARE_WIRING.md            # Wiring diagrams
    └── README.md                     # This file
```

---

## 🚀 Quick Start

### 1. Hardware Setup

```bash
# Connect hardware sesuai docs/HARDWARE_WIRING.md
# - ESP32 via USB to Raspberry Pi
# - Camera via CSI port (25° downward)
# - LiDAR via USB
# - Motors to BTS7960, servos to ESP32
```

### 2. Upload ESP32 Firmware

```bash
# Di Arduino IDE:
# 1. Open: esp32/main_autonomous/main_autonomous.ino
# 2. Board: ESP32S3 Dev Module
# 3. Upload (Ctrl+U)
# 4. Verify Serial Monitor shows: "READY"
```

### 3. Install Raspberry Pi Dependencies

```bash
cd /home/mikoto/Documents/Robotik/raspi

# System packages
sudo apt update
sudo apt install -y python3-opencv python3-picamera2 python3-serial

# Python packages
pip3 install --break-system-packages -r requirements.txt

# Permissions
sudo usermod -a -G dialout,video $USER
sudo reboot
```

### 4. Calibrate System

```bash
cd /home/mikoto/Documents/Robotik/raspi

# Color calibration (WAJIB!)
python3 tools/color_calibration.py
# Adjust HSV untuk semua 12 target colors
# Press 's' to save, 'n' for next color

# Distance calibration (Recommended)
python3 tools/distance_calibration.py
# Place black object at 10-15 different distances
# Press SPACE to capture, input actual distance

# Update config.py dengan hasil calibration
```

### 5. Test Modules

```bash
# Test individual components
python3 tools/test_modules.py

# Test akan check:
# - Configuration
# - Utilities
# - Vision system
# - LiDAR (atau simulator)
# - ESP32 serial communication
```

### 6. Run Autonomous Mission

```bash
# Full autonomous mode
./launch.sh

# Atau dengan options:
./launch.sh --simulator    # Use LiDAR simulator
./launch.sh --verbose      # Enable verbose logging

# Emergency stop: Press Ctrl+C
```

---

## 📊 System Architecture

### Software Architecture

```
┌─────────────────────────────────────────────────┐
│           Raspberry Pi 5 (Master)               │
│                                                 │
│  ┌──────────────────────────────────────────┐  │
│  │         main.py (State Machine)          │  │
│  │  - GOTO_WATER_STATION                    │  │
│  │  - APPROACH_BOTTLE                       │  │
│  │  - GRAB_BOTTLE                           │  │
│  │  - FIND_POT                              │  │
│  │  - GOTO_WATERING_ZONE                    │  │
│  │  - RELEASE_BOTTLE                        │  │
│  │  - CHECK_MORE_BOTTLES (loop)             │  │
│  │  - RETURN_HOME                           │  │
│  └──────────────────────────────────────────┘  │
│         │              │              │         │
│    ┌────▼───┐    ┌────▼────┐    ┌────▼────┐   │
│    │ Vision │    │  LiDAR  │    │ Serial  │   │
│    │ System │    │   Nav   │    │  Comm   │   │
│    └────────┘    └─────────┘    └────┬────┘   │
│                                       │         │
└───────────────────────────────────────┼─────────┘
                                        │ USB Serial
                                        │ 115200 baud
┌───────────────────────────────────────▼─────────┐
│              ESP32-S3 (Slave)                   │
│                                                 │
│  ┌──────────────────────────────────────────┐  │
│  │    Serial Command Parser                 │  │
│  │    Format: <L>,<R>,<Lift>,<Grip>\n      │  │
│  └──────────────────────────────────────────┘  │
│         │                            │          │
│    ┌────▼────┐                  ┌───▼──────┐   │
│    │ Motor   │                  │  Servo   │   │
│    │ Control │                  │ Control  │   │
│    │ PWM     │                  │          │   │
│    └────┬────┘                  └───┬──────┘   │
│         │                           │          │
└─────────┼───────────────────────────┼──────────┘
          │                           │
     ┌────▼────┐              ┌───────▼────────┐
     │ BTS7960 │              │  MG90 Servos   │
     │ Drivers │              │  - Lift        │
     │  x2     │              │  - Gripper     │
     └────┬────┘              └────────────────┘
          │
     ┌────▼────┐
     │ 4 Wheels│
     │ DC Motor│
     └─────────┘
```

### Sensor Fusion

```
Navigation Decision = Vision Detection + LiDAR Obstacle Avoidance

Priority Levels:
1. LiDAR Critical (<30cm)     → EMERGENCY STOP
2. Vision Black Obstacles     → AVOID (turn away)
3. LiDAR Warning (30-50cm)    → SLOW + AVOID
4. Vision Target Navigation   → APPROACH with PID steering
```

---

## ⚙️ Configuration

### Serial Ports (`raspi/src/config.py`)

```python
SERIAL_PORT = "/dev/ttyUSB0"  # ESP32
LIDAR_PORT = "/dev/ttyUSB1"   # LiDAR

# Check actual ports:
# ls -l /dev/ttyUSB* /dev/ttyACM*
```

### HSV Color Ranges

```python
# Kalibrasi menggunakan tools/color_calibration.py
COLOR_RANGES = {
    "home_base": ([140, 50, 150], [170, 255, 255]),      # Pink
    "water_station": ([100, 100, 50], [130, 255, 150]),  # Dark Blue
    "red_bottle": ([0, 120, 70], [10, 255, 255]),
    # ... dst
}
```

### Navigation Parameters

```python
MAX_SPEED = 150              # PWM value (0-255)
APPROACH_SPEED = 80          # Slower speed saat mendekati target
TURN_SPEED = 100             # Rotation speed

KP_STEERING = 0.3            # PID proportional gain
KD_STEERING = 0.1            # PID derivative gain

OBSTACLE_DISTANCE = 50       # cm threshold LiDAR
```

---

## 🧪 Testing & Debugging

### Component Tests

```bash
# Test vision only
cd raspi/src
python3 vision.py

# Test LiDAR only
python3 lidar_nav.py --simulator

# Test ESP32 communication
python3 serial_comm.py

# Test utilities
python3 utils.py
```

### Debug Mode

```python
# Edit config.py:
ENABLE_VISUALIZATION = True      # Show camera feed with detections
ENABLE_VERBOSE_LOGGING = True    # Detailed console output
SAVE_DEBUG_IMAGES = True         # Save frames to disk (disk-intensive!)
```

### Logs

```bash
# Logs disimpan di:
raspi/logs/YYYYMMDD_HHMMSS_robot_autonomous.log

# Monitor real-time:
tail -f raspi/logs/*.log
```

---

## 🔍 Troubleshooting

### Camera tidak terdeteksi

```bash
# Check camera
vcgencmd get_camera
# Should return: supported=1 detected=1

# Test capture
libcamera-hello

# Enable camera di raspi-config
sudo raspi-config
# Interface Options → Camera → Enable
```

### Serial port permission denied

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Reboot
sudo reboot

# Verify
groups | grep dialout
```

### Motor tidak bergerak

```bash
# Check ESP32 connection
ls -l /dev/ttyUSB*

# Test manual command
python3 src/serial_comm.py

# Verify di Serial Monitor Arduino IDE:
# Should see: OK response untuk setiap command
```

### Color detection tidak akurat

```bash
# Re-calibrate colors
python3 tools/color_calibration.py

# Tips:
# - Adjust lighting (consistent, no shadows)
# - Letakkan target di berbagai jarak
# - HSV ranges perlu relaxed untuk banner/paper textures
```

### Robot stuck atau tidak navigasi smooth

```bash
# Check:
# 1. Camera angle (25° downward optimal)
# 2. LiDAR working (python3 src/lidar_nav.py)
# 3. PID parameters (adjust KP, KD di config.py)
# 4. Speed settings (reduce MAX_SPEED untuk testing)
```

---

## 📖 Documentation

### Detailed Guides

- **Installation:** `docs/INSTALLATION.md` - Complete setup instructions
- **Hardware Wiring:** `docs/HARDWARE_WIRING.md` - Pin connections & diagrams
- **Color Calibration:** Use `tools/color_calibration.py` with target objects
- **Distance Calibration:** Use `tools/distance_calibration.py` with measuring tape

### Key Concepts

**State Machine:** Robot menggunakan finite state machine dengan 10 states untuk manage mission flow.

**Multi-Trip:** Robot akan loop 4x untuk deliver semua botol, tracking `bottles_delivered` dan `delivered_colors` list.

**Sensor Fusion:** Combining vision (color detection) dengan LiDAR (obstacle avoidance) untuk robust navigation.

**PID Controller:** Smooth steering menggunakan proportional-derivative control berdasarkan vision error.

---

## 🛠️ Development

### Adding New Features

```python
# 1. Add new state to RobotState enum (main.py)
class RobotState(Enum):
    NEW_STATE = 11

# 2. Implement handler method
def handle_new_state(self):
    # Your logic here
    pass

# 3. Add to state machine dispatcher
elif self.state == RobotState.NEW_STATE:
    self.handle_new_state()
```

### Modifying Navigation

```python
# Edit navigate_with_obstacle_avoidance() di main.py
# Adjust priority levels, thresholds, atau add new sensors
```

### Custom Color Targets

```python
# 1. Add to COLOR_RANGES di config.py
"new_target": ([H_min, S_min, V_min], [H_max, S_max, V_max])

# 2. Calibrate dengan tools/color_calibration.py
# 3. Use di vision.py:
obj = self.detect_object(frame, 'new_target', min_area)
```

---

## 📝 Notes

### Limitations

- **Indoor Environment:** System dirancang untuk indoor 3x3m dengan controlled lighting
- **Flat Surface:** Robot menggunakan wheeled locomotion, butuh flat ground
- **Color Dependency:** Detection tergantung HSV calibration yang accurate
- **Single Object:** Gripper design untuk 1 botol at a time

### Future Improvements

- [ ] Add path planning (A* atau RRT)
- [ ] Implement SLAM untuk mapping
- [ ] Add battery monitoring
- [ ] Improve stuck detection & recovery
- [ ] Add voice feedback atau status LED
- [ ] Support multiple pot positions
- [ ] Implement learning dari previous runs

---

## 🤝 Contributing

Project ini adalah student project untuk autonomous robotics. Suggestions welcome!

---

## 📄 License

Educational project - developed untuk robotics competition/learning.

---

## 👥 Credits

**Hardware:**
- Raspberry Pi Foundation (Raspberry Pi 5)
- Espressif (ESP32-S3)
- YDLIDAR (YDS2MD-1)

**Software Libraries:**
- OpenCV (Computer Vision)
- Picamera2 (Raspberry Pi Camera)
- PySerial (Serial Communication)
- NumPy, SciPy (Numerical Computing)

---

## 📞 Support

Jika ada issues atau questions:

1. Check `docs/TROUBLESHOOTING.md` (coming soon)
2. Review logs di `raspi/logs/`
3. Test individual modules dengan `tools/test_modules.py`
4. Verify hardware connections di `docs/HARDWARE_WIRING.md`

---

**Status:** ✅ Full Implementation Complete

**Last Updated:** 2026-10-17

**Version:** 1.0.0
