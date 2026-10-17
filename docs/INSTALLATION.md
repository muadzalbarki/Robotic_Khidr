# Installation Guide - Raspberry Pi 5 Setup

## Sistem Requirements

- **Hardware:**
  - Raspberry Pi 5 (4GB+ RAM recommended)
  - MicroSD Card (32GB+ Class 10)
  - Raspberry Pi Camera Module (with CSI cable)
  - YDLIDAR YDS2MD-1
  - ESP32-S3 (with USB cable)
  - Power supply 5V/3A untuk Raspberry Pi
  - 12V battery untuk motor

- **Operating System:**
  - Raspberry Pi OS (64-bit) Bookworm or later

---

## 1. Raspberry Pi OS Installation

### 1.1 Flash OS ke SD Card

```bash
# Download Raspberry Pi Imager
# https://www.raspberrypi.com/software/

# Flash "Raspberry Pi OS (64-bit)" ke SD card
# Enable SSH dan set username/password saat setup
```

### 1.2 First Boot Setup

```bash
# Update system
sudo apt update
sudo apt upgrade -y

# Install essential tools
sudo apt install -y git vim python3-pip python3-venv
```

---

## 2. Camera Setup

### 2.1 Enable Camera Interface

```bash
# Enable camera via raspi-config
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable

# Reboot
sudo reboot
```

### 2.2 Test Camera

```bash
# Test with libcamera
libcamera-hello

# Should show camera preview for 5 seconds
# If error, check cable connection
```

### 2.3 Install Picamera2

```bash
# Install picamera2 (should be pre-installed on Bookworm)
sudo apt install -y python3-picamera2

# If not available, install from source:
sudo apt install -y python3-libcamera python3-kms++
sudo apt install -y python3-pyqt5 python3-prctl libatlas-base-dev
pip3 install picamera2
```

---

## 3. Python Dependencies

### 3.1 Install System Packages

```bash
# OpenCV dependencies
sudo apt install -y python3-opencv

# Serial communication
sudo apt install -y python3-serial

# Scientific computing
sudo apt install -y python3-numpy python3-scipy
```

### 3.2 Install Python Packages

```bash
cd /home/mikoto/Documents/Robotik/raspi

# Install from requirements.txt
pip3 install --break-system-packages -r requirements.txt

# Or install individually:
pip3 install --break-system-packages numpy opencv-python picamera2 pyserial scipy
```

**Note:** `--break-system-packages` flag diperlukan di Raspberry Pi OS Bookworm untuk install via pip.

---

## 4. Serial Port Permissions

### 4.1 Add User to Groups

```bash
# Add user to dialout group (for serial access)
sudo usermod -a -G dialout $USER

# Add user to video group (for camera access)
sudo usermod -a -G video $USER

# Logout and login kembali untuk apply changes
# Atau reboot:
sudo reboot
```

### 4.2 Verify Serial Ports

```bash
# List serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Should show devices with permissions for dialout group
# Example output:
# crw-rw---- 1 root dialout ... /dev/ttyUSB0
# crw-rw---- 1 root dialout ... /dev/ttyUSB1
```

---

## 5. ESP32 Setup

### 5.1 Upload Code ke ESP32

```bash
# Install Arduino IDE di komputer lain (Windows/Mac/Linux)
# https://www.arduino.cc/en/software

# Install ESP32 Board Support:
# File → Preferences → Additional Board Manager URLs:
# https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# Tools → Board Manager → Search "ESP32" → Install

# Install ESP32Servo Library:
# Tools → Manage Libraries → Search "ESP32Servo" → Install
```

### 5.2 Configure Arduino IDE

```
Board: "ESP32S3 Dev Module"
Upload Speed: 921600
USB Mode: "Hardware CDC and JTAG"
USB CDC On Boot: "Enabled"
Port: (select port ESP32 terhubung)
```

### 5.3 Upload Sketch

```
1. Open: esp32/main_autonomous/main_autonomous.ino
2. Verify/Compile (Ctrl+R)
3. Upload (Ctrl+U)
4. Open Serial Monitor (Ctrl+Shift+M) untuk verify
5. Should see: "=== ESP32 Autonomous Robot Controller ==="
```

---

## 6. LiDAR Setup

### 6.1 Connect YDLIDAR YDS2MD-1

```bash
# Connect LiDAR via USB to Raspberry Pi
# LiDAR akan muncul sebagai /dev/ttyUSB0 atau /dev/ttyACM0

# Check connection
ls -l /dev/ttyUSB* /dev/ttyACM*

# Test read data
sudo cat /dev/ttyUSB1  # Adjust port number
# Should see binary data stream
```

### 6.2 Set Permissions

```bash
# Create udev rule for persistent permissions
sudo nano /etc/udev/rules.d/99-lidar.rules

# Add line:
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Reconnect LiDAR
```

---

## 7. Project Setup

### 7.1 Clone/Copy Project

```bash
# If using git:
cd /home/mikoto/Documents
git clone <your-repo-url> Robotik

# Or if files already exist, just navigate:
cd /home/mikoto/Documents/Robotik
```

### 7.2 Directory Structure

```
/home/mikoto/Documents/Robotik/
├── esp32/
│   ├── main_autonomous/
│   │   └── main_autonomous.ino
│   └── mainServo_backup/
│       └── mainServo.ino
├── raspi/
│   ├── src/
│   │   ├── main.py
│   │   ├── config.py
│   │   ├── vision.py
│   │   ├── lidar_nav.py
│   │   ├── serial_comm.py
│   │   └── utils.py
│   ├── tools/
│   │   ├── color_calibration.py
│   │   ├── distance_calibration.py
│   │   └── test_modules.py
│   ├── requirements.txt
│   └── launch.sh
└── docs/
```

### 7.3 Verify Installation

```bash
cd /home/mikoto/Documents/Robotik/raspi

# Test modules
python3 tools/test_modules.py

# Follow prompts untuk test setiap module
```

---

## 8. Configuration

### 8.1 Update Serial Ports di config.py

```bash
cd /home/mikoto/Documents/Robotik/raspi/src

# Check actual ports
ls -l /dev/ttyUSB* /dev/ttyACM*

# Edit config.py
nano config.py

# Update:
SERIAL_PORT = "/dev/ttyUSB0"  # ESP32 port
LIDAR_PORT = "/dev/ttyUSB1"   # LiDAR port

# Save: Ctrl+O, Enter, Ctrl+X
```

---

## 9. Camera Mounting

### 9.1 Physical Setup

```
Camera Mount Requirements:
- Angle: 25° downward from horizontal
- Height: 15-20cm from ground
- Position: Front-center of robot
- Stable mounting (no vibration)
```

### 9.2 Verify Field of View

```bash
# Test camera view
python3 -c "from picamera2 import Picamera2; import time; p = Picamera2(); p.start(); time.sleep(2); p.capture_file('test.jpg'); p.stop(); print('Saved to test.jpg')"

# Check image untuk verify angle dan coverage
```

---

## 10. Troubleshooting

### Camera Issues

```bash
# Check camera detected
vcgencmd get_camera

# Should return: supported=1 detected=1

# If not detected:
# 1. Check cable connection (blue side to camera, connector clips up)
# 2. Enable camera in raspi-config
# 3. Reboot
```

### Serial Port Issues

```bash
# Check permissions
groups
# Should include: dialout video

# Check port availability
sudo dmesg | grep tty
# Should show USB serial devices

# Test communication
sudo minicom -D /dev/ttyUSB0 -b 115200
# Should connect (Ctrl+A X to exit)
```

### Python Import Errors

```bash
# Verify installations
python3 -c "import cv2; print('OpenCV OK')"
python3 -c "import picamera2; print('Picamera2 OK')"
python3 -c "import serial; print('PySerial OK')"
python3 -c "import numpy; print('NumPy OK')"

# If errors, reinstall:
pip3 install --break-system-packages --force-reinstall <package-name>
```

### Permission Denied Errors

```bash
# Re-add to groups
sudo usermod -a -G dialout,video $USER

# Logout/login or reboot
sudo reboot

# Verify
groups
# Should list: dialout video
```

---

## 11. Optional: Auto-start on Boot

```bash
# Create systemd service
sudo nano /etc/systemd/system/autonomous-robot.service

# Add:
[Unit]
Description=Autonomous Robot Service
After=network.target

[Service]
Type=simple
User=mikoto
WorkingDirectory=/home/mikoto/Documents/Robotik/raspi
ExecStart=/home/mikoto/Documents/Robotik/raspi/launch.sh
Restart=on-failure

[Install]
WantedBy=multi-user.target

# Enable service
sudo systemctl daemon-reload
sudo systemctl enable autonomous-robot.service

# Start/stop manually:
# sudo systemctl start autonomous-robot
# sudo systemctl stop autonomous-robot
```

---

## 12. Next Steps

Setelah installation complete:

1. **Calibrate Colors:**
   ```bash
   python3 tools/color_calibration.py
   ```

2. **Calibrate Distance:**
   ```bash
   python3 tools/distance_calibration.py
   ```

3. **Test All Modules:**
   ```bash
   python3 tools/test_modules.py
   ```

4. **Run Autonomous Mission:**
   ```bash
   ./launch.sh
   ```

Lihat `docs/HARDWARE_WIRING.md` untuk connection diagram.
