1. Initial Setup
# Clone repository
git clone https://github.com/muadzalbarki/Robotic_Khidr
cd Robotic_Khidr

# Install system dependencies (GUI & Camera support)
sudo apt update
sudo apt install libgtk-3-dev pkg-config libcamera-dev python3-libcamera xauth x11-apps
2. Virtual Environment
# Create venv with system access
python3 -m venv --system-site-packages venv
source venv/bin/activate

# Install dependencies
pip install -r raspi/requirements.txt
3. Firmware Upload
- Unggah file esp32/main_autonomous/main_autonomous.ino ke ESP32-S3 menggunakan Arduino IDE (pastikan Library ESP32Servo sudah terinstal).
4. Running the Robot
Untuk menjalankan robot, gunakan skrip otomatis yang telah disediakan:
Mode Headless (Tanpa GUI - Untuk Lomba):
./main.py
Mode GUI (Untuk Kalibrasi/Monitoring via SSH -X):
./run_robot.sh --gui