"""
Configuration file untuk Robot Otonom
Berisi semua parameter untuk Vision, LiDAR, Navigation, dan Hardware

IMPORTANT: HSV values perlu dikalibrasi menggunakan tools/color_calibration.py
untuk kondisi lighting spesifik di area testing Anda.
"""

import os

# ===== SERIAL COMMUNICATION =====
SERIAL_PORT = "/dev/ttyACM0"  # ESP32-S3 USB connection
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1

# ===== LiDAR CONFIGURATION =====
LIDAR_PORT = "/dev/ttyUSB0"     # YDLIDAR YDS2MD-1
LIDAR_BAUD = 115200             # YDLIDAR YDS2MD-1 default baud
LIDAR_TIMEOUT = 1.0

# Sector definitions (degrees)
LIDAR_SECTORS = {
    "front": (330, 30),         # -30° to +30°
    "front_left": (300, 330),
    "front_right": (30, 60),
    "left": (270, 330),
    "right": (30, 90),
    "back": (150, 210)
}

# ===== MISSION PARAMETERS =====
MAX_BOTTLES = 4                 # Total bottles to deliver (red, yellow, green, blue)
BOTTLE_PRIORITY = ['red', 'yellow', 'green', 'blue']  # Detection priority

# Watering zone mapping (warna botol -> warna lantai zone)
WATERING_ZONE_MAPPING = {
    "red": "red_floor",
    "yellow": "yellow_floor",
    "green": "green_floor",
    "blue": "blue_floor"
}

# ===== LOGGING =====
LOG_LEVEL = "INFO"              # DEBUG, INFO, WARNING, ERROR
LOG_TO_FILE = True
LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "logs")
LOG_FILE = "robot_autonomous.log"

# Console colors for logging
LOG_COLORS = {
    "DEBUG": "\033[36m",        # Cyan
    "INFO": "\033[32m",         # Green
    "WARNING": "\033[33m",      # Yellow
    "ERROR": "\033[31m",        # Red
    "CRITICAL": "\033[35m",     # Magenta
    "RESET": "\033[0m"
}

# ===== DEBUG & VISUALIZATION =====
ENABLE_VISUALIZATION = True     # Show camera feed dengan detection overlay
ENABLE_VERBOSE_LOGGING = True   # Detailed logging untuk debugging
SAVE_DEBUG_IMAGES = False       # Save frames untuk post-analysis (disk-intensive!)

# Visualization settings
VIS_WINDOW_NAME = "Robot Vision"
VIS_DRAW_CONTOURS = True
VIS_DRAW_CENTROIDS = True
VIS_DRAW_LABELS = True
VIS_DRAW_DISTANCES = True

# ===== SAFETY & RECOVERY =====
EMERGENCY_STOP_ENABLED = True
MAX_STUCK_TIME = 10.0           # seconds, if no progress -> recovery action
MAX_RETRY_ATTEMPTS = 3          # Max retries per state before error
WATCHDOG_TIMEOUT = 0.5          # seconds, ESP32 watchdog (must match ESP32 code)

# Recovery behaviors
ENABLE_REVERSE_ON_STUCK = True
REVERSE_DURATION = 1.0          # seconds
ENABLE_ROTATION_SEARCH = True
ROTATION_SEARCH_ANGLE = 360     # degrees to search

# ===== HARDWARE PINS (for reference, actual control via ESP32) =====
# Motor Driver BTS7960
PIN_MOTOR_LEFT_RPWM = 17
PIN_MOTOR_LEFT_LPWM = 18
PIN_MOTOR_RIGHT_RPWM = 6
PIN_MOTOR_RIGHT_LPWM = 7

# Servo
PIN_SERVO_LIFT = 4
PIN_SERVO_GRIPPER = 5

# ===== CALIBRATION STATUS =====
# Update ini ke True setelah kalibrasi warna dan distance selesai
COLOR_CALIBRATED = False        # Set True after running color_calibration.py
DISTANCE_CALIBRATED = False     # Set True after running distance_calibration.py

# ===== NOTES =====
"""
IMPORTANT CALIBRATION STEPS:

1. Color Calibration (WAJIB):
   python3 tools/color_calibration.py
   - Kalibrasi setiap warna target di kondisi lighting area testing
   - Update HSV ranges di COLOR_RANGES
   - Set COLOR_CALIBRATED = True

2. Distance Calibration (RECOMMENDED):
   python3 tools/distance_calibration.py
   - Place black object at berbagai jarak (30cm - 200cm)
   - Generate calibration formula
   - Update DISTANCE_CALIBRATION values
   - Set DISTANCE_CALIBRATED = True

3. Testing Individual Modules:
   python3 tools/test_modules.py
   - Verify vision detection
   - Verify LiDAR scanning
   - Verify serial communication
   - Verify all sensors working

4. Full Mission Testing:
   python3 src/main.py --verbose
"""
