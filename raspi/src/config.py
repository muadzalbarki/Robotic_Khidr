"""
Configuration file untuk Robot Otonom
Berisi semua parameter untuk Vision, LiDAR, Navigation, dan Hardware

IMPORTANT: HSV values perlu dikalibrasi menggunakan tools/color_calibration.py
untuk kondisi lighting spesifik di area testing Anda.
"""

import os

# ===== SERIAL COMMUNICATION =====
SERIAL_PORT = "/dev/ttyUSB0"  # ESP32 USB connection
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.1

# ===== CAMERA CONFIGURATION =====
CAMERA_INDEX = 0
CAMERA_ANGLE_DEG = 25           # Downward tilt from horizontal
CAMERA_HEIGHT_CM = 20           # Height from ground
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_FPS = 30
FRAME_CENTER_X = FRAME_WIDTH // 2
FRAME_CENTER_Y = FRAME_HEIGHT // 2

# ===== HSV COLOR RANGES =====
# Format: (lower_hsv, upper_hsv) where HSV = [Hue 0-180, Saturation 0-255, Value 0-255]
# CATATAN: Values ini adalah estimasi awal. WAJIB dikalibrasi dengan color_calibration.py!

COLOR_RANGES = {
    # Navigation Targets (50x50cm colored papers)
    "home_base": ([140, 50, 150], [170, 255, 255]),      # Pink/Magenta paper
    "water_station": ([100, 100, 50], [130, 255, 150]),  # Dark Blue paper
    
    # Bottles (small objects)
    "red_bottle": ([0, 120, 70], [10, 255, 255]),
    "red_bottle_wrap": ([170, 120, 70], [180, 255, 255]),  # Hue wraparound for red
    "yellow_bottle": ([20, 100, 100], [30, 255, 255]),
    "green_bottle": ([40, 50, 50], [80, 255, 255]),
    "blue_bottle": ([90, 80, 80], [110, 255, 255]),
    
    # Watering Zone Floors (50x50cm colored papers, same colors as bottles)
    # Slightly relaxed ranges untuk accommodate lighting variations di lantai
    "red_floor": ([0, 80, 50], [10, 255, 200]),
    "red_floor_wrap": ([170, 80, 50], [180, 255, 200]),
    "yellow_floor": ([20, 80, 80], [30, 255, 200]),
    "green_floor": ([40, 40, 40], [80, 255, 200]),
    "blue_floor": ([90, 60, 60], [110, 255, 200]),
    
    # Pot Detection (white pot)
    "pot_white": ([0, 0, 180], [180, 30, 255]),          # High V, low S = white
    
    # Obstacle Detection (black physical objects)
    "obstacle_black": ([0, 0, 0], [180, 255, 50])        # Low V = dark/black
}

# ===== OBJECT DETECTION THRESHOLDS =====
# Minimum pixel area untuk valid detection
MIN_AREA_THRESHOLD = 500        # General minimum
BOTTLE_MIN_AREA = 800           # Bottles (small objects)
STATION_MIN_AREA = 2000         # Navigation targets (50x50cm papers)
FLOOR_ZONE_MIN_AREA = 2000      # Watering zone floors
POT_MIN_AREA = 3000             # Pot (large object)
OBSTACLE_MIN_AREA = 500         # Black obstacles

# YOLO26n Configuration
YOLO_MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "models", "yolo26n.pt")
YOLO_CONFIDENCE_THRESHOLD = 0.60
YOLO_BOTTLE_CLASS_ID = 39       # COCO class ID for bottle

# Pot-specific detection
POT_MIN_CIRCULARITY = 0.6       # Circle detection threshold (0-1)
POT_SEARCH_ZONE_WIDTH = 200     # Pixels, width area scan beside pot

# Floor detection zone (dengan camera 25° downward)
FLOOR_DETECTION_ZONE = {
    "y_min": 320,               # Bottom half of frame (near field)
    "y_max": 480,
    "min_area": 2000
}

# ===== DISTANCE ESTIMATION =====
# Distance estimation from Y position and area (NEEDS CALIBRATION!)
# Gunakan tools/distance_calibration.py untuk kalibrasi akurat

DISTANCE_CALIBRATION = {
    "frame_bottom_cm": 30,      # Camera dapat lihat 30cm di depan robot (near field)
    "frame_top_cm": 300,        # Far field horizon (~3m)
    "area_reference": 5000      # Reference area for 1m distance object
}

# Distance-based area thresholds
DISTANCE_THRESHOLDS = {
    "very_close": 15000,        # area > 15000 pixel (~30cm)
    "close": 8000,              # area > 8000 pixel (~50cm)
    "medium": 3000,             # area > 3000 pixel (~1m)
    "far": 1000                 # area > 1000 pixel (~2m)
}

# ===== OBSTACLE AVOIDANCE =====
# Vision-based (black obstacles)
BLACK_OBSTACLE_CRITICAL_DISTANCE = 50   # cm
BLACK_OBSTACLE_WARNING_DISTANCE = 100   # cm

# LiDAR-based (all physical obstacles)
LIDAR_CRITICAL_THRESHOLD = 30           # cm - emergency stop
LIDAR_WARNING_THRESHOLD = 50            # cm - slow + avoid
LIDAR_SAFE_DISTANCE = 80                # cm - maintain this distance

# ===== NAVIGATION PARAMETERS =====
# Motor speeds (PWM values 0-255)
MAX_SPEED = 150
MIN_SPEED = 60
APPROACH_SPEED = 80             # Speed when approaching target
TURN_SPEED = 100                # Speed for rotation
SEARCH_SPEED = 70               # Speed when searching for target

# PID Controller parameters
KP_STEERING = 0.3               # Proportional gain for steering
KI_STEERING = 0.0               # Integral gain (usually 0 for simple robots)
KD_STEERING = 0.1               # Derivative gain for smoothing
KP_DISTANCE = 0.5               # Proportional gain for distance control

# Steering thresholds
STEERING_DEADZONE = 10          # Pixels, no correction if error < this
ALIGNED_THRESHOLD = 20          # Pixels, considered aligned if error < this

# ===== SERVO POSITIONS =====
SERVO_LIFT_UP = 120             # Lift position for transport
SERVO_LIFT_MID = 90             # Middle position
SERVO_LIFT_DOWN = 45            # Down position for grab/release
SERVO_GRIP_OPEN = 30            # Gripper open
SERVO_GRIP_CLOSE = 90           # Gripper close/holding

# ===== STATE MACHINE TIMEOUTS =====
GRAB_SEQUENCE_TIME = 3.0        # seconds for grab sequence
RELEASE_SEQUENCE_TIME = 2.0     # seconds for release sequence
NAVIGATION_TIMEOUT = 120.0      # seconds max per navigation state
POT_SEARCH_TIMEOUT = 30.0       # seconds to find pot
BOTTLE_SEARCH_TIMEOUT = 20.0    # seconds to find bottle at water station
TARGET_SEARCH_TIMEOUT = 30.0    # seconds to find navigation target
APPROACH_TIMEOUT = 15.0         # seconds for final approach

# Search behavior
SEARCH_ROTATION_SPEED = 60      # Slow rotation when searching
SEARCH_ROTATION_STEP = 0.5      # seconds per rotation step

# ===== LIDAR CONFIGURATION =====
LIDAR_PORT = "/dev/ttyUSB0"     # Update ke port yang terdeteksi
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
