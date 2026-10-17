#!/bin/bash

# Launch Script untuk Autonomous Robot
# Usage: ./launch.sh [options]

echo "=========================================="
echo "  AUTONOMOUS ROBOT - LAUNCH SCRIPT"
echo "=========================================="
echo ""

# Check if running on Raspberry Pi
if [ ! -f /proc/device-tree/model ]; then
    echo "[WARNING] Not running on Raspberry Pi?"
fi

# Parse arguments
USE_SIMULATOR=false
VERBOSE=false

for arg in "$@"; do
    case $arg in
        --simulator|--sim)
            USE_SIMULATOR=true
            shift
            ;;
        --verbose|-v)
            VERBOSE=true
            shift
            ;;
        --help|-h)
            echo "Usage: ./launch.sh [options]"
            echo ""
            echo "Options:"
            echo "  --simulator, --sim    Use LiDAR simulator (no hardware)"
            echo "  --verbose, -v         Enable verbose logging"
            echo "  --help, -h            Show this help message"
            echo ""
            exit 0
            ;;
    esac
done

# Check Python version
echo "[CHECK] Python version..."
python3 --version

# Check if in correct directory
if [ ! -f "src/main.py" ]; then
    echo "[ERROR] Please run this script from raspi/ directory"
    echo "  cd /home/mikoto/Documents/Robotik/raspi"
    echo "  ./launch.sh"
    exit 1
fi

# Check dependencies
echo "[CHECK] Checking dependencies..."
python3 -c "import cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "[ERROR] OpenCV not installed. Run: pip3 install opencv-python"
    exit 1
fi

python3 -c "import picamera2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "[ERROR] Picamera2 not installed. Run: sudo apt install python3-picamera2"
    exit 1
fi

python3 -c "import serial" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "[ERROR] PySerial not installed. Run: pip3 install pyserial"
    exit 1
fi

echo "[PASS] All dependencies installed"

# Check hardware connections
echo ""
echo "[CHECK] Hardware connections..."

# Check camera
if [ -e /dev/video0 ] || [ -e /dev/video10 ]; then
    echo "[PASS] Camera detected"
else
    echo "[WARNING] Camera not detected"
fi

# Check serial ports
if ls /dev/ttyUSB* 1> /dev/null 2>&1 || ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo "[PASS] Serial devices detected:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | while read port; do
        echo "  - $port"
    done
else
    echo "[WARNING] No serial devices detected (ESP32/LiDAR)"
    echo "  Make sure devices are connected via USB"
fi

# Create logs directory
mkdir -p logs

# Build command
CMD="python3 src/main.py"

if [ "$USE_SIMULATOR" = true ]; then
    CMD="$CMD --simulator"
    echo ""
    echo "[MODE] Using LiDAR SIMULATOR"
fi

if [ "$VERBOSE" = true ]; then
    CMD="$CMD --verbose"
fi

echo ""
echo "=========================================="
echo "  STARTING AUTONOMOUS ROBOT"
echo "=========================================="
echo ""
echo "Command: $CMD"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the robot
cd "$(dirname "$0")"
exec $CMD
