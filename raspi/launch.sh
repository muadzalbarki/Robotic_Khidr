#!/bin/bash
# Robot Launch Script: Main entry point for the autonomous robot

# Activate venv
if [ -d "venv" ]; then
    source venv/bin/activate
elif [ -d "../venv" ]; then
    source ../venv/bin/activate
elif [ -d "../../venv" ]; then
    source ../../venv/bin/activate
else
    echo "[ERROR] Virtual environment 'venv' not found."
    exit 1
fi

# Launch main application using absolute or correct relative path from project root
# Assuming launch.sh is executed from project root or inside raspi/
if [ -f "raspi/src/main.py" ]; then
    MAIN_PATH="raspi/src/main.py"
elif [ -f "src/main.py" ]; then
    MAIN_PATH="src/main.py"
else
    echo "[ERROR] main.py not found."
    exit 1
fi

echo "Starting Robot Autonomous Mission..."
python3 "$MAIN_PATH"

