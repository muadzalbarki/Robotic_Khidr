#!/bin/bash
# Robot Launch Script: Main entry point for the robot

GUI_MODE=false
MODE="run"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --gui) GUI_MODE=true ;;
        --test) MODE="test" ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Activate venv
if [ -d "venv" ]; then
    source venv/bin/activate
elif [ -d "../venv" ]; then
    source ../venv/bin/activate
else
    echo "[ERROR] Virtual environment 'venv' not found."
    exit 1
fi

# Configure visualization
CONFIG_FILE="raspi/src/config.py"
if [ "$GUI_MODE" = true ]; then
    echo "Visualization Enabled."
    sed -i 's/ENABLE_VISUALIZATION = False/ENABLE_VISUALIZATION = True/' $CONFIG_FILE
else
    echo "Visualization Disabled (Headless)."
    sed -i 's/ENABLE_VISUALIZATION = True/ENABLE_VISUALIZATION = False/' $CONFIG_FILE
fi

# Execute mode
if [ "$MODE" = "test" ]; then
    echo "Launching Test Modules..."
    python3 raspi/src/test_modules.py
else
    echo "Starting Autonomous Mission..."
    python3 raspi/src/main.py
fi
