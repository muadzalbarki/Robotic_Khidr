#!/bin/bash
# Robot Launch Script: Main entry point for the autonomous robot

# Activate venv
if [ -d "venv" ]; then
    source venv/bin/activate
else
    # Fallback to project root if venv is one level up
    if [ -d "../venv" ]; then
        source ../venv/bin/activate
    else
        echo "[ERROR] Virtual environment 'venv' not found."
        exit 1
    fi
fi

# Launch main application
echo "Starting Robot Autonomous Mission..."
python3 raspi/src/main.py
