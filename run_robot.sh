#!/bin/bash
# Robot Launch Script: Toggle between GUI/Headless modes and use venv

GUI_MODE=false

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --gui) GUI_MODE=true ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Activate venv
if [ -d "venv" ]; then
    source venv/bin/activate
else
    echo "[ERROR] Virtual environment 'venv' not found. Please create it first."
    exit 1
fi

# Toggle config
CONFIG_FILE="raspi/src/config.py"
if [ "$GUI_MODE" = true ]; then
    echo "Launching in GUI Mode (Visualization Enabled)..."
    python3 -c "
with open('$CONFIG_FILE', 'r') as f:
    content = f.read()
content = content.replace('ENABLE_VISUALIZATION = False', 'ENABLE_VISUALIZATION = True')
with open('$CONFIG_FILE', 'w') as f:
    f.write(content)
"
else
    echo "Launching in Production Mode (Headless)..."
    python3 -c "
with open('$CONFIG_FILE', 'r') as f:
    content = f.read()
content = content.replace('ENABLE_VISUALIZATION = True', 'ENABLE_VISUALIZATION = False')
with open('$CONFIG_FILE', 'w') as f:
    f.write(content)
"
fi

# Run test modules
python3 raspi/src/test_modules.py
