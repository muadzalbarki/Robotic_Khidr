import serial
import time
from config import SERIAL_PORT, SERIAL_BAUD

def emergency_stop():
    print("[EMERGENCY] Sending STOP command to ESP32...")
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        ser.write(b"0,0,90,30\n")
        ser.close()
        print("[EMERGENCY] STOP command sent successfully.")
    except Exception as e:
        print(f"[ERROR] Could not send stop command: {e}")

if __name__ == "__main__":
    emergency_stop()
