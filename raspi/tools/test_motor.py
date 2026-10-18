import time
from serial_comm import ESPController

def test_motor_movement():
    print("[TEST] Starting manual movement test...")
    esp = ESPController()
    
    if not esp.connected:
        print("[ERROR] ESP32 not connected. Check port configuration.")
        return

    print("[TEST] Moving forward for 2 seconds...")
    esp.forward(100)
    time.sleep(2)
    
    print("[TEST] Stopping...")
    esp.stop()
    
    print("[TEST] Turning right for 1 second...")
    esp.turn_right(80)
    time.sleep(1)
    
    print("[TEST] Stopping...")
    esp.stop()
    
    esp.close()
    print("[TEST] Test completed.")

if __name__ == "__main__":
    test_motor_movement()
