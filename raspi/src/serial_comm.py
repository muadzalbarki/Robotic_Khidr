"""
Serial Communication Module untuk ESP32 Controller
Mengirim perintah motor dan servo ke ESP32 via USB Serial

Protocol: <speedLeft>,<speedRight>,<liftAngle>,<gripAngle>\n
Example: 150,-100,90,30\n
"""

import serial
import time
import glob
from config import *

class ESPController:
    def __init__(self, port=SERIAL_PORT, baud=SERIAL_BAUD, timeout=SERIAL_TIMEOUT):
        """
        Initialize serial connection to ESP32
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0')
            baud: Baud rate (default 115200)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self.connected = False
        
        # Current state tracking
        self.current_speed_left = 0
        self.current_speed_right = 0
        self.current_lift = SERVO_LIFT_MID
        self.current_grip = SERVO_GRIP_OPEN
        
        self._connect()
    
    def _connect(self):
        """Establish serial connection with auto-detection fallback"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2)  # Wait for ESP32 to initialize
            
            # Flush buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # Wait for READY signal from ESP32
            start_time = time.time()
            while time.time() - start_time < 5:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if "READY" in line:
                            self.connected = True
                            print(f"[SERIAL] Connected to ESP32 on {self.port}")
                            return
                    except Exception:
                        continue
            
            print("[SERIAL] Warning: No READY signal from ESP32, but connected")
            self.connected = True
            
        except serial.SerialException as e:
            print(f"[SERIAL] Error connecting to {self.port}: {e}")
            print("[SERIAL] Attempting auto-detection...")
            
            # Try to auto-detect port
            detected_port = self._auto_detect_port()
            if detected_port:
                self.port = detected_port
                self._connect()
            else:
                print("[SERIAL] ERROR: Could not connect to ESP32")
                self.connected = False
    
    def _auto_detect_port(self):
        """Auto-detect ESP32 USB port"""
        # Prioritize ttyACM0 for ESP32-S3
        possible_ports = ['/dev/ttyACM0', '/dev/ttyUSB1'] + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        
        for port in possible_ports:
            try:
                test_ser = serial.Serial(port, self.baud, timeout=1)
                time.sleep(1)
                test_ser.close()
                print(f"[SERIAL] Found potential port: {port}")
                return port
            except:
                continue
        
        return None
    
    def send_command(self, speed_left, speed_right, lift=None, grip=None):
        """
        Send command to ESP32
        """
        if not self.connected or self.ser is None:
            print("[SERIAL] ERROR: Not connected")
            return False
        
        # Constrain values and ensure integer format
        speed_left = int(max(-255, min(255, int(speed_left))))
        speed_right = int(max(-255, min(255, int(speed_right))))
        
        # Use current values if not specified
        if lift is None:
            lift = self.current_lift
        else:
            lift = int(max(SERVO_LIFT_DOWN, min(SERVO_LIFT_UP, int(lift))))
            self.current_lift = lift
        
        if grip is None:
            grip = self.current_grip
        else:
            grip = int(max(SERVO_GRIP_OPEN, min(SERVO_GRIP_CLOSE, int(grip))))
            self.current_grip = grip
        
        # Update state
        self.current_speed_left = speed_left
        self.current_speed_right = speed_right
        
        # Build command string (using comma-separated format)
        cmd = f"{speed_left},{speed_right},{lift},{grip}\n"
        
        try:
            self.ser.write(cmd.encode('utf-8'))
            
            # Wait for acknowledgment
            # Wait a short moment to capture response
            time.sleep(0.01)
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if "OK" in response:
                    return True
                elif "ERR" in response:
                    print(f"[SERIAL] ESP32 Error: {response}")
                    return False
            
            return True
            
        except serial.SerialException as e:
            print(f"[SERIAL] Send error: {e}")
            self.connected = False
            return False
    
    def stop(self):
        """Stop all motors immediately"""
        return self.send_command(0, 0)
    
    def forward(self, speed=MAX_SPEED):
        """Move forward"""
        return self.send_command(speed, speed)
    
    def backward(self, speed=MAX_SPEED):
        """Move backward"""
        return self.send_command(-speed, -speed)
    
    def turn_left(self, speed=TURN_SPEED):
        """Turn left (rotate in place)"""
        return self.send_command(-speed, speed)
    
    def turn_right(self, speed=TURN_SPEED):
        """Turn right (rotate in place)"""
        return self.send_command(speed, -speed)
    
    def arc_left(self, speed=MAX_SPEED):
        """Arc turn left (left wheel slower)"""
        return self.send_command(int(speed * 0.3), speed)
    
    def arc_right(self, speed=MAX_SPEED):
        """Arc turn right (right wheel slower)"""
        return self.send_command(speed, int(speed * 0.3))
    
    def set_servo_lift(self, angle):
        """Set lift servo angle"""
        return self.send_command(0, 0, lift=angle, grip=self.current_grip)
    
    def set_servo_grip(self, angle):
        """Set gripper servo angle"""
        return self.send_command(0, 0, lift=self.current_lift, grip=angle)
    
    def grab_bottle(self, verbose=True):
        """
        Execute grab sequence:
        1. Lower lift
        2. Open gripper
        3. Close gripper
        4. Raise lift
        """
        if verbose:
            print("[GRIPPER] Starting grab sequence...")
        
        # Lower lift, gripper open
        self.send_command(0, 0, SERVO_LIFT_DOWN, SERVO_GRIP_OPEN)
        time.sleep(1.0)
        
        # Close gripper
        self.send_command(0, 0, SERVO_LIFT_DOWN, SERVO_GRIP_CLOSE)
        time.sleep(1.0)
        
        # Raise lift with bottle
        self.send_command(0, 0, SERVO_LIFT_UP, SERVO_GRIP_CLOSE)
        time.sleep(1.0)
        
        if verbose:
            print("[GRIPPER] Grab sequence complete")
        
        return True
    
    def release_bottle(self, verbose=True):
        """
        Execute release sequence:
        1. Lower lift
        2. Open gripper
        3. Raise lift (empty)
        """
        if verbose:
            print("[GRIPPER] Starting release sequence...")
        
        # Lower lift with bottle
        self.send_command(0, 0, SERVO_LIFT_DOWN, SERVO_GRIP_CLOSE)
        time.sleep(1.0)
        
        # Open gripper
        self.send_command(0, 0, SERVO_LIFT_DOWN, SERVO_GRIP_OPEN)
        time.sleep(0.5)
        
        # Raise lift (empty)
        self.send_command(0, 0, SERVO_LIFT_MID, SERVO_GRIP_OPEN)
        time.sleep(0.5)
        
        if verbose:
            print("[GRIPPER] Release sequence complete")
        
        return True
    
    def read_response(self):
        """Read any available response from ESP32"""
        if self.ser and self.ser.in_waiting > 0:
            try:
                return self.ser.readline().decode('utf-8').strip()
            except:
                return None
        return None
    
    def close(self):
        """Close serial connection"""
        if self.ser:
            self.stop()
            time.sleep(0.1)
            self.ser.close()
            self.connected = False
            print("[SERIAL] Connection closed")
    
    def __del__(self):
        """Cleanup on object destruction"""
        self.close()


# Test function
if __name__ == "__main__":
    print("=== ESP32 Serial Communication Test ===\n")
    
    try:
        esp = ESPController()
        
        if not esp.connected:
            print("Failed to connect to ESP32")
            exit(1)
        
        print("\nTest 1: Forward")
        esp.forward(100)
        time.sleep(2)
        
        print("Test 2: Stop")
        esp.stop()
        time.sleep(1)
        
        print("Test 3: Turn Right")
        esp.turn_right(80)
        time.sleep(1)
        
        print("Test 4: Stop")
        esp.stop()
        time.sleep(1)
        
        print("Test 5: Backward")
        esp.backward(100)
        time.sleep(2)
        
        print("Test 6: Stop")
        esp.stop()
        time.sleep(1)
        
        print("\nTest 7: Servo Test")
        esp.set_servo_lift(SERVO_LIFT_DOWN)
        time.sleep(1)
        esp.set_servo_lift(SERVO_LIFT_UP)
        time.sleep(1)
        esp.set_servo_lift(SERVO_LIFT_MID)
        
        print("\nTest 8: Gripper Test")
        esp.set_servo_grip(SERVO_GRIP_OPEN)
        time.sleep(1)
        esp.set_servo_grip(SERVO_GRIP_CLOSE)
        time.sleep(1)
        esp.set_servo_grip(SERVO_GRIP_OPEN)
        
        print("\nTest 9: Grab Sequence")
        esp.grab_bottle()
        
        print("\nTest 10: Release Sequence")
        esp.release_bottle()
        
        print("\n=== All tests complete ===")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    finally:
        if 'esp' in locals():
            esp.close()
