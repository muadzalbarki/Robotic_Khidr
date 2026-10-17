"""
LiDAR Navigation Module untuk YDLIDAR YDS2MD-1
Menangani obstacle detection dan avoidance menggunakan 360° LiDAR scan

YDLIDAR YDS2MD-1 Specifications:
- Range: 0.12m - 8m
- Scan frequency: 5Hz
- Angular resolution: ~1°
- Interface: UART (115200 baud default)
"""

import struct
import time
import math
import numpy as np
from config import *


class LidarNavigator:
    def __init__(self, port=LIDAR_PORT, baud=LIDAR_BAUD):
        """
        Initialize YDLIDAR connection
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB1')
            baud: Baud rate (default 115200)
        """
        self.port = port
        self.baud = baud
        self.ser = None
        self.connected = False
        
        # Scan data storage
        self.scan_data = {}  # {angle: distance_cm}
        self.last_scan_time = 0
        
        self._connect()
    
    def _connect(self):
        """Establish serial connection to LiDAR"""
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=LIDAR_TIMEOUT,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            time.sleep(0.5)
            self.connected = True
            print(f"[LIDAR] Connected to YDLIDAR on {self.port}")
            
            # Start scanning
            self._start_scan()
            
        except serial.SerialException as e:
            print(f"[LIDAR] ERROR: Failed to connect to {self.port}: {e}")
            self.connected = False
    
    def _start_scan(self):
        """Send command to start scanning"""
        try:
            # YDLIDAR protocol: Start scan command (device-specific)
            # For many YDLIDAR models: 0xA5 0x60
            start_cmd = bytes([0xA5, 0x60])
            self.ser.write(start_cmd)
            time.sleep(0.1)
            print("[LIDAR] Scan started")
        except Exception as e:
            print(f"[LIDAR] Warning: Could not send start command: {e}")
    
    def _parse_scan_packet(self):
        """
        Parse LiDAR scan data packet
        
        YDLIDAR Protocol (simplified for YDS2MD-1):
        Each scan point contains:
        - Distance (mm)
        - Angle (0.01 degree units)
        - Quality/Intensity
        
        Returns:
            List of (angle_deg, distance_cm) tuples
        """
        points = []
        
        try:
            # Read available data
            if self.ser.in_waiting < 10:
                return points
            
            # Simple parsing approach for YDLIDAR
            # Note: Actual protocol may vary, this is a generic implementation
            # For production, consider using official ydlidar Python SDK
            
            buffer = self.ser.read(self.ser.in_waiting)
            
            # Look for packet header (0x55 0xAA is common for some YDLIDAR models)
            # This is simplified - actual parsing depends on specific protocol
            
            # For now, implementing basic distance extraction
            # User may need to adjust based on actual YDS2MD-1 protocol
            
            i = 0
            while i < len(buffer) - 5:
                # Check for sync bytes
                if buffer[i] == 0x55 and buffer[i+1] == 0xAA:
                    try:
                        # Extract angle and distance (format varies by model)
                        angle_raw = struct.unpack('<H', buffer[i+2:i+4])[0]
                        distance_raw = struct.unpack('<H', buffer[i+4:i+6])[0]
                        
                        angle = (angle_raw / 100.0) % 360  # Convert to degrees
                        distance = distance_raw / 10.0      # Convert to cm
                        
                        if 0 <= angle < 360 and 12 <= distance <= 800:  # Valid range
                            points.append((angle, distance))
                        
                        i += 6
                    except:
                        i += 1
                else:
                    i += 1
            
        except Exception as e:
            print(f"[LIDAR] Parse error: {e}")
        
        return points
    
    def update_scan(self):
        """
        Update scan data from LiDAR
        Should be called regularly (e.g., 10Hz)
        """
        if not self.connected or self.ser is None:
            return False
        
        points = self._parse_scan_packet()
        
        if points:
            # Update scan data dictionary
            for angle, distance in points:
                self.scan_data[int(angle)] = distance
            
            self.last_scan_time = time.time()
            return True
        
        return False
    
    def get_sector_distances(self):
        """
        Get minimum distance in each sector
        
        Returns:
            Dict with sector names and minimum distances
        """
        if not self.scan_data:
            return {sector: 999 for sector in LIDAR_SECTORS.keys()}
        
        sector_distances = {}
        
        for sector_name, (start_angle, end_angle) in LIDAR_SECTORS.items():
            distances = []
            
            # Handle angle wraparound (e.g., 330° to 30°)
            if start_angle > end_angle:
                # Wraparound case (e.g., 330-360 and 0-30)
                for angle in range(start_angle, 360):
                    if angle in self.scan_data:
                        distances.append(self.scan_data[angle])
                for angle in range(0, end_angle + 1):
                    if angle in self.scan_data:
                        distances.append(self.scan_data[angle])
            else:
                # Normal case
                for angle in range(start_angle, end_angle + 1):
                    if angle in self.scan_data:
                        distances.append(self.scan_data[angle])
            
            # Get minimum distance in sector (closest obstacle)
            if distances:
                sector_distances[sector_name] = min(distances)
            else:
                sector_distances[sector_name] = 999  # No data = assume clear
        
        return sector_distances
    
    def check_obstacle(self, sector='front', threshold=LIDAR_WARNING_THRESHOLD):
        """
        Check if there's an obstacle in specific sector
        
        Args:
            sector: Sector name from LIDAR_SECTORS
            threshold: Distance threshold in cm
        
        Returns:
            (has_obstacle, distance)
        """
        sector_distances = self.get_sector_distances()
        
        if sector not in sector_distances:
            return (False, 999)
        
        distance = sector_distances[sector]
        has_obstacle = distance < threshold
        
        return (has_obstacle, distance)
    
    def get_safe_direction(self):
        """
        Analyze all sectors and recommend safe direction
        
        Returns:
            Direction string: 'forward', 'left', 'right', 'backward', 'stop'
        """
        sectors = self.get_sector_distances()
        
        front = sectors.get('front', 999)
        front_left = sectors.get('front_left', 999)
        front_right = sectors.get('front_right', 999)
        left = sectors.get('left', 999)
        right = sectors.get('right', 999)
        
        # Critical obstacle directly ahead
        if front < LIDAR_CRITICAL_THRESHOLD:
            # Emergency: choose best escape direction
            if left > right:
                return 'left'
            else:
                return 'right'
        
        # Warning zone ahead
        if front < LIDAR_WARNING_THRESHOLD:
            # Obstacle ahead, turn to clearer side
            if front_left > front_right and left > LIDAR_SAFE_DISTANCE:
                return 'left'
            elif front_right > front_left and right > LIDAR_SAFE_DISTANCE:
                return 'right'
            elif left > right:
                return 'left'
            else:
                return 'right'
        
        # Close on left side
        if left < LIDAR_CRITICAL_THRESHOLD:
            return 'right'
        
        # Close on right side
        if right < LIDAR_CRITICAL_THRESHOLD:
            return 'left'
        
        # Path is clear
        return 'forward'
    
    def is_path_clear(self, min_distance=LIDAR_WARNING_THRESHOLD):
        """
        Check if forward path is clear
        
        Returns:
            Boolean
        """
        has_obstacle, distance = self.check_obstacle('front', min_distance)
        return not has_obstacle
    
    def get_closest_obstacle(self):
        """
        Find closest obstacle in any direction
        
        Returns:
            (angle, distance) or (None, 999) if no obstacles
        """
        if not self.scan_data:
            return (None, 999)
        
        min_distance = 999
        min_angle = None
        
        for angle, distance in self.scan_data.items():
            if distance < min_distance:
                min_distance = distance
                min_angle = angle
        
        return (min_angle, min_distance)
    
    def get_scan_data_raw(self):
        """Get raw scan data dictionary"""
        return self.scan_data.copy()
    
    def clear_old_data(self, max_age=2.0):
        """Clear scan data older than max_age seconds"""
        if time.time() - self.last_scan_time > max_age:
            self.scan_data.clear()
            print("[LIDAR] Cleared stale scan data")
    
    def close(self):
        """Stop scanning and close connection"""
        if self.ser:
            try:
                # Stop scan command (0xA5 0x65 for many YDLIDAR)
                stop_cmd = bytes([0xA5, 0x65])
                self.ser.write(stop_cmd)
                time.sleep(0.1)
            except:
                pass
            
            self.ser.close()
            self.connected = False
            print("[LIDAR] Connection closed")
    
    def __del__(self):
        """Cleanup"""
        self.close()


# Simplified LiDAR simulator for testing without hardware
class LidarSimulator:
    """
    Simulated LiDAR for testing when hardware is not available
    Returns random but plausible scan data
    """
    def __init__(self):
        self.connected = True
        self.scan_data = {}
        print("[LIDAR] Using SIMULATOR mode (no hardware)")
    
    def update_scan(self):
        """Generate simulated scan data"""
        # Simulate clear path with occasional obstacles
        self.scan_data.clear()
        
        for angle in range(0, 360, 5):
            # Most angles show clear path (100-300cm)
            base_distance = np.random.uniform(100, 300)
            
            # Simulate occasional obstacles in certain sectors
            if 340 <= angle or angle <= 20:  # Front
                if np.random.random() < 0.1:  # 10% chance of obstacle
                    base_distance = np.random.uniform(30, 80)
            
            self.scan_data[angle] = base_distance
        
        return True
    
    def get_sector_distances(self):
        """Return simulated sector distances"""
        return {
            'front': np.random.uniform(80, 200),
            'front_left': np.random.uniform(100, 250),
            'front_right': np.random.uniform(100, 250),
            'left': np.random.uniform(120, 300),
            'right': np.random.uniform(120, 300),
            'back': np.random.uniform(150, 300)
        }
    
    def check_obstacle(self, sector='front', threshold=50):
        sectors = self.get_sector_distances()
        distance = sectors.get(sector, 999)
        return (distance < threshold, distance)
    
    def get_safe_direction(self):
        return 'forward'
    
    def is_path_clear(self, min_distance=50):
        return True
    
    def get_closest_obstacle(self):
        return (0, 150)
    
    def get_scan_data_raw(self):
        return self.scan_data.copy()
    
    def clear_old_data(self, max_age=2.0):
        pass
    
    def close(self):
        print("[LIDAR] Simulator closed")


# Factory function
def create_lidar(use_simulator=False):
    """
    Create LiDAR instance (real or simulator)
    
    Args:
        use_simulator: If True, use simulator instead of hardware
    
    Returns:
        LidarNavigator or LidarSimulator instance
    """
    if use_simulator:
        return LidarSimulator()
    else:
        return LidarNavigator()


# Test function
if __name__ == "__main__":
    print("=== LiDAR Navigation Test ===\n")
    print("Use simulator? (y/n): ", end='')
    
    use_sim = input().lower().startswith('y')
    
    lidar = create_lidar(use_simulator=use_sim)
    
    if not lidar.connected:
        print("Failed to connect to LiDAR")
        exit(1)
    
    try:
        while True:
            lidar.update_scan()
            
            sectors = lidar.get_sector_distances()
            
            print("\n--- Sector Distances (cm) ---")
            for sector, distance in sectors.items():
                status = "CLEAR" if distance > LIDAR_WARNING_THRESHOLD else "OBSTACLE"
                print(f"{sector:12s}: {distance:6.1f} cm  [{status}]")
            
            direction = lidar.get_safe_direction()
            print(f"\nRecommended direction: {direction}")
            
            closest_angle, closest_dist = lidar.get_closest_obstacle()
            print(f"Closest obstacle: {closest_dist:.1f}cm at {closest_angle}°")
            
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    
    finally:
        lidar.close()
