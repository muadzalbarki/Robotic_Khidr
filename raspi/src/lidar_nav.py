"""
LiDAR Navigation Module untuk YDLIDAR X2 360° Laser Scanner
Menangani obstacle detection dan avoidance menggunakan 360° LiDAR scan
"""

import struct
import time
import math
import numpy as np
import serial
from config import *


class LidarNavigator:
    def __init__(self, port=LIDAR_PORT, baud=LIDAR_BAUD):
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
            print(f"[LIDAR] Connected to YDLIDAR X2 on {self.port}")
            
            # Start scanning command (A5 60)
            self._start_scan()
            
        except serial.SerialException as e:
            print(f"[LIDAR] ERROR: Failed to connect to {self.port}: {e}")
            self.connected = False
    
    def _start_scan(self):
        """Send command to start scanning"""
        try:
            start_cmd = bytes([0xA5, 0x60])
            self.ser.write(start_cmd)
            time.sleep(0.1)
            print("[LIDAR] Scan started")
        except Exception as e:
            print(f"[LIDAR] Warning: Could not send start command: {e}")
    
    def _parse_scan_packet(self):
        """
        Parse YDLIDAR X-series packet protocol:
        Header: 0xAA 0x55
        CT: 1 byte
        LSN: 1 byte (Sample Count)
        FSA: 2 bytes (First Angle)
        LSA: 2 bytes (Last Angle)
        CS: 2 bytes (Checksum)
        Samples: LSN * 2 bytes
        """
        points = []
        try:
            if self.ser.in_waiting < 40:
                return points
            
            buffer = self.ser.read(self.ser.in_waiting)
            i = 0
            while i < len(buffer) - 10:
                # Look for header AA 55
                if buffer[i] == 0xAA and buffer[i+1] == 0x55:
                    try:
                        ct = buffer[i+2]
                        lsn = buffer[i+3]
                        if lsn == 0:
                            i += 2
                            continue
                            
                        fsa = struct.unpack('<H', buffer[i+4:i+6])[0]
                        lsa = struct.unpack('<H', buffer[i+6:i+8])[0]
                        
                        # Angles are in format where bit 0 is fixed flag, angle in 0.01 degree
                        fsa_angle = (fsa >> 1) / 64.0
                        lsa_angle = (lsa >> 1) / 64.0
                        
                        packet_len = 10 + lsn * 2
                        if i + packet_len > len(buffer):
                            break
                            
                        samples_data = buffer[i+10:i+packet_len]
                        
                        # Interpolate angles for each sample
                        diff_angle = lsa_angle - fsa_angle
                        if diff_angle < 0:
                            diff_angle += 360.0
                            
                        for s in range(lsn):
                            dist_raw = struct.unpack('<H', samples_data[s*2:s*2+2])[0]
                            distance = (dist_raw >> 2) / 4.0 / 10.0  # Convert to cm
                            
                            if distance > 0:
                                sample_angle = fsa_angle + (diff_angle / max(1, lsn - 1)) * s
                                sample_angle = sample_angle % 360.0
                                points.append((sample_angle, distance))
                                
                        i += packet_len
                    except Exception:
                        i += 1
                else:
                    i += 1
        except Exception as e:
            print(f"[LIDAR] Parse error: {e}")
            
        return points
    
    def update_scan(self):
        """Update scan data from LiDAR"""
        if not self.connected or self.ser is None:
            return False
        
        points = self._parse_scan_packet()
        if points:
            for angle, distance in points:
                self.scan_data[int(angle)] = distance
            self.last_scan_time = time.time()
            return True
        return False
    
    def get_sector_distances(self):
        """Get minimum distance in each sector"""
        if not self.scan_data:
            return {sector: 999 for sector in LIDAR_SECTORS.keys()}
        
        sector_distances = {}
        for sector_name, (start_angle, end_angle) in LIDAR_SECTORS.items():
            distances = []
            if start_angle > end_angle:
                for angle in range(start_angle, 360):
                    if angle in self.scan_data:
                        distances.append(self.scan_data[angle])
                for angle in range(0, end_angle + 1):
                    if angle in self.scan_data:
                        distances.append(self.scan_data[angle])
            else:
                for angle in range(start_angle, end_angle + 1):
                    if angle in self.scan_data:
                        distances.append(self.scan_data[angle])
            
            if distances:
                sector_distances[sector_name] = min(distances)
            else:
                sector_distances[sector_name] = 999
        return sector_distances
    
    def check_obstacle(self, sector='front', threshold=LIDAR_WARNING_THRESHOLD):
        sector_distances = self.get_sector_distances()
        if sector not in sector_distances:
            return (False, 999)
        distance = sector_distances[sector]
        return (distance < threshold, distance)
    
    def get_safe_direction(self):
        sectors = self.get_sector_distances()
        front = sectors.get('front', 999)
        left = sectors.get('left', 999)
        right = sectors.get('right', 999)
        
        if front < LIDAR_CRITICAL_THRESHOLD:
            return 'left' if left > right else 'right'
        if front < LIDAR_WARNING_THRESHOLD:
            return 'left' if left > right else 'right'
        return 'forward'
    
    def get_closest_obstacle(self):
        if not self.scan_data:
            return (None, 999)
        min_distance = 999
        min_angle = None
        for angle, distance in self.scan_data.items():
            if distance < min_distance:
                min_distance = distance
                min_angle = angle
        return (min_angle, min_distance)
    
    def close(self):
        if self.ser:
            try:
                stop_cmd = bytes([0xA5, 0x65])
                self.ser.write(stop_cmd)
                time.sleep(0.1)
            except:
                pass
            self.ser.close()
            self.connected = False
            print("[LIDAR] Connection closed")
    
    def __del__(self):
        self.close()


def create_lidar():
    return LidarNavigator()


if __name__ == "__main__":
    print("=== LiDAR X2 Navigation Test ===\n")
    lidar = create_lidar()
    
    if not lidar.connected:
        print("Sambungkan lidar, lidar belum terhubung")
        exit(1)
    
    try:
        while True:
            lidar.update_scan()
            sectors = lidar.get_sector_distances()
            
            print("\n--- Sector Distances (cm) ---")
            for sector, distance in sectors.items():
                status = "CLEAR" if distance > LIDAR_WARNING_THRESHOLD else "OBSTACLE"
                print(f"{sector:12s}: {distance:6.1f} cm  [{status}]")
            
            closest_angle, closest_dist = lidar.get_closest_obstacle()
            print(f"Closest obstacle: {closest_dist:.1f}cm at {closest_angle}°")
            
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    finally:
        lidar.close()
