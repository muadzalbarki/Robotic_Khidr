"""
LiDAR Navigation Module untuk YDLIDAR X2 360° Laser Scanner
Menangani obstacle detection, avoidance, wall following, dan odometry (scan matching)
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
        
        # Odometry state
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}  # x, y in meters, theta in radians
        self.prev_scan = None
        self.prev_scan_time = time.time()
        self.prev_points = None
        
        # Wall following state
        self.wall_follow_side = 'right'  # 'left' or 'right'
        self.prev_wall_error = 0.0
        
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
    
    def _polar_to_cartesian(self, points):
        """Convert polar scan points to cartesian coordinates"""
        points_xy = []
        for angle_deg, dist_cm in points:
            if dist_cm <= 0 or dist_cm > 5000:  # Filter invalid readings
                continue
            angle_rad = math.radians(angle_deg)
            x = dist_cm * math.cos(math.radians(angle_deg))
            y = dist_cm * math.sin(math.radians(angle_deg))
            points_xy.append((x, y, angle_deg))
        return points_xy
    
    def _polar_to_cartesian(self, points):
        """Convert polar points to cartesian numpy array"""
        pts = []
        for angle_deg, dist_cm in points:
            if dist_cm <= 0 or dist_cm > 5000:
                continue
            angle_rad = math.radians(angle_deg)
            x = dist_cm * math.cos(math.radians(angle_deg))
            y = dist_cm * math.sin(math.radians(angle_deg))
            pts.append([x, y])
        return np.array(pts) if pts else np.empty((0, 2))
    
    def _icp_step(self, prev_points, curr_points):
        """
        Simple ICP (Iterative Closest Point) for scan matching
        Returns: dx, dy, dtheta (in meters, radians)
        """
        if len(prev_points) < 10 or len(curr_points) < 10:
            return 0.0, 0.0, 0.0
        
        prev_pts = np.array([[p[0], p[1]] for p in prev_points])
        curr_pts = np.array([[p[0], p[1]] for p in curr_points])
        
        if len(prev_pts) < 10 or len(curr_pts) < 10:
            return 0.0, 0.0, 0.0
        
        # Simple ICP - find closest points and estimate transform
        # Use centroid-based alignment as initial guess
        prev_centroid = np.mean(prev_pts, axis=0)
        curr_centroid = np.mean(curr_pts, axis=0)
        
        # Translate to centroids
        prev_centered = prev_pts - prev_centroid
        curr_centered = curr_pts - curr_centroid
        
        # SVD for rotation
        H = curr_centered.T @ prev_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Ensure proper rotation (no reflection)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Extract translation and rotation
        dx = curr_centroid[0] - prev_centroid[0]
        dy = curr_centroid[1] - prev_centroid[1]
        
        # Extract angle from rotation matrix
        dtheta = math.atan2(R[1, 0], R[0, 0])
        
        # Convert cm to meters
        dx_m = dx / 100.0
        dy_m = dy / 100.0
        
        return dx_m, dy_m, dtheta

    def _polar_to_cartesian(self, points):
        """Convert polar scan points to cartesian coordinates"""
        points_xy = []
        for angle_deg, dist_cm in points:
            if dist_cm <= 0 or dist_cm > 5000:  # Filter invalid readings
                continue
            angle_rad = math.radians(angle_deg)
            x = dist_cm * math.cos(math.radians(angle_deg))
            y = dist_cm * math.sin(math.radians(angle_deg))
            points_xy.append((x, y, angle_deg))
        return points_xy
    
    def _polar_to_cartesian(self, points):
        """Convert polar points to cartesian numpy array"""
        pts = []
        for angle_deg, dist_cm in points:
            if dist_cm <= 0 or dist_cm > 5000:
                continue
            angle_rad = math.radians(angle_deg)
            x = dist_cm * math.cos(math.radians(angle_deg))
            y = dist_cm * math.sin(math.radians(angle_deg))
            pts.append([x, y])
        return np.array(pts) if pts else np.empty((0, 2))
    
    def _icp_step(self, prev_points, curr_points):
        """Single ICP iteration - returns dx, dy, dtheta"""
        if len(prev_points) < 10 or len(curr_points) < 10:
            return 0.0, 0.0, 0.0
        
        prev_pts = self._polar_to_cartesian(prev_points)
        curr_pts = self._polar_to_cartesian(curr_points)
        
        if len(prev_pts) < 10 or len(curr_pts) < 10:
            return 0.0, 0.0, 0.0
        
        prev_pts = np.array(prev_pts)
        curr_pts = np.array(curr_pts)
        
        # Find closest points (simple nearest neighbor)
        from scipy.spatial import KDTree
        try:
            tree = KDTree(prev_pts)
            distances, indices = tree.query(curr_pts, k=1)
            
            # Filter outliers
            mask = distances < 100  # 100cm threshold
            if np.sum(mask) < 10:
                return 0.0, 0.0, 0.0
            
            src = curr_pts[mask]
            dst = prev_pts[indices[mask]]
            
            # Compute transform using SVD
            src_centroid = np.mean(src, axis=0)
            dst_centroid = np.mean(dst, axis=0)
            
            src_centered = src - src_centroid
            dst_centered = dst - dst_centroid
            
            H = src_centered.T @ dst_centered
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            dx = dst_centroid[0] - src_centroid[0]
            dy = dst_centroid[1] - src_centroid[1]
            dtheta = math.atan2(R[1, 0], R[0, 0])
            
            return dx / 100.0, dy / 100.0, dtheta
        except:
            return 0.0, 0.0, 0.0

    def _update_odometry(self, points):
        """Update pose using scan matching"""
        if self.prev_points is not None and len(points) > 20:
            dx, dy, dtheta = self._icp_step(self.prev_points, points)
            
            # Update pose
            self.pose['x'] += dx * math.cos(self.pose['theta']) - dy * math.sin(self.pose['theta'])
            self.pose['y'] += dx * math.sin(self.pose['theta']) + dy * math.cos(self.pose['theta'])
            self.pose['theta'] = (self.pose['theta'] + dtheta) % (2 * math.pi)
            
        self.prev_points = points
        self.prev_scan_time = time.time()
    
    def _get_wall_distance(self, side='right'):
        """Get average distance to wall on specified side"""
        sector = 'front_right' if side == 'right' else 'front_left'
        sectors = self.get_sector_distances()
        return sectors.get(sector, 999)

    def _wall_follow_control(self, target_dist_cm=20):
        """Wall following PID control"""
        # Use front_right for right wall following
        right_dist = self.get_sector_distances().get('front_right', 999)
        front_dist = self.get_sector_distances().get('front', 999)
        
        # Safety: stop if front obstacle too close
        if self.get_sector_distances().get('front', 999) < 30:
            return 0, 0
        
        # Wall following PID
        target_dist = LIDAR_WALL_FOLLOW_DIST / 10.0  # cm
        current_dist = self._get_wall_distance('right')
        
        error = (target_dist - current_dist) / 10.0  # Convert mm to cm for error
        
        # PID constants
        Kp = 2.0
        Kd = 0.5
        
        derivative = error - self.prev_wall_error
        correction = Kp * error + Kd * (error - self.prev_wall_error)
        self.prev_wall_error = error
        
        # Limit correction
        correction = max(-80, min(80, correction))
        
        base_speed = 80
        left_speed = 100 - correction
        right_speed = 100 + correction
        
        # Clamp speeds
        left_speed = max(0, min(255, left_speed))
        right_speed = max(0, min(255, right_speed))
        
        return left_speed, right_speed

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
                return []
            
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
        """Update scan data from LiDAR and update odometry"""
        if not self.connected or self.ser is None:
            return False
        
        points = self._parse_scan_packet()
        if points:
            for angle, distance in points:
                self.scan_data[int(angle)] = distance
            self.last_scan_time = time.time()
            
            # Update odometry using scan matching
            self._update_odometry(points)
            return True
        return False
    
    def _update_odometry(self, points):
        """Update pose using scan matching"""
        if self.prev_points is not None and len(points) > 20:
            dx, dy, dtheta = self._icp_step(self.prev_points, points)
            
            # Update pose
            self.pose['x'] += dx * math.cos(self.pose['theta']) - dy * math.sin(self.pose['theta'])
            self.pose['y'] += dx * math.sin(self.pose['theta']) + dy * math.cos(self.pose['theta'])
            self.pose['theta'] = (self.pose['theta'] + dtheta) % (2 * math.pi)
            
        self.prev_points = points
        self.prev_scan_time = time.time()
    
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
    
    def get_motor_speeds(self):
        """Get motor speeds for wall following with obstacle avoidance"""
        sectors = self.get_sector_distances()
        front_dist = sectors.get('front', 999)
        front_left = sectors.get('front_left', 999)
        front_right = sectors.get('front_right', 999)
        
        # Emergency stop
        if front_dist < 30:
            return 0, 0
        
        # Wall following
        left_speed, right_speed = self._wall_follow_control()
        
        # Slow down if obstacle ahead
        if front_dist < 70:
            return int(left_speed * 0.5), int(right_speed * 0.5)
        
        return left_speed, right_speed

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
    
    def get_motor_speeds(self):
        """Get motor speeds for wall following with obstacle avoidance"""
        sectors = self.get_sector_distances()
        front_dist = sectors.get('front', 999)
        front_left = sectors.get('front_left', 999)
        front_right = sectors.get('front_right', 999)
        
        # Emergency stop
        if front_dist < 30:
            return 0, 0
        
        # Wall following
        left_speed, right_speed = self._wall_follow_control()
        
        # Slow down if obstacle ahead
        if front_dist < 70:
            return int(left_speed * 0.5), int(right_speed * 0.5)
        
        return left_speed, right_speed

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
    
    def get_motor_speeds(self):
        """Get motor speeds for wall following with obstacle avoidance"""
        sectors = self.get_sector_distances()
        front_dist = sectors.get('front', 999)
        front_left = sectors.get('front_left', 999)
        front_right = sectors.get('front_right', 999)
        
        # Emergency stop
        if front_dist < 30:
            return 0, 0
        
        # Wall following
        left_speed, right_speed = self._wall_follow_control()
        
        # Slow down if obstacle ahead
        if front_dist < 70:
            return int(left_speed * 0.5), int(right_speed * 0.5)
        
        return left_speed, right_speed

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