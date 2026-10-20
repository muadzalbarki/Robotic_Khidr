"""
Arena Mapper Module untuk Robot Otonom
Mengelola mapping arena, landmark tracking, dan pose estimation
"""

import json
import math
import time
import os
from config import *

class ArenaMap:
    def __init__(self):
        self.landmarks = {}  # color -> {'x':, 'y':, 'theta':, 'timestamp':}
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.waypoints = []  # untuk path planning
        self.home_base = None
        self.water_station = None
        self.watering_zones = {}  # color -> {'x':, 'y':, 'theta':}
        
        self.map_file = os.path.join(os.path.dirname(__file__), "..", "data", "arena_map.json")
        self.load_map()
    
    def update_pose(self, dx, dy, dtheta):
        """Update pose dari LiDAR odometry (dalam meter dan radian)"""
        self.robot_pose['x'] += dx * math.cos(self.robot_pose['theta']) - dy * math.sin(self.robot_pose['theta'])
        self.robot_pose['y'] += dx * math.sin(self.robot_pose['theta']) + dy * math.cos(self.robot_pose['theta'])
        self.robot_pose['theta'] = (self.robot_pose['theta'] + dtheta) % (2 * math.pi)
    
    def correct_pose_from_landmark(self, color, detected_bearing, detected_distance):
        """Koreksi pose absolut saat landmark terlihat"""
        if color in self.landmarks:
            landmark = self.landmarks[color]
            # Hitung pose absolut dari landmark
            abs_x = landmark['x'] + detected_distance * math.cos(landmark['theta'] + detected_bearing)
            abs_y = landmark['y'] + detected_distance * math.sin(landmark['theta'] + detected_bearing)
            # Koreksi pose (fusion sederhana: 70% odometry, 30% landmark)
            self.robot_pose['x'] = 0.7 * self.robot_pose['x'] + 0.3 * abs_x
            self.robot_pose['y'] = 0.7 * self.robot_pose['y'] + 0.3 * abs_y
            # Update theta jika landmark punya orientasi
            if 'theta' in landmark:
                self.robot_pose['theta'] = 0.7 * self.robot_pose['theta'] + 0.3 * landmark['theta']
    
    def save_landmark(self, color, bearing, distance):
        """Simpan landmark saat pertama kali terdeteksi"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.landmarks[color] = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_zone(self, color, bearing, distance):
        """Simpan zona watering zone"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.watering_zones[color] = {
            'x': x, 
            'y': y, 
            'theta': 0.0,
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_station(self, bearing, distance):
        """Simpan water station position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.water_station = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_home_base(self, bearing, distance):
        """Simpan home base position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.home_base = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base
    
    def update_pose(self, dx, dy, dtheta):
        """Update pose dari LiDAR odometry (dalam meter dan radian)"""
        self.robot_pose['x'] += dx * math.cos(self.robot_pose['theta']) - dy * math.sin(self.robot_pose['theta'])
        self.robot_pose['y'] += dx * math.sin(self.robot_pose['theta']) + dy * math.cos(self.robot_pose['theta'])
        self.robot_pose['theta'] = (self.robot_pose['theta'] + dtheta) % (2 * math.pi)
    
    def correct_pose_from_landmark(self, color, detected_bearing, detected_distance):
        """Koreksi pose absolut saat landmark terlihat"""
        if color in self.landmarks:
            landmark = self.landmarks[color]
            # Hitung pose absolut dari landmark
            abs_x = landmark['x'] + detected_distance * math.cos(landmark['theta'] + detected_bearing)
            abs_y = landmark['y'] + detected_distance * math.sin(landmark['theta'] + detected_bearing)
            # Koreksi pose (fusion sederhana: 70% odometry, 30% landmark)
            self.robot_pose['x'] = 0.7 * self.robot_pose['x'] + 0.3 * abs_x
            self.robot_pose['y'] = 0.7 * self.robot_pose['y'] + 0.3 * abs_y
            # Update theta jika landmark punya orientasi
            if 'theta' in landmark:
                self.robot_pose['theta'] = 0.7 * self.robot_pose['theta'] + 0.3 * landmark['theta']
    
    def save_landmark(self, color, bearing, distance):
        """Simpan landmark saat pertama kali terdeteksi"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.landmarks[color] = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_zone(self, color, bearing, distance):
        """Simpan zona watering zone"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.watering_zones[color] = {
            'x': x, 
            'y': y, 
            'theta': 0.0,
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_station(self, bearing, distance):
        """Simpan water station position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.water_station = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_home_base(self, bearing, distance):
        """Simpan home base position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.home_base = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base
    
    def update_pose(self, dx, dy, dtheta):
        """Update pose dari LiDAR odometry (dalam meter dan radian)"""
        self.robot_pose['x'] += dx * math.cos(self.robot_pose['theta']) - dy * math.sin(self.robot_pose['theta'])
        self.robot_pose['y'] += dx * math.sin(self.robot_pose['theta']) + dy * math.cos(self.robot_pose['theta'])
        self.robot_pose['theta'] = (self.robot_pose['theta'] + dtheta) % (2 * math.pi)
    
    def correct_pose_from_landmark(self, color, detected_bearing, detected_distance):
        """Koreksi pose absolut saat landmark terlihat"""
        if color in self.landmarks:
            landmark = self.landmarks[color]
            # Hitung pose absolut dari landmark
            abs_x = landmark['x'] + detected_distance * math.cos(landmark['theta'] + detected_bearing)
            abs_y = landmark['y'] + detected_distance * math.sin(landmark['theta'] + detected_bearing)
            # Koreksi pose (fusion sederhana: 70% odometry, 30% landmark)
            self.robot_pose['x'] = 0.7 * self.robot_pose['x'] + 0.3 * abs_x
            self.robot_pose['y'] = 0.7 * self.robot_pose['y'] + 0.3 * abs_y
            # Update theta jika landmark punya orientasi
            if 'theta' in landmark:
                self.robot_pose['theta'] = 0.7 * self.robot_pose['theta'] + 0.3 * landmark['theta']
    
    def save_landmark(self, color, bearing, distance):
        """Simpan landmark saat pertama kali terdeteksi"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.landmarks[color] = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_zone(self, color, bearing, distance):
        """Simpan zona watering zone"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.watering_zones[color] = {
            'x': x, 
            'y': y, 
            'theta': 0.0,
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_station(self, bearing, distance):
        """Simpan water station position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.water_station = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_home_base(self, bearing, distance):
        """Simpan home base position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.home_base = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base
    
    def update_pose(self, dx, dy, dtheta):
        """Update pose dari LiDAR odometry (dalam meter dan radian)"""
        self.robot_pose['x'] += dx * math.cos(self.robot_pose['theta']) - dy * math.sin(self.robot_pose['theta'])
        self.robot_pose['y'] += dx * math.sin(self.robot_pose['theta']) + dy * math.cos(self.robot_pose['theta'])
        self.robot_pose['theta'] = (self.robot_pose['theta'] + dtheta) % (2 * math.pi)
    
    def correct_pose_from_landmark(self, color, detected_bearing, detected_distance):
        """Koreksi pose absolut saat landmark terlihat"""
        if color in self.landmarks:
            landmark = self.landmarks[color]
            # Hitung pose absolut dari landmark
            abs_x = landmark['x'] + detected_distance * math.cos(landmark['theta'] + detected_bearing)
            abs_y = landmark['y'] + detected_distance * math.sin(landmark['theta'] + detected_bearing)
            # Koreksi pose (fusion sederhana: 70% odometry, 30% landmark)
            self.robot_pose['x'] = 0.7 * self.robot_pose['x'] + 0.3 * abs_x
            self.robot_pose['y'] = 0.7 * self.robot_pose['y'] + 0.3 * abs_y
            # Update theta jika landmark punya orientasi
            if 'theta' in landmark:
                self.robot_pose['theta'] = 0.7 * self.robot_pose['theta'] + 0.3 * landmark['theta']
    
    def save_landmark(self, color, bearing, distance):
        """Simpan landmark saat pertama kali terdeteksi"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.landmarks[color] = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_zone(self, color, bearing, distance):
        """Simpan zona watering zone"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.watering_zones[color] = {
            'x': x, 
            'y': y, 
            'theta': 0.0,
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_station(self, bearing, distance):
        """Simpan water station position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.water_station = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_home_base(self, bearing, distance):
        """Simpan home base position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.home_base = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base
    
    def update_pose(self, dx, dy, dtheta):
        """Update pose dari LiDAR odometry (dalam meter dan radian)"""
        self.robot_pose['x'] += dx * math.cos(self.robot_pose['theta']) - dy * math.sin(self.robot_pose['theta'])
        self.robot_pose['y'] += dx * math.sin(self.robot_pose['theta']) + dy * math.cos(self.robot_pose['theta'])
        self.robot_pose['theta'] = (self.robot_pose['theta'] + dtheta) % (2 * math.pi)
    
    def correct_pose_from_landmark(self, color, detected_bearing, detected_distance):
        """Koreksi pose absolut saat landmark terlihat"""
        if color in self.landmarks:
            landmark = self.landmarks[color]
            # Hitung pose absolut dari landmark
            abs_x = landmark['x'] + detected_distance * math.cos(landmark['theta'] + detected_bearing)
            abs_y = landmark['y'] + detected_distance * math.sin(landmark['theta'] + detected_bearing)
            # Koreksi pose (fusion sederhana: 70% odometry, 30% landmark)
            self.robot_pose['x'] = 0.7 * self.robot_pose['x'] + 0.3 * abs_x
            self.robot_pose['y'] = 0.7 * self.robot_pose['y'] + 0.3 * abs_y
            # Update theta jika landmark punya orientasi
            if 'theta' in landmark:
                self.robot_pose['theta'] = 0.7 * self.robot_pose['theta'] + 0.3 * landmark['theta']
    
    def save_landmark(self, color, bearing, distance):
        """Simpan landmark saat pertama kali terdeteksi"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.landmarks[color] = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_zone(self, color, bearing, distance):
        """Simpan zona watering zone"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.watering_zones[color] = {
            'x': x, 
            'y': y, 
            'theta': 0.0,
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_station(self, bearing, distance):
        """Simpan water station position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.water_station = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_home_base(self, bearing, distance):
        """Simpan home base position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.home_base = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base
    
    def update_pose(self, dx, dy, dtheta):
        """Update pose dari LiDAR odometry (dalam meter dan radian)"""
        self.robot_pose['x'] += dx * math.cos(self.robot_pose['theta']) - dy * math.sin(self.robot_pose['theta'])
        self.robot_pose['y'] += dx * math.sin(self.robot_pose['theta']) + dy * math.cos(self.robot_pose['theta'])
        self.robot_pose['theta'] = (self.robot_pose['theta'] + dtheta) % (2 * math.pi)
    
    def correct_pose_from_landmark(self, color, detected_bearing, detected_distance):
        """Koreksi pose absolut saat landmark terlihat"""
        if color in self.landmarks:
            landmark = self.landmarks[color]
            # Hitung pose absolut dari landmark
            abs_x = landmark['x'] + detected_distance * math.cos(landmark['theta'] + detected_bearing)
            abs_y = landmark['y'] + detected_distance * math.sin(landmark['theta'] + detected_bearing)
            # Koreksi pose (fusion sederhana: 70% odometry, 30% landmark)
            self.robot_pose['x'] = 0.7 * self.robot_pose['x'] + 0.3 * abs_x
            self.robot_pose['y'] = 0.7 * self.robot_pose['y'] + 0.3 * abs_y
            # Update theta jika landmark punya orientasi
            if 'theta' in landmark:
                self.robot_pose['theta'] = 0.7 * self.robot_pose['theta'] + 0.3 * landmark['theta']
    
    def save_landmark(self, color, bearing, distance):
        """Simpan landmark saat pertama kali terdeteksi"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.landmarks[color] = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_zone(self, color, bearing, distance):
        """Simpan zona watering zone"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.watering_zones[color] = {
            'x': x, 
            'y': y, 
            'theta': 0.0,
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_station(self, bearing, distance):
        """Simpan water station position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.water_station = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_home_base(self, bearing, distance):
        """Simpan home base position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.home_base = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base
    
    def update_pose(self, dx, dy, dtheta):
        """Update pose dari LiDAR odometry (dalam meter dan radian)"""
        self.robot_pose['x'] += dx * math.cos(self.robot_pose['theta']) - dy * math.sin(self.robot_pose['theta'])
        self.robot_pose['y'] += dx * math.sin(self.robot_pose['theta']) + dy * math.cos(self.robot_pose['theta'])
        self.robot_pose['theta'] = (self.robot_pose['theta'] + dtheta) % (2 * math.pi)
    
    def correct_pose_from_landmark(self, color, detected_bearing, detected_distance):
        """Koreksi pose absolut saat landmark terlihat"""
        if color in self.landmarks:
            landmark = self.landmarks[color]
            # Hitung pose absolut dari landmark
            abs_x = landmark['x'] + detected_distance * math.cos(landmark['theta'] + detected_bearing)
            abs_y = landmark['y'] + detected_distance * math.sin(landmark['theta'] + detected_bearing)
            # Koreksi pose (fusion sederhana: 70% odometry, 30% landmark)
            self.robot_pose['x'] = 0.7 * self.robot_pose['x'] + 0.3 * abs_x
            self.robot_pose['y'] = 0.7 * self.robot_pose['y'] + 0.3 * abs_y
            # Update theta jika landmark punya orientasi
            if 'theta' in landmark:
                self.robot_pose['theta'] = 0.7 * self.robot_pose['theta'] + 0.3 * landmark['theta']
    
    def save_landmark(self, color, bearing, distance):
        """Simpan landmark saat pertama kali terdeteksi"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.landmarks[color] = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_zone(self, color, bearing, distance):
        """Simpan zona watering zone"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.watering_zones[color] = {
            'x': x, 
            'y': y, 
            'theta': 0.0,
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_station(self, bearing, distance):
        """Simpan water station position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.water_station = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_home_base(self, bearing, distance):
        """Simpan home base position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.home_base = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base
    
    def update_pose(self, dx, dy, dtheta):
        """Update pose dari LiDAR odometry (dalam meter dan radian)"""
        self.robot_pose['x'] += dx * math.cos(self.robot_pose['theta']) - dy * math.sin(self.robot_pose['theta'])
        self.robot_pose['y'] += dx * math.sin(self.robot_pose['theta']) + dy * math.cos(self.robot_pose['theta'])
        self.robot_pose['theta'] = (self.robot_pose['theta'] + dtheta) % (2 * math.pi)
    
    def correct_pose_from_landmark(self, color, detected_bearing, detected_distance):
        """Koreksi pose absolut saat landmark terlihat"""
        if color in self.landmarks:
            landmark = self.landmarks[color]
            # Hitung pose absolut dari landmark
            abs_x = landmark['x'] + detected_distance * math.cos(landmark['theta'] + detected_bearing)
            abs_y = landmark['y'] + detected_distance * math.sin(landmark['theta'] + detected_bearing)
            # Koreksi pose (fusion sederhana: 70% odometry, 30% landmark)
            self.robot_pose['x'] = 0.7 * self.robot_pose['x'] + 0.3 * abs_x
            self.robot_pose['y'] = 0.7 * self.robot_pose['y'] + 0.3 * abs_y
            # Update theta jika landmark punya orientasi
            if 'theta' in landmark:
                self.robot_pose['theta'] = 0.7 * self.robot_pose['theta'] + 0.3 * landmark['theta']
    
    def save_landmark(self, color, bearing, distance):
        """Simpan landmark saat pertama kali terdeteksi"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.landmarks[color] = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_zone(self, color, bearing, distance):
        """Simpan zona watering zone"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.watering_zones[color] = {
            'x': x, 
            'y': y, 
            'theta': 0.0,
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_station(self, bearing, distance):
        """Simpan water station position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.water_station = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_home_base(self, bearing, distance):
        """Simpan home base position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.home_base = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base
    
    def save_map(self):
        """Save map to JSON file"""
        data = {
            'robot_pose': self.robot_pose,
            'landmarks': self.landmarks,
            'watering_zones': self.watering_zones,
            'water_station': self.water_station,
            'home_base': self.home_base
        }
        try:
            with open(self.map_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"[MAP] ERROR saving map: {e}")
    
    def load_map(self):
        """Load map from JSON file"""
        if os.path.exists(self.map_file):
            try:
                with open(self.map_file, 'r') as f:
                    data = json.load(f)
                self.robot_pose = data.get('robot_pose', {'x': 0.0, 'y': 0.0, 'theta': 0.0})
                self.landmarks = data.get('landmarks', {})
                self.watering_zones = data.get('watering_zones', {})
                self.water_station = data.get('water_station')
                self.home_base = data.get('home_base')
                print(f"[MAP] Loaded map with {len(self.landmarks)} landmarks")
            except Exception as e:
                print(f"[MAP] ERROR loading map: {e}")

    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base


class ArenaMap:
    def __init__(self):
        self.landmarks = {}  # color -> {'x':, 'y':, 'theta':, 'timestamp':}
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.waypoints = []  # untuk path planning
        self.home_base = None
        self.water_station = None
        self.watering_zones = {}
        
        self.map_file = os.path.join(os.path.dirname(__file__), "..", "data", "arena_map.json")
        self.load_map()
    
    def update_pose(self, dx, dy, dtheta):
        """Update pose dari LiDAR odometry (dalam meter dan radian)"""
        self.robot_pose['x'] += dx * math.cos(self.robot_pose['theta']) - dy * math.sin(self.robot_pose['theta'])
        self.robot_pose['y'] += dx * math.sin(self.robot_pose['theta']) + dy * math.cos(self.robot_pose['theta'])
        self.robot_pose['theta'] = (self.robot_pose['theta'] + dtheta) % (2 * math.pi)
    
    def correct_pose_from_landmark(self, color, detected_bearing, detected_distance):
        """Koreksi pose absolut saat landmark terlihat"""
        if color in self.landmarks:
            landmark = self.landmarks[color]
            # Hitung pose absolut dari landmark
            abs_x = landmark['x'] + detected_distance * math.cos(landmark['theta'] + detected_bearing)
            abs_y = landmark['y'] + detected_distance * math.sin(landmark['theta'] + detected_bearing)
            # Koreksi pose (fusion sederhana: 70% odometry, 30% landmark)
            self.robot_pose['x'] = 0.7 * self.robot_pose['x'] + 0.3 * abs_x
            self.robot_pose['y'] = 0.7 * self.robot_pose['y'] + 0.3 * abs_y
            # Update theta jika landmark punya orientasi
            if 'theta' in landmark:
                self.robot_pose['theta'] = 0.7 * self.robot_pose['theta'] + 0.3 * landmark['theta']
    
    def save_landmark(self, color, bearing, distance):
        """Simpan landmark saat pertama kali terdeteksi"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.landmarks[color] = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_zone(self, color, bearing, distance):
        """Simpan zona watering zone"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.watering_zones[color] = {
            'x': x, 
            'y': y, 
            'theta': 0.0,
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_station(self, bearing, distance):
        """Simpan water station position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.water_station = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
        
    def save_home_base(self, bearing, distance):
        """Simpan home base position"""
        x = self.robot_pose['x'] + distance * math.cos(self.robot_pose['theta'] + bearing)
        y = self.robot_pose['y'] + distance * math.sin(self.robot_pose['theta'] + bearing)
        self.home_base = {
            'x': x, 
            'y': y, 
            'theta': self.robot_pose['theta'],
            'timestamp': time.time()
        }
        self.save_map()
    
    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base
    
    def save_map(self):
        """Save map to JSON file"""
        data = {
            'robot_pose': self.robot_pose,
            'landmarks': self.landmarks,
            'watering_zones': self.watering_zones,
            'water_station': self.water_station,
            'home_base': self.home_base
        }
        try:
            with open(self.map_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"[MAP] ERROR saving map: {e}")
    
    def load_map(self):
        """Load map from JSON file"""
        if os.path.exists(self.map_file):
            try:
                with open(self.map_file, 'r') as f:
                    data = json.load(f)
                self.robot_pose = data.get('robot_pose', {'x': 0.0, 'y': 0.0, 'theta': 0.0})
                self.landmarks = data.get('landmarks', {})
                self.watering_zones = data.get('watering_zones', {})
                self.water_station = data.get('water_station')
                self.home_base = data.get('home_base')
                print(f"[MAP] Loaded map with {len(self.landmarks)} landmarks")
            except Exception as e:
                print(f"[MAP] ERROR loading map: {e}")

    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base
    
    def save_map(self):
        """Save map to JSON file"""
        data = {
            'robot_pose': self.robot_pose,
            'landmarks': self.landmarks,
            'watering_zones': self.watering_zones,
            'water_station': self.water_station,
            'home_base': self.home_base
        }
        try:
            with open(self.map_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"[MAP] ERROR saving map: {e}")
    
    def load_map(self):
        """Load map from JSON file"""
        if os.path.exists(self.map_file):
            try:
                with open(self.map_file, 'r') as f:
                    data = json.load(f)
                self.robot_pose = data.get('robot_pose', {'x': 0.0, 'y': 0.0, 'theta': 0.0})
                self.landmarks = data.get('landmarks', {})
                self.watering_zones = data.get('watering_zones', {})
                self.water_station = data.get('water_station')
                self.home_base = data.get('home_base')
                print(f"[MAP] Loaded map with {len(self.landmarks)} landmarks")
            except Exception as e:
                print(f"[MAP] ERROR loading map: {e}")

    def get_zone_position(self, color):
        return self.watering_zones.get(color)
    
    def get_station_position(self):
        return self.water_station
    
    def get_home_position(self):
        return self.home_base