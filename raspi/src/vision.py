"""
Vision System Module untuk Robot Otonom
Mendeteksi berbagai objek menggunakan Raspberry Pi Camera dengan OpenCV:
- Navigation targets (home base, water station)
- Bottles (red, yellow, green, blue)
- Watering zone floors
- White pot (circle detection)
- Black physical obstacles

Camera mount: 25° downward angle dari horizontal
"""

import cv2
import numpy as np
from picamera2 import Picamera2
import time
from config import *


class VisionSystem:
    def __init__(self, camera_index=CAMERA_INDEX):
        """Initialize Raspberry Pi Camera"""
        self.camera_index = camera_index
        self.picam = None
        self.frame_width = FRAME_WIDTH
        self.frame_height = FRAME_HEIGHT
        self.frame_center_x = FRAME_CENTER_X
        self.frame_center_y = FRAME_CENTER_Y
        
        self._init_camera()
    
    def _init_camera(self):
        """Initialize Picamera2"""
        try:
            self.picam = Picamera2()
            config = self.picam.create_preview_configuration(
                main={"size": (self.frame_width, self.frame_height), "format": "RGB888"}
            )
            self.picam.configure(config)
            self.picam.start()
            time.sleep(2)  # Wait for camera to warm up
            print("[VISION] Camera initialized successfully")
        except Exception as e:
            print(f"[VISION] ERROR: Failed to initialize camera: {e}")
            self.picam = None
    
    def capture_frame(self):
        """Capture a frame from camera"""
        if self.picam is None:
            print("[VISION] ERROR: Camera not initialized")
            return None
        
        try:
            frame = self.picam.capture_array()
            return frame
        except Exception as e:
            print(f"[VISION] ERROR: Failed to capture frame: {e}")
            return None
    
    def detect_color_mask(self, frame, color_key):
        """
        Create binary mask for specific color
        
        Args:
            frame: BGR/RGB image
            color_key: Key from COLOR_RANGES config
        
        Returns:
            Binary mask
        """
        if color_key not in COLOR_RANGES:
            print(f"[VISION] WARNING: Unknown color key '{color_key}'")
            return None
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Get color range
        lower, upper = COLOR_RANGES[color_key]
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        
        # Create mask
        mask = cv2.inRange(hsv, lower, upper)
        
        # Morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask
    
    def find_largest_contour(self, mask, min_area=MIN_AREA_THRESHOLD):
        """Find largest contour in mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find largest contour by area
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < min_area:
            return None
        
        # Calculate centroid
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        return {
            'contour': largest,
            'centroid': (cx, cy),
            'area': area,
            'moments': M
        }
    
    def detect_object(self, frame, color_key, min_area=MIN_AREA_THRESHOLD):
        """
        Detect single object by color
        
        Returns:
            Dict with centroid, area, distance_est or None
        """
        mask = self.detect_color_mask(frame, color_key)
        if mask is None:
            return None
        
        result = self.find_largest_contour(mask, min_area)
        if result is None:
            return None
        
        # Estimate distance
        distance_est = self.estimate_distance(result['centroid'][1], result['area'])
        
        return {
            'centroid': result['centroid'],
            'area': result['area'],
            'distance_est': distance_est,
            'contour': result['contour']
        }
    
    def detect_red_bottle(self, frame):
        """Detect red bottle (handle hue wraparound)"""
        # Red requires two masks due to hue wraparound (0° and 180°)
        mask1 = self.detect_color_mask(frame, 'red_bottle')
        mask2 = self.detect_color_mask(frame, 'red_bottle_wrap')
        
        if mask1 is None or mask2 is None:
            return None
        
        # Combine masks
        mask = cv2.bitwise_or(mask1, mask2)
        
        result = self.find_largest_contour(mask, BOTTLE_MIN_AREA)
        if result is None:
            return None
        
        distance_est = self.estimate_distance(result['centroid'][1], result['area'])
        
        return {
            'centroid': result['centroid'],
            'area': result['area'],
            'distance_est': distance_est,
            'contour': result['contour']
        }
    
    def detect_bottles(self, frame):
        """
        Detect all bottles in priority order
        
        Returns:
            Dict with detected bottles
        """
        bottles = {}
        
        # Red (special handling for hue wraparound)
        red = self.detect_red_bottle(frame)
        if red:
            bottles['red_bottle'] = red
        
        # Other colors
        for color in ['yellow_bottle', 'green_bottle', 'blue_bottle']:
            obj = self.detect_object(frame, color, BOTTLE_MIN_AREA)
            if obj:
                bottles[color] = obj
        
        return bottles
    
    def detect_pot(self, frame):
        """
        Detect white pot using color + circle shape detection
        
        Returns:
            Dict with centroid, radius, area, circularity or None
        """
        mask = self.detect_color_mask(frame, 'pot_white')
        if mask is None:
            return None
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_pot = None
        max_score = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < POT_MIN_AREA:
                continue
            
            # Calculate circularity
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            if circularity < POT_MIN_CIRCULARITY:
                continue
            
            # Get enclosing circle
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            
            # Score based on area and circularity
            score = area * circularity
            
            if score > max_score:
                max_score = score
                best_pot = {
                    'centroid': (int(x), int(y)),
                    'radius': int(radius),
                    'area': area,
                    'circularity': circularity,
                    'contour': cnt
                }
        
        return best_pot
    
    def find_watering_zone_near_pot(self, frame, pot_centroid, pot_radius, bottle_color):
        """
        Find colored floor zone beside pot
        
        Args:
            pot_centroid: (x, y) of pot center
            pot_radius: pot radius in pixels
            bottle_color: 'red', 'yellow', 'green', 'blue'
        
        Returns:
            Dict with zone centroid, side ('left'/'right'), area or None
        """
        if bottle_color not in WATERING_ZONE_MAPPING:
            return None
        
        floor_color_key = WATERING_ZONE_MAPPING[bottle_color]
        
        # Handle red floor (hue wraparound)
        if bottle_color == 'red':
            mask1 = self.detect_color_mask(frame, 'red_floor')
            mask2 = self.detect_color_mask(frame, 'red_floor_wrap')
            if mask1 is None or mask2 is None:
                return None
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = self.detect_color_mask(frame, floor_color_key)
            if mask is None:
                return None
        
        pot_x, pot_y = pot_centroid
        
        # Define left and right search zones
        left_x_start = max(0, pot_x - pot_radius - POT_SEARCH_ZONE_WIDTH)
        left_x_end = pot_x - pot_radius
        
        right_x_start = pot_x + pot_radius
        right_x_end = min(self.frame_width, pot_x + pot_radius + POT_SEARCH_ZONE_WIDTH)
        
        # Extract zones
        left_zone = mask[:, left_x_start:left_x_end] if left_x_end > left_x_start else None
        right_zone = mask[:, right_x_start:right_x_end] if right_x_end > right_x_start else None
        
        def analyze_zone(zone_mask, x_offset):
            if zone_mask is None or zone_mask.size == 0:
                return None
            
            contours, _ = cv2.findContours(zone_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None
            
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area < FLOOR_ZONE_MIN_AREA:
                return None
            
            M = cv2.moments(largest)
            if M["m00"] == 0:
                return None
            
            cx = int(M["m10"] / M["m00"]) + x_offset
            cy = int(M["m01"] / M["m00"])
            
            return {'centroid': (cx, cy), 'area': area}
        
        # Analyze both zones
        left_result = analyze_zone(left_zone, left_x_start)
        right_result = analyze_zone(right_zone, right_x_start)
        
        # Return best result
        if left_result and right_result:
            if left_result['area'] > right_result['area']:
                left_result['side'] = 'left'
                return left_result
            else:
                right_result['side'] = 'right'
                return right_result
        elif left_result:
            left_result['side'] = 'left'
            return left_result
        elif right_result:
            right_result['side'] = 'right'
            return right_result
        
        return None
    
    def detect_black_obstacles(self, frame):
        """
        Detect black physical obstacles
        
        Returns:
            List of obstacles sorted by distance (closest first)
        """
        mask = self.detect_color_mask(frame, 'obstacle_black')
        if mask is None:
            return []
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < OBSTACLE_MIN_AREA:
                continue
            
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            distance_est = self.estimate_distance(cy, area)
            
            # Priority classification
            if distance_est < BLACK_OBSTACLE_CRITICAL_DISTANCE:
                priority = 'critical'
            elif distance_est < BLACK_OBSTACLE_WARNING_DISTANCE:
                priority = 'warning'
            else:
                priority = 'safe'
            
            obstacles.append({
                'centroid': (cx, cy),
                'area': area,
                'distance_est': distance_est,
                'priority': priority,
                'contour': cnt
            })
        
        # Sort by distance (closest first)
        obstacles.sort(key=lambda x: x['distance_est'])
        
        return obstacles
    
    def detect_all_objects(self, frame):
        """
        Detect all objects in single frame
        
        Returns:
            Dict with all detected objects
        """
        objects = {}
        
        # Navigation targets
        home = self.detect_object(frame, 'home_base', STATION_MIN_AREA)
        if home:
            objects['home_base'] = home
        
        station = self.detect_object(frame, 'water_station', STATION_MIN_AREA)
        if station:
            objects['water_station'] = station
        
        # Bottles
        bottles = self.detect_bottles(frame)
        objects.update(bottles)
        
        # Pot
        pot = self.detect_pot(frame)
        if pot:
            objects['pot'] = pot
        
        # Black obstacles
        obstacles = self.detect_black_obstacles(frame)
        if obstacles:
            objects['black_obstacles'] = obstacles
        
        return objects
    
    def estimate_distance(self, y_pixel, area):
        """
        Estimate distance from Y position and area
        Camera mounted 25° downward
        
        Args:
            y_pixel: Y coordinate in frame (0 = top, FRAME_HEIGHT = bottom)
            area: Object area in pixels
        
        Returns:
            Estimated distance in cm
        """
        # Normalize Y position (0 = top/far, 1 = bottom/near)
        normalized_y = y_pixel / self.frame_height
        
        # Linear interpolation for distance based on Y
        frame_bottom_dist = DISTANCE_CALIBRATION['frame_bottom_cm']
        frame_top_dist = DISTANCE_CALIBRATION['frame_top_cm']
        
        # Bottom of frame = near, top = far
        distance_from_y = frame_bottom_dist + (frame_top_dist - frame_bottom_dist) * (1 - normalized_y)
        
        # Adjust with area (larger area = closer)
        area_ref = DISTANCE_CALIBRATION['area_reference']
        if area > 0:
            area_factor = np.sqrt(area_ref / area)
            distance = distance_from_y * area_factor
        else:
            distance = distance_from_y
        
        # Clamp to reasonable range
        return max(10, min(300, distance))
    
    def calculate_steering_error(self, target_centroid):
        """
        Calculate steering error from target centroid
        Negative = turn left, Positive = turn right
        
        Returns:
            Error in pixels
        """
        target_x = target_centroid[0]
        error = target_x - self.frame_center_x
        return error
    
    def visualize(self, frame, objects, window_name=VIS_WINDOW_NAME):
        """Draw detection results on frame for debugging"""
        if not ENABLE_VISUALIZATION:
            return
        
        vis_frame = frame.copy()
        
        # Draw navigation targets
        for key in ['home_base', 'water_station']:
            if key in objects:
                obj = objects[key]
                cv2.circle(vis_frame, obj['centroid'], 10, (255, 0, 255), -1)
                cv2.putText(vis_frame, f"{key}: {obj['distance_est']:.0f}cm", 
                           (obj['centroid'][0] + 15, obj['centroid'][1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # Draw bottles
        bottle_colors_vis = {
            'red_bottle': (0, 0, 255),
            'yellow_bottle': (0, 255, 255),
            'green_bottle': (0, 255, 0),
            'blue_bottle': (255, 0, 0)
        }
        for key, color in bottle_colors_vis.items():
            if key in objects:
                obj = objects[key]
                cv2.circle(vis_frame, obj['centroid'], 8, color, -1)
                cv2.putText(vis_frame, f"{key.split('_')[0]}", 
                           (obj['centroid'][0] + 12, obj['centroid'][1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw pot
        if 'pot' in objects:
            pot = objects['pot']
            cv2.circle(vis_frame, pot['centroid'], pot['radius'], (255, 255, 255), 2)
            cv2.putText(vis_frame, "POT", 
                       (pot['centroid'][0] - 20, pot['centroid'][1]),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Draw obstacles
        if 'black_obstacles' in objects:
            for obs in objects['black_obstacles']:
                color = (0, 0, 255) if obs['priority'] == 'critical' else (0, 165, 255)
                cv2.drawContours(vis_frame, [obs['contour']], -1, color, 2)
                cv2.putText(vis_frame, f"OBS {obs['distance_est']:.0f}cm", 
                           (obs['centroid'][0], obs['centroid'][1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Draw center crosshair
        cv2.line(vis_frame, (self.frame_center_x - 20, self.frame_center_y),
                (self.frame_center_x + 20, self.frame_center_y), (0, 255, 0), 1)
        cv2.line(vis_frame, (self.frame_center_x, self.frame_center_y - 20),
                (self.frame_center_x, self.frame_center_y + 20), (0, 255, 0), 1)
        
        cv2.imshow(window_name, vis_frame)
        cv2.waitKey(1)
    
    def close(self):
        """Cleanup"""
        if self.picam:
            self.picam.stop()
            print("[VISION] Camera closed")
        cv2.destroyAllWindows()


# Test function
if __name__ == "__main__":
    print("=== Vision System Test ===\n")
    
    vision = VisionSystem()
    
    try:
        while True:
            frame = vision.capture_frame()
            if frame is None:
                print("Failed to capture frame")
                break
            
            objects = vision.detect_all_objects(frame)
            
            print(f"\n--- Detected Objects ---")
            for key, obj in objects.items():
                if key == 'black_obstacles':
                    print(f"{key}: {len(obj)} obstacles")
                else:
                    print(f"{key}: area={obj['area']}, dist={obj['distance_est']:.1f}cm")
            
            vision.visualize(frame, objects)
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    
    finally:
        vision.close()
