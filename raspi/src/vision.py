"""
Vision System Module untuk Robot Otonom (OpenCV Color-based Detection)
Mendeteksi objek menggunakan contour detection berbasis HSV.
"""

import cv2
import numpy as np
from picamera2 import Picamera2
import time
import importlib
from config import *


class VisionSystem:
    def __init__(self, camera_index=CAMERA_INDEX):
        """Initialize Raspberry Pi Camera"""
        self.camera_index = camera_index
        self.picam = None
        self.frame_width = FRAME_WIDTH
        self.frame_height = FRAME_HEIGHT
        
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
            time.sleep(2)
            print("[VISION] Camera initialized successfully")
        except Exception as e:
            print(f"[VISION] ERROR: Failed to initialize camera: {e}")
            self.picam = None
    
    def capture_frame(self) -> np.ndarray | None:
        """Capture a frame from camera"""
        if self.picam is None:
            return None
        try:
            return self.picam.capture_array()
        except Exception as e:
            print(f"[VISION] ERROR: Failed to capture frame: {e}")
            return None
            
    def detect_objects(self, frame: np.ndarray) -> list[dict]:
        """Detect objects using HSV contour detection"""
        detected_objects = []
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        height, width, _ = frame.shape
        left_limit = width // 3
        right_limit = 2 * width // 3

        # Iterasi setiap warna target yang didefinisikan di COLOR_RANGES
        for color_name, (lower, upper) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Khusus Red Wraparound
            if "red" in color_name and "wrap" in color_name:
                continue # Red wrap sudah digabung di logic detect_environment atau handling terpisah
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > MIN_AREA_THRESHOLD:
                    x, y, w, h = cv2.boundingRect(cnt)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    position = "LEFT" if center_x < left_limit else ("RIGHT" if center_x > right_limit else "CENTER")
                    
                    detected_objects.append({
                        "class": color_name,
                        "area": area,
                        "bbox": (x, y, w, h),
                        "center": (center_x, center_y),
                        "position": position
                    })
        return detected_objects

    def draw_debug(self, frame: np.ndarray, detections: list, window_name=VIS_WINDOW_NAME):
        """Draw debug visualization"""
        if not ENABLE_VISUALIZATION:
            return
            
        vis_frame = frame.copy()
        for det in detections:
            x, y, w, h = det['bbox']
            label = f"{det['class']} ({det['position']})"
            cv2.rectangle(vis_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(vis_frame, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.imshow(window_name, vis_frame)
        cv2.waitKey(1)

    def reload_color_ranges(self):
        """Hot-reload COLOR_RANGES from config.py"""
        import importlib
        import config
        importlib.reload(config)
        from config import COLOR_RANGES, MIN_AREA_THRESHOLD, FLOOR_DETECTION_ZONE, FRAME_WIDTH, FRAME_HEIGHT, FRAME_CENTER_X, FRAME_CENTER_Y
        globals().update({
            'COLOR_RANGES': config.COLOR_RANGES,
            'MIN_AREA_THRESHOLD': config.MIN_AREA_THRESHOLD,
            'FLOOR_DETECTION_ZONE': config.FLOOR_DETECTION_ZONE,
            'FRAME_WIDTH': config.FRAME_WIDTH,
            'FRAME_HEIGHT': config.FRAME_HEIGHT,
            'FRAME_CENTER_X': config.FRAME_CENTER_X,
            'FRAME_CENTER_Y': config.FRAME_CENTER_Y,
        })
        # Update instance attributes
        self.frame_width = FRAME_WIDTH
        self.frame_height = FRAME_HEIGHT
        self.frame_center_x = FRAME_CENTER_X
        self.frame_center_y = FRAME_CENTER_Y
        print("[VISION] COLOR_RANGES hot-reloaded from config.py")

    def detect_environment(self, frame: np.ndarray):
        """
        Detect floor color, obstacles, and pots using HSV.
        """
        results = {
            "floor": None,
            "obstacles": [],
            "pots": []
        }
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Crop area for floor detection
        y_min, y_max = FLOOR_DETECTION_ZONE["y_min"], FLOOR_DETECTION_ZONE["y_max"]
        floor_roi = frame[y_min:y_max, :]
        hsv_floor = cv2.cvtColor(floor_roi, cv2.COLOR_RGB2HSV)
        
        # 1. Detect Floor
        max_area = 0
        for color in ['red', 'yellow', 'green', 'blue']:
            key = f"{color}_floor"
            if key in COLOR_RANGES:
                mask = cv2.inRange(hsv, np.array(COLOR_RANGES[key][0]), np.array(COLOR_RANGES[key][1]))
                if color == 'red' and 'red_floor_wrap' in COLOR_RANGES:
                    mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(COLOR_RANGES['red_floor_wrap'][0]), np.array(COLOR_RANGES['red_floor_wrap'][1])))
                area = cv2.countNonZero(mask)
                if area > FLOOR_ZONE_MIN_AREA and area > max_area:
                    max_area = area
                    results["floor"] = color

        # 2. Detect Obstacles & Pots
        hsv_full = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Black obstacles
        obstacles = []
        obs_mask = cv2.inRange(hsv_full, np.array(COLOR_RANGES["obstacle_black"][0]), np.array(COLOR_RANGES["obstacle_black"][1]))
        contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > OBSTACLE_MIN_AREA:
                obstacles.append(cv2.boundingRect(cnt))
        
        # White pots
        pot_mask = cv2.inRange(hsv_full, np.array(COLOR_RANGES["pot_white"][0]), np.array(COLOR_RANGES["pot_white"][1]))
        contours, _ = cv2.findContours(pot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > POT_MIN_AREA:
                results["pots"].append(cv2.boundingRect(cnt))
                
        results["obstacles"] = obstacles
        return results

    def close(self):
        if self.picam:
            self.picam.stop()
            print("[VISION] Camera closed")
        cv2.destroyAllWindows()