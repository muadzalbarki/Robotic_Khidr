"""
Vision System Module untuk Robot Otonom (OpenCV Color-based Detection)
Mendeteksi objek menggunakan contour detection berbasis HSV.
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

    def close(self):
        if self.picam:
            self.picam.stop()
        cv2.destroyAllWindows()