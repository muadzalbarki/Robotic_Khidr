"""
Vision System Module untuk Robot Otonom (Hybrid YOLO26n + HSV)
Mendeteksi objek dengan model YOLO26n (Bounding Box) dan klasifikasi warna via HSV.
"""

import cv2
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO
import time
from config import *


class VisionSystem:
    def __init__(self, camera_index=CAMERA_INDEX):
        """Initialize Raspberry Pi Camera and YOLO26n model"""
        self.camera_index = camera_index
        self.picam = None
        self.frame_width = FRAME_WIDTH
        self.frame_height = FRAME_HEIGHT
        self.frame_center_x = FRAME_CENTER_X
        self.frame_center_y = FRAME_CENTER_Y
        
        self.model = None
        self._init_yolo()
        self._init_camera()
    
    def _init_yolo(self):
        """Load YOLO26n model with error handling"""
        try:
            print(f"[VISION] Loading YOLO26n model from {YOLO_MODEL_PATH}...")
            self.model = YOLO(YOLO_MODEL_PATH)
            print("[VISION] YOLO26n model loaded successfully")
        except Exception as e:
            print(f"[VISION] ERROR: Failed to load YOLO26n model: {e}")
            self.model = None

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
    
    def detect_bottles(self, frame: np.ndarray) -> list[dict]:
        """
        Detect bottles using YOLO26n for bounding box and HSV for color classification.
        
        Args:
            frame: Input image (RGB)
            
        Returns:
            List of detected bottles dictionaries:
            [{"color": str, "bbox": (x1, y1, x2, y2), "center": (cx, cy), "area": float}]
        """
        detected_bottles = []
        if self.model is None or frame is None:
            return detected_bottles
        
        try:
            # Run YOLO26n inference
            results = self.model(frame, conf=YOLO_CONFIDENCE_THRESHOLD, verbose=False)
            
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                # Filter class for bottle (COCO class ID 39)
                if cls_id == YOLO_BOTTLE_CLASS_ID:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Ensure bounding box is within frame limits
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(self.frame_width, x2), min(self.frame_height, y2)
                    
                    if x2 <= x1 or y2 <= y1:
                        continue
                    
                    # Crop ROI for color analysis
                    roi = frame[y1:y2, x1:x2]
                    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
                    
                    # Determine color using COLOR_RANGES
                    max_pixels = 0
                    best_color = "unknown"
                    
                    color_keys = ['red_bottle', 'yellow_bottle', 'green_bottle', 'blue_bottle']
                    for color_key in color_keys:
                        if color_key not in COLOR_RANGES:
                            continue
                        
                        lower, upper = COLOR_RANGES[color_key]
                        lower = np.array(lower, dtype=np.uint8)
                        upper = np.array(upper, dtype=np.uint8)
                        
                        mask = cv2.inRange(hsv_roi, lower, upper)
                        
                        # Handle red wraparound if needed
                        if color_key == 'red_bottle' and 'red_bottle_wrap' in COLOR_RANGES:
                            lw_low, lw_up = COLOR_RANGES['red_bottle_wrap']
                            mask_wrap = cv2.inRange(hsv_roi, np.array(lw_low, dtype=np.uint8), np.array(lw_up, dtype=np.uint8))
                            mask = cv2.bitwise_or(mask, mask_wrap)
                        
                        pixel_count = cv2.countNonZero(mask)
                        if pixel_count > max_pixels:
                            max_pixels = pixel_count
                            best_color = color_key.replace('_bottle', '')
                    
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    area = (x2 - x1) * (y2 - y1)
                    
                    detected_bottles.append({
                        "color": best_color,
                        "bbox": (x1, y1, x2, y2),
                        "center": (cx, cy),
                        "area": float(area)
                    })
        except Exception as e:
            print(f"[VISION] ERROR during bottle detection: {e}")
            
        return detected_bottles

    def detect_environment(self, frame: np.ndarray):
        """
        Detect floor color, obstacles, and pots using HSV.
        """
        results = {
            "floor": None,
            "obstacles": [],
            "pots": []
        }
        
        # Crop area for floor detection
        y_min, y_max = FLOOR_DETECTION_ZONE["y_min"], FLOOR_DETECTION_ZONE["y_max"]
        floor_roi = frame[y_min:y_max, :]
        hsv_floor = cv2.cvtColor(floor_roi, cv2.COLOR_RGB2HSV)
        
        # 1. Detect Floor
        max_area = 0
        for color in ['red', 'yellow', 'green', 'blue']:
            key = f"{color}_floor"
            if key in COLOR_RANGES:
                mask = cv2.inRange(hsv_floor, np.array(COLOR_RANGES[key][0]), np.array(COLOR_RANGES[key][1]))
                if color == 'red' and 'red_floor_wrap' in COLOR_RANGES:
                    mask = cv2.bitwise_or(mask, cv2.inRange(hsv_floor, np.array(COLOR_RANGES['red_floor_wrap'][0]), np.array(COLOR_RANGES['red_floor_wrap'][1])))
                area = cv2.countNonZero(mask)
                if area > FLOOR_ZONE_MIN_AREA and area > max_area:
                    max_area = area
                    results["floor"] = color

        # 2. Detect Obstacles & Pots
        hsv_full = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Black obstacles
        obs_mask = cv2.inRange(hsv_full, np.array(COLOR_RANGES["obstacle_black"][0]), np.array(COLOR_RANGES["obstacle_black"][1]))
        contours, _ = cv2.findContours(obs_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > OBSTACLE_MIN_AREA:
                results["obstacles"].append(cv2.boundingRect(cnt))
        
        # White pots
        pot_mask = cv2.inRange(hsv_full, np.array(COLOR_RANGES["pot_white"][0]), np.array(COLOR_RANGES["pot_white"][1]))
        contours, _ = cv2.findContours(pot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > POT_MIN_AREA:
                results["pots"].append(cv2.boundingRect(cnt))
                
        return results

    def draw_debug(self, frame: np.ndarray, detections: dict, env_data: dict, window_name=VIS_WINDOW_NAME):
        """Draw all detections for debugging"""
        if not ENABLE_VISUALIZATION:
            return
            
        vis_frame = frame.copy()
        
        # Draw Floor Text
        if env_data["floor"]:
            cv2.putText(vis_frame, f"Floor: {env_data['floor']}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
        # Draw Obstacles
        for (x, y, w, h) in env_data["obstacles"]:
            cv2.rectangle(vis_frame, (x, y), (x+w, y+h), (0, 0, 0), 2)
            cv2.putText(vis_frame, "Obs", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
        # Draw Pots
        for (x, y, w, h) in env_data["pots"]:
            cv2.rectangle(vis_frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
            cv2.putText(vis_frame, "Pot", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Existing bottle drawing...
        # [REUSED existing logic for bottles...]
        self._draw_bottles(vis_frame, detections)
        
        cv2.imshow(window_name, vis_frame)
        cv2.waitKey(1)

    def _draw_bottles(self, frame, detections):
        color_map = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'unknown': (128, 128, 128)
        }
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            color_name = det['color']
            bgr_color = color_map.get(color_name, (255, 255, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), bgr_color, 2)
            cv2.circle(frame, det['center'], 5, bgr_color, -1)
            label = f"{color_name} bottle"
            cv2.putText(frame, label, (x1, max(20, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr_color, 2)



    def close(self):
        """Cleanup camera and windows"""
        if self.picam:
            self.picam.stop()
            print("[VISION] Camera closed")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    print("=== Hybrid Vision System Test (YOLO26n + HSV) ===\n")
    vision = VisionSystem()
    
    try:
        while True:
            frame = vision.capture_frame()
            if frame is None:
                print("Failed to capture frame")
                break
            
            bottles = vision.detect_bottles(frame)
            env_data = vision.detect_environment(frame)
            print(f"Detected {len(bottles)} bottles:", bottles)
            
            vision.draw_debug(frame, bottles, env_data)

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        vision.close()
