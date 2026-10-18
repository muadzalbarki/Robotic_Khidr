"""
Interactive Color Calibration Tool untuk HSV Range Tuning
Gunakan tool ini untuk kalibrasi warna semua target detection:
- Home base (pink)
- Water station (dark blue)
- Bottles (red, yellow, green, blue)
- Watering zone floors (red, yellow, green, blue)
- Pot (white)
- Obstacles (black)

Usage:
    python3 color_calibration.py

Controls:
    q - Quit
    s - Save current HSV values
    n - Next color target
    p - Previous color target
"""

import cv2
import numpy as np
from picamera2 import Picamera2
import time
import json
import os


# Color targets to calibrate
COLOR_TARGETS = [
    ("home_base", "Pink/Magenta Home Base Paper"),
    ("water_station", "Dark Blue Water Station Paper"),
    ("red_bottle", "Red Bottle"),
    ("yellow_bottle", "Yellow Bottle"),
    ("green_bottle", "Green Bottle"),
    ("blue_bottle", "Blue Bottle"),
    ("red_floor", "Red Floor/Watering Zone"),
    ("yellow_floor", "Yellow Floor/Watering Zone"),
    ("green_floor", "Green Floor/Watering Zone"),
    ("blue_floor", "Blue Floor/Watering Zone"),
    ("pot_white", "White Pot"),
    ("obstacle_black", "Black Obstacle Object"),
]

# Default HSV ranges (starting point - updated with non-overlapping ranges & max hue 179)
DEFAULT_RANGES = {
    "home_base": {"h_min": 140, "h_max": 170, "s_min": 50, "s_max": 255, "v_min": 150, "v_max": 255},
    "water_station": {"h_min": 115, "h_max": 135, "s_min": 100, "s_max": 255, "v_min": 50, "v_max": 150},
    "red_bottle": {"h_min": 0, "h_max": 10, "s_min": 120, "s_max": 255, "v_min": 70, "v_max": 255},
    "yellow_bottle": {"h_min": 20, "h_max": 35, "s_min": 100, "s_max": 255, "v_min": 100, "v_max": 255},
    "green_bottle": {"h_min": 45, "h_max": 80, "s_min": 50, "s_max": 255, "v_min": 50, "v_max": 255},
    "blue_bottle": {"h_min": 90, "h_max": 110, "s_min": 80, "s_max": 255, "v_min": 80, "v_max": 255},
    "red_floor": {"h_min": 0, "h_max": 10, "s_min": 80, "s_max": 255, "v_min": 50, "v_max": 200},
    "yellow_floor": {"h_min": 20, "h_max": 35, "s_min": 80, "s_max": 255, "v_min": 80, "v_max": 200},
    "green_floor": {"h_min": 45, "h_max": 80, "s_min": 40, "s_max": 255, "v_min": 40, "v_max": 200},
    "blue_floor": {"h_min": 90, "h_max": 110, "s_min": 60, "s_max": 255, "v_min": 60, "v_max": 200},
    "pot_white": {"h_min": 0, "h_max": 179, "s_min": 0, "s_max": 60, "v_min": 180, "v_max": 255},
    "obstacle_black": {"h_min": 0, "h_max": 179, "s_min": 0, "s_max": 255, "v_min": 0, "v_max": 50},
}


class ColorCalibrator:
    def __init__(self):
        self.picam = None
        self.current_index = 0
        self.calibrated_values = {}
        
        # Load existing calibration if available
        self.load_calibration()
        
        # Initialize camera
        self._init_camera()
        
        # Create windows and trackbars
        self._create_windows()
    
    def _init_camera(self):
        """Initialize Raspberry Pi Camera"""
        try:
            self.picam = Picamera2()
            config = self.picam.create_preview_configuration(
                main={"size": (640, 480), "format": "RGB888"}
            )
            self.picam.configure(config)
            self.picam.start()
            time.sleep(2)
            print("[CAMERA] Initialized successfully")
        except Exception as e:
            print(f"[ERROR] Failed to initialize camera: {e}")
            exit(1)
    
    def _create_windows(self):
        """Create OpenCV windows and trackbars"""
        cv2.namedWindow('Original')
        cv2.namedWindow('Mask')
        cv2.namedWindow('Result')
        cv2.namedWindow('Controls')
        
        # Create trackbars for HSV adjustment (Fix max Hue to 179)
        cv2.createTrackbar('H_min', 'Controls', 0, 179, self._nothing)
        cv2.createTrackbar('H_max', 'Controls', 179, 179, self._nothing)
        cv2.createTrackbar('S_min', 'Controls', 0, 255, self._nothing)
        cv2.createTrackbar('S_max', 'Controls', 255, 255, self._nothing)
        cv2.createTrackbar('V_min', 'Controls', 0, 255, self._nothing)
        cv2.createTrackbar('V_max', 'Controls', 255, 255, self._nothing)
        
        # Load initial values for first target
        self._load_target_values(0)
    
    def _nothing(self, x):
        """Trackbar callback (print current values for guidance)"""
        values = self._get_current_values()
        print(f"[CALIBRATION] H:{values['h_min']}-{values['h_max']} S:{values['s_min']}-{values['s_max']} V:{values['v_min']}-{values['v_max']}", end='\r')

    
    def _load_target_values(self, index):
        """Load HSV values for specific target"""
        target_key = COLOR_TARGETS[index][0]
        
        if target_key in self.calibrated_values:
            values = self.calibrated_values[target_key]
        elif target_key in DEFAULT_RANGES:
            values = DEFAULT_RANGES[target_key]
        else:
            values = {"h_min": 0, "h_max": 180, "s_min": 0, "s_max": 255, "v_min": 0, "v_max": 255}
        
        cv2.setTrackbarPos('H_min', 'Controls', values['h_min'])
        cv2.setTrackbarPos('H_max', 'Controls', values['h_max'])
        cv2.setTrackbarPos('S_min', 'Controls', values['s_min'])
        cv2.setTrackbarPos('S_max', 'Controls', values['s_max'])
        cv2.setTrackbarPos('V_min', 'Controls', values['v_min'])
        cv2.setTrackbarPos('V_max', 'Controls', values['v_max'])
    
    def _get_current_values(self):
        """Get current trackbar values"""
        h_min = cv2.getTrackbarPos('H_min', 'Controls')
        h_max = cv2.getTrackbarPos('H_max', 'Controls')
        s_min = cv2.getTrackbarPos('S_min', 'Controls')
        s_max = cv2.getTrackbarPos('S_max', 'Controls')
        v_min = cv2.getTrackbarPos('V_min', 'Controls')
        v_max = cv2.getTrackbarPos('V_max', 'Controls')
        
        return {
            'h_min': h_min,
            'h_max': h_max,
            's_min': s_min,
            's_max': s_max,
            'v_min': v_min,
            'v_max': v_max
        }
    
    def save_current_target(self):
        """Save current HSV values for current target"""
        target_key = COLOR_TARGETS[self.current_index][0]
        values = self._get_current_values()
        self.calibrated_values[target_key] = values
        print(f"[SAVED] {target_key}: {values}")
    
    def next_target(self):
        """Move to next color target"""
        self.save_current_target()
        self.current_index = (self.current_index + 1) % len(COLOR_TARGETS)
        self._load_target_values(self.current_index)
        print(f"\n[TARGET] {COLOR_TARGETS[self.current_index][1]}")
    
    def prev_target(self):
        """Move to previous color target"""
        self.save_current_target()
        self.current_index = (self.current_index - 1) % len(COLOR_TARGETS)
        self._load_target_values(self.current_index)
        print(f"\n[TARGET] {COLOR_TARGETS[self.current_index][1]}")
    
    def load_calibration(self):
        """Load existing calibration from file"""
        calib_file = os.path.join(os.path.dirname(__file__), 'color_calibration.json')
        
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    self.calibrated_values = json.load(f)
                print(f"[LOADED] Existing calibration from {calib_file}")
            except Exception as e:
                print(f"[WARNING] Could not load calibration: {e}")
                self.calibrated_values = {}
        else:
            self.calibrated_values = {}
    
    def save_calibration(self):
        """Save calibration to file"""
        self.save_current_target()
        
        calib_file = os.path.join(os.path.dirname(__file__), 'color_calibration.json')
        
        try:
            with open(calib_file, 'w') as f:
                json.dump(self.calibrated_values, f, indent=2)
            print(f"\n[SAVED] Calibration saved to {calib_file}")
            
            # Generate config.py format
            self._generate_config_format()
            
        except Exception as e:
            print(f"[ERROR] Could not save calibration: {e}")
    
    def _generate_config_format(self):
        """Generate Python code for config.py"""
        print("\n" + "="*60)
        print("Copy this to config.py COLOR_RANGES:")
        print("="*60)
        
        print("\nCOLOR_RANGES = {")
        
        for target_key, desc in COLOR_TARGETS:
            if target_key in self.calibrated_values:
                v = self.calibrated_values[target_key]
                
                # Handle red wraparound
                if target_key == 'red_bottle':
                    print(f'    "{target_key}": ([{v["h_min"]}, {v["s_min"]}, {v["v_min"]}], [{v["h_max"]}, {v["s_max"]}, {v["v_max"]}]),')
                    print(f'    "red_bottle_wrap": ([170, {v["s_min"]}, {v["v_min"]}], [179, {v["s_max"]}, {v["v_max"]}]),')
                elif target_key == 'red_floor':
                    print(f'    "{target_key}": ([{v["h_min"]}, {v["s_min"]}, {v["v_min"]}], [{v["h_max"]}, {v["s_max"]}, {v["v_max"]}]),')
                    print(f'    "red_floor_wrap": ([170, {v["s_min"]}, {v["v_min"]}], [179, {v["s_max"]}, {v["v_max"]}]),')
                else:
                    print(f'    "{target_key}": ([{v["h_min"]}, {v["s_min"]}, {v["v_min"]}], [{v["h_max"]}, {v["s_max"]}, {v["v_max"]}]),')
        
        print("}")
        print("\n" + "="*60)
    
    def run(self):
        """Main calibration loop"""
        print("\n" + "="*60)
        print("   COLOR CALIBRATION TOOL")
        print("="*60)
        print("\nInstructions:")
        print("  1. Place target object in camera view")
        print("  2. Adjust HSV trackbars until only target is visible in mask")
        print("  3. Press 's' to save current target")
        print("  4. Press 'n' for next target, 'p' for previous")
        print("  5. Press 'q' to quit and save all")
        print("\nControls:")
        print("  q - Quit and save")
        print("  s - Save current target")
        print("  n - Next target")
        print("  p - Previous target")
        print("\n" + "="*60)
        
        print(f"\n[TARGET] {COLOR_TARGETS[self.current_index][1]}")
        
        try:
            while True:
                # Capture frame
                frame = self.picam.capture_array()
                
                # Convert to HSV
                hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
                
                # Get current HSV values
                values = self._get_current_values()
                
                # Create mask
                lower = np.array([values['h_min'], values['s_min'], values['v_min']])
                upper = np.array([values['h_max'], values['s_max'], values['v_max']])
                mask = cv2.inRange(hsv, lower, upper)
                
                # Apply mask
                result = cv2.bitwise_and(frame, frame, mask=mask)
                
                # Add text overlay
                target_name = COLOR_TARGETS[self.current_index][1]
                info_text = f"Target: {target_name} ({self.current_index+1}/{len(COLOR_TARGETS)})"
                cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                hsv_text = f"H:{values['h_min']}-{values['h_max']} S:{values['s_min']}-{values['s_max']} V:{values['v_min']}-{values['v_max']}"
                cv2.putText(frame, hsv_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Calculate detection area
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest)
                    cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Area: {int(area)} px", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # Show windows
                cv2.imshow('Original', frame)
                cv2.imshow('Mask', mask)
                cv2.imshow('Result', result)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    print("\n[QUIT] Saving and exiting...")
                    break
                elif key == ord('s'):
                    self.save_current_target()
                elif key == ord('n'):
                    self.next_target()
                elif key == ord('p'):
                    self.prev_target()
        
        except KeyboardInterrupt:
            print("\n\n[INTERRUPTED] Saving and exiting...")
        
        finally:
            self.save_calibration()
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        if self.picam:
            self.picam.stop()
        cv2.destroyAllWindows()
        print("\n[DONE] Calibration complete!")


if __name__ == "__main__":
    calibrator = ColorCalibrator()
    calibrator.run()
