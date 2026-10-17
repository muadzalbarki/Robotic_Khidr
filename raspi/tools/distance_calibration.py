"""
Distance Calibration Tool untuk Vision System
Mengkalibrasi distance estimation berdasarkan Y position dan area object

Workflow:
1. Letakkan objek hitam di berbagai jarak (30cm - 200cm)
2. Capture Y position dan area dari detection
3. Input actual distance (measured dengan meteran)
4. Generate calibration formula
5. Update config.py dengan hasil

Usage:
    python3 distance_calibration.py
"""

import cv2
import numpy as np
from picamera2 import Picamera2
import time
import json
import os
from scipy.optimize import curve_fit


class DistanceCalibrator:
    def __init__(self):
        self.picam = None
        self.calibration_data = []
        self.frame_width = 640
        self.frame_height = 480
        
        self._init_camera()
        
        print("\n" + "="*60)
        print("   DISTANCE CALIBRATION TOOL")
        print("="*60)
        print("\nThis tool calibrates distance estimation from vision")
        print("Camera angle: 25° downward from horizontal")
        print("\nYou will need:")
        print("  - Black object (e.g., black box, black paper)")
        print("  - Measuring tape/ruler")
        print("  - 10-15 distance measurements (30cm - 200cm)")
        print("\n" + "="*60)
    
    def _init_camera(self):
        """Initialize camera"""
        try:
            self.picam = Picamera2()
            config = self.picam.create_preview_configuration(
                main={"size": (self.frame_width, self.frame_height), "format": "RGB888"}
            )
            self.picam.configure(config)
            self.picam.start()
            time.sleep(2)
            print("\n[CAMERA] Initialized successfully")
        except Exception as e:
            print(f"[ERROR] Failed to initialize camera: {e}")
            exit(1)
    
    def detect_black_object(self, frame):
        """Detect largest black object in frame"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Black mask
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, mask
        
        # Get largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < 500:
            return None, mask
        
        # Calculate centroid
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None, mask
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        return {
            'centroid': (cx, cy),
            'area': area,
            'contour': largest
        }, mask
    
    def capture_measurement(self):
        """Capture single measurement"""
        print("\n" + "-"*60)
        print("Place black object at known distance")
        print("Press SPACE to capture, 'q' to finish calibration")
        print("-"*60)
        
        captured = False
        detection_data = None
        
        while not captured:
            frame = self.picam.capture_array()
            
            detection, mask = self.detect_black_object(frame)
            
            # Visualize
            vis_frame = frame.copy()
            
            if detection:
                # Draw detection
                cv2.drawContours(vis_frame, [detection['contour']], -1, (0, 255, 0), 2)
                cv2.circle(vis_frame, detection['centroid'], 5, (0, 0, 255), -1)
                
                # Display info
                info_text = f"Y: {detection['centroid'][1]} | Area: {int(detection['area'])}"
                cv2.putText(vis_frame, info_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(vis_frame, "Press SPACE to capture", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                detection_data = detection
            else:
                cv2.putText(vis_frame, "No black object detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow('Calibration', vis_frame)
            cv2.imshow('Mask', mask)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord(' ') and detection_data:
                # Capture measurement
                captured = True
            elif key == ord('q'):
                return None
        
        # Get actual distance from user
        print("\nObject detected:")
        print(f"  Y position: {detection_data['centroid'][1]} pixels")
        print(f"  Area: {detection_data['area']:.0f} pixels")
        
        while True:
            try:
                distance_str = input("\nEnter actual distance (cm) [or 'skip']: ")
                
                if distance_str.lower() == 'skip':
                    return None
                
                actual_distance = float(distance_str)
                
                if actual_distance < 10 or actual_distance > 300:
                    print("Distance should be between 10-300 cm")
                    continue
                
                break
            except ValueError:
                print("Invalid input. Enter number in cm")
        
        # Store measurement
        measurement = {
            'y_pixel': detection_data['centroid'][1],
            'area': detection_data['area'],
            'actual_distance_cm': actual_distance
        }
        
        self.calibration_data.append(measurement)
        
        print(f"\n✓ Captured: Y={measurement['y_pixel']}, Area={measurement['area']:.0f}, Distance={actual_distance}cm")
        print(f"Total measurements: {len(self.calibration_data)}")
        
        return measurement
    
    def run_calibration(self):
        """Run full calibration process"""
        print("\nStarting calibration...")
        print("Recommended: 10-15 measurements at various distances")
        print("Example distances: 30, 50, 70, 90, 110, 130, 150, 170, 190 cm")
        
        while True:
            result = self.capture_measurement()
            
            if result is None:
                break
            
            # Check if enough data
            if len(self.calibration_data) >= 5:
                continue_str = input("\nCapture more measurements? (y/n): ")
                if continue_str.lower() != 'y':
                    break
        
        if len(self.calibration_data) < 5:
            print("\n[ERROR] Need at least 5 measurements for calibration")
            return False
        
        print(f"\n[SUCCESS] Captured {len(self.calibration_data)} measurements")
        return True
    
    def analyze_data(self):
        """Analyze calibration data and generate formulas"""
        if len(self.calibration_data) < 5:
            print("[ERROR] Not enough data for analysis")
            return
        
        print("\n" + "="*60)
        print("   CALIBRATION ANALYSIS")
        print("="*60)
        
        # Extract data
        y_pixels = np.array([d['y_pixel'] for d in self.calibration_data])
        areas = np.array([d['area'] for d in self.calibration_data])
        distances = np.array([d['actual_distance_cm'] for d in self.calibration_data])
        
        # Normalize Y (0 = top/far, 1 = bottom/near)
        y_normalized = y_pixels / self.frame_height
        
        # Model 1: Distance from Y position (linear)
        # distance = a * (1 - y_norm) + b
        def linear_model(y_norm, a, b):
            return a * (1 - y_norm) + b
        
        try:
            params_y, _ = curve_fit(linear_model, y_normalized, distances)
            frame_bottom_cm = params_y[1]
            frame_top_cm = params_y[0] + params_y[1]
            
            print(f"\n1. Y-Position Model (Linear):")
            print(f"   Frame bottom distance: {frame_bottom_cm:.1f} cm")
            print(f"   Frame top distance: {frame_top_cm:.1f} cm")
            
            # Calculate R-squared
            residuals = distances - linear_model(y_normalized, *params_y)
            ss_res = np.sum(residuals**2)
            ss_tot = np.sum((distances - np.mean(distances))**2)
            r_squared_y = 1 - (ss_res / ss_tot)
            print(f"   R² = {r_squared_y:.3f}")
        
        except Exception as e:
            print(f"[ERROR] Y-position fitting failed: {e}")
            frame_bottom_cm = 30
            frame_top_cm = 300
        
        # Model 2: Area-based correction
        # Estimate reference area at 100cm
        ref_distances = np.array([100])
        ref_areas = []
        
        for i, dist in enumerate(distances):
            if 80 <= dist <= 120:
                ref_areas.append(areas[i])
        
        if ref_areas:
            area_reference = int(np.mean(ref_areas))
        else:
            area_reference = 5000
        
        print(f"\n2. Area Reference:")
        print(f"   Reference area (at ~100cm): {area_reference} pixels")
        
        # Test combined model
        print(f"\n3. Prediction Test:")
        print(f"   {'Y_px':>6} {'Area':>8} {'Actual':>8} {'Predicted':>10} {'Error':>8}")
        print(f"   {'-'*6} {'-'*8} {'-'*8} {'-'*10} {'-'*8}")
        
        total_error = 0
        for data in self.calibration_data:
            y_norm = data['y_pixel'] / self.frame_height
            dist_from_y = frame_bottom_cm + (frame_top_cm - frame_bottom_cm) * (1 - y_norm)
            
            if data['area'] > 0:
                area_factor = np.sqrt(area_reference / data['area'])
                predicted = dist_from_y * area_factor
            else:
                predicted = dist_from_y
            
            predicted = max(10, min(300, predicted))
            
            error = abs(predicted - data['actual_distance_cm'])
            total_error += error
            
            print(f"   {data['y_pixel']:6d} {data['area']:8.0f} {data['actual_distance_cm']:8.1f} {predicted:10.1f} {error:8.1f}")
        
        avg_error = total_error / len(self.calibration_data)
        print(f"\n   Average error: {avg_error:.1f} cm")
        
        # Generate config
        self.generate_config(frame_bottom_cm, frame_top_cm, area_reference)
    
    def generate_config(self, frame_bottom_cm, frame_top_cm, area_reference):
        """Generate config.py format"""
        print("\n" + "="*60)
        print("Copy this to config.py:")
        print("="*60)
        
        print(f"""
DISTANCE_CALIBRATION = {{
    "frame_bottom_cm": {frame_bottom_cm:.1f},
    "frame_top_cm": {frame_top_cm:.1f},
    "area_reference": {area_reference}
}}
""")
        
        print("="*60)
        
        # Save to file
        calib_file = os.path.join(os.path.dirname(__file__), 'distance_calibration.json')
        
        calib_result = {
            'frame_bottom_cm': float(frame_bottom_cm),
            'frame_top_cm': float(frame_top_cm),
            'area_reference': int(area_reference),
            'measurements': self.calibration_data
        }
        
        try:
            with open(calib_file, 'w') as f:
                json.dump(calib_result, f, indent=2)
            print(f"\n[SAVED] Calibration data saved to {calib_file}")
        except Exception as e:
            print(f"[ERROR] Could not save: {e}")
    
    def cleanup(self):
        """Cleanup resources"""
        if self.picam:
            self.picam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    calibrator = DistanceCalibrator()
    
    try:
        if calibrator.run_calibration():
            calibrator.analyze_data()
        else:
            print("\n[CANCELLED] Calibration cancelled or insufficient data")
    
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Calibration interrupted")
    
    finally:
        calibrator.cleanup()
        print("\n[DONE] Distance calibration complete!")
