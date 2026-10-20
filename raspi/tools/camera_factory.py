"""
Camera Factory untuk Color Calibration Tool
Support: Raspberry Pi Camera (Picamera2) & Laptop Webcam (OpenCV)
"""

import cv2
import numpy as np
from abc import ABC, abstractmethod
import time


class CameraInterface(ABC):
    """Abstract base class for camera interface"""
    
    @abstractmethod
    def start(self):
        """Initialize and start camera"""
        pass
    
    @abstractmethod
    def capture_array(self) -> np.ndarray:
        """Capture frame as RGB numpy array"""
        pass
    
    @abstractmethod
    def stop(self):
        """Stop and release camera"""
        pass
    
    @abstractmethod
    def is_opened(self) -> bool:
        """Check if camera is opened"""
        pass


class PiCameraWrapper:
    """Wrapper for Raspberry Pi Camera (Picamera2)"""
    
    def __init__(self, width=640, height=480, format="RGB888"):
        self.width = width
        self.height = height
        self.format = format
        self.picam = None
        self._initialized = False
    
    def start(self):
        from picamera2 import Picamera2
        self.picam = Picamera2()
        config = self.picam.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam.configure(config)
        self.picam.start()
        time.sleep(2)
        print("[CAMERA] PiCamera initialized successfully")
    
    def capture_array(self) -> np.ndarray:
        if self.picam is None:
            return None
        return self.picam.capture_array()
    
    def stop(self):
        if self.picam:
            self.picam.stop()
            print("[CAMERA] PiCamera stopped")
    
    def is_opened(self) -> bool:
        return self.picam is not None


class WebcamWrapper:
    """Wrapper for USB Webcam using OpenCV"""
    
    def __init__(self, camera_index=0, width=640, height=480):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None
        self._backend = cv2.CAP_ANY
    
    def start(self):
        # Try different backends for better compatibility
        backends = [cv2.CAP_V4L2, cv2.CAP_GSTREAMER, cv2.CAP_DSHOW, cv2.CAP_ANY]
        
        for backend in [cv2.CAP_V4L2, cv2.CAP_ANY]:
            self.cap = cv2.VideoCapture(self.camera_index, backend)
            if self.cap.isOpened():
                print(f"[CAMERA] Opened webcam with backend: {backend}")
                break
        
        if not self.cap or not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {self.camera_index}")
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Set buffer size to minimum for low latency
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        time.sleep(1)
        print("[CAMERA] Webcam initialized successfully")
    
    def capture_array(self) -> np.ndarray:
        if self.cap is None or not self.cap.isOpened():
            return None
        
        # Grab and retrieve for lowest latency
        self.cap.grab()
        ret, frame = self.cap.retrieve()
        
        if not ret or frame is None:
            return None
        
        # Convert BGR to RGB (OpenCV uses BGR, we need RGB)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return frame_rgb
    
    def stop(self):
        if self.cap:
            self.cap.release()
            print("[CAMERA] Webcam stopped")
    
    def is_opened(self) -> bool:
        return self.cap is not None and self.cap.isOpened()


def create_camera(source='auto', camera_index=0):
    """
    Factory function to create camera instance
    
    Args:
        source: 'auto', 'pi', 'webcam'
        camera_index: Camera index for webcam (default 0)
    
    Returns:
        Camera instance with start(), capture_array(), stop() methods
    """
    
    if source == 'pi':
        try:
            from picamera2 import Picamera2
            cam = PiCameraWrapper()
            cam.start()
            return cam
        except Exception as e:
            print(f"[CAMERA] PiCamera failed: {e}, falling back to webcam")
            # Fall through to webcam
    
    if source == 'webcam' or source == 'auto':
        try:
            cam = WebcamWrapper(camera_index=0)
            cam.start()
            return cam
        except Exception as e:
            print(f"[CAMERA] Webcam failed: {e}")
            raise
    
    raise ValueError(f"Unknown camera source: {source}")


class PiCameraWrapper:
    """Wrapper for Raspberry Pi Camera (Picamera2)"""
    
    def __init__(self, width=640, height=480, format="RGB888"):
        self.width = width
        self.height = height
        self.format = format
        self.picam = None
    
    def start(self):
        from picamera2 import Picamera2
        self.picam = Picamera2()
        config = self.picam.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam.configure(config)
        self.picam.start()
        time.sleep(2)
        print("[CAMERA] PiCamera initialized successfully")
    
    def capture_array(self):
        if self.picam is None:
            return None
        return self.picam.capture_array()
    
    def stop(self):
        if self.picam:
            self.picam.stop()
            print("[CAMERA] PiCamera stopped")
    
    def is_opened(self):
        return self.picam is not None


class WebcamWrapper:
    """Wrapper for USB Webcam using OpenCV"""
    
    def __init__(self, camera_index=0, width=640, height=480):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None
    
    def start(self):
        # Try different backends for better compatibility
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        
        for backend in [cv2.CAP_V4L2, cv2.CAP_ANY]:
            self.cap = cv2.VideoCapture(0, backend)
            if self.cap.isOpened():
                print(f"[CAMERA] Opened webcam with backend: {backend}")
                break
        
        if not self.cap or not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera index 0")
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Set buffer size to minimum for low latency
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        time.sleep(1)
        print("[CAMERA] Webcam initialized successfully")
    
    def capture_array(self):
        if self.cap is None or not self.cap.isOpened():
            return None
        
        # Grab and retrieve for lowest latency
        self.cap.grab()
        ret, frame = self.cap.retrieve()
        
        if not ret or frame is None:
            return None
        
        # Convert BGR to RGB (OpenCV uses BGR, we need RGB)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return frame_rgb
    
    def stop(self):
        if self.cap:
            self.cap.release()
            print("[CAMERA] Webcam stopped")
    
    def is_opened(self):
        return self.cap is not None and self.cap.isOpened()


def create_camera(source='auto', camera_index=0):
    """
    Factory function to create camera instance
    
    Args:
        source: 'auto', 'pi', 'webcam'
        camera_index: Camera index for webcam (default 0)
    
    Returns:
        Camera instance with start(), capture_array(), stop() methods
    """
    
    if source == 'pi':
        try:
            from picamera2 import Picamera2
            cam = PiCameraWrapper()
            cam.start()
            return cam
        except Exception as e:
            print(f"[CAMERA] PiCamera failed: {e}, falling back to webcam")
            # Fall through to webcam
    
    if source == 'webcam' or source == 'auto':
        try:
            cam = WebcamWrapper(camera_index=0)
            cam.start()
            return cam
        except Exception as e:
            print(f"[CAMERA] Webcam failed: {e}")
            raise
    
    raise ValueError(f"Unknown camera source: {source}")


if __name__ == "__main__":
    # Test camera factory
    import sys
    
    print("Testing camera factory...")
    
    # Test webcam
    print("\n=== Testing Webcam ===")
    try:
        cam = create_camera('webcam')
        print("Camera started successfully")
        
        for i in range(5):
            frame = cam.capture_array()
            if frame is not None:
                print(f"Frame {i}: shape={frame.shape}, dtype={frame.dtype}")
            time.sleep(0.1)
        
        cam.stop()
        print("Webcam test passed!")
        
    except Exception as e:
        print(f"Webcam test failed: {e}")