import cv2
from ultralytics import YOLO
import os
from config import YOLO_MODEL_PATH, YOLO_CONFIDENCE_THRESHOLD

def test_yolo():
    print(f"Loading YOLO26n model from: {YOLO_MODEL_PATH}")
    
    if not os.path.exists(YOLO_MODEL_PATH):
        print(f"Error: Model file not found at {YOLO_MODEL_PATH}")
        return

    try:
        model = YOLO(YOLO_MODEL_PATH)
        print("Model loaded successfully.")
        
        # Test dengan gambar dummy atau webcam (OpenCV 0)
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Could not open camera.")
            return
            
        print("Capturing frame for inference...")
        ret, frame = cap.read()
        if ret:
            results = model(frame, conf=YOLO_CONFIDENCE_THRESHOLD)
            print(f"Detection successful. Found {len(results[0].boxes)} objects.")
            for box in results[0].boxes:
                print(f"Class: {int(box.cls)}, Conf: {box.conf.item():.2f}")
        
        cap.release()
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    test_yolo()
