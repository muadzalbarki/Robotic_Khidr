import time
from serial_comm import ESPController
from vision import VisionSystem
from lidar_nav import LidarNavigator
from mapper import ArenaMap
import config

class MissionController:
    def __init__(self):
        self.esp = ESPController()
        self.vision = VisionSystem()
        self.lidar = LidarNavigator()
        self.map = ArenaMap()
        self.state = "EXPLORE" # EXPLORE, NAV_STATION, SCANNING_BOTTLE, APPROACHING_BOTTLE, GRIPPING, NAV_ZONE, PLACING_BOTTLE, EVALUATING, RETURN_HOME
        self.bottles_delivered = {"red": 0, "yellow": 0, "green": 0, "blue": 0}
        self.target_bottle = None
        self.ultrasonic_dist = 999.0

    def run(self):
        print("[MISSION] Starting autonomous mission...")
        try:
            while True:
                # 1. Update Sensor Data
                sensor_dist = self.esp.read_sensor_data()
                if sensor_dist is not None:
                    self.ultrasonic_dist = sensor_dist

                frame = self.vision.capture_frame()
                env = self.vision.detect_environment(frame)
                objects = self.vision.detect_bottles(frame)

                # 2. State Machine Logic
                if self.state == "EXPLORE":
                    # Simple wall following logic
                    print("[EXPLORE] Scanning...")
                    self.esp.forward(80)
                    if env["obstacles"]:
                        self.esp.turn_right(100)
                        time.sleep(0.5)
                    # Jika menemukan target station, transisi
                    if env["obstacles"] == []: # Placeholder
                         pass
                
                elif self.state == "SCANNING_BOTTLE":
                    print("[SCAN] Berputar mencari botol...")
                    self.esp.turn_right(60) 
                    
                    if len(objects) > 0:
                        # Cari objek botol
                        for obj in objects:
                            if "botol" in obj['class'].lower():
                                self.esp.stop()
                                self.target_bottle = obj
                                print(f"[VISION] Terkunci pada: {obj['class']}")
                                self.state = "APPROACHING_BOTTLE"
                                break

                elif self.state == "APPROACHING_BOTTLE":
                    if self.target_bottle:
                        cx = self.target_bottle['center'][0]
                        pusat_kamera = config.FRAME_CENTER_X 
                        
                        if cx < (pusat_kamera - 40):
                            self.esp.arc_left(60)
                        elif cx > (pusat_kamera + 40):
                            self.esp.arc_right(60)
                        else:
                            self.esp.forward(60)
                    
                    print(f"[APPROACH] Jarak: {self.ultrasonic_dist} cm")
                    
                    if self.ultrasonic_dist <= 5.0:
                        print("[APPROACH] Jarak ideal! Menghentikan motor.")
                        self.esp.stop()
                        self.state = "GRIPPING"

                elif self.state == "GRIPPING":
                    print("[ACTION] Mengeksekusi jepitan...")
                    self.esp.grab_bottle(verbose=True)
                    self.state = "NAV_ZONE"

                elif self.state == "NAV_ZONE":
                    print("[NAV] Mundur & bergerak ke area pot...")
                    self.esp.backward(80)
                    time.sleep(1.5)
                    self.esp.turn_left(80)
                    time.sleep(1)
                    self.esp.stop()
                    self.state = "PLACING_BOTTLE"

                elif self.state == "PLACING_BOTTLE":
                    print("[ACTION] Melepaskan botol ke zona...")
                    self.esp.release_bottle(verbose=True)
                    # Kembali eksplorasi
                    self.state = "EXPLORE"

                if config.ENABLE_VISUALIZATION:
                    self.vision.draw_debug(frame, objects, env)

                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n[MISSION] Dihentikan paksa. Mematikan motor...")
            self.esp.stop()
            self.vision.close()

    def nav_to_station(self):
        print("[NAV] Moving to water station...")
        self.esp.forward(100)
        time.sleep(2)
        self.esp.stop()
        pass


if __name__ == "__main__":
    robot = MissionController()
    robot.run()
