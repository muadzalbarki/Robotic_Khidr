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
                bottles = self.vision.detect_bottles(frame)

                # 2. State Machine Logic
                if self.state == "EXPLORE":
                    # Simple wall following logic
                    print("[EXPLORE] Scanning...")
                    self.esp.forward(80)
                    if env["obstacles"]:
                        self.esp.turn_right(100)
                        time.sleep(0.5)
                
                elif self.state == "NAV_STATION":
                    print("[NAV] Moving to station...")
                    # Add pathfinding to station coords here
                    self.state = "SCANNING_BOTTLE"
                
                elif self.state == "APPROACHING_BOTTLE":
                    # Use ultrasonic to stop at 5cm
                    if self.ultrasonic_dist < 5.0:
                        self.esp.stop()
                        self.state = "GRIPPING"
                    else:
                        self.esp.forward(60)

                elif self.state == "GRIPPING":
                    print("[ACTION] Gripping...")
                    self.esp.grab_bottle()
                    self.state = "NAV_ZONE"

                # ... (add other states)

                time.sleep(0.1)
        except KeyboardInterrupt:
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
