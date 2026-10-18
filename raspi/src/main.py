import time
from serial_comm import ESPController
from vision import VisionSystem
from lidar_nav import LidarNavigator
from mapper import ArenaMap
import config

class AutonomousRobot:
    def __init__(self):
        self.esp = ESPController()
        self.vision = VisionSystem()
        self.lidar = LidarNavigator()
        self.map = ArenaMap()
        self.state = "EXPLORE" # EXPLORE, NAV_STATION, GRAB, NAV_ZONE, RELEASE, RETURN_HOME
        self.bottles_delivered = {"red": 0, "yellow": 0, "green": 0, "blue": 0}

    def run(self):
        print("[MISSION] Starting autonomous mission...")
        try:
            while True:
                frame = self.vision.capture_frame()
                env = self.vision.detect_environment(frame)
                bottles = self.vision.detect_bottles(frame)

                if self.state == "EXPLORE":
                    self.explore_cycle(env)
                elif self.state == "NAV_STATION":
                    self.nav_to_station()
                # ... (state logic selanjutnya)
                
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.esp.stop()
            self.vision.close()

    def explore_cycle(self, env):
        # Wall following + mapping logic here
        print("[EXPLORE] Scanning...")
        # Placeholder
        pass

    def nav_to_station(self):
        # Navigation logic here
        pass

if __name__ == "__main__":
    robot = AutonomousRobot()
    robot.run()
