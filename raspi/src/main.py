import time
import math
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
        self.state = "EXPLORE"  # EXPLORE, SCANNING_BOTTLE, APPROACHING_BOTTLE, GRIPPING, NAV_ZONE, PLACING_BOTTLE, RETURN_HOME
        self.bottles_delivered = {"red": 0, "yellow": 0, "green": 0, "blue": 0}
        self.target_bottle = None
        self.target_zone_color = None
        self.ultrasonic_dist = 999.0
        self.last_obstacle_time = 0
        self.stuck_count = 0
        
    def run(self):
        print("[MISSION] Starting autonomous mission...")
        try:
            while True:
                # 1. Update Sensor Data
                sensor_dist = self.esp.read_sensor_data()
                if sensor_dist is not None:
                    self.ultrasonic_dist = sensor_dist

                frame = self.vision.capture_frame()
                if frame is None:
                    time.sleep(0.1)
                    continue
                    
                env = self.vision.detect_environment(frame)
                objects = self.vision.detect_objects(frame)

                # Update LiDAR and pose
                self.lidar.update_scan()
                self.update_pose_from_lidar()
                
                # Check for landmarks and correct pose
                self.correct_pose_from_landmarks(env)
                
                # 2. State Machine Logic
                if self.state == "EXPLORE":
                    # Wall following with LiDAR
                    print("[EXPLORE] Wall following & mapping...")
                    left_speed, right_speed = self.lidar.get_motor_speeds()
                    self.esp.set_motor_speeds(left_speed, right_speed)
                    
                    # Check for water station
                    if self.check_water_station(env):
                        print("[MISSION] Water station found!")
                        self.state = "SCANNING_BOTTLE"
                        self.esp.stop()
                    
                    # Check for watering zones
                    self.check_watering_zones(env)
                    
                    # Check if completed full loop (back to home)
                    if self.check_loop_closure():
                        print("[MISSION] Completed full arena loop, switching to search watering zone")
                        self.state = "SEARCH_WATERING_ZONE"
                        self.esp.stop()
                
                elif self.state == "SEARCH_WATERING_ZONE":
                    print("[SEARCH_ZONE] Searching for watering zone...")
                    self.lidar.update_scan()
                    left_speed, right_speed = self.lidar.get_motor_speeds()
                    self.esp.set_motor_speeds(left_speed, right_speed)
                    
                    # Check for watering zone
                    zone = self.check_watering_zones(self.vision.detect_environment(self.vision.capture_frame()))
                    if zone:
                        self.target_zone_color = zone
                        print(f"[MISSION] Found watering zone: {zone}")
                        self.state = "SEARCH_BOTTLE"
                        self.esp.stop()
                
                elif self.state == "SCANNING_BOTTLE":
                    print("[SCAN] Berputar mencari botol...")
                    self.esp.turn_right(60)
                    time.sleep(0.1)
                    
                    frame = self.vision.capture_frame()
                    if frame is not None:
                        objects = self.vision.detect_objects(frame)
                        for obj in objects:
                            if "botol" in obj['class'].lower():
                                self.esp.stop()
                                self.target_bottle = obj
                                print(f"[VISION] Terkunci pada: {obj['class']}")
                                self.state = "APPROACHING_BOTTLE"
                                break
                
                elif self.state == "APPROACHING_BOTTLE":
                    if self.target_bottle:
                        frame = self.vision.capture_frame()
                        if frame is not None:
                            objects = self.vision.detect_objects(frame)
                            # Update target position
                            for obj in objects:
                                if obj['class'] == self.target_bottle['class'] and obj['center'] == self.target_bottle['center']:
                                    self.target_bottle = obj
                                    break
                        
                        cx = self.target_bottle['center'][0]
                        pusat_kamera = config.FRAME_CENTER_X
                        
                        if cx < (self.frame_center_x - 40):
                            self.esp.arc_left(60)
                        elif cx > (self.frame_center_x + 40):
                            self.esp.arc_right(60)
                        else:
                            self.esp.forward(60)
                    
                    print(f"[APPROACH] Jarak: {self.ultrasonic_dist} cm")
                    
                    if self.ultrasonic_dist <= config.ULTRASONIC_GRAB_DISTANCE:  # 6cm threshold
                        print("[APPROACH] Jarak ideal! Menghentikan motor.")
                        self.esp.stop()
                        self.state = "GRIPPING"
                    elif self.ultrasonic_dist > 50:
                        self.target_bottle = None
                        self.state = "SCANNING_BOTTLE"
                
                elif self.state == "GRIPPING":
                    print("[ACTION] Mengeksekusi jepitan...")
                    self.esp.grab_bottle(verbose=True)
                    self.state = "NAV_ZONE"
                
                elif self.state == "NAV_ZONE":
                    print("[NAV] Navigasi ke zona warna...")
                    target_zone = self.map.get_zone_position(self.target_zone_color)
                    if target_zone:
                        # Simple navigation: turn towards zone, move forward
                        # For now, use simple timed movement
                        self.esp.backward(80)
                        time.sleep(2)
                        self.esp.turn_left(80)
                        time.sleep(1)
                        self.esp.forward(100)
                        time.sleep(3)
                        self.esp.stop()
                        self.state = "PLACING_BOTTLE"
                    else:
                        print("[WARN] Target zone not found, returning to explore")
                        self.state = "EXPLORE"

                elif self.state == "PLACING_BOTTLE":
                    print("[ACTION] Melepaskan botol ke zona...")
                    self.esp.release_bottle(verbose=True)
                    self.bottles_delivered[self.target_zone_color] = self.bottles_delivered.get(self.target_zone_color, 0) + 1
                    print(f"[MISSION] Delivered bottle to {self.target_zone_color} zone. Total: {self.bottles_delivered[self.target_zone_color]}")
                    
                    # Check if all bottles delivered
                    total_delivered = sum(self.bottles_delivered.values())
                    if total_delivered >= 4:
                        print("[MISSION] All bottles delivered! Returning home...")
                        self.state = "RETURN_HOME"
                    else:
                        self.state = "EXPLORE"
                
                elif self.state == "RETURN_HOME":
                    print("[RETURN] Kembali ke home base...")
                    home_pos = self.map.get_home_position()
                    if home_pos:
                        # Simple return: turn around and go back
                        self.esp.turn_left(180)
                        time.sleep(2)
                        self.esp.forward(100)
                        time.sleep(5)
                        self.esp.stop()
                        print("[MISSION] MISSION COMPLETE!")
                        break
                    else:
                        # No home position saved, just stop
                        self.esp.stop()
                        break

                if config.ENABLE_VISUALIZATION:
                    frame = self.vision.capture_frame()
                    if frame is not None:
                        objects = self.vision.detect_objects(frame)
                        env = self.vision.detect_environment(frame)
                        self.vision.draw_debug(frame, objects, env)

                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n[MISSION] Dihentikan paksa. Mematikan motor...")
            self.esp.stop()
            self.vision.close()
        except Exception as e:
            print(f"[MISSION] ERROR: {e}")
            self.esp.stop()
            self.vision.close()
            raise

    def update_pose_from_lidar(self):
        """Update robot pose from LiDAR odometry"""
        self.lidar.update_scan()
        # Odometry is updated internally in lidar.update_scan()

    def correct_pose_from_landmarks(self, env):
        """Correct pose using visible landmarks"""
        # Check for known landmarks in environment
        if env.get('floor'):
            color = env['floor']
            # If we see a known floor color, we can correct pose
            pass
    
    def check_water_station(self, env):
        """Check if water station is visible"""
        # Water station is detected as 'water_station' in env or objects
        # For now, check if we see water_station color
        return False  # Placeholder
    
    def check_watering_zones(self, env):
        """Check for watering zones and save positions"""
        # Check env for watering zones
        if env.get('floor'):
            color = env['floor']
            if color in ['red_floor', 'yellow_floor', 'green_floor', 'blue_floor']:
                zone_color = color.replace('_floor', '')
                if zone_color not in self.map.watering_zones:
                    # Estimate position from LiDAR
                    sectors = self.lidar.get_sector_distances()
                    front_dist = sectors.get('front', 1000)
                    if front_dist < 100:  # Close enough
                        color_name = color.replace('_floor', '')
                        self.map.save_zone(color.replace('_floor', ''), 0, 50)
                        print(f"[MAP] Saved watering zone: {color_name}")
                        return color_name
        return None
    
    def check_loop_closure(self):
        """Check if we've completed a full loop (back to start)"""
        # Simple check: if we're back near home position
        if self.map.home_base:
            sectors = self.lidar.get_sector_distances()
            front_dist = sectors.get('front', 999)
            # Simple heuristic: if we see home_base color and close to start
            return False  # Placeholder
        return False
    
    def check_water_station(self, env):
        """Check if water station is detected"""
        # Water station detection logic
        return False
    
    def check_loop_closure(self):
        """Check if we've completed a full loop"""
        return False  # Placeholder
    
    def update_pose_from_lidar(self):
        """Update robot pose from LiDAR odometry"""
        self.lidar.update_scan()
        # Pose is updated internally in lidar.update_scan()

    def correct_pose_from_landmarks(self, env):
        """Correct pose using visible landmarks"""
        # Check for known landmarks in environment
        pass


    def check_water_station(self, env):
        """Check if water station is visible"""
        # Water station is detected as 'water_station' in env or objects
        # For now, check if we see water_station color
        return False  # Placeholder
    
    def check_watering_zones(self, env):
        """Check for watering zones and save positions"""
        # Check env for watering zones
        if env.get('floor'):
            color = env['floor']
            if color in ['red_floor', 'yellow_floor', 'green_floor', 'blue_floor']:
                zone_color = color.replace('_floor', '')
                if zone_color not in self.map.watering_zones:
                    # Estimate position from LiDAR
                    sectors = self.lidar.get_sector_distances()
                    front_dist = sectors.get('front', 1000)
                    if front_dist < 100:  # Close enough
                        color_name = color.replace('_floor', '')
                        self.map.save_zone(color_name, 0, 50)
                        print(f"[MAP] Saved watering zone: {color_name}")
                        return color_name
        return None
    
    def check_loop_closure(self):
        """Check if we've completed a full loop (back to start)"""
        # Simple check: if we're back near home position
        if self.map.home_base:
            sectors = self.lidar.get_sector_distances()
            front_dist = sectors.get('front', 999)
            # Simple heuristic: if we see home_base color and close to start
            return False  # Placeholder
        return False


if __name__ == "__main__":
    robot = MissionController()
    robot.run()