"""
Main Autonomous Navigation System - Multi-Trip State Machine
Robot akan mengambil 4 botol (red, yellow, green, blue) dari Water Station
dan menempatkannya di Watering Zone yang sesuai, kemudian kembali ke Home Base

State Flow:
    IDLE → GOTO_WATER_STATION → APPROACH_BOTTLE → GRAB_BOTTLE → 
    FIND_POT → GOTO_WATERING_ZONE → APPROACH_WATERING_ZONE → RELEASE_BOTTLE →
    CHECK_MORE_BOTTLES → (loop atau) RETURN_HOME → DONE
"""

import time
import sys
from enum import Enum
import signal

from config import *
from vision import VisionSystem
from lidar_nav import create_lidar
from serial_comm import ESPController
from utils import *


class RobotState(Enum):
    """Robot state machine states"""
    IDLE = 0
    GOTO_WATER_STATION = 1
    APPROACH_BOTTLE = 2
    GRAB_BOTTLE = 3
    FIND_POT = 4
    GOTO_WATERING_ZONE = 5
    APPROACH_WATERING_ZONE = 6
    RELEASE_BOTTLE = 7
    CHECK_MORE_BOTTLES = 8
    RETURN_HOME = 9
    DONE = 10
    ERROR = 99


class AutonomousRobot:
    """Main autonomous robot controller"""
    
    def __init__(self, use_lidar_simulator=False):
        """Initialize all subsystems"""
        self.logger = Logger("AutonomousRobot")
        self.logger.info("="*50)
        self.logger.info("AUTONOMOUS ROBOT SYSTEM INITIALIZING")
        self.logger.info("="*50)
        
        # Check calibration status
        if not COLOR_CALIBRATED:
            self.logger.warning("COLOR NOT CALIBRATED! Run tools/color_calibration.py first")
        if not DISTANCE_CALIBRATED:
            self.logger.warning("DISTANCE NOT CALIBRATED! Run tools/distance_calibration.py first")
        
        # Initialize subsystems
        self.logger.info("Initializing Vision System...")
        self.vision = VisionSystem()
        
        self.logger.info("Initializing LiDAR Navigator...")
        self.lidar = create_lidar(use_simulator=use_lidar_simulator)
        
        self.logger.info("Initializing ESP32 Controller...")
        self.esp = ESPController()
        
        if not self.esp.connected:
            self.logger.error("Failed to connect to ESP32!")
            raise Exception("ESP32 connection failed")
        
        # State machine
        self.state = RobotState.IDLE
        self.previous_state = None
        
        # Mission tracking
        self.bottles_delivered = 0
        self.delivered_colors = []
        self.current_bottle_color = None
        self.watering_zone_target = None
        
        # Controllers
        self.steering_pid = PIDController(
            kp=KP_STEERING,
            ki=KI_STEERING,
            kd=KD_STEERING,
            output_limits=(-100, 100)
        )
        
        # Timers and utilities
        self.state_timer = Timer(NAVIGATION_TIMEOUT)
        self.state_stats = StateTimer()
        self.stuck_detector = StuckDetector()
        self.rate_limiter = RateLimiter(10)  # 10Hz control loop
        
        # Retry mechanism
        self.retry_count = 0
        self.max_retries = MAX_RETRY_ATTEMPTS
        
        # Emergency stop flag
        self.emergency_stop = False
        
        self.logger.info("All subsystems initialized successfully!")
        self.logger.info("="*50)
    
    def run(self):
        """Main execution loop"""
        self.logger.info("STARTING AUTONOMOUS MISSION")
        self.state = RobotState.GOTO_WATER_STATION
        
        try:
            while not self.emergency_stop:
                # Rate limiting
                self.rate_limiter.wait_if_needed()
                
                # Update sensors
                self.lidar.update_scan()
                
                # State machine dispatcher
                if self.state == RobotState.IDLE:
                    self.handle_idle()
                
                elif self.state == RobotState.GOTO_WATER_STATION:
                    self.handle_goto_water_station()
                
                elif self.state == RobotState.APPROACH_BOTTLE:
                    self.handle_approach_bottle()
                
                elif self.state == RobotState.GRAB_BOTTLE:
                    self.handle_grab_bottle()
                
                elif self.state == RobotState.FIND_POT:
                    self.handle_find_pot()
                
                elif self.state == RobotState.GOTO_WATERING_ZONE:
                    self.handle_goto_watering_zone()
                
                elif self.state == RobotState.APPROACH_WATERING_ZONE:
                    self.handle_approach_watering_zone()
                
                elif self.state == RobotState.RELEASE_BOTTLE:
                    self.handle_release_bottle()
                
                elif self.state == RobotState.CHECK_MORE_BOTTLES:
                    self.handle_check_more_bottles()
                
                elif self.state == RobotState.RETURN_HOME:
                    self.handle_return_home()
                
                elif self.state == RobotState.DONE:
                    self.logger.info("MISSION COMPLETE!")
                    self.esp.stop()
                    break
                
                elif self.state == RobotState.ERROR:
                    self.logger.error("MISSION FAILED!")
                    self.esp.stop()
                    break
                
                # Check state timeout
                if self.state_timer.is_timeout():
                    self.logger.warning(f"State {self.state.name} TIMEOUT after {self.state_timer.elapsed():.1f}s")
                    self.retry_or_fail()
        
        except KeyboardInterrupt:
            self.logger.warning("Mission interrupted by user")
        
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.shutdown()
    
    def transition_state(self, new_state):
        """Transition to new state with logging"""
        self.state_stats.end_state()
        
        self.previous_state = self.state
        self.state = new_state
        
        self.logger.info(f"STATE TRANSITION: {self.previous_state.name} → {new_state.name}")
        
        # Reset state-specific variables
        self.state_timer.reset()
        self.steering_pid.reset()
        self.retry_count = 0
        
        # Set appropriate timeout
        if new_state in [RobotState.GRAB_BOTTLE, RobotState.RELEASE_BOTTLE]:
            self.state_timer = Timer(GRAB_SEQUENCE_TIME + 1)
        elif new_state == RobotState.FIND_POT:
            self.state_timer = Timer(POT_SEARCH_TIMEOUT)
        elif new_state == RobotState.APPROACH_BOTTLE:
            self.state_timer = Timer(BOTTLE_SEARCH_TIMEOUT)
        else:
            self.state_timer = Timer(NAVIGATION_TIMEOUT)
        
        self.state_stats.start_state(new_state.name)
    
    def navigate_with_obstacle_avoidance(self, target_object_key, frame, objects):
        """
        Core navigation logic dengan sensor fusion (Vision + LiDAR)
        
        Returns:
            'reached', 'navigating', 'avoiding', 'searching', or 'failed'
        """
        # Priority 1: LiDAR Critical Zone
        if not self.lidar.is_path_clear(LIDAR_CRITICAL_THRESHOLD):
            self.logger.warning("EMERGENCY: LiDAR critical obstacle!")
            self.esp.stop()
            time.sleep(0.3)
            
            safe_dir = self.lidar.get_safe_direction()
            if safe_dir == 'left':
                self.esp.turn_left(TURN_SPEED)
            else:
                self.esp.turn_right(TURN_SPEED)
            
            return 'avoiding'
        
        # Priority 2: Vision Black Obstacles
        black_obstacles = objects.get('black_obstacles', [])
        critical_obstacles = [obs for obs in black_obstacles if obs['priority'] == 'critical']
        
        if critical_obstacles:
            obs = critical_obstacles[0]
            self.logger.warning(f"Vision: Black obstacle at {obs['distance_est']:.0f}cm")
            
            if obs['centroid'][0] < FRAME_CENTER_X:
                self.esp.turn_right(TURN_SPEED)
            else:
                self.esp.turn_left(TURN_SPEED)
            
            return 'avoiding'
        
        # Priority 3: LiDAR Warning Zone
        if not self.lidar.is_path_clear(LIDAR_WARNING_THRESHOLD):
            safe_dir = self.lidar.get_safe_direction()
            self.logger.debug(f"LiDAR warning: turning {safe_dir}")
            
            if safe_dir == 'left':
                self.esp.turn_left(TURN_SPEED // 2)
            elif safe_dir == 'right':
                self.esp.turn_right(TURN_SPEED // 2)
            
            return 'avoiding'
        
        # Priority 4: Navigate to target
        if target_object_key in objects:
            target = objects[target_object_key]
            
            steering_error = self.vision.calculate_steering_error(target['centroid'])
            distance_est = target['distance_est']
            
            self.logger.debug(f"Target: {target_object_key}, dist={distance_est:.0f}cm, error={steering_error}")
            
            # Check if reached
            if distance_est < DISTANCE_THRESHOLDS['very_close'] and abs(steering_error) < ALIGNED_THRESHOLD:
                self.esp.stop()
                self.logger.info(f"Reached {target_object_key}!")
                return 'reached'
            
            # Calculate motor speeds with PID
            steering_correction = self.steering_pid.update(steering_error)
            
            # Speed based on distance
            if distance_est > 150:
                base_speed = MAX_SPEED
            elif distance_est > 80:
                base_speed = APPROACH_SPEED
            else:
                base_speed = MIN_SPEED
            
            # Apply steering
            speed_left = int(base_speed - steering_correction)
            speed_right = int(base_speed + steering_correction)
            
            # Constrain speeds
            speed_left = constrain(speed_left, -MAX_SPEED, MAX_SPEED)
            speed_right = constrain(speed_right, -MAX_SPEED, MAX_SPEED)
            
            self.esp.send_command(speed_left, speed_right)
            
            return 'navigating'
        
        else:
            # Target not visible, search
            self.logger.debug(f"Searching for {target_object_key}...")
            self.esp.turn_right(SEARCH_ROTATION_SPEED)
            return 'searching'
    
    def handle_idle(self):
        """Idle state - waiting to start"""
        self.logger.info("Robot idle, press Enter to start mission...")
        input()
        self.transition_state(RobotState.GOTO_WATER_STATION)
    
    def handle_goto_water_station(self):
        """Navigate to water station (dark blue paper)"""
        frame = self.vision.capture_frame()
        if frame is None:
            return
        
        objects = self.vision.detect_all_objects(frame)
        
        if ENABLE_VISUALIZATION:
            self.vision.visualize(frame, objects)
        
        status = self.navigate_with_obstacle_avoidance('water_station', frame, objects)
        
        if status == 'reached':
            self.transition_state(RobotState.APPROACH_BOTTLE)
    
    def handle_approach_bottle(self):
        """Detect and approach bottle at water station"""
        frame = self.vision.capture_frame()
        if frame is None:
            return
        
        objects = self.vision.detect_all_objects(frame)
        
        if ENABLE_VISUALIZATION:
            self.vision.visualize(frame, objects)
        
        # Find available bottle based on priority
        for color in BOTTLE_PRIORITY:
            bottle_key = f"{color}_bottle"
            
            if color in self.delivered_colors:
                continue  # Already delivered
            
            if bottle_key in objects:
                self.current_bottle_color = color
                target = objects[bottle_key]
                
                steering_error = self.vision.calculate_steering_error(target['centroid'])
                distance_est = target['distance_est']
                
                self.logger.debug(f"Bottle {color}: dist={distance_est:.0f}cm, error={steering_error}")
                
                # Check if ready to grab
                if distance_est < 40 and abs(steering_error) < ALIGNED_THRESHOLD:
                    self.esp.stop()
                    self.logger.info(f"Ready to grab {color} bottle!")
                    self.transition_state(RobotState.GRAB_BOTTLE)
                    return
                
                # Approach slowly
                steering_correction = self.steering_pid.update(steering_error)
                
                speed_left = int(60 - steering_correction * 0.5)
                speed_right = int(60 + steering_correction * 0.5)
                
                self.esp.send_command(speed_left, speed_right)
                return
        
        # No bottle found
        self.logger.warning("No available bottles detected")
        self.esp.turn_right(SEARCH_ROTATION_SPEED)
    
    def handle_grab_bottle(self):
        """Execute grab sequence"""
        self.logger.info(f"Grabbing {self.current_bottle_color} bottle...")
        self.esp.grab_bottle(verbose=True)
        
        self.logger.info(f"{self.current_bottle_color.upper()} bottle grabbed successfully!")
        time.sleep(0.5)
        
        self.transition_state(RobotState.FIND_POT)
    
    def handle_find_pot(self):
        """Search and approach pot"""
        frame = self.vision.capture_frame()
        if frame is None:
            return
        
        objects = self.vision.detect_all_objects(frame)
        
        if ENABLE_VISUALIZATION:
            self.vision.visualize(frame, objects)
        
        pot = objects.get('pot')
        
        if pot:
            self.logger.debug(f"Pot detected: radius={pot['radius']}, circularity={pot['circularity']:.2f}")
            
            # Check if close enough to scan for watering zone
            if pot['radius'] > 50:  # Close enough
                self.logger.info("Pot found! Searching for watering zone...")
                
                # Find watering zone beside pot
                zone = self.vision.find_watering_zone_near_pot(
                    frame,
                    pot['centroid'],
                    pot['radius'],
                    self.current_bottle_color
                )
                
                if zone:
                    self.watering_zone_target = zone
                    self.logger.info(f"Watering zone found on {zone['side']} side of pot!")
                    self.transition_state(RobotState.GOTO_WATERING_ZONE)
                    return
                else:
                    # Move closer to pot
                    self.logger.debug("Watering zone not visible, moving closer...")
                    self.esp.forward(APPROACH_SPEED)
            else:
                # Approach pot
                steering_error = self.vision.calculate_steering_error(pot['centroid'])
                steering_correction = self.steering_pid.update(steering_error)
                
                speed_left = int(APPROACH_SPEED - steering_correction * 0.5)
                speed_right = int(APPROACH_SPEED + steering_correction * 0.5)
                
                self.esp.send_command(speed_left, speed_right)
        else:
            # Pot not visible, search
            self.logger.debug("Searching for pot...")
            self.esp.turn_right(SEARCH_ROTATION_SPEED)
    
    def handle_goto_watering_zone(self):
        """Navigate to watering zone colored floor"""
        frame = self.vision.capture_frame()
        if frame is None:
            return
        
        objects = self.vision.detect_all_objects(frame)
        
        if ENABLE_VISUALIZATION:
            self.vision.visualize(frame, objects)
        
        # Check obstacle avoidance first
        if not self.lidar.is_path_clear(LIDAR_WARNING_THRESHOLD):
            safe_dir = self.lidar.get_safe_direction()
            if safe_dir == 'left':
                self.esp.turn_left(TURN_SPEED // 2)
            elif safe_dir == 'right':
                self.esp.turn_right(TURN_SPEED // 2)
            return
        
        # Navigate based on stored watering zone target
        if self.watering_zone_target:
            # Simple approach: move toward remembered position
            # In production, continuously detect colored floor
            
            # For now, move forward slowly
            self.esp.forward(APPROACH_SPEED)
            
            # After some time, assume reached
            if self.state_timer.elapsed() > 5.0:
                self.logger.info("Arrived at watering zone")
                self.transition_state(RobotState.APPROACH_WATERING_ZONE)
    
    def handle_approach_watering_zone(self):
        """Final approach and positioning"""
        self.logger.info("Final positioning...")
        self.esp.forward(MIN_SPEED)
        time.sleep(1)
        self.esp.stop()
        
        self.transition_state(RobotState.RELEASE_BOTTLE)
    
    def handle_release_bottle(self):
        """Execute release sequence"""
        self.logger.info(f"Releasing {self.current_bottle_color} bottle at watering zone...")
        self.esp.release_bottle(verbose=True)
        
        self.delivered_colors.append(self.current_bottle_color)
        self.bottles_delivered += 1
        
        self.logger.info(f"{self.current_bottle_color.upper()} bottle delivered! ({self.bottles_delivered}/{MAX_BOTTLES})")
        
        # Back up a bit
        self.esp.backward(APPROACH_SPEED)
        time.sleep(1.5)
        self.esp.stop()
        
        self.transition_state(RobotState.CHECK_MORE_BOTTLES)
    
    def handle_check_more_bottles(self):
        """Check if more bottles need to be delivered"""
        self.logger.info(f"Progress: {self.bottles_delivered}/{MAX_BOTTLES} bottles delivered")
        self.logger.info(f"Delivered colors: {', '.join(self.delivered_colors)}")
        
        if self.bottles_delivered >= MAX_BOTTLES:
            self.logger.info("All bottles delivered! Returning to home base...")
            self.transition_state(RobotState.RETURN_HOME)
        else:
            remaining = [c for c in BOTTLE_PRIORITY if c not in self.delivered_colors]
            self.logger.info(f"Remaining bottles: {', '.join(remaining)}")
            self.logger.info("Returning to water station for next bottle...")
            
            time.sleep(1)
            self.transition_state(RobotState.GOTO_WATER_STATION)
    
    def handle_return_home(self):
        """Navigate back to home base (pink paper)"""
        frame = self.vision.capture_frame()
        if frame is None:
            return
        
        objects = self.vision.detect_all_objects(frame)
        
        if ENABLE_VISUALIZATION:
            self.vision.visualize(frame, objects)
        
        status = self.navigate_with_obstacle_avoidance('home_base', frame, objects)
        
        if status == 'reached':
            self.logger.info("HOME BASE REACHED!")
            self.transition_state(RobotState.DONE)
    
    def retry_or_fail(self):
        """Handle retry logic on timeout/failure"""
        self.retry_count += 1
        
        if self.retry_count >= self.max_retries:
            self.logger.error(f"Max retries ({self.max_retries}) exceeded in state {self.state.name}")
            self.transition_state(RobotState.ERROR)
        else:
            self.logger.warning(f"Retry {self.retry_count}/{self.max_retries} for state {self.state.name}")
            
            # Recovery action: back up and retry
            self.esp.stop()
            time.sleep(0.5)
            self.esp.backward(APPROACH_SPEED)
            time.sleep(1.5)
            self.esp.stop()
            
            # Reset timer
            self.state_timer.reset()
    
    def shutdown(self):
        """Clean shutdown of all subsystems"""
        self.logger.info("="*50)
        self.logger.info("SHUTTING DOWN ROBOT SYSTEM")
        self.logger.info("="*50)
        
        # Stop motors
        self.esp.stop()
        time.sleep(0.5)
        
        # Print statistics
        self.logger.info(f"\nMission Statistics:")
        self.logger.info(f"  Bottles Delivered: {self.bottles_delivered}/{MAX_BOTTLES}")
        self.logger.info(f"  Delivered Colors: {', '.join(self.delivered_colors) if self.delivered_colors else 'None'}")
        
        self.state_stats.end_state()
        self.state_stats.print_statistics()
        
        # Close connections
        self.esp.close()
        self.lidar.close()
        self.vision.close()
        
        self.logger.info("System shutdown complete")


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\nEmergency stop signal received!")
    sys.exit(0)


if __name__ == "__main__":
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("="*60)
    print("   AUTONOMOUS ROBOT - MULTI-TRIP WATERING MISSION")
    print("="*60)
    print("\nMission: Deliver 4 bottles from Water Station to Watering Zones")
    print("Press Ctrl+C at any time for emergency stop\n")
    
    # Parse arguments
    use_simulator = '--simulator' in sys.argv or '--sim' in sys.argv
    
    if use_simulator:
        print("[WARNING] Using LiDAR SIMULATOR mode\n")
    
    # Create and run robot
    robot = AutonomousRobot(use_lidar_simulator=use_simulator)
    robot.run()
