"""
Utility Functions untuk Robot Otonom
Berisi PID controller, logging, dan helper functions
"""

import time
import os
import sys
from datetime import datetime
from config import *


class PIDController:
    """Simple PID controller untuk steering dan distance control"""
    
    def __init__(self, kp, ki=0.0, kd=0.0, output_limits=(-100, 100)):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: (min, max) tuple for output clamping
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        # State variables
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def update(self, error, dt=None):
        """
        Calculate PID output
        
        Args:
            error: Current error value (setpoint - measured)
            dt: Time delta (if None, auto-calculated)
        
        Returns:
            Control output
        """
        current_time = time.time()
        
        if dt is None:
            dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.01  # Prevent division by zero
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()


class Logger:
    """Custom logger with console colors and file output"""
    
    def __init__(self, name="RobotLog", log_to_file=LOG_TO_FILE):
        self.name = name
        self.log_to_file = log_to_file
        self.log_file_path = None
        
        if self.log_to_file:
            self._init_log_file()
    
    def _init_log_file(self):
        """Initialize log file"""
        try:
            # Create logs directory if not exists
            os.makedirs(LOG_DIR, exist_ok=True)
            
            # Create timestamped log file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{timestamp}_{LOG_FILE}"
            self.log_file_path = os.path.join(LOG_DIR, filename)
            
            # Write header
            with open(self.log_file_path, 'w') as f:
                f.write(f"=== Robot Autonomous Log ===\n")
                f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"="*50 + "\n\n")
            
            print(f"[LOG] Logging to: {self.log_file_path}")
        
        except Exception as e:
            print(f"[LOG] Warning: Could not create log file: {e}")
            self.log_to_file = False
    
    def _log(self, level, message):
        """Internal logging method"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_line = f"[{timestamp}] [{level:8s}] {message}"
        
        # Console output with colors
        color = LOG_COLORS.get(level, LOG_COLORS["RESET"])
        reset = LOG_COLORS["RESET"]
        print(f"{color}{log_line}{reset}")
        
        # File output
        if self.log_to_file and self.log_file_path:
            try:
                with open(self.log_file_path, 'a') as f:
                    f.write(log_line + "\n")
            except:
                pass
    
    def debug(self, message):
        """Debug level logging"""
        if ENABLE_VERBOSE_LOGGING:
            self._log("DEBUG", message)
    
    def info(self, message):
        """Info level logging"""
        self._log("INFO", message)
    
    def warning(self, message):
        """Warning level logging"""
        self._log("WARNING", message)
    
    def error(self, message):
        """Error level logging"""
        self._log("ERROR", message)
    
    def critical(self, message):
        """Critical level logging"""
        self._log("CRITICAL", message)


def constrain(value, min_val, max_val):
    """Constrain value to range [min_val, max_val]"""
    return max(min_val, min(max_val, value))


def map_range(value, in_min, in_max, out_min, out_max):
    """Map value from one range to another"""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def calculate_distance_2d(point1, point2):
    """Calculate Euclidean distance between two 2D points"""
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)**0.5


def calculate_angle(point1, point2):
    """
    Calculate angle from point1 to point2 in degrees
    0° = right, 90° = up, 180° = left, 270° = down
    """
    import math
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    angle = math.atan2(dy, dx) * 180 / math.pi
    return angle % 360


def moving_average(values, window_size=5):
    """Calculate moving average of list"""
    if len(values) < window_size:
        return sum(values) / len(values) if values else 0
    
    return sum(values[-window_size:]) / window_size


class Timer:
    """Simple timer for timeout detection"""
    
    def __init__(self, timeout_seconds):
        self.timeout = timeout_seconds
        self.start_time = time.time()
    
    def reset(self):
        """Reset timer"""
        self.start_time = time.time()
    
    def elapsed(self):
        """Get elapsed time in seconds"""
        return time.time() - self.start_time
    
    def is_timeout(self):
        """Check if timeout reached"""
        return self.elapsed() >= self.timeout
    
    def remaining(self):
        """Get remaining time in seconds"""
        return max(0, self.timeout - self.elapsed())


class StateTimer:
    """Timer for tracking state durations"""
    
    def __init__(self):
        self.state_times = {}
        self.current_state = None
        self.state_start_time = None
    
    def start_state(self, state_name):
        """Start timing a state"""
        if self.current_state:
            self.end_state()
        
        self.current_state = state_name
        self.state_start_time = time.time()
    
    def end_state(self):
        """End current state timing"""
        if self.current_state and self.state_start_time:
            duration = time.time() - self.state_start_time
            
            if self.current_state not in self.state_times:
                self.state_times[self.current_state] = []
            
            self.state_times[self.current_state].append(duration)
        
        self.current_state = None
        self.state_start_time = None
    
    def get_statistics(self):
        """Get timing statistics"""
        stats = {}
        for state, times in self.state_times.items():
            if times:
                stats[state] = {
                    'count': len(times),
                    'total': sum(times),
                    'average': sum(times) / len(times),
                    'min': min(times),
                    'max': max(times)
                }
        return stats
    
    def print_statistics(self):
        """Print timing statistics"""
        print("\n=== State Timing Statistics ===")
        stats = self.get_statistics()
        
        for state, data in stats.items():
            print(f"\n{state}:")
            print(f"  Count: {data['count']}")
            print(f"  Total: {data['total']:.2f}s")
            print(f"  Average: {data['average']:.2f}s")
            print(f"  Min: {data['min']:.2f}s")
            print(f"  Max: {data['max']:.2f}s")


class RateLimiter:
    """Limit execution rate of a function"""
    
    def __init__(self, max_rate_hz):
        """
        Args:
            max_rate_hz: Maximum execution rate in Hz
        """
        self.min_interval = 1.0 / max_rate_hz
        self.last_execution = 0
    
    def can_execute(self):
        """Check if enough time has passed"""
        now = time.time()
        if now - self.last_execution >= self.min_interval:
            self.last_execution = now
            return True
        return False
    
    def wait_if_needed(self):
        """Block until can execute"""
        now = time.time()
        elapsed = now - self.last_execution
        if elapsed < self.min_interval:
            time.sleep(self.min_interval - elapsed)
        self.last_execution = time.time()


def is_aligned(error, threshold=ALIGNED_THRESHOLD):
    """Check if steering error is within alignment threshold"""
    return abs(error) < threshold


def is_in_deadzone(error, deadzone=STEERING_DEADZONE):
    """Check if error is in deadzone (no correction needed)"""
    return abs(error) < deadzone


def format_time(seconds):
    """Format seconds to readable time string"""
    if seconds < 60:
        return f"{seconds:.1f}s"
    elif seconds < 3600:
        minutes = int(seconds // 60)
        secs = seconds % 60
        return f"{minutes}m {secs:.0f}s"
    else:
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        return f"{hours}h {minutes}m"


def print_progress_bar(iteration, total, prefix='', suffix='', length=50):
    """Print progress bar to console"""
    percent = 100 * (iteration / float(total))
    filled_length = int(length * iteration // total)
    bar = '█' * filled_length + '-' * (length - filled_length)
    
    sys.stdout.write(f'\r{prefix} |{bar}| {percent:.1f}% {suffix}')
    sys.stdout.flush()
    
    if iteration == total:
        print()


class StuckDetector:
    """Detect if robot is stuck (not making progress)"""
    
    def __init__(self, window_size=10, threshold=20):
        """
        Args:
            window_size: Number of position samples to track
            threshold: Maximum allowed movement (pixels) to consider stuck
        """
        self.window_size = window_size
        self.threshold = threshold
        self.positions = []
    
    def update(self, position):
        """
        Update with new position (x, y)
        
        Returns:
            True if stuck, False otherwise
        """
        self.positions.append(position)
        
        if len(self.positions) > self.window_size:
            self.positions.pop(0)
        
        if len(self.positions) < self.window_size:
            return False
        
        # Calculate total movement
        total_movement = 0
        for i in range(1, len(self.positions)):
            movement = calculate_distance_2d(self.positions[i-1], self.positions[i])
            total_movement += movement
        
        # If total movement is very small, robot is stuck
        return total_movement < self.threshold
    
    def reset(self):
        """Reset stuck detector"""
        self.positions.clear()


def safe_division(numerator, denominator, default=0):
    """Safe division with default value for zero denominator"""
    return numerator / denominator if denominator != 0 else default


def exponential_smoothing(new_value, old_value, alpha=0.3):
    """
    Exponential smoothing filter
    
    Args:
        new_value: New measurement
        old_value: Previous smoothed value
        alpha: Smoothing factor (0-1, higher = less smoothing)
    
    Returns:
        Smoothed value
    """
    return alpha * new_value + (1 - alpha) * old_value


# Test functions
if __name__ == "__main__":
    print("=== Utils Module Test ===\n")
    
    # Test PID controller
    print("Test 1: PID Controller")
    pid = PIDController(kp=0.5, ki=0.0, kd=0.1)
    
    for i in range(10):
        error = 50 - i * 5  # Decreasing error
        output = pid.update(error, dt=0.1)
        print(f"  Error: {error:5.1f} -> Output: {output:6.2f}")
    
    # Test Logger
    print("\nTest 2: Logger")
    logger = Logger("TestLog", log_to_file=False)
    logger.debug("This is a debug message")
    logger.info("This is an info message")
    logger.warning("This is a warning message")
    logger.error("This is an error message")
    
    # Test Timer
    print("\nTest 3: Timer")
    timer = Timer(2.0)
    print(f"  Timer started, waiting 1s...")
    time.sleep(1)
    print(f"  Elapsed: {timer.elapsed():.2f}s, Timeout: {timer.is_timeout()}")
    time.sleep(1.1)
    print(f"  Elapsed: {timer.elapsed():.2f}s, Timeout: {timer.is_timeout()}")
    
    # Test StuckDetector
    print("\nTest 4: StuckDetector")
    stuck = StuckDetector(window_size=5, threshold=10)
    
    # Simulate moving robot
    for i in range(8):
        pos = (i * 5, i * 5)  # Moving diagonally
        is_stuck = stuck.update(pos)
        print(f"  Position: {pos}, Stuck: {is_stuck}")
    
    # Simulate stuck robot
    for i in range(5):
        pos = (40, 40)  # Not moving
        is_stuck = stuck.update(pos)
        print(f"  Position: {pos}, Stuck: {is_stuck}")
    
    print("\n=== All tests complete ===")
