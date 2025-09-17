#!/usr/bin/env python3
import math
from .movement_api import MovementApi
from .calibration_api import CalibrationApi
from .config_manager import ConfigManager

"""
TeleopApi Entry Points:
Core Movement:
- move_dist(distance) - Move forward by specified distance in meters
- move_time(seconds) - Move forward at default speed for specified duration
Core Turning:
- turn_amount(angle) - Turn in place by specified angle in radians
- turn_time(seconds) - Turn at default speed for specified duration
Control:
- stop() - Stop robot immediately with zero velocity
- move_continuous(linear, angular) - Send velocity commands without auto-stop
Settings:
- set_linear_speed(speed) - Set default linear velocity with safety check
- set_angular_speed(speed) - Set default angular velocity with safety check
Status:
- get_status() - Return current speed settings and limits as dictionary
Helpers:
- check_limits(linear, angular) - Validate speeds are within safety bounds
- cmd_vel_helper(linear, angular, seconds) - Send velocity commands for duration
"""

class TeleopApi:
    """
    High-level teleoperation API that orchestrates movement and calibration.
    Provides a unified interface for robot control operations.
    """
    def __init__(self, config_manager: ConfigManager = None):
        """Initialize teleop API with movement and calibration APIs."""
        self.config = config_manager or ConfigManager()
        self.movement = MovementApi(self.config)
        self.calibration = CalibrationApi(self.movement, self.config)
    
    # Core Movement Methods - Delegate to MovementApi
    def move_dist(self, distance):
        """Move robot forward a specified distance."""
        return self.movement.move_dist(distance)

    def move_time(self, seconds):
        """Move robot forward at default speed for specified duration."""
        return self.movement.move_time(seconds)

    # Core Turning Methods - Delegate to MovementApi
    def turn_amount(self, angle: float):
        """Turn robot in place by a given angle in radians."""
        return self.movement.turn_amount(angle)

    def turn_degrees(self, degrees: float):
        """Turn robot in place by a given angle in degrees."""
        return self.movement.turn_degrees(degrees)

    def turn_time(self, seconds: float):
        """Turn robot at a specified speed for a given duration."""
        return self.movement.turn_time(seconds)

    # Calibration Methods - Delegate to CalibrationApi
    def calibrate_square(self, side_length: float):
        """Move robot in a square pattern for calibration."""
        return self.calibration.calibrate_square(side_length)

    # Control Methods - Delegate to MovementApi
    def stop(self):
        """Stop the robot immediately."""
        return self.movement.stop()

    def move_continuous(self, linear, angular):
        """Send continuous velocity commands without auto-stop."""
        return self.movement.move_continuous(linear, angular)

    # Settings Methods - Delegate to MovementApi
    def set_linear_speed(self, speed):
        """Set default linear velocity with safety check."""
        return self.movement.set_linear_speed(speed)

    def set_angular_speed(self, speed):
        """Set default angular velocity with safety check."""
        return self.movement.set_angular_speed(speed)

    # Status Methods - Delegate to MovementApi
    def get_status(self):
        """Return current speed settings and limits."""
        return self.movement.get_status()

    def get_topics(self):
        """Get list of currently published topics."""
        return self.movement.get_topics()

    # Additional helper methods - Delegate to MovementApi
    def send_cmd_vel(self, linear, angular):
        """Send velocity command directly."""
        return self.movement.send_cmd_vel(linear, angular)

    def get_current_position(self):
        """Get current x, y position from odometry."""
        return self.movement.get_current_position()

    def destroy_node(self):
        """Clean up resources and shutdown nodes."""
        self.movement.destroy_node()
        if hasattr(self.calibration, 'destroy_node'):
            self.calibration.destroy_node()

if __name__ == '__main__':
    toap = TeleopApi()
    toap.destroy_node()
