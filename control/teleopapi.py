#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from roscli.msg import Robogym  # Commented out to remove message dependency
from nav_msgs.msg import Odometry
import time
import math

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

class TeleopApi(Node):
    """
    ROS2 teleoperation API for robot movement control.
    Provides safe velocity commands with automatic stopping.
    """
    def __init__(self):
        """Initialize teleop API node and publisher."""
        rclpy.init()
        super().__init__('teleop_api')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        # self.robogym_pub = self.create_publisher(Robogym, 'cli', 1)  # Commented out to remove message dependency
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.get_logger().info("TeleopApi node initialized")
        self.linear_min = -0.5
        self.linear_max = 0.5
        self.angular_min = -1.0
        self.angular_max = 1.0
        self.linear:float  = 0.3
        self.angular:float  = 0.4
        self.current_pose = None
    
    # Core Movement Methods
    def move_dist(self, distance):
        """Move robot forward a specified distance."""
        seconds = abs(distance) / self.linear
        actual_speed = self.linear if distance >= 0 else -self.linear
        self.cmd_vel_helper(actual_speed, 0.0, seconds)
        
    def move_time(self, seconds):
        """Move robot forward at default speed for specified duration."""
        self.cmd_vel_helper(self.linear, 0.0, seconds)

    # Core Turning Methods
    def turn_amount(self, angle: float):
        """Turn robot in place by a given angle in radians."""
        if self.angular == 0: 
            self.get_logger().warn("Angular speed is zero, cannot turn.")
            return
        seconds = abs(angle) / self.angular
        angular_speed = self.angular if angle >= 0 else -self.angular
        self.cmd_vel_helper(0.0, angular_speed, seconds)
        
    def turn_time(self, seconds:float):
        """Turn robot at a specified speed for a given duration."""
        self.cmd_vel_helper(0.0, self.angular, seconds)

    # Control Methods
    def stop(self):
        """Stop the robot immediately."""
        self.cmd_vel_helper(0.0, 0.0, 0.0)

    def move_continuous(self, linear, angular):
        """Send continuous velocity commands without auto-stop."""
        if not self.check_limits(linear, angular):
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    # Settings Methods
    def set_linear_speed(self, speed):
        """Set default linear velocity with safety check."""
        if self.check_limits(speed, self.angular):
            self.linear = speed

    def set_angular_speed(self, speed):
        """Set default angular velocity with safety check."""
        if self.check_limits(self.linear, speed):
            self.angular = speed

    # Status Methods
    def get_status(self):
        """Return current speed settings and limits."""
        return {
            'linear': self.linear,
            'angular': self.angular,
            'linear_limits': [self.linear_min, self.linear_max],
            'angular_limits': [self.angular_min, self.angular_max]
        }

    # Helper Methods
    def check_limits(self, linear, angular):
        """Check if speeds are within safe limits."""
        if not (self.linear_min <= linear <= self.linear_max and self.angular_min <= angular <= self.angular_max):
            self.get_logger().warn(f"Speed out of bounds: linear={linear}, angular={angular}. Linear must be [{self.linear_min}, {self.linear_max}], angular must be [{self.angular_min}, {self.angular_max}]")
            return False
        return True

    def cmd_vel_helper(self, linear, angular, seconds):
        """Send velocity commands for specified duration with safety limits."""
        if not self.check_limits(linear, angular):
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start_time = time.time()
        rate_hz = 10
        sleep_duration = 1.0 / rate_hz
        
        while rclpy.ok() and (time.time() - start_time) < seconds:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=sleep_duration)
            time.sleep(sleep_duration)
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def send_robogym_command(self, command, params):
        """Send robogym command with parameters. (Disabled - message dependency removed)"""
        self.get_logger().info(f"Robogym command disabled: {command} with params: {params}")
        # Legacy robogym functionality disabled to remove custom message dependency
        pass

    def odom_callback(self, msg):
        """Callback for odometry updates."""
        self.current_pose = msg.pose.pose

    def send_cmd_vel(self, linear, angular):
        """Send velocity command directly."""
        if not self.check_limits(linear, angular):
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def get_current_position(self):
        """Get current x, y position from odometry."""
        if self.current_pose is None:
            return None
        return (self.current_pose.position.x, self.current_pose.position.y)

    def navigate_to_point(self, target_x, target_y, tolerance=0.1):
        """Navigate to a specific point using odometry."""
        rate_hz = 10
        sleep_duration = 1.0 / rate_hz
        
        while rclpy.ok():
            current_pos = self.get_current_position()
            if current_pos is None:
                self.get_logger().warn("No odometry data available")
                time.sleep(sleep_duration)
                continue
                
            current_x, current_y = current_pos
            
            # Calculate distance to target
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Check if we've arrived
            if distance < tolerance:
                self.send_cmd_vel(0.0, 0.0)
                return True
                
            # Calculate angle to target
            target_angle = math.atan2(dy, dx)
            
            # Get current orientation (yaw)
            orientation = self.current_pose.orientation
            current_yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
            )
            
            # Calculate angle difference
            angle_diff = target_angle - current_yaw
            # Normalize angle to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
                
            # Simple proportional controller
            linear_vel = min(self.linear, distance * 2.0)  # Slow down as we approach
            angular_vel = angle_diff * 2.0  # Proportional to angle error
            
            # Clamp velocities
            linear_vel = max(min(linear_vel, self.linear_max), self.linear_min)
            angular_vel = max(min(angular_vel, self.angular_max), self.angular_min)
            
            self.send_cmd_vel(linear_vel, angular_vel)
            rclpy.spin_once(self, timeout_sec=sleep_duration)
            time.sleep(sleep_duration)
            
        return False

    def destroy_node(self):
        """Clean up resources and shutdown node."""
        self.get_logger().info("Shutting down TeleopApi node")
        super().destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    toap = TeleopApi()
    toap.destroy_node()
    rclpy.shutdown()
