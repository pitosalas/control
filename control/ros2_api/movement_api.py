#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

from ..commands.config_manager import ConfigManager
from .base_api import BaseApi


class MovementApi(BaseApi):
    """ROS2 movement API for robot velocity control and odometry.
    Provides safe velocity commands with automatic stopping.
    """

    def __init__(self, config_manager: ConfigManager = None):
        super().__init__("movement_api", config_manager)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )
        self.battery_sub = self.create_subscription(
            BatteryState,
            "/battery_state",
            self.battery_callback,
            10,
        )

        self.linear_min = -0.5
        self.linear_max = 0.5
        self.angular_min = -1.0
        self.angular_max = 1.0

        self.linear = self.config.get_variable("linear_speed")
        self.angular = self.config.get_variable("angular_speed")
        self.current_pose = None
        self.current_voltage = None

    def move_dist(self, distance: float):
        seconds = abs(distance) / self.linear
        actual_speed = self.linear if distance >= 0 else -self.linear
        self.cmd_vel_helper(actual_speed, 0.0, seconds)

    def move_time(self, seconds: float):
        self.cmd_vel_helper(self.linear, 0.0, seconds)

    def turn_amount(self, angle: float):
        if self.angular == 0:
            self.log_warn("Angular speed is zero, cannot turn.")
            return
        seconds = abs(angle) / self.angular
        angular_speed = self.angular if angle >= 0 else -self.angular
        self.cmd_vel_helper(0.0, angular_speed, seconds)

    def turn_degrees(self, degrees: float):
        """Turn robot in place by a given angle in degrees."""
        radians = math.radians(degrees)
        self.turn_amount(radians)

    def turn_time(self, seconds: float):
        """Turn robot at a specified speed for a given duration."""
        self.cmd_vel_helper(0.0, self.angular, seconds)

    def stop(self):
        """Stop the robot immediately."""
        self.cmd_vel_helper(0.0, 0.0, 0.0)

    def move_continuous(self, linear: float, angular: float):
        """Send continuous velocity commands without auto-stop."""
        if not self.check_velocity_limits(linear, angular):
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def send_cmd_vel(self, linear: float, angular: float):
        """Send velocity command directly."""
        if not self.check_velocity_limits(linear, angular):
            return
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def set_linear_speed(self, speed: float):
        """Set default linear velocity with safety check."""
        if self.check_velocity_limits(speed, self.angular):
            self.linear = speed

    def set_angular_speed(self, speed: float):
        """Set default angular velocity with safety check."""
        if self.check_velocity_limits(self.linear, speed):
            self.angular = speed

    def get_status(self):
        """Return current speed settings and limits."""
        return {
            "linear": self.linear,
            "angular": self.angular,
            "linear_limits": [self.linear_min, self.linear_max],
            "angular_limits": [self.angular_min, self.angular_max],
        }

    def get_current_position(self):
        """Get current x, y position from odometry."""
        if self.current_pose is None:
            return None
        return (self.current_pose.position.x, self.current_pose.position.y)

    def check_velocity_limits(self, linear: float, angular: float) -> bool:
        """Check if speeds are within safe limits."""
        linear_ok = self.check_bounds(
            linear,
            self.linear_min,
            self.linear_max,
            "linear velocity",
        )
        angular_ok = self.check_bounds(
            angular,
            self.angular_min,
            self.angular_max,
            "angular velocity",
        )
        return linear_ok and angular_ok

    def cmd_vel_helper(self, linear: float, angular: float, seconds: float):
        """Send velocity commands for specified duration with safety limits."""
        if not self.check_velocity_limits(linear, angular):
            return

        if self.config.is_dry_run():
            self.log_info(f"DRY RUN: Would move linear={linear}, angular={angular} for {seconds}s")
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

    def odom_callback(self, msg):
        """Callback for odometry updates."""
        self.current_pose = msg.pose.pose

    def battery_callback(self, msg):
        """Callback for battery state updates."""
        self.current_voltage = msg.voltage

    def get_voltage(self):
        """Get current battery voltage."""
        return self.current_voltage
