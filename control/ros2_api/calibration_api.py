#!/usr/bin/env python3
import math
import time
import rclpy

from ..commands.config_manager import ConfigManager
from .base_api import BaseApi
from .movement_api import MovementApi


class CalibrationApi(BaseApi):
    """ROS2 calibration API for robot movement patterns and testing.
    Provides predefined movement patterns for calibration purposes.
    """

    def __init__(self, movement_api: MovementApi, config_manager: ConfigManager):
        super().__init__("calibration_api", config_manager)
        self.movement = movement_api

    def run_square_pattern(self, side_length: float):
        """Move robot in a square pattern."""
        self.log_info(f"Starting square pattern with {side_length}m sides")

        for i in range(4):
            self.log_info(f"Square side {i + 1}/4")
            self.movement.move_dist(side_length)
            self.movement.turn_amount(math.pi / 2)

        self.log_info("Square pattern completed")

    def calibrate_circle(self, radius: float, angular_speed: float):
        circumference = 2 * math.pi * radius
        linear_speed = self.movement.linear
        total_time = circumference / linear_speed

        self.log_info(f"Starting circle calibration with {radius}m radius")
        self.movement.move_continuous(linear_speed, angular_speed)

        time.sleep(total_time)
        self.movement.stop()
        self.log_info("Circle calibration completed")

    def calibrate_figure_eight(self, radius: float):
        """Move robot in a figure-8 pattern for calibration."""
        self.log_info(f"Starting figure-8 calibration with {radius}m radius")

        # First circle clockwise
        self.calibrate_circle(radius, self.movement.angular)

        # Second circle counter-clockwise
        self.calibrate_circle(radius, -self.movement.angular)

        self.log_info("Figure-8 calibration completed")

    def test_movement_speeds(self):
        """Test different movement speeds for calibration."""
        test_distances = [0.1, 0.2, 0.5]
        test_angles = [math.pi / 4, math.pi / 2, math.pi]

        self.log_info("Starting movement speed tests")

        for distance in test_distances:
            self.log_info(f"Testing forward movement: {distance}m")
            self.movement.move_dist(distance)

        for angle in test_angles:
            self.log_info(f"Testing turn: {math.degrees(angle)} degrees")
            self.movement.turn_amount(angle)

        self.log_info("Movement speed tests completed")

    def run_rotate_stress(self):
        """Run continuous rotation stress test."""
        self.log_info("Starting rotation stress test - Press Ctrl+C to stop")

        # Get rotation speed from config
        slow_angular = self.config.get_variable("stress_test_rotation_speed")
        if slow_angular is None:
            slow_angular = 0.2  # Default fallback
        full_rotation = 2 * math.pi
        num_rotations = 10

        self.log_info(f"Using rotation speed: {slow_angular} rad/s")

        cycle = 0
        start_time = time.time()
        last_print_time = start_time

        try:
            while rclpy.ok():
                cycle += 1

                # Print elapsed time every 10 seconds
                current_time = time.time()
                if current_time - last_print_time >= 10:
                    elapsed = current_time - start_time
                    print(f"Elapsed time: {elapsed:.1f}s")
                    self.log_info(f"Elapsed time: {elapsed:.1f}s")
                    last_print_time = current_time

                self.log_info(f"Starting {num_rotations} rotations...")
                for rotation in range(num_rotations):
                    self.log_info(f"Rotation {rotation + 1}/{num_rotations}")
                    time_for_rotation = full_rotation / slow_angular
                    self.movement.cmd_vel_helper(0.0, slow_angular, time_for_rotation)

                self.log_info(f"Completed {num_rotations} rotations")

        except KeyboardInterrupt:
            self.log_info("Rotation stress test stopped by user")
            self.movement.stop()
            print("\nRotation stress test stopped")

        self.movement.stop()

    def run_circle_stress(self, diameter: float):
        """Run continuous circle stress test with specified diameter."""
        radius = diameter / 2.0
        self.log_info(f"Starting circle stress test - diameter {diameter}m (radius {radius}m) - Press Ctrl+C to stop")

        # Get speed from config
        linear_speed = self.config.get_variable("linear_speed")
        if linear_speed is None:
            linear_speed = 0.2  # Default fallback

        # Calculate angular speed for the circle
        # v = r * ω, so ω = v / r
        angular_speed = linear_speed / radius

        self.log_info(f"Using linear speed: {linear_speed} m/s, angular speed: {angular_speed:.3f} rad/s")

        cycle = 0
        start_time = time.time()
        last_print_time = start_time

        try:
            while rclpy.ok():
                cycle += 1

                # Print elapsed time every 10 seconds
                current_time = time.time()
                if current_time - last_print_time >= 10:
                    elapsed = current_time - start_time
                    print(f"Elapsed time: {elapsed:.1f}s")
                    self.log_info(f"Elapsed time: {elapsed:.1f}s")
                    last_print_time = current_time

                print(f"Cycle {cycle}: Starting circle...")
                self.log_info(f"Cycle {cycle}: Starting circle pattern")

                # Calculate time for one complete circle
                circumference = 2 * math.pi * radius
                circle_time = circumference / linear_speed

                # Execute circle movement
                self.movement.cmd_vel_helper(linear_speed, angular_speed, circle_time)

                self.log_info(f"Cycle {cycle}: Completed circle")

        except KeyboardInterrupt:
            self.log_info("Circle stress test stopped by user")
            self.movement.stop()
            print("\nCircle stress test stopped")

        self.movement.stop()
