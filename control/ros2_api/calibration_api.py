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

    def run_stress_test(self):
        """Run continuous stress test: rotate in place, measure voltage between rotations."""
        self.log_info("Starting stress test - Press Ctrl+C to stop")

        slow_angular = 0.2
        full_rotation = 2 * math.pi
        num_rotations = 10

        cycle = 0
        try:
            while rclpy.ok():
                cycle += 1

                voltage = self.movement.get_voltage()
                if voltage is not None:
                    print(f"Cycle {cycle}: Battery voltage = {voltage:.2f}V")
                    self.log_info(f"Cycle {cycle}: Battery voltage = {voltage:.2f}V")
                else:
                    print(f"Cycle {cycle}: Battery voltage = N/A (no data)")
                    self.log_info(f"Cycle {cycle}: Battery voltage unavailable")

                self.log_info(f"Starting {num_rotations} rotations...")
                for rotation in range(num_rotations):
                    self.log_info(f"Rotation {rotation + 1}/{num_rotations}")
                    time_for_rotation = full_rotation / slow_angular
                    self.movement.cmd_vel_helper(0.0, slow_angular, time_for_rotation)

                self.log_info(f"Completed {num_rotations} rotations")

        except KeyboardInterrupt:
            self.log_info("Stress test stopped by user")
            self.movement.stop()
            print("\nStress test stopped")

        self.movement.stop()
