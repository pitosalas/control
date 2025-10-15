#!/usr/bin/env python3
import math
import time

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

    def calibrate_square(self, side_length: float):
        """Move robot in a square pattern for calibration."""
        self.log_info(f"Starting square calibration with {side_length}m sides")

        for i in range(4):
            self.log_info(f"Square side {i + 1}/4")
            self.movement.move_dist(side_length)
            self.movement.turn_amount(math.pi / 2)

        self.log_info("Square calibration completed")

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
