#!/usr/bin/env python3
"""
ROS2 calibration API for robot movement patterns and testing
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

import math
import time

import rclpy

import control.commands.config_manager as cm
import control.ros2_api.base_api as base
import control.ros2_api.movement_api as movement


class CalibrationApi(base.BaseApi):
    """ROS2 calibration API for robot movement patterns and testing.
    Provides predefined movement patterns for calibration purposes.
    """

    def __init__(self, movement_api: movement.MovementApi, config_manager: cm.ConfigManager):
        super().__init__("calibration_api", config_manager)
        self.movement = movement_api

    def _print_elapsed_time(self, current_time, start_time, last_print_time):
        if current_time - last_print_time >= 10:
            elapsed = current_time - start_time
            print(f"Elapsed time: {elapsed:.1f}s")
            self.log_info(f"Elapsed time: {elapsed:.1f}s")
            return current_time
        return last_print_time

    def run_square_pattern(self, side_length: float):
        self.log_info(f"Starting square pattern with {side_length}m sides")

        for i in range(4):
            self.log_info(f"Square side {i + 1}/4")
            self.movement.move_dist(side_length)
            self.movement.turn_amount(math.pi / 2)

        self.log_info("Square pattern completed")

    def run_rotate_stress(self):
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

                current_time = time.time()
                last_print_time = self._print_elapsed_time(current_time, start_time, last_print_time)

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
        radius = diameter / 2.0
        self.log_info(
            f"Starting circle stress test - diameter {diameter}m (radius {radius}m) - Press Ctrl+C to stop"
        )

        # Get speed from config
        linear_speed = self.config.get_variable("linear_speed")
        angular_speed = self.config.get_variable("angular_speed")

        self.log_info(
            f"Using linear speed: {linear_speed} m/s, angular speed: {angular_speed:.3f} rad/s"
        )

        cycle = 0
        start_time = time.time()
        last_print_time = start_time

        try:
            while rclpy.ok():
                cycle += 1

                current_time = time.time()
                last_print_time = self._print_elapsed_time(current_time, start_time, last_print_time)

                # Calculate time for one complete circle
                circumference = 2 * math.pi * radius
                circle_time = circumference / linear_speed

                # Execute circle movement
                self.movement.cmd_vel_helper(linear_speed, angular_speed, circle_time)

        except KeyboardInterrupt:
            self.log_info("Circle stress test stopped by user")
            self.movement.stop()
            print("\nCircle stress test stopped")

        self.movement.stop()
