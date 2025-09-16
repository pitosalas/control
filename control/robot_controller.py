#!/usr/bin/env python3
from typing import Any, Dict, Optional
from dataclasses import dataclass
from .movement_api import MovementApi
from .calibration_api import CalibrationApi
from .process_api import ProcessApi
from .config_manager import ConfigManager

@dataclass
class CommandResponse:
    success: bool
    message: str
    data: Any = None

class RobotController:
    """
    Business logic orchestration layer for robot operations.
    Coordinates multiple APIs and provides unified interface for CLI and REST.
    """

    def __init__(self, config_manager: ConfigManager):
        self.config = config_manager
        self.movement = MovementApi(self.config)
        self.calibration = CalibrationApi(self.movement, self.config)
        self.process = ProcessApi(self.config)

        # Track singleton processes
        self.nav_stack_process_id = None

    def move_distance(self, distance: float) -> CommandResponse:
        self.movement.move_dist(distance)
        return CommandResponse(True, f"Moved {distance} meters")

    def move_for_time(self, seconds: float) -> CommandResponse:
        self.movement.move_time(seconds)
        return CommandResponse(True, f"Moved for {seconds} seconds")

    def turn_by_radians(self, radians: float) -> CommandResponse:
        self.movement.turn_amount(radians)
        return CommandResponse(True, f"Turned {radians} radians")

    def turn_by_degrees(self, degrees: float) -> CommandResponse:
        self.movement.turn_degrees(degrees)
        return CommandResponse(True, f"Turned {degrees} degrees")

    def turn_for_time(self, seconds: float) -> CommandResponse:
        self.movement.turn_time(seconds)
        return CommandResponse(True, f"Turned for {seconds} seconds")

    def stop_robot(self) -> CommandResponse:
        self.movement.stop()
        return CommandResponse(True, "Robot stopped")

    def calibrate_square(self, meters: float) -> CommandResponse:
        self.calibration.calibrate_square(meters)
        return CommandResponse(True, f"Completed square calibration with {meters}m sides")

    def set_variable(self, name: str, value: str) -> CommandResponse:
        self.config.set_variable(name, value)
        stored_value = self.config.get_variable(name)
        return CommandResponse(True, f"Set {name} = {stored_value} ({type(stored_value).__name__})")

    def get_variable(self, name: str) -> CommandResponse:
        value = self.config.get_variable(name)
        return CommandResponse(True, f"{name} = {value} ({type(value).__name__})", {"value": value})

    def get_all_variables(self) -> CommandResponse:
        variables = self.config.get_all_variables()
        return CommandResponse(True, "Configuration variables", {"variables": variables})

    def get_topics(self) -> CommandResponse:
        topics = self.movement.get_topics()
        topic_list = []
        for topic_name, topic_types in topics:
            type_names = [t for t in topic_types]
            topic_list.append({"name": topic_name, "types": type_names})
        return CommandResponse(True, "Active ROS topics", {"topics": topic_list})

    def get_robot_status(self) -> CommandResponse:
        status = self.movement.get_status()
        return CommandResponse(True, "Robot status", {"status": status})

    def start_navigation_stack(self, use_sim_time: bool = False, **kwargs) -> CommandResponse:
        # Kill existing nav stack if running
        if self.nav_stack_process_id and self.process.is_process_running(self.nav_stack_process_id):
            self.process.kill_process(self.nav_stack_process_id)

        self.nav_stack_process_id = self.process.start_navigation(use_sim_time, **kwargs)
        return CommandResponse(True, f"Started navigation stack", {"process_id": self.nav_stack_process_id})

    def save_current_map(self, filename: str) -> CommandResponse:
        process_id = self.process.save_map(filename)
        return CommandResponse(True, f"Saving map to {filename}", {"process_id": process_id})

    def start_slam(self, use_sim_time: bool = False, **kwargs) -> CommandResponse:
        process_id = self.process.start_slam(use_sim_time, **kwargs)
        return CommandResponse(True, f"Started SLAM", {"process_id": process_id})

    def launch_file(self, package: str, launch_file: str, **kwargs) -> CommandResponse:
        process_id = self.process.launch_file(package, launch_file, **kwargs)
        return CommandResponse(True, f"Launched {package}/{launch_file}", {"process_id": process_id})

    def run_ros_node(self, package: str, executable: str, **kwargs) -> CommandResponse:
        process_id = self.process.run_ros_node(package, executable, **kwargs)
        return CommandResponse(True, f"Started {package}/{executable}", {"process_id": process_id})

    def launch_command(self, command: str) -> CommandResponse:
        process_id = self.process.launch_command(command)
        return CommandResponse(True, f"Launched command: {command}", {"process_id": process_id})

    def kill_process(self, process_id: str) -> CommandResponse:
        """Kill managed process"""
        success = self.process.kill_process(process_id)
        if success:
            # Clear nav stack ID if it was killed
            if process_id == self.nav_stack_process_id:
                self.nav_stack_process_id = None
            return CommandResponse(True, f"Killed process {process_id}")
        return CommandResponse(False, f"Failed to kill process {process_id}")

    def kill_navigation_stack(self) -> CommandResponse:
        if not self.nav_stack_process_id:
            return CommandResponse(False, "No navigation stack running")

        if not self.process.is_process_running(self.nav_stack_process_id):
            self.nav_stack_process_id = None
            return CommandResponse(False, "Navigation stack not running")

        success = self.process.kill_process(self.nav_stack_process_id)
        if success:
            self.nav_stack_process_id = None
            return CommandResponse(True, "Navigation stack stopped")
        return CommandResponse(False, "Failed to stop navigation stack")

    def get_active_processes(self) -> CommandResponse:
        processes = self.process.get_running_processes()
        return CommandResponse(True, "Active processes", {"processes": processes})

    def get_process_output(self, process_id: str, lines: Optional[int] = None) -> CommandResponse:
        output = self.process.get_process_output(process_id, lines)
        return CommandResponse(True, f"Output for process {process_id}", {"output": output})

    def is_process_running(self, process_id: str) -> CommandResponse:
        running = self.process.is_process_running(process_id)
        status = "running" if running else "stopped"
        return CommandResponse(True, f"Process {process_id} is {status}", {"running": running})

    def set_robot_speeds(self, linear: float, angular: float) -> CommandResponse:
        self.movement.set_linear_speed(linear)
        self.movement.set_angular_speed(angular)
        return CommandResponse(True, f"Set speeds: linear={linear}, angular={angular}")

    def move_continuous(self, linear: float, angular: float) -> CommandResponse:
        self.movement.move_continuous(linear, angular)
        return CommandResponse(True, f"Continuous movement: linear={linear}, angular={angular}")

    def get_current_position(self) -> CommandResponse:
        position = self.movement.get_current_position()
        if position:
            x, y = position
            return CommandResponse(True, f"Position: x={x:.3f}, y={y:.3f}", {"x": x, "y": y})
        return CommandResponse(False, "No position data available")

    def save_config(self):
        """Save configuration to file"""
        self.config.save_config()

    def cleanup(self):
        """Clean up all resources"""
        self.process.destroy_node()
        self.movement.destroy_node()
        if hasattr(self.calibration, 'destroy_node'):
            self.calibration.destroy_node()