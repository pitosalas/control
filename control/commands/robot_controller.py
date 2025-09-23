#!/usr/bin/env python3
from typing import Any, Dict, Optional
from dataclasses import dataclass
from ..ros2_api.movement_api import MovementApi
from ..ros2_api.calibration_api import CalibrationApi
from ..ros2_api.process_api import ProcessApi
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

        # Track singleton launch processes
        self.launch_process_ids = {
            "nav": None,
            "slam": None,
            "map_server": None
        }

    def _is_launch_running(self, launch_type: str) -> bool:
        """Check if a specific launch type is running"""
        process_id = self.launch_process_ids.get(launch_type)
        if not process_id:
            return False
        return self.process.is_process_running(process_id)

    def _check_launch_conflict(self, launch_type: str) -> Optional[CommandResponse]:
        """Check if launch type is already running and return error response if so"""
        if self._is_launch_running(launch_type):
            return CommandResponse(False, f"{launch_type} is already running, stop it first")
        return None

    def _start_launch(self, launch_type: str, **params) -> CommandResponse:
        """Start a launch process with conflict checking"""
        conflict = self._check_launch_conflict(launch_type)
        if conflict:
            return conflict

        try:
            # Start new launch process using ProcessApi
            process_id = self.process.launch_by_type(launch_type, **params)
            self.launch_process_ids[launch_type] = process_id
            return CommandResponse(True, f"Started {launch_type}", {"process_id": process_id})
        except ValueError as e:
            return CommandResponse(False, str(e))

    def _stop_launch(self, launch_type: str) -> CommandResponse:
        """Stop a specific launch type"""
        process_id = self.launch_process_ids.get(launch_type)
        if not process_id:
            return CommandResponse(False, f"No {launch_type} running")

        if not self.process.is_process_running(process_id):
            self.launch_process_ids[launch_type] = None
            return CommandResponse(False, f"{launch_type} not running")

        success = self.process.kill_by_type(launch_type, process_id)
        if success:
            self.launch_process_ids[launch_type] = None
            return CommandResponse(True, f"{launch_type} stopped")
        return CommandResponse(False, f"Failed to stop {launch_type}")

    def launch_list(self) -> CommandResponse:
        """List all available launch types with descriptions"""
        launch_info = []
        for launch_type in self.process.get_available_launch_types():
            config = self.process.get_launch_config(launch_type)
            is_running = self._is_launch_running(launch_type)
            status = "RUNNING" if is_running else "STOPPED"
            launch_info.append(f"{launch_type:<12} {status:<8} {config.description}")

        if launch_info:
            header = f"{'TYPE':<12} {'STATUS':<8} DESCRIPTION"
            separator = "-" * 50
            formatted_output = f"{header}\n{separator}\n" + "\n".join(launch_info)
            return CommandResponse(True, formatted_output)
        else:
            return CommandResponse(True, "No launch types available")

    def launch_start(self, launch_type: str, **kwargs) -> CommandResponse:
        """Start a launch process by type"""
        return self._start_launch(launch_type, **kwargs)

    def launch_kill(self, launch_type: str) -> CommandResponse:
        """Stop a launch process by type"""
        return self._stop_launch(launch_type)

    def launch_status(self, launch_type: str = None) -> CommandResponse:
        """Get status of launch processes"""
        if launch_type:
            # Status for specific launch type
            process_id = self.launch_process_ids.get(launch_type)
            if not process_id:
                return CommandResponse(True, f"{launch_type} status", {
                    "launch_type": launch_type,
                    "running": False,
                    "process_id": None,
                    "pid": None
                })

            is_running = self.process.is_process_running(process_id)
            if not is_running:
                self.launch_process_ids[launch_type] = None

            process_info = self.process.processes.get(process_id)
            pid = process_info.pid if process_info else None

            return CommandResponse(True, f"{launch_type} status", {
                "launch_type": launch_type,
                "running": is_running,
                "process_id": process_id if is_running else None,
                "pid": pid if is_running else None
            })
        else:
            # Status for all launch types
            return CommandResponse(True, "All launch status", {
                "launches": self._get_launch_status()
            })

    def _get_launch_status(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all tracked launch processes with PIDs"""
        status = {}
        for launch_type, process_id in self.launch_process_ids.items():
            if process_id and self.process.is_process_running(process_id):
                process_info = self.process.processes.get(process_id)
                pid = process_info.pid if process_info else "unknown"
                status[launch_type] = {
                    "running": True,
                    "process_id": process_id,
                    "pid": pid
                }
            else:
                # Clean up stale process IDs
                if process_id:
                    self.launch_process_ids[launch_type] = None
                status[launch_type] = {
                    "running": False,
                    "process_id": None,
                    "pid": None
                }
        return status

    def _get_process_status(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all tracked processes with PIDs - now using launch tracking"""
        return self._get_launch_status()

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
        return self._start_launch("nav", use_sim_time=use_sim_time, **kwargs)

    def save_current_map(self, filename: str) -> CommandResponse:
        # Check if map_server is running
        if not self.launch_process_ids.get("map_server"):
            return CommandResponse(False, "Map server is not running. Start it with: launch start map_server")

        # Use service call to save map
        success = self.process.save_map_via_service(filename)
        if success:
            return CommandResponse(True, f"Map saved to maps/{filename}")
        else:
            return CommandResponse(False, "Failed to save map via service call")

    def list_maps(self) -> CommandResponse:
        from pathlib import Path
        maps_dir = Path("maps")

        if not maps_dir.exists():
            return CommandResponse(True, "No maps directory found", {"maps": []})

        # Find all .yaml files (map metadata files)
        yaml_files = list(maps_dir.glob("*.yaml"))

        if not yaml_files:
            return CommandResponse(True, "No maps found in maps/ folder", {"maps": []})

        # Get just the filenames without extension
        map_names = [f.stem for f in yaml_files]
        map_names.sort()

        return CommandResponse(True, f"Found {len(map_names)} maps", {"maps": map_names})

    def load_map(self, filename: str) -> CommandResponse:
        from pathlib import Path
        maps_dir = Path("maps")
        map_path = maps_dir / f"{filename}.yaml"

        if not map_path.exists():
            return CommandResponse(False, f"Map '{filename}' not found in maps/ folder")

        # Check if map_server is running
        if not self.launch_process_ids.get("map_server"):
            return CommandResponse(False, "Map server is not running. Start it with: launch start map_server")

        # Use service call to load map
        success = self.process.load_map_via_service(str(map_path))
        if success:
            return CommandResponse(True, f"Map loaded from maps/{filename}.yaml")
        else:
            return CommandResponse(False, "Failed to load map via service call")

    def start_slam(self, use_sim_time: bool = False, **kwargs) -> CommandResponse:
        return self._start_launch("slam", use_sim_time=use_sim_time, **kwargs)

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
            # Clear launch process IDs if they were killed
            for launch_type, tracked_id in self.launch_process_ids.items():
                if tracked_id == process_id:
                    self.launch_process_ids[launch_type] = None
                    break
            return CommandResponse(True, f"Killed process {process_id}")
        return CommandResponse(False, f"Failed to kill process {process_id}")

    def kill_navigation_stack(self) -> CommandResponse:
        return self._stop_launch("nav")

    def stop_slam(self) -> CommandResponse:
        return self._stop_launch("slam")


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

    def move_continuous(self, linear: float, angular: float) -> CommandResponse:
        self.movement.move_continuous(linear, angular)
        return CommandResponse(True, f"Continuous movement: linear={linear}, angular={angular}")

    def get_current_position(self) -> CommandResponse:
        position = self.movement.get_current_position()
        if position:
            x, y = position
            return CommandResponse(True, f"Position: x={x:.3f}, y={y:.3f}", {"x": x, "y": y})
        return CommandResponse(False, "No position data available")

    def turn_radians(self, radians: float) -> CommandResponse:
        return self.movement.turn_radians(radians)

    def turn_degrees(self, degrees: float) -> CommandResponse:
        return self.movement.turn_degrees(degrees)

    def get_robot_status(self) -> CommandResponse:
        linear_speed = self.config.get_variable('linear_speed')
        angular_speed = self.config.get_variable('angular_speed')

        # Check node health
        nodes_status = {
            "movement_api": "running" if hasattr(self, 'movement') and self.movement else "not available",
            "calibration_api": "running" if hasattr(self, 'calibration') and self.calibration else "not available",
            "process_api": "running" if hasattr(self, 'process') and self.process else "not available"
        }

        status = {
            "speeds": {
                "linear": linear_speed,
                "angular": angular_speed
            },
            "processes": self._get_process_status(),
            "nodes": nodes_status
        }
        return CommandResponse(True, "Robot status retrieved", {"status": status})

    def get_all_variables(self) -> CommandResponse:
        variables = self.config.get_all_variables()
        return CommandResponse(True, "All variables retrieved", {"variables": variables})

    def list_topics(self) -> CommandResponse:
        try:
            topics = self.movement.get_topic_names_and_types()
            topic_list = [name for name, _ in topics]
            return CommandResponse(True, "Active ROS topics", {"topics": topic_list})
        except Exception as e:
            return CommandResponse(False, f"Failed to list topics: {str(e)}")

    def save_config(self):
        """Save configuration to file"""
        self.config.save_config()

    def cleanup(self):
        """Clean up all resources"""
        self.process.destroy_node()
        self.movement.destroy_node()
        if hasattr(self.calibration, 'destroy_node'):
            self.calibration.destroy_node()