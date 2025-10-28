#!/usr/bin/env python3
from typing import Any, Dict, Optional
from dataclasses import dataclass

from control.ros2_api.movement_api import MovementApi
from control.ros2_api.calibration_api import CalibrationApi
from control.ros2_api.process_api import ProcessApi
from control.commands.config_manager import ConfigManager

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
            "map": None
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
            # Status for specific launch type - format as single row table plus log file
            process_id = self.launch_process_ids.get(launch_type)
            if not process_id:
                status = "STOPPED"
                pid = "N/A"
                proc_id = "N/A"
                log_file = None
            else:
                is_running = self.process.is_process_running(process_id)
                if not is_running:
                    self.launch_process_ids[launch_type] = None
                    status = "STOPPED"
                    pid = "N/A"
                    proc_id = "N/A"
                    log_file = None
                else:
                    status = "RUNNING"
                    process_info = self.process.processes.get(process_id)
                    pid = str(process_info.pid) if process_info else "Unknown"
                    proc_id = process_id[:8]
                    log_file = self.process.get_process_log_file(process_id)

            header = f"{'TYPE':<12} {'STATUS':<8} {'PID':<8} {'PROC_ID':<10}"
            separator = "-" * 40
            row = f"{launch_type:<12} {status:<8} {pid:<8} {proc_id:<10}"
            formatted_output = f"{header}\n{separator}\n{row}"

            if log_file:
                formatted_output += f"\nLog file: {log_file}"

            return CommandResponse(True, formatted_output)
        else:
            # Status for all launch types - format as table
            status_data = self._get_launch_status()
            if not status_data:
                return CommandResponse(True, "No launch processes tracked")

            header = f"{'TYPE':<12} {'STATUS':<8} {'PID':<8} {'PROC_ID':<10}"
            separator = "-" * 40
            rows = []
            log_files = []

            for launch_type, info in status_data.items():
                status = "RUNNING" if info["running"] else "STOPPED"
                pid = str(info["pid"]) if info["pid"] else "N/A"
                proc_id = info["process_id"][:8] if info["process_id"] else "N/A"
                rows.append(f"{launch_type:<12} {status:<8} {pid:<8} {proc_id:<10}")

                # Get log file if process is running
                if info["running"] and info["process_id"]:
                    log_file = self.process.get_process_log_file(info["process_id"])
                    if log_file:
                        log_files.append(f"  {launch_type}: {log_file}")

            formatted_output = f"{header}\n{separator}\n" + "\n".join(rows)

            if log_files:
                formatted_output += "\n\nLog files:\n" + "\n".join(log_files)

            return CommandResponse(True, formatted_output)

    def launch_doctor(self) -> CommandResponse:
        """Diagnose launch process conflicts and suggest fixes"""
        import subprocess
        import re

        issues = []
        suggestions = []

        # Get our tracked processes
        tracked_status = self._get_launch_status()

        # Detect external processes for each launch type
        launch_types = {
            "nav": ["navigation_launch", "nav2_bringup"],
            "slam": ["slam_toolbox", "online_async_launch"],
            "map": ["nav2_map_server", "map_server"]
        }

        external_processes = {}

        for launch_type, patterns in launch_types.items():
            external_pids = []

            for pattern in patterns:
                try:
                    # Find processes matching pattern
                    result = subprocess.run(['pgrep', '-f', pattern],
                                          capture_output=True, text=True)
                    if result.returncode == 0:
                        pids = [int(pid.strip()) for pid in result.stdout.split() if pid.strip()]

                        # Filter out our tracked processes
                        tracked_pid = None
                        if tracked_status.get(launch_type, {}).get("running"):
                            tracked_pid = tracked_status[launch_type].get("pid")

                        for pid in pids:
                            if tracked_pid != pid:
                                external_pids.append(pid)
                except Exception:
                    continue

            if external_pids:
                external_processes[launch_type] = external_pids

        # Check for conflicts
        for launch_type, external_pids in external_processes.items():
            tracked = tracked_status.get(launch_type, {})
            is_tracked_running = tracked.get("running", False)

            if external_pids:
                if is_tracked_running:
                    issues.append(f"Multiple {launch_type} processes detected: "
                                f"tracked PID {tracked.get('pid')} + external PIDs {external_pids}")
                    suggestions.append(f"launch kill-all {launch_type}  # Kill ALL {launch_type} processes")
                else:
                    issues.append(f"External {launch_type} processes detected: PIDs {external_pids}")
                    suggestions.append(f"launch kill-all {launch_type}  # Kill ALL {launch_type} processes")

        # Check for orphaned tracked processes
        for launch_type, status in tracked_status.items():
            if status.get("process_id") and not status.get("running"):
                issues.append(f"Orphaned {launch_type} process tracking (process ended but still tracked)")
                suggestions.append(f"launch status  # Check current status and clear stale tracking")

        # Check for stale processes (only if not already detected)
        if "nav" not in external_processes:
            try:
                result = subprocess.run(['pgrep', '-f', 'ros2.*launch.*navigation_launch'],
                                      capture_output=True, text=True)
                stale_pids = [pid.strip() for pid in result.stdout.split() if pid.strip()]
                if len(stale_pids) > 1:
                    issues.append(f"Multiple navigation launch processes: {stale_pids}")
                    suggestions.append("launch kill-all nav  # Kill ALL navigation processes")
            except Exception:
                pass

        # Format output
        if not issues:
            return CommandResponse(True, "DIAGNOSIS: No launch conflicts detected. All systems appear healthy.")

        diagnosis = "DIAGNOSIS:\n"
        for issue in issues:
            diagnosis += f"✗ {issue}\n"

        if suggestions:
            diagnosis += "\nRECOMMENDED ACTIONS:\n"
            for i, suggestion in enumerate(suggestions, 1):
                diagnosis += f"{i}. {suggestion}\n"

        diagnosis += "\nADDITIONAL COMMANDS:\n"
        diagnosis += "- launch status  # Check current tracking status\n"
        diagnosis += "- launch list    # Show available launch types\n"
        diagnosis += "- launch kill <type>  # Kill tracked process only"

        return CommandResponse(True, diagnosis.strip())

    def launch_kill_all(self, launch_type: str) -> CommandResponse:
        """Kill ALL instances of a launch type (tracked + external) or everything with *"""
        import subprocess

        # Nuclear option - kill ALL ROS2 processes
        if launch_type == "*":
            return self._nuclear_ros_cleanup()

        if launch_type not in ["nav", "slam", "map"]:
            return CommandResponse(False, f"Unknown launch type: {launch_type}. Use: nav, slam, map, or * for everything")

        # Define process patterns for each launch type
        kill_patterns = {
            "nav": ["navigation_launch", "nav2_bringup"],
            "slam": ["slam_toolbox", "online_async_launch"],
            "map": ["nav2_map_server", "map_server"]
        }

        patterns = kill_patterns[launch_type]
        killed_pids = []
        errors = []

        # Kill our tracked process first (graceful)
        tracked_killed = False
        if self.launch_process_ids.get(launch_type):
            result = self._stop_launch(launch_type)
            if result.success:
                tracked_killed = True

        # Kill all external processes matching patterns
        for pattern in patterns:
            try:
                # Find PIDs
                result = subprocess.run(['pgrep', '-f', pattern],
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    pids = [pid.strip() for pid in result.stdout.split() if pid.strip()]

                    # Kill each PID
                    for pid in pids:
                        try:
                            subprocess.run(['kill', '-TERM', pid], check=True)
                            killed_pids.append(pid)
                        except subprocess.CalledProcessError:
                            # Try SIGKILL if TERM fails
                            try:
                                subprocess.run(['kill', '-KILL', pid], check=True)
                                killed_pids.append(pid)
                            except subprocess.CalledProcessError:
                                errors.append(f"Failed to kill PID {pid}")

            except Exception as e:
                errors.append(f"Error finding {pattern} processes: {str(e)}")

        # Clear our tracking
        if launch_type in self.launch_process_ids:
            self.launch_process_ids[launch_type] = None

        # Format response
        if killed_pids:
            message = f"Killed {len(killed_pids)} {launch_type} processes: {killed_pids}"
            if tracked_killed:
                message += " (including tracked process)"
        elif tracked_killed:
            message = f"Killed tracked {launch_type} process"
        else:
            message = f"No {launch_type} processes found to kill"

        if errors:
            message += f"\nErrors: {'; '.join(errors)}"
            return CommandResponse(False, message)

        return CommandResponse(True, message)

    def _nuclear_ros_cleanup(self) -> CommandResponse:
        """Nuclear option: Kill ALL ROS2 nodes and processes"""
        import subprocess

        # Clear all our tracking first
        for launch_type in self.launch_process_ids:
            self.launch_process_ids[launch_type] = None

        commands = [
            ["pkill", "-f", "ros2"],
            ["pkill", "-f", "_ros"],
            ["pkill", "-f", "rcl"],
            ["pkill", "-f", "launch"]
        ]

        results = []
        total_killed = 0

        for cmd in commands:
            try:
                # Get PIDs before killing to count them
                pattern = cmd[2]
                pid_result = subprocess.run(['pgrep', '-f', pattern],
                                          capture_output=True, text=True)
                if pid_result.returncode == 0:
                    pid_count = len([p for p in pid_result.stdout.split() if p.strip()])
                else:
                    pid_count = 0

                # Execute the kill command
                result = subprocess.run(cmd, capture_output=True, text=True)
                if pid_count > 0:
                    results.append(f"Killed {pid_count} processes matching '{pattern}'")
                    total_killed += pid_count
                else:
                    results.append(f"No processes found matching '{pattern}'")

            except Exception as e:
                results.append(f"Error with command {' '.join(cmd)}: {str(e)}")

        if total_killed > 0:
            message = f"NUCLEAR CLEANUP: Killed {total_killed} ROS processes total\n"
            message += "\n".join(results)
            message += "\n\nWARNING: All ROS2 processes have been terminated!"
            return CommandResponse(True, message)
        else:
            return CommandResponse(True, "No ROS2 processes found to kill")

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

    def move_forward(self, meters: float) -> CommandResponse:
        self.movement.move_dist(abs(meters))
        return CommandResponse(True, f"Moved forward {abs(meters)} meters")

    def move_backward(self, meters: float) -> CommandResponse:
        self.movement.move_dist(-abs(meters))
        return CommandResponse(True, f"Moved backward {abs(meters)} meters")

    def turn_by_radians(self, radians: float) -> CommandResponse:
        self.movement.turn_amount(radians)
        return CommandResponse(True, f"Turned {radians} radians")

    def turn_by_degrees(self, degrees: float) -> CommandResponse:
        self.movement.turn_degrees(degrees)
        return CommandResponse(True, f"Turned {degrees} degrees")

    def turn_clockwise(self, degrees: float) -> CommandResponse:
        self.movement.turn_degrees(-abs(degrees))
        return CommandResponse(True, f"Turned clockwise {abs(degrees)} degrees")

    def turn_counterclockwise(self, degrees: float) -> CommandResponse:
        self.movement.turn_degrees(abs(degrees))
        return CommandResponse(True, f"Turned counterclockwise {abs(degrees)} degrees")

    def turn_for_time(self, seconds: float) -> CommandResponse:
        self.movement.turn_time(seconds)
        return CommandResponse(True, f"Turned for {seconds} seconds")

    def stop_robot(self) -> CommandResponse:
        self.movement.stop()
        return CommandResponse(True, "Robot stopped")

    def script_square(self, meters: float) -> CommandResponse:
        self.calibration.run_square_pattern(meters)
        return CommandResponse(True, f"Completed square pattern with {meters}m sides")

    def script_rotate_stress(self) -> CommandResponse:
        self.calibration.run_rotate_stress()
        return CommandResponse(True, "Rotation stress test completed")

    def script_circle_stress(self, diameter: float) -> CommandResponse:
        self.calibration.run_circle_stress(diameter)
        return CommandResponse(True, f"Circle stress test with {diameter}m diameter completed")

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


    def save_current_map(self, filename: str) -> CommandResponse:
        if not self.launch_process_ids.get("map"):
            return CommandResponse(False, "Map server is not running. Start it with: launch start map")

        self.config.ensure_subdirs()
        success = self.process.save_map_via_service(filename)
        if success:
            return CommandResponse(True, f"Map saved to {self.config.get_maps_dir() / filename}")
        else:
            return CommandResponse(False, "Failed to save map via service call")

    def list_maps(self) -> CommandResponse:
        self.config.ensure_subdirs()
        maps_dir = self.config.get_maps_dir()
        yaml_files = list(maps_dir.glob("*.yaml"))

        if not yaml_files:
            return CommandResponse(True, f"No maps found in {maps_dir}", {"maps": []})

        map_names = [f.stem for f in yaml_files]
        map_names.sort()

        return CommandResponse(True, f"Found {len(map_names)} maps", {"maps": map_names})


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
        self.movement.turn_amount(radians)
        return CommandResponse(True, f"Turned {radians} radians")

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