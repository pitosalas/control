#!/usr/bin/env python3
"""Robot Controller - Orchestrates robot operations and APIs.

Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

from __future__ import annotations

from dataclasses import dataclass

from control.commands.config_manager import ConfigManager
from control.ros2_api.calibration_api import CalibrationApi
from control.ros2_api.movement_api import MovementApi
from control.ros2_api.process_api import ProcessApi


@dataclass
class CommandResponse:
    success: bool
    message: str
    data: dict | None = None


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

        # Track singleton launch processes - initialized from config templates
        launch_templates = self.config.get_launch_templates()
        self.launch_process_ids = dict.fromkeys(launch_templates.keys())

    def _is_launch_running(self, launch_type: str) -> bool:
        """Check if a specific launch type is running"""
        process_id = self.launch_process_ids.get(launch_type)
        if not process_id:
            return False
        return self.process.is_process_running(process_id)

    def _check_launch_conflict(self, launch_type: str):
        if self._is_launch_running(launch_type):
            return CommandResponse(
                False, f"{launch_type} is already running, stop it first"
            )
        return None

    def _start_launch(self, launch_type: str, **params) -> CommandResponse:
        """Start a launch process with conflict checking and extra debug logging"""
        print(f"[DEBUG] Attempting to start launch: {launch_type} with params: {params}")
        conflict = self._check_launch_conflict(launch_type)
        if conflict:
            print(f"[DEBUG] Launch conflict detected: {conflict.message}")
            return conflict

        try:
            # Start new launch process using ProcessApi
            process_id = self.process.launch_by_type(launch_type, **params)
            print(f"[DEBUG] launch_by_type returned process_id: {process_id}")
            self.launch_process_ids[launch_type] = process_id
            return CommandResponse(
                True, f"Started {launch_type}", {"process_id": process_id}
            )
        except ValueError as e:
            print(f"[DEBUG] Exception in _start_launch: {e}")
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
        """List all available launch templates from config"""
        launch_info = []
        for launch_type in self.process.get_available_launch_types():
            config = self.process.get_launch_config(launch_type)
            name = config.launch_type
            description = config.description
            launch_info.append(f"{name:<15} {description}")

        if launch_info:
            header = f"{'NAME':<15} DESCRIPTION"
            separator = "-" * 70
            formatted_output = f"{header}\n{separator}\n" + "\n".join(launch_info)
            return CommandResponse(True, formatted_output)
        return CommandResponse(True, "No launch templates available")

    def launch_info(self, launch_type: str) -> CommandResponse:
        """Show detailed information about a launch template"""
        config = self.process.get_launch_config(launch_type)
        if not config:
            return CommandResponse(False, f"Unknown launch type: {launch_type}")

        # Build info display
        info_lines = []
        info_lines.append(f"Launch: {config.launch_type}")
        info_lines.append(f"Description: {config.description}")
        info_lines.append("\nCommand:")
        info_lines.append(f"  {config.command_template}")

        if config.default_params:
            info_lines.append("\nDefault Parameters:")
            for key, value in config.default_params.items():
                info_lines.append(f"  {key}: {value}")

        info_lines.append("\nUsage:")
        info_lines.append(f"  launch start {launch_type}")
        # Show map parameter option if map is in default params
        if config.default_params and "map" in config.default_params:
            info_lines.append(f"  launch start {launch_type} --map <mapname>")
        if "map" in launch_type:
            info_lines.append(f"  launch start {launch_type} --map-name <name>")
        info_lines.append(f"  launch start {launch_type} --use-sim-time")

        return CommandResponse(True, "\n".join(info_lines))

    def launch_start(self, launch_type: str, **kwargs) -> CommandResponse:
        """Start a launch process by type"""
        return self._start_launch(launch_type, **kwargs)

    def launch_stop(self, launch_type: str) -> CommandResponse:
        """Stop a launch process by type"""
        return self._stop_launch(launch_type)

    def _get_launch_status(self):
        """Get status of all tracked launch processes with PIDs"""
        status = {}
        for launch_type, process_id in self.launch_process_ids.items():
            if process_id and self.process.is_process_running(process_id):
                process_info = self.process.processes.get(process_id)
                pid = process_info.pid if process_info else "unknown"
                status[launch_type] = {
                    "running": True,
                    "process_id": process_id,
                    "pid": pid,
                }
            else:
                # Clean up stale process IDs
                if process_id:
                    self.launch_process_ids[launch_type] = None
                status[launch_type] = {
                    "running": False,
                    "process_id": None,
                    "pid": None,
                }
        return status

    def _get_process_status(self):
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
        return CommandResponse(
            True, f"Circle stress test with {diameter}m diameter completed"
        )

    def script_list(self) -> CommandResponse:
        scripts = {
            "square": "Drive in a square pattern with specified side length",
            "rotate_stress": "Run continuous rotation stress test",
            "circle_stress": "Run continuous circle stress test with specified diameter",
        }
        return CommandResponse(True, "Available scripts", {"scripts": scripts})

    def set_variable(self, name: str, value: str) -> CommandResponse:
        self.config.set_variable(name, value)
        stored_value = self.config.get_variable(name)
        return CommandResponse(
            True, f"Set {name} = {stored_value} ({type(stored_value).__name__})"
        )

    def get_variable(self, name: str) -> CommandResponse:
        value = self.config.get_variable(name)
        return CommandResponse(
            True, f"{name} = {value} ({type(value).__name__})", {"value": value}
        )

    def map_save(self) -> CommandResponse:
        """Save current map using map_saver_cli"""
        map_name = self.config.get_variable("map_name")
        if not map_name:
            return CommandResponse(False, "map_name variable not set. Use 'config set map_name <name>' first.")

        self.config.ensure_subdirs()
        maps_dir = self.config.get_maps_dir()
        full_path = maps_dir / map_name

        cmd = (
            f"ros2 run nav2_map_server map_saver_cli "
            f"-f {full_path} "
            f"--ros-args -p save_map_timeout:=10000."
        )

        # Use ProcessApi to run command with logging
        success, output, log_file = self.process.run_command_sync(
            cmd, log_name="map_save", timeout=15.0
        )

        if success:
            msg = f"Map saved to {maps_dir}/{map_name}.yaml and {map_name}.pgm"
            if log_file:
                msg += f"\nLog: {log_file}"
            return CommandResponse(True, msg)

        error_msg = "Failed to save map"
        if "timed out" in output.lower():
            error_msg = "Map save timeout after 15 seconds"
        elif output:
            # Extract error message from output
            error_lines = [line for line in output.split("\n") if line.strip()]
            if error_lines:
                error_msg = f"Failed to save map: {error_lines[-1]}"

        if log_file:
            error_msg += f"\nSee log: {log_file}"

        return CommandResponse(False, error_msg)

    def map_serialize(self) -> CommandResponse:
        """Save current map in SLAM Toolbox serialized format"""
        map_name = self.config.get_variable("map_name")
        if not map_name:
            return CommandResponse(False, "map_name variable not set. Use 'config set map_name <name>' first.")

        self.config.ensure_subdirs()
        maps_dir = self.config.get_maps_dir()
        full_path = maps_dir / map_name

        cmd = (
            f"ros2 service call /slam_toolbox/serialize_map "
            f"slam_toolbox/srv/SerializePoseGraph "
            f'"{{filename: \'{full_path}\'}}"'
        )

        # Use ProcessApi to run command with logging
        success, output, log_file = self.process.run_command_sync(
            cmd, log_name="map_serialize", timeout=15.0
        )

        if success:
            msg = f"Map serialized to {full_path}.posegraph and {map_name}.data"
            if log_file:
                msg += f"\nLog: {log_file}"
            return CommandResponse(True, msg)

        error_msg = "Failed to serialize map"
        if "timed out" in output.lower():
            error_msg = "Map serialize timeout after 15 seconds"
        elif output:
            # Extract error message from output
            error_lines = [line for line in output.split("\n") if line.strip()]
            if error_lines:
                error_msg = f"Failed to serialize map: {error_lines[-1]}"

        if log_file:
            error_msg += f"\nSee log: {log_file}"

        return CommandResponse(False, error_msg)

    def list_maps(self) -> CommandResponse:
        self.config.ensure_subdirs()
        maps_dir = self.config.get_maps_dir()

        # Find all map-related files
        all_files = list(maps_dir.glob("*"))

        # Group files by base name
        map_groups = {}
        for f in all_files:
            if f.is_file():
                base_name = f.stem
                ext = f.suffix
                if base_name not in map_groups:
                    map_groups[base_name] = []
                map_groups[base_name].append(ext)

        if not map_groups:
            return CommandResponse(True, f"No maps found in {maps_dir}", {"maps": []})

        # Build formatted output
        output_lines = []
        output_lines.append(f"Maps in {maps_dir}:")
        output_lines.append("-" * 70)

        for map_name in sorted(map_groups.keys()):
            extensions = sorted(map_groups[map_name])
            ext_str = ", ".join(extensions)
            output_lines.append(f"  {map_name:<30} [{ext_str}]")

        output_lines.append("-" * 70)
        output_lines.append(f"Total: {len(map_groups)} map(s)")

        message = "\n".join(output_lines)
        return CommandResponse(True, message, {"maps": list(map_groups.keys())})

    def turn_radians(self, radians: float) -> CommandResponse:
        self.movement.turn_amount(radians)
        return CommandResponse(True, f"Turned {radians} radians")

    def turn_degrees(self, degrees: float) -> CommandResponse:
        return self.movement.turn_degrees(degrees)

    def get_robot_status(self) -> CommandResponse:
        linear_speed = self.config.get_variable("linear_speed")
        angular_speed = self.config.get_variable("angular_speed")

        # Check node health
        nodes_status = {
            "movement_api": "running"
            if hasattr(self, "movement") and self.movement
            else "not available",
            "calibration_api": "running"
            if hasattr(self, "calibration") and self.calibration
            else "not available",
            "process_api": "running"
            if hasattr(self, "process") and self.process
            else "not available",
        }

        status = {
            "speeds": {"linear": linear_speed, "angular": angular_speed},
            "processes": self._get_process_status(),
            "nodes": nodes_status,
        }
        return CommandResponse(True, "Robot status retrieved", {"status": status})

    def get_all_variables(self) -> CommandResponse:
        all_vars = self.config.get_all_variables()
        variables = {k: v for k, v in all_vars.items() if k != "launch_templates"}
        return CommandResponse(
            True, "All variables retrieved", {"variables": variables}
        )

    def list_topics(self) -> CommandResponse:
        try:
            topics = self.movement.get_topic_names_and_types()
            topic_list = [name for name, _ in topics]
            return CommandResponse(True, "Active ROS topics", {"topics": topic_list})
        except Exception as e:
            return CommandResponse(False, f"Failed to list topics: {e!s}")

    def list_ros_processes(self) -> CommandResponse:
        """List all ROS-related processes running on the system"""
        import subprocess

        try:
            # Get all processes with ros2, nav2, slam, or other ROS-related keywords
            result = subprocess.run(
                ["ps", "aux"], check=False, capture_output=True, text=True, timeout=5
            )

            if result.returncode != 0:
                return CommandResponse(False, "Failed to list processes")

            lines = result.stdout.split("\n")

            # Filter for ROS-related processes
            ros_keywords = [
                "ros2",
                "nav2",
                "slam",
                "rviz",
                "gazebo",
                "amcl",
                "map_server",
                "robot_state",
            ]
            process_info = []

            for line in lines[1:]:
                if any(keyword in line.lower() for keyword in ros_keywords):
                    parts = line.split(None, 10)  # Split into max 11 parts
                    if len(parts) >= 11:
                        pid = parts[1]
                        cpu = parts[2]
                        mem = parts[3]
                        command = parts[10]

                        # Truncate command to fit in 120 char display
                        # Format: #  PID    CPU  MEM  COMMAND
                        #         1  12345  1.2  3.4  command text...
                        # Reserve: 2 for number, 7 for PID, 5 for CPU, 5 for MEM = 19 chars
                        # Leaves ~101 chars for command
                        max_cmd_len = 101
                        if len(command) > max_cmd_len:
                            command = command[: max_cmd_len - 3] + "..."

                        process_info.append(
                            {"pid": pid, "cpu": cpu, "mem": mem, "command": command}
                        )

            if not process_info:
                return CommandResponse(True, "No ROS processes found")

            # Format output
            output_lines = []
            output_lines.append("#  PID     CPU  MEM  COMMAND")
            output_lines.append("-" * 120)

            for i, proc in enumerate(process_info, 1):
                line = f"{i:<2} {proc['pid']:<7} {proc['cpu']:>4} {proc['mem']:>4}  {proc['command']}"
                output_lines.append(line)

            output_lines.append("-" * 120)
            output_lines.append(f"Total: {len(process_info)} ROS processes")
            output_lines.append("Use 'system kill <PID>' to kill a process")

            return CommandResponse(True, "\n".join(output_lines))

        except subprocess.TimeoutExpired:
            return CommandResponse(False, "Process listing timed out")
        except Exception as e:
            return CommandResponse(False, f"Error listing processes: {e}")

    def list_launch_processes(self) -> CommandResponse:
        """List all ros2 launch processes running on the system, filtering out child processes"""
        import subprocess

        try:
            # Get all processes with parent PID and PGID info
            result = subprocess.run(
                ["ps", "axo", "pid,pgid,ppid,user,%cpu,%mem,command"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )

            if result.returncode != 0:
                return CommandResponse(False, "Failed to list processes")

            lines = result.stdout.split("\n")

            # First pass: collect all ros2 launch processes
            all_processes = {}
            parent_pids = set()

            for line in lines[1:]:
                if "ros2" in line.lower() and "launch" in line.lower():
                    parts = line.split(None, 6)
                    if len(parts) >= 7:
                        pid = parts[0]
                        pgid = parts[1]
                        ppid = parts[2]
                        cpu = parts[4]
                        mem = parts[5]
                        command = parts[6]

                        all_processes[pid] = {
                            "pid": pid,
                            "pgid": pgid,
                            "ppid": ppid,
                            "cpu": cpu,
                            "mem": mem,
                            "command": command,
                        }
                        parent_pids.add(ppid)

            # Second pass: filter out child processes
            # Only keep processes whose PID is NOT in the parent_pids set
            # (i.e., they are not parents of other ros2 launch processes)
            # AND whose parent is not another ros2 launch process
            parent_processes = []
            for pid, proc in all_processes.items():
                # Skip if this process is a child of another ros2 launch process
                if proc["ppid"] in all_processes:
                    continue

                # This is a parent process we want to show
                command = proc["command"]
                # Truncate command to fit in 120 char display
                # Format: #  PID    PGID   CPU  MEM  COMMAND
                # Reserve: 2 for #, 7 for PID, 7 for PGID, 5 for CPU, 5 for MEM = 26 chars
                # Leaves ~94 chars for command
                max_cmd_len = 94
                if len(command) > max_cmd_len:
                    command = command[: max_cmd_len - 3] + "..."

                parent_processes.append(
                    {
                        "pid": proc["pid"],
                        "pgid": proc["pgid"],
                        "cpu": proc["cpu"],
                        "mem": proc["mem"],
                        "command": command,
                    }
                )

            if not parent_processes:
                return CommandResponse(True, "No ros2 launch processes found")

            # Format output
            output_lines = []
            output_lines.append("#  PID     PGID    CPU  MEM  COMMAND")
            output_lines.append("-" * 120)

            for i, proc in enumerate(parent_processes, 1):
                line = f"{i:<2} {proc['pid']:<7} {proc['pgid']:<7} {proc['cpu']:>4} {proc['mem']:>4}  {proc['command']}"
                output_lines.append(line)

            output_lines.append("-" * 120)
            output_lines.append(
                f"Total: {len(parent_processes)} launch processes (parent processes only)"
            )
            output_lines.append(
                "Use 'system kill <PID>' to kill (kills process group if leader)"
            )

            return CommandResponse(True, "\n".join(output_lines))

        except subprocess.TimeoutExpired:
            return CommandResponse(False, "Process listing timed out")
        except Exception as e:
            return CommandResponse(False, f"Error listing processes: {e}")

    def kill_ros_process(self, pid: int) -> CommandResponse:
        """Kill a ROS process by its PID, kills entire process group if PID is group leader"""
        import subprocess

        try:
            # First, check if this PID is a process group leader (PID == PGID)
            ps_result = subprocess.run(
                ["ps", "-o", "pid,pgid", "-p", str(pid)],
                check=False,
                capture_output=True,
                text=True,
                timeout=2,
            )

            is_group_leader = False
            if ps_result.returncode == 0:
                lines = ps_result.stdout.strip().split("\n")
                if len(lines) > 1:
                    parts = lines[1].split()
                    if len(parts) >= 2:
                        proc_pid = parts[0].strip()
                        proc_pgid = parts[1].strip()
                        is_group_leader = proc_pid == proc_pgid

            # If it's a group leader, kill the entire process group
            if is_group_leader:
                try:
                    # Kill process group with negative PID
                    subprocess.run(["kill", "-TERM", f"-{pid}"], check=True, timeout=2)
                    return CommandResponse(
                        True,
                        f"Sent SIGTERM to process group {pid} (and all children)\nRun 'system ps' to verify they stopped",
                    )
                except subprocess.CalledProcessError:
                    # Try SIGKILL for the group
                    try:
                        subprocess.run(
                            ["kill", "-KILL", f"-{pid}"], check=True, timeout=2
                        )
                        return CommandResponse(
                            True, f"Force killed process group {pid} (SIGKILL)"
                        )
                    except subprocess.CalledProcessError:
                        return CommandResponse(
                            False,
                            f"Failed to kill process group {pid}. May require sudo",
                        )
            else:
                # Not a group leader, just kill the single process
                try:
                    subprocess.run(["kill", "-TERM", str(pid)], check=True, timeout=2)
                    return CommandResponse(
                        True,
                        f"Sent SIGTERM to process {pid}\nRun 'system ps' to verify it stopped",
                    )
                except subprocess.CalledProcessError:
                    # Try SIGKILL if SIGTERM fails
                    try:
                        subprocess.run(
                            ["kill", "-KILL", str(pid)], check=True, timeout=2
                        )
                        return CommandResponse(
                            True, f"Force killed process {pid} (SIGKILL)"
                        )
                    except subprocess.CalledProcessError:
                        return CommandResponse(
                            False,
                            f"Failed to kill process {pid}. May already be stopped or requires sudo",
                        )

        except subprocess.TimeoutExpired:
            return CommandResponse(False, "Kill command timed out")
        except Exception as e:
            return CommandResponse(False, f"Error killing process: {e}")

