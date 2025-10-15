#!/usr/bin/env python3
import os
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from nav2_msgs.srv import SaveMap

from ..commands.config_manager import ConfigManager
from .base_api import BaseApi


@dataclass
class ProcessInfo:
    process: subprocess.Popen
    command: str
    output: List[str]
    is_running: bool
    pid: int
    start_time: float
    log_file: Optional[str] = None


@dataclass
class LaunchConfig:
    launch_type: str
    command_template: str
    description: str
    default_params: Dict[str, str]


# Static launch configuration table
LAUNCH_CONFIGS = {
    "nav": LaunchConfig(
        launch_type="nav",
        command_template="ros2 launch nav2_bringup navigation_launch.py {params}",
        description="Navigation stack with path planning and obstacle avoidance",
        default_params={"use_sim_time": "false"},
    ),
    "slam": LaunchConfig(
        launch_type="slam",
        command_template="ros2 launch slam_toolbox online_async_launch.py {params}",
        description="SLAM mapping and localization",
        default_params={"use_sim_time": "false"},
    ),
    "map": LaunchConfig(
        launch_type="map",
        command_template="ros2 run nav2_map_server map_server {map_args} {params}",
        description="Map server and map saver for loading and saving maps",
        default_params={"use_sim_time": "false"},
    ),
}


class ProcessApi(BaseApi):
    """
    ROS2 process management API for launching and controlling external processes.
    Handles launch files, shell commands, and process lifecycle management.
    """

    def __init__(self, config_manager: ConfigManager = None):
        super().__init__("process_api", config_manager)
        self.processes: Dict[str, ProcessInfo] = {}

        # Service client for map save operation
        self.save_map_client = self.create_client(SaveMap, "/map_saver/save_map")

    def get_available_launch_types(self) -> List[str]:
        """Get list of available launch types"""
        return list(LAUNCH_CONFIGS.keys())

    def get_launch_config(self, launch_type: str) -> Optional[LaunchConfig]:
        """Get launch configuration for a specific type"""
        return LAUNCH_CONFIGS.get(launch_type)

    def _format_launch_params(self, params: Dict[str, str]) -> str:
        """Format parameters for launch command"""
        if not params:
            return ""
        param_parts = []
        for key, value in params.items():
            param_parts.append(f"{key}:={value}")
        return " ".join(param_parts)

    def launch_by_type(self, launch_type: str, **params) -> str:
        """Launch a process by launch type with optional parameters"""
        config = self.get_launch_config(launch_type)
        if not config:
            raise ValueError(f"Unknown launch type: {launch_type}")

        # Handle map special case with map_name parameter
        if launch_type == "map" and "map_name" in params:
            map_name = params.pop("map_name")  # Remove from params to avoid duplication
            # Maps stored in ~/.control/maps/
            maps_dir = self.config.get_control_dir() / "maps"
            map_file = maps_dir / f"{map_name}.yaml"
            if not map_file.exists():
                raise ValueError(f"Map file not found: {map_file}")
            map_file_abs = map_file.resolve()
            map_args = f"--ros-args -p yaml_filename:={map_file_abs}"
        else:
            map_args = ""

        # Merge default params with provided params
        final_params = config.default_params.copy()
        final_params.update({k: str(v) for k, v in params.items()})

        # Format parameters and build command
        params_str = self._format_launch_params(final_params)

        # Special handling for map type which uses ros2 run instead of ros2 launch
        if launch_type == "map" and params_str:
            params_str = "--ros-args " + " ".join([f"-p {param}" for param in params_str.split()])

        # Format command based on launch type
        if launch_type == "map":
            command = config.command_template.format(params=params_str, map_args=map_args).strip()
        else:
            command = config.command_template.format(params=params_str).strip()

        self.log_debug(f"Launching {launch_type}: {command}")
        return self.launch_command(command, log_name=launch_type)

    def kill_by_type(self, launch_type: str, tracked_process_id: str) -> bool:
        """Kill a process by launch type using its tracked process ID"""
        if not tracked_process_id:
            self.log_warn(f"No process ID provided for {launch_type}")
            return False

        return self.kill_process(tracked_process_id)

    def get_launch_status(self, launch_type: str = None) -> Dict:
        """Get status of launch processes"""
        if launch_type:
            # Return status for specific launch type (requires external tracking)
            # This will be handled by RobotController which tracks process IDs
            return {"launch_type": launch_type, "status": "unknown"}
        else:
            # Return status for all tracked processes
            return self.get_running_processes()

    def launch_command(self, command: str, log_name: Optional[str] = None) -> str:
        """Launch shell command and return process ID"""
        process_id = str(uuid.uuid4())

        self.log_debug(f"Launching: {command}")

        # Create log file if log_name is provided
        log_file_path = None
        if log_name:
            self.config.ensure_subdirs()
            log_dir_config = self.config.get_variable("log_dir") or "logs"
            logs_dir = self.config.resolve_path(log_dir_config)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            log_file_path = str(logs_dir / f"{log_name}_{timestamp}.log")

        try:
            proc = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setpgrp,  # Create process group for clean killing
            )

            process_info = ProcessInfo(
                process=proc,
                command=command,
                output=[],
                is_running=True,
                pid=proc.pid,
                start_time=time.time(),
                log_file=log_file_path,
            )

            # Start output capture thread
            threading.Thread(
                target=self._capture_output,
                args=(process_id, process_info),
                daemon=True,
            ).start()

            self.processes[process_id] = process_info
            if log_file_path:
                self.log_debug(f"Process launched with ID: {process_id}, PID: {proc.pid}, Log: {log_file_path}")
            else:
                self.log_debug(f"Process launched with ID: {process_id}, PID: {proc.pid}")
            return process_id

        except Exception as e:
            self.log_error(f"Failed to launch command '{command}': {e}")
            raise

    def save_map_via_service(self, filename: str) -> bool:
        """Save current map using ROS2 service call to persistent map_saver"""
        if not self.save_map_client.wait_for_service(timeout_sec=2.0):
            self.log_error("Map saver service not available")
            return False

        self.config.ensure_subdirs()
        map_path = self.config.get_maps_dir() / filename

        request = SaveMap.Request()
        request.map_topic = "map"
        request.map_url = str(map_path)
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        try:
            future = self.save_map_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result() is not None:
                result = future.result()
                if result.result:
                    self.log_debug(f"Map saved successfully to {map_path}")
                    return True
                else:
                    self.log_error(f"Failed to save map: {result.result}")
                    return False
            else:
                self.log_error("Map save service call failed")
                return False
        except Exception as e:
            self.log_error(f"Error calling map save service: {e}")
            return False


    def run_ros_node(self, package: str, executable: str, **kwargs) -> str:
        """Run a ROS2 node with optional parameters"""
        params = []
        for key, value in kwargs.items():
            params.append(f"--ros-args -p {key}:={value}")

        param_string = " ".join(params)
        command = f"ros2 run {package} {executable} {param_string}"
        return self.launch_command(command)

    def launch_file(self, package: str, launch_file: str, **kwargs) -> str:
        """Launch a ROS2 launch file with optional parameters"""
        params = []
        for key, value in kwargs.items():
            params.append(f"{key}:={value}")

        param_string = " ".join(params)
        command = f"ros2 launch {package} {launch_file} {param_string}"
        return self.launch_command(command)

    def kill_process(self, process_id: str) -> bool:
        """Kill process and all children (Ctrl+C equivalent)"""
        if process_id not in self.processes:
            self.log_warn(f"Process ID {process_id} not found")
            return False

        proc_info = self.processes[process_id]
        if not proc_info.is_running:
            self.log_debug(f"Process {process_id} already stopped")
            return True

        try:
            self.log_debug(f"Terminating process {process_id}: {proc_info.command}")

            # Kill process group (handles launch files with multiple nodes)
            try:
                os.killpg(os.getpgid(proc_info.pid), signal.SIGTERM)
            except ProcessLookupError:
                # Process already dead
                proc_info.is_running = False
                return True

            # Wait for graceful shutdown
            try:
                proc_info.process.wait(timeout=5)
                self.log_debug(f"Process {process_id} terminated gracefully")
            except subprocess.TimeoutExpired:
                self.log_warn(f"Force killing process {process_id}")
                try:
                    os.killpg(os.getpgid(proc_info.pid), signal.SIGKILL)
                    proc_info.process.wait(timeout=2)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass

        except Exception as e:
            self.log_error(f"Error killing process {process_id}: {e}")
            return False

        proc_info.is_running = False
        return True

    def kill_all_processes(self) -> int:
        """Kill all managed processes"""
        killed_count = 0
        for process_id in list(self.processes.keys()):
            if self.kill_process(process_id):
                killed_count += 1
        return killed_count

    def get_process_output(
        self, process_id: str, lines: Optional[int] = None
    ) -> List[str]:
        """Get captured output from process"""
        if process_id not in self.processes:
            return []

        output = self.processes[process_id].output
        if lines is None:
            return output.copy()
        return output[-lines:] if lines > 0 else []

    def get_process_log_file(self, process_id: str) -> Optional[str]:
        """Get log file path for a process"""
        if process_id not in self.processes:
            return None
        return self.processes[process_id].log_file

    def get_running_processes(self) -> Dict[str, dict]:
        """Get status of all managed processes"""
        result = {}
        for process_id, proc_info in self.processes.items():
            # Check if process is actually still running
            if proc_info.is_running:
                try:
                    proc_info.process.poll()
                    if proc_info.process.returncode is not None:
                        proc_info.is_running = False
                except (OSError, AttributeError):
                    proc_info.is_running = False

            result[process_id] = {
                "command": proc_info.command,
                "pid": proc_info.pid,
                "is_running": proc_info.is_running,
                "start_time": proc_info.start_time,
                "runtime": time.time() - proc_info.start_time,
                "output_lines": len(proc_info.output),
            }
        return result

    def is_process_running(self, process_id: str) -> bool:
        """Check if process is still running"""
        if process_id not in self.processes:
            return False

        proc_info = self.processes[process_id]
        if not proc_info.is_running:
            return False

        # Double-check process status
        try:
            proc_info.process.poll()
            if proc_info.process.returncode is not None:
                proc_info.is_running = False
                return False
        except (OSError, AttributeError):
            proc_info.is_running = False
            return False

        return True

    def wait_for_process(
        self, process_id: str, timeout: Optional[float] = None
    ) -> bool:
        """Wait for process to complete"""
        if process_id not in self.processes:
            return False

        proc_info = self.processes[process_id]
        try:
            proc_info.process.wait(timeout=timeout)
            proc_info.is_running = False
            return True
        except subprocess.TimeoutExpired:
            return False

    def _capture_output(self, process_id: str, process_info: ProcessInfo):
        """Capture output in background thread and write to log file if configured"""
        log_file_handle = None
        try:
            # Open log file if configured
            if process_info.log_file:
                log_file_handle = open(process_info.log_file, 'w')
                log_file_handle.write(f"=== Process Log ===\n")
                log_file_handle.write(f"Command: {process_info.command}\n")
                log_file_handle.write(f"PID: {process_info.pid}\n")
                log_file_handle.write(f"Started: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(process_info.start_time))}\n")
                log_file_handle.write(f"==================\n\n")
                log_file_handle.flush()

            for line in iter(process_info.process.stdout.readline, ""):
                if line:
                    stripped_line = line.strip()
                    process_info.output.append(stripped_line)

                    # Write to log file if configured
                    if log_file_handle:
                        log_file_handle.write(line)
                        log_file_handle.flush()

                # Check if process has ended
                if process_info.process.poll() is not None:
                    break
        except Exception as e:
            self.log_error(f"Error capturing output for process {process_id}: {e}")
        finally:
            # Close log file
            if log_file_handle:
                log_file_handle.write(f"\n=== Process Ended ===\n")
                log_file_handle.write(f"Ended: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                log_file_handle.close()

            process_info.is_running = False
            self.log_debug(f"Process {process_id} finished")

    def cleanup_finished_processes(self):
        """Remove finished processes from tracking"""
        finished = []
        for process_id, proc_info in self.processes.items():
            if not proc_info.is_running:
                try:
                    proc_info.process.poll()
                    if proc_info.process.returncode is not None:
                        finished.append(process_id)
                except (OSError, AttributeError):
                    finished.append(process_id)

        for process_id in finished:
            del self.processes[process_id]
            self.log_debug(f"Cleaned up finished process {process_id}")

    def destroy_node(self):
        """Clean up all processes and shutdown"""
        self.log_info("Shutting down ProcessApi - killing all managed processes")
        self.kill_all_processes()
        super().destroy_node()
