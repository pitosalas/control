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

class ProcessApi(BaseApi):
    """
    ROS2 process management API for launching and controlling external processes.
    Handles launch files, shell commands, and process lifecycle management.
    """

    def __init__(self, config_manager: ConfigManager = None):
        super().__init__("process_api", config_manager)
        self.processes: Dict[str, ProcessInfo] = {}

        # Load launch templates from config
        self._load_launch_configs()

    def _load_launch_configs(self):
        """Load launch configurations from config file"""
        templates = self.config.get_launch_templates() if self.config else {}

        if templates:
            # Build LaunchConfig objects from config file
            self.launch_configs = {}
            for launch_type, template in templates.items():
                self.launch_configs[launch_type] = LaunchConfig(
                    launch_type=launch_type,
                    command_template=template.get("command", ""),
                    description=template.get("description", ""),
                    default_params=template.get("default_params", {})
                )
        else:
            # No launch templates found in config
            self.launch_configs = {}
            self.log_warn("No launch_templates found in config file. Launch commands will not be available.")

    def get_available_launch_types(self) -> List[str]:
        """Get list of available launch types"""
        return list(self.launch_configs.keys())

    def get_launch_config(self, launch_type: str) -> Optional[LaunchConfig]:
        """Get launch configuration for a specific type"""
        return self.launch_configs.get(launch_type)

    def _format_launch_params(self, params: Dict[str, str], format_type: str = "ros2") -> str:
        """
        Format parameters for launch command

        Args:
            params: Dictionary of parameters
            format_type: 'ros2' for 'key:=value' format, 'cli' for '--key value' format
        """
        if not params:
            return ""
        param_parts = []
        for key, value in params.items():
            if format_type == "cli":
                param_parts.append(f"--{key} {value}")
            else:  # ros2 format
                param_parts.append(f"{key}:={value}")
        return " ".join(param_parts)

    def launch_by_type(self, launch_type: str, **params) -> str:
        """Launch a process by launch type with optional parameters"""
        config = self.get_launch_config(launch_type)
        if not config:
            raise ValueError(f"Unknown launch type: {launch_type}")

        # Start with the base command from template
        command = config.command_template

        # Merge default params with provided params (filter out None values)
        final_params = config.default_params.copy()
        final_params.update({k: str(v) for k, v in params.items() if v is not None})

        # Handle map parameter - resolve to full path if it's just a filename
        if "map" in final_params:
            map_value = final_params["map"]
            # If it's not already an absolute path, resolve it in maps directory
            if not map_value.startswith("/"):
                maps_dir = self.config.get_maps_dir()
                # Remove .yaml extension if present, we'll add it
                map_name = map_value.replace(".yaml", "")
                map_file = maps_dir / f"{map_name}.yaml"
                if map_file.exists():
                    final_params["map"] = str(map_file.resolve())
                else:
                    self.log_warn(f"Map file not found: {map_file}, using as-is")

        # Handle map_name parameter for map server launch type
        if launch_type == "map" and "map_name" in params:
            map_name = params.pop("map_name")
            maps_dir = self.config.get_maps_dir()
            map_file = maps_dir / f"{map_name}.yaml"
            if not map_file.exists():
                raise ValueError(f"Map file not found: {map_file}")
            map_file_abs = map_file.resolve()
            command += f" --ros-args -p yaml_filename:={map_file_abs}"
            # Remove map_name from final_params if it exists
            final_params.pop("map_name", None)

        # Add parameters to command
        if final_params:
            # Detect if command is a "bl" (better_launch) command (starts with two letters "bl")
            is_bl_command = command.strip().split()[0] == "bl"

            if is_bl_command:
                # For bl commands, use CLI format: --param value
                params_str = self._format_launch_params(final_params, format_type="cli")
                if params_str:
                    command += f" {params_str}"
            elif "ros2 run" in command:
                # For ros2 run commands, use --ros-args -p format
                params_str = self._format_launch_params(final_params, format_type="ros2")
                if params_str:
                    if "--ros-args" not in command:
                        command += " --ros-args"
                    for param in params_str.split():
                        command += f" -p {param}"
            else:
                # For ros2 launch commands, use param:=value format
                params_str = self._format_launch_params(final_params, format_type="ros2")
                if params_str:
                    command += f" {params_str}"

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

    def run_command_sync(self, command: str, log_name: Optional[str] = None, timeout: float = 30.0) -> tuple[bool, str, Optional[str]]:
        """
        Run a command synchronously and wait for completion.
        Returns (success, output, log_file_path) tuple.

        Args:
            command: Shell command to run
            log_name: Optional name for log file (e.g., 'map_save')
            timeout: Timeout in seconds (default 30)

        Returns:
            Tuple of (success: bool, output: str, log_file_path: Optional[str])
        """
        self.log_debug(f"Running command: {command}")

        # Create log file if log_name is provided
        log_file_path = None
        if log_name:
            self.config.ensure_subdirs()
            log_dir_config = self.config.get_variable("log_dir") or "logs"
            logs_dir = self.config.resolve_path(log_dir_config)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            log_file_path = str(logs_dir / f"{log_name}_{timestamp}.log")

        try:
            start_time = time.time()
            result = subprocess.run(
                command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout
            )

            output = result.stdout + result.stderr
            success = result.returncode == 0

            # Write to log file if configured
            if log_file_path:
                try:
                    with open(log_file_path, 'w') as f:
                        f.write(f"=== Command Log ===\n")
                        f.write(f"Command: {command}\n")
                        f.write(f"Started: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(start_time))}\n")
                        f.write(f"Return Code: {result.returncode}\n")
                        f.write(f"==================\n\n")
                        f.write(output)
                        f.write(f"\n=== Command Completed ===\n")
                        f.write(f"Ended: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                except IOError as e:
                    self.log_warn(f"Failed to write log file {log_file_path}: {e}")

            if success:
                self.log_debug(f"Command completed successfully")
            else:
                self.log_error(f"Command failed with return code {result.returncode}")

            return (success, output, log_file_path)

        except subprocess.TimeoutExpired:
            error_msg = f"Command timed out after {timeout} seconds"
            self.log_error(error_msg)
            return (False, error_msg, log_file_path)
        except Exception as e:
            error_msg = f"Failed to run command: {e}"
            self.log_error(error_msg)
            return (False, error_msg, log_file_path)

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
                start_new_session=True,  # Detach from terminal and create new process group
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

    def _capture_output(self, process_id: str, process_info: ProcessInfo):
        """Capture initial output then detach from process"""
        log_file_handle = None
        max_initial_lines = 20  # Capture first 20 lines then detach
        lines_captured = 0

        try:
            # Open log file if configured
            if process_info.log_file:
                log_file_handle = open(process_info.log_file, 'w')
                log_file_handle.write(f"=== Process Log ===\n")
                log_file_handle.write(f"Command: {process_info.command}\n")
                log_file_handle.write(f"PID: {process_info.pid}\n")
                log_file_handle.write(f"Started: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(process_info.start_time))}\n")
                log_file_handle.write(f"==================\n\n")
                log_file_handle.write(f">> Initial output (first {max_initial_lines} lines):\n\n")
                log_file_handle.flush()

            # Capture initial output with timeout
            start_time = time.time()
            timeout_seconds = 3.0  # Wait up to 3 seconds for initial output

            for line in iter(process_info.process.stdout.readline, ""):
                # Check timeout
                if time.time() - start_time > timeout_seconds:
                    break

                if line:
                    stripped_line = line.strip()
                    process_info.output.append(stripped_line)

                    # Write to log file if configured
                    if log_file_handle:
                        log_file_handle.write(line)
                        log_file_handle.flush()

                    lines_captured += 1

                    # Stop after capturing initial lines
                    if lines_captured >= max_initial_lines:
                        break

                # Check if process has ended early
                if process_info.process.poll() is not None:
                    break

        except Exception as e:
            self.log_error(f"Error capturing output for process {process_id}: {e}")
        finally:
            # Close log file with detachment notice
            if log_file_handle:
                log_file_handle.write(f"\n{'='*50}\n")
                log_file_handle.write(f"Process detached from control.py monitoring.\n")
                log_file_handle.write(f"The process continues to run independently (PID: {process_info.pid}).\n")
                log_file_handle.write(f"Check system logs for continued output.\n")
                log_file_handle.write(f"Detached at: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                log_file_handle.write(f"{'='*50}\n")
                log_file_handle.close()

            # Close stdout to fully detach
            try:
                process_info.process.stdout.close()
            except:
                pass

            # Mark as no longer being monitored (but still running)
            self.log_debug(f"Process {process_id} detached after capturing {lines_captured} lines")

    def destroy_node(self):
        """Clean up all processes and shutdown"""
        self.log_info("Shutting down ProcessApi - killing all managed processes")
        self.kill_all_processes()
        super().destroy_node()
