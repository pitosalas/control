#!/usr/bin/env python3
import subprocess
import threading
import uuid
import os
import signal
import time
from typing import Dict, Optional, List
from dataclasses import dataclass
from .base_api import BaseApi
from ..commands.config_manager import ConfigManager

@dataclass
class ProcessInfo:
    process: subprocess.Popen
    command: str
    output: List[str]
    is_running: bool
    pid: int
    start_time: float

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
        default_params={"use_sim_time": "false"}
    ),
    "slam": LaunchConfig(
        launch_type="slam",
        command_template="ros2 launch slam_toolbox online_async_launch.py {params}",
        description="SLAM mapping and localization",
        default_params={"use_sim_time": "false"}
    ),
    "map_server": LaunchConfig(
        launch_type="map_server",
        command_template="ros2 run nav2_map_server map_server {params}",
        description="Map server for loading and saving maps",
        default_params={}
    )
}

class ProcessApi(BaseApi):
    """
    ROS2 process management API for launching and controlling external processes.
    Handles launch files, shell commands, and process lifecycle management.
    """

    def __init__(self, config_manager: ConfigManager = None):
        super().__init__('process_api', config_manager)
        self.processes: Dict[str, ProcessInfo] = {}

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

        # Merge default params with provided params
        final_params = config.default_params.copy()
        final_params.update({k: str(v) for k, v in params.items()})

        # Format parameters and build command
        params_str = self._format_launch_params(final_params)
        command = config.command_template.format(params=params_str).strip()

        self.log_info(f"Launching {launch_type}: {command}")
        return self.launch_command(command)

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

    def launch_command(self, command: str) -> str:
        """Launch shell command and return process ID"""
        process_id = str(uuid.uuid4())

        self.log_info(f"Launching: {command}")

        try:
            proc = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setpgrp  # Create process group for clean killing
            )

            process_info = ProcessInfo(
                process=proc,
                command=command,
                output=[],
                is_running=True,
                pid=proc.pid,
                start_time=time.time()
            )

            # Start output capture thread
            threading.Thread(
                target=self._capture_output,
                args=(process_id, process_info),
                daemon=True
            ).start()

            self.processes[process_id] = process_info
            self.log_info(f"Process launched with ID: {process_id}, PID: {proc.pid}")
            return process_id

        except Exception as e:
            self.log_error(f"Failed to launch command '{command}': {e}")
            raise

    def save_map(self, filename: str) -> str:
        """Save current map using map_saver_cli"""
        command = f"ros2 run nav2_map_server map_saver_cli -f {filename}"
        return self.launch_command(command)

    def load_map(self, map_file: str) -> str:
        """Load a map using nav2_map_server"""
        command = f"ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={map_file}"
        return self.launch_command(command)

    def start_navigation(self, use_sim_time: bool = False, **kwargs) -> str:
        """Start navigation stack with optional parameters"""
        params = []
        params.append(f"use_sim_time:={str(use_sim_time).lower()}")

        for key, value in kwargs.items():
            params.append(f"{key}:={value}")

        param_string = " ".join(params)
        command = f"ros2 launch nav2_bringup navigation_launch.py {param_string}"
        return self.launch_command(command)

    def start_slam(self, use_sim_time: bool = False, **kwargs) -> str:
        """Start SLAM toolbox"""
        params = []
        params.append(f"use_sim_time:={str(use_sim_time).lower()}")

        for key, value in kwargs.items():
            params.append(f"{key}:={value}")

        param_string = " ".join(params)
        command = f"ros2 launch slam_toolbox online_async_launch.py {param_string}"
        return self.launch_command(command)

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
            self.log_info(f"Process {process_id} already stopped")
            return True

        try:
            self.log_info(f"Terminating process {process_id}: {proc_info.command}")

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
                self.log_info(f"Process {process_id} terminated gracefully")
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

    def get_process_output(self, process_id: str, lines: Optional[int] = None) -> List[str]:
        """Get captured output from process"""
        if process_id not in self.processes:
            return []

        output = self.processes[process_id].output
        if lines is None:
            return output.copy()
        return output[-lines:] if lines > 0 else []

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
                except:
                    proc_info.is_running = False

            result[process_id] = {
                'command': proc_info.command,
                'pid': proc_info.pid,
                'is_running': proc_info.is_running,
                'start_time': proc_info.start_time,
                'runtime': time.time() - proc_info.start_time,
                'output_lines': len(proc_info.output)
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
        except:
            proc_info.is_running = False
            return False

        return True

    def wait_for_process(self, process_id: str, timeout: Optional[float] = None) -> bool:
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
        """Capture output in background thread"""
        try:
            for line in iter(process_info.process.stdout.readline, ''):
                if line:
                    process_info.output.append(line.strip())
                # Check if process has ended
                if process_info.process.poll() is not None:
                    break
        except Exception as e:
            self.log_error(f"Error capturing output for process {process_id}: {e}")
        finally:
            process_info.is_running = False
            self.log_info(f"Process {process_id} finished")

    def cleanup_finished_processes(self):
        """Remove finished processes from tracking"""
        finished = []
        for process_id, proc_info in self.processes.items():
            if not proc_info.is_running:
                try:
                    proc_info.process.poll()
                    if proc_info.process.returncode is not None:
                        finished.append(process_id)
                except:
                    finished.append(process_id)

        for process_id in finished:
            del self.processes[process_id]
            self.log_info(f"Cleaned up finished process {process_id}")

    def destroy_node(self):
        """Clean up all processes and shutdown"""
        self.log_info("Shutting down ProcessApi - killing all managed processes")
        self.kill_all_processes()
        super().destroy_node()