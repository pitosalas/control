#!/usr/bin/env python3
import os
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass
class CommandConfig:
    command: str
    log_name: Optional[str]
    timeout: float

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

from dome_control.commands.config_manager import ConfigManager
from dome_control.ros2_api.base_api import BaseApi

SUBSYSTEM_KEYWORDS = {
    "gendrv": ["micro_ros_agent"],
    "nav": ["slam_manager_node", "explorer_manager_node", "slam_toolbox", "bt_navigator", "amcl"],
    "semantic": ["semantic_map_node"],
    "control": ["behavior_manager"],
    "mission": ["mission_node"],
    "vision": ["dome_vision_ros_node"],
}


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

    def __init__(self, config_manager: ConfigManager):
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

    def _format_launch_params(self, params: Dict[str, str], format_type: str) -> str:
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

        command = config.command_template
        final_params = config.default_params.copy()
        final_params.update({k: str(v) for k, v in params.items() if v is not None})

        # Handle map parameter - resolve to full path if it's just a filename
        if "map" in final_params:
            map_value = final_params["map"]
            if map_value.startswith("/"):
                pass
            else:
                maps_dir = self.config.get_maps_dir()
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
            final_params.pop("map_name", None)

        # Add parameters to command (flattened logic)
        if not final_params:
            self.log_debug(f"Launching {launch_type}: {command}")
            return self.launch_command(command, log_name=launch_type)

        is_bl_command = command.strip().split()[0] == "bl"
        params_str = ""
        if is_bl_command:
            params_str = self._format_launch_params(final_params, format_type="cli")
            if params_str:
                command += f" {params_str}"
        elif "ros2 run" in command:
            params_str = self._format_launch_params(final_params, format_type="ros2")
            if params_str:
                if "--ros-args" not in command:
                    command += " --ros-args"
                for param in params_str.split():
                    command += f" -p {param}"
        else:
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

    def get_launch_status(self, launch_type: str) -> Dict:
        """Get status of launch processes"""
        if launch_type:
            # Return status for specific launch type (requires external tracking)
            # This will be handled by RobotController which tracks process IDs
            return {"launch_type": launch_type, "status": "unknown"}
        else:
            # Return status for all tracked processes
            return self.get_running_processes()

    def run_command_sync(self, config: CommandConfig) -> tuple[bool, str, Optional[str]]:
        """
        Run a command synchronously and wait for completion.
        Returns (success, output, log_file_path) tuple.
        Args:
            config: CommandConfig dataclass with command, log_name, timeout
        Returns:
            Tuple of (success: bool, output: str, log_file_path: Optional[str])
        """
        self.log_debug(f"Running command: {config.command}")

        log_file_path = None
        if config.log_name:
            self.config.ensure_subdirs()
            log_dir_config = self.config.get_variable("log_dir") or "logs"
            logs_dir = self.config.resolve_path(log_dir_config)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            log_file_path = str(logs_dir / f"{config.log_name}_{timestamp}.log")

        try:
            start_time = time.time()
            result = subprocess.run(
                config.command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=config.timeout
            )

            output = result.stdout + result.stderr
            success = result.returncode == 0

            if log_file_path:
                try:
                    with open(log_file_path, 'w') as f:
                        f.write(f"=== Command Log ===\n")
                        f.write(f"Command: {config.command}\n")
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
            error_msg = f"Command timed out after {config.timeout} seconds"
            self.log_error(error_msg)
            return (False, error_msg, log_file_path)
        except Exception as e:
            error_msg = f"Failed to run command: {e}"
            self.log_error(error_msg)
            return (False, error_msg, log_file_path)

    def launch_command(self, command: str, log_name: Optional[str]) -> str:
        process_id = str(uuid.uuid4())

        self.log_debug(f"Launching: {command}")

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
                stdin=subprocess.DEVNULL,  # Detach stdin from terminal
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
            self.processes[process_id] = process_info
            if log_file_path:
                self.log_debug(f"Process launched with ID: {process_id}, PID: {proc.pid}, Log: {log_file_path}")
            else:
                self.log_debug(f"Process launched with ID: {process_id}, PID: {proc.pid}")

            # Always start a thread to capture output and print to console in real time
            output_thread = threading.Thread(target=self._capture_output, args=(process_id, process_info), daemon=True)
            output_thread.start()

            return process_id

        except Exception as e:
            self.log_error(f"Failed to launch command '{command}': {e}")
            if log_file_path:
                try:
                    with open(log_file_path, 'w') as f:
                        f.write(f"Failed to launch command: {e}\n")
                    self.log_debug(f"Exception info written to log file: {log_file_path}")
                except Exception as ex:
                    self.log_warn(f"Failed to write exception log file: {ex}")
            raise

    def kill_process(self, process_id: str) -> bool:
        """Gracefully stop process with Ctrl+C (SIGINT), capture all output, and write to log file."""
        if process_id not in self.processes:
            self.log_warn(f"Process ID {process_id} not found")
            return False

        proc_info = self.processes[process_id]
        if not proc_info.is_running:
            self.log_debug(f"Process {process_id} already stopped")
            return True

        try:
            self.log_debug(f"Sending SIGINT to process {process_id}: {proc_info.command}")
            proc_info.process.send_signal(signal.SIGINT)
            stdout, _ = proc_info.process.communicate()
            proc_info.is_running = False

            if proc_info.log_file:
                try:
                    with open(proc_info.log_file, 'w') as f:
                        f.write(stdout)
                    self.log_debug(f"Output written to log file: {proc_info.log_file}")
                except Exception as e:
                    self.log_warn(f"Failed to write shutdown log file: {e}")

            self.log_debug(f"Process {process_id} terminated gracefully with SIGINT")
            return True
        except Exception as e:
            self.log_error(f"Error stopping process {process_id}: {e}")
            return False

    def kill_all_processes(self) -> int:
        """Kill all managed processes"""
        killed_count = 0
        for process_id in list(self.processes.keys()):
            if self.kill_process(process_id):
                killed_count += 1
        return killed_count

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
        """Continuously capture all output, log to file, and print to console until process ends."""
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

            # Continuously read and log output until process ends
            for line in iter(process_info.process.stdout.readline, ""):
                if not line:
                    break
                stripped_line = line.rstrip()
                process_info.output.append(stripped_line)
                # Write to log file if configured
                if log_file_handle:
                    log_file_handle.write(line)
                    log_file_handle.flush()
                # Print to console
                print(f"[PROCESS {process_id}] {stripped_line}")
                # Check if process has ended
                if process_info.process.poll() is not None:
                    break

        except Exception as e:
            self.log_error(f"Error capturing output for process {process_id}: {e}")
        finally:
            # Close log file with detachment notice
            if log_file_handle:
                log_file_handle.write(f"\n{'='*50}\n")
                log_file_handle.write(f"Process output capture complete.\n")
                log_file_handle.write(f"Ended at: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                log_file_handle.write(f"{'='*50}\n")
                log_file_handle.close()
            # Close stdout
            try:
                process_info.process.stdout.close()
            except:
                pass
            self.log_debug(f"Process {process_id} output capture finished.")

    def get_process_pid(self, process_id: str) -> int | str:
        info = self.processes.get(process_id)
        return info.pid if info else "unknown"

    def kill_ros_process(self, pid: int) -> "CommandResponse":
        from dome_control.commands.robot_controller import CommandResponse
        try:
            ps_result = subprocess.run(
                ["ps", "-o", "pid,pgid", "-p", str(pid)],
                check=False, capture_output=True, text=True, timeout=2,
            )
            is_group_leader = False
            if ps_result.returncode == 0:
                lines = ps_result.stdout.strip().split("\n")
                if len(lines) > 1:
                    parts = lines[1].split()
                    if len(parts) >= 2:
                        is_group_leader = parts[0].strip() == parts[1].strip()

            target = f"-{pid}" if is_group_leader else str(pid)
            label = f"process group {pid}" if is_group_leader else f"process {pid}"
            try:
                subprocess.run(["kill", "-TERM", target], check=True, timeout=2)
                return CommandResponse(True, f"Sent SIGTERM to {label}\nRun 'system ps' to verify stopped")
            except subprocess.CalledProcessError:
                subprocess.run(["kill", "-KILL", target], check=True, timeout=2)
                return CommandResponse(True, f"Force killed {label} (SIGKILL)")

        except subprocess.CalledProcessError:
            return CommandResponse(False, f"Failed to kill {pid}. May require sudo")
        except subprocess.TimeoutExpired:
            return CommandResponse(False, "Kill command timed out")
        except OSError as e:
            return CommandResponse(False, f"Error killing process: {e}")

    def _parse_ps_aux_line(self, line: str) -> dict | None:
        parts = line.split(None, 10)
        if len(parts) < 11:
            return None
        return {"pid": parts[1], "cpu": parts[2], "mem": parts[3], "command": parts[10]}

    def list_ros_processes(self) -> "CommandResponse":
        from dome_control.commands.robot_controller import CommandResponse
        try:
            result = subprocess.run(
                ["ps", "aux"], check=False, capture_output=True, text=True, timeout=5
            )
            if result.returncode != 0:
                return CommandResponse(False, "Failed to list processes")

            ros_keywords = ["ros2", "nav2", "slam", "rviz", "gazebo", "amcl", "map_server", "robot_state"]
            process_info = []
            for line in result.stdout.split("\n")[1:]:
                if not any(kw in line.lower() for kw in ros_keywords):
                    continue
                info = self._parse_ps_aux_line(line)
                if info is None:
                    continue
                command = info["command"]
                if len(command) > 101:
                    command = command[:98] + "..."
                process_info.append({**info, "command": command})

            if not process_info:
                return CommandResponse(True, "No ROS processes found")

            lines = ["#  PID     CPU  MEM  COMMAND", "-" * 120]
            for i, proc in enumerate(process_info, 1):
                lines.append(f"{i:<2} {proc['pid']:<7} {proc['cpu']:>4} {proc['mem']:>4}  {proc['command']}")
            lines += ["-" * 120, f"Total: {len(process_info)} ROS processes", "Use 'system kill <PID>' to kill a process"]
            return CommandResponse(True, "\n".join(lines))

        except subprocess.TimeoutExpired:
            return CommandResponse(False, "Process listing timed out")
        except Exception as e:
            return CommandResponse(False, f"Error listing processes: {e}")

    def get_subsystem_status(self) -> dict[str, dict]:
        """Report running/not-running and matching processes for each of
        SUBSYSTEM_KEYWORDS, from a single `ps aux` snapshot."""
        status = {name: {"running": False, "processes": []} for name in SUBSYSTEM_KEYWORDS}
        try:
            result = subprocess.run(
                ["ps", "aux"], check=False, capture_output=True, text=True, timeout=5
            )
        except subprocess.TimeoutExpired:
            return status
        if result.returncode != 0:
            return status

        for line in result.stdout.split("\n")[1:]:
            info = self._parse_ps_aux_line(line)
            if info is None:
                continue
            line_lower = line.lower()
            for name, keywords in SUBSYSTEM_KEYWORDS.items():
                if any(kw in line_lower for kw in keywords):
                    status[name]["running"] = True
                    status[name]["processes"].append(info)

        return status

    def get_mission_state(self, timeout_s: float = 1.0) -> str:
        """Read the latest /mission/state value (transient-local String),
        waiting up to timeout_s. Returns "unknown" if nothing arrives —
        e.g. dome_mission not running, or running without the publisher."""
        received = {"value": None}

        def on_state(msg: String) -> None:
            received["value"] = msg.data

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        subscription = self.create_subscription(String, "/mission/state", on_state, qos)
        try:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline and received["value"] is None:
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_subscription(subscription)

        return received["value"] if received["value"] is not None else "unknown"

    def list_launch_processes(self) -> "CommandResponse":
        from dome_control.commands.robot_controller import CommandResponse
        try:
            result = subprocess.run(
                ["ps", "axo", "pid,pgid,ppid,user,%cpu,%mem,command"],
                check=False, capture_output=True, text=True, timeout=5,
            )
            if result.returncode != 0:
                return CommandResponse(False, "Failed to list processes")

            all_processes = {}
            for line in result.stdout.split("\n")[1:]:
                if "ros2" in line.lower() and "launch" in line.lower():
                    parts = line.split(None, 6)
                    if len(parts) >= 7:
                        all_processes[parts[0]] = {
                            "pid": parts[0], "pgid": parts[1], "ppid": parts[2],
                            "cpu": parts[4], "mem": parts[5], "command": parts[6],
                        }

            parent_processes = []
            for pid, proc in all_processes.items():
                if proc["ppid"] in all_processes:
                    continue
                command = proc["command"]
                if len(command) > 94:
                    command = command[:91] + "..."
                parent_processes.append({**proc, "command": command})

            if not parent_processes:
                return CommandResponse(True, "No ros2 launch processes found")

            lines = ["#  PID     PGID    CPU  MEM  COMMAND", "-" * 120]
            for i, proc in enumerate(parent_processes, 1):
                lines.append(f"{i:<2} {proc['pid']:<7} {proc['pgid']:<7} {proc['cpu']:>4} {proc['mem']:>4}  {proc['command']}")
            lines += ["-" * 120,
                      f"Total: {len(parent_processes)} launch processes (parent processes only)",
                      "Use 'system kill <PID>' to kill (kills process group if leader)"]
            return CommandResponse(True, "\n".join(lines))

        except subprocess.TimeoutExpired:
            return CommandResponse(False, "Process listing timed out")
        except Exception as e:
            return CommandResponse(False, f"Error listing processes: {e}")

    def destroy_node(self):
        """Clean up all processes and shutdown"""
        self.log_info("Shutting down ProcessApi - killing all managed processes")
        self.kill_all_processes()
        super().destroy_node()
