#!/usr/bin/env python3
"""Robot Controller - Orchestrates robot operations and APIs.

Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

import subprocess
import time
from dataclasses import dataclass

from dome_control.commands.config_manager import ConfigManager
from dome_control.ros2_api.calibration_api import CalibrationApi
from dome_control.ros2_api.intent_api import IntentApi
from dome_control.ros2_api.movement_api import MovementApi
from dome_control.ros2_api.process_api import CommandConfig, ProcessApi
from dome_control.ros2_api.speech_api import SpeechApi
from dome_control.ros2_api.survey_api import SurveyApi


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
        self.movement_node = None
        self.calibration_node = None
        self.process_node = None
        self.intent_node = None
        self.speech_node = None
        self.survey_node = None
        self.launch_process_ids = dict.fromkeys(self.config.get_launch_templates().keys())

    @property
    def movement(self) -> MovementApi:
        if self.movement_node is None:
            self.movement_node = MovementApi(self.config)
        return self.movement_node

    @property
    def calibration(self) -> CalibrationApi:
        if self.calibration_node is None:
            self.calibration_node = CalibrationApi(self.movement, self.config)
        return self.calibration_node

    @property
    def process(self) -> ProcessApi:
        if self.process_node is None:
            self.process_node = ProcessApi(self.config)
        return self.process_node

    @property
    def intent(self) -> IntentApi:
        if self.intent_node is None:
            self.intent_node = IntentApi(self.config)
            # allow DDS publisher-subscriber discovery before first publish
            time.sleep(0.5)
        return self.intent_node

    @property
    def survey(self) -> SurveyApi:
        if self.survey_node is None:
            self.survey_node = SurveyApi()
        return self.survey_node

    @property
    def speech(self) -> SpeechApi:
        if self.speech_node is None:
            self.speech_node = SpeechApi(self.config)
            time.sleep(0.5)
        return self.speech_node

    def is_launch_running(self, launch_type: str) -> bool:
        process_id = self.launch_process_ids.get(launch_type)
        if not process_id:
            return False
        return self.process.is_process_running(process_id)

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

        info_lines = []
        info_lines.append(f"Description: {config.description}")
        info_lines.append(f"Command: {config.command_template}")

        return CommandResponse(True, "\n".join(info_lines))

    def launch_start(self, launch_type: str, **kwargs) -> CommandResponse:
        if self.is_launch_running(launch_type):
            return CommandResponse(False, f"{launch_type} is already running, stop it first")
        try:
            process_id = self.process.launch_by_type(launch_type, **kwargs)
            self.launch_process_ids[launch_type] = process_id
            return CommandResponse(True, f"Started {launch_type}", {"process_id": process_id})
        except ValueError as e:
            return CommandResponse(False, str(e))

    def launch_stop(self, launch_type: str) -> CommandResponse:
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

    def get_launch_status(self):
        status = {}
        for launch_type, process_id in self.launch_process_ids.items():
            if process_id and self.process.is_process_running(process_id):
                status[launch_type] = {
                    "running": True,
                    "process_id": process_id,
                    "pid": self.process.get_process_pid(process_id),
                }
            else:
                if process_id:
                    self.launch_process_ids[launch_type] = None
                status[launch_type] = {
                    "running": False,
                    "process_id": None,
                    "pid": None,
                }
        return status

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

    def speak_text(self, text: str) -> CommandResponse:
        self.speech.speak(text)
        return CommandResponse(True, f"Speaking: {text!r}")

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

    def map_path(self):
        """Return (map_name, maps_dir, full_path) or None if map_name not set."""
        map_name = self.config.get_variable("map_name")
        if not map_name:
            return None, None, None
        self.config.ensure_subdirs()
        maps_dir = self.config.get_maps_dir()
        return map_name, maps_dir, maps_dir / map_name

    def run_map_command(self, config: CommandConfig, success_msg: str, error_prefix: str) -> CommandResponse:
        success, output, log_file = self.process.run_command_sync(config)

        if success:
            msg = success_msg + (f"\nLog: {log_file}" if log_file else "")
            return CommandResponse(True, msg)

        if "timed out" in output.lower():
            error_msg = f"{error_prefix} timeout after 15 seconds"
        else:
            error_lines = [line for line in output.split("\n") if line.strip()]
            error_msg = f"{error_prefix}: {error_lines[-1]}" if error_lines else error_prefix

        if log_file:
            error_msg += f"\nSee log: {log_file}"
        return CommandResponse(False, error_msg)

    def map_save(self) -> CommandResponse:
        map_name, maps_dir, full_path = self.map_path()
        if not map_name:
            return CommandResponse(False, "map_name variable not set. Use 'config set map_name <name>' first.")
        cmd = f"ros2 run nav2_map_server map_saver_cli -f {full_path} --ros-args -p save_map_timeout:=10000."
        config = CommandConfig(command=cmd, log_name="map_save", timeout=15.0)
        return self.run_map_command(config,
                                    f"Map saved to {maps_dir}/{map_name}.yaml and {map_name}.pgm",
                                    "Failed to save map")

    def map_serialize(self) -> CommandResponse:
        map_name, _, full_path = self.map_path()
        if not map_name:
            return CommandResponse(False, "map_name variable not set. Use 'config set map_name <name>' first.")
        cmd = (f"ros2 service call /slam_toolbox/serialize_map "
               f"slam_toolbox/srv/SerializePoseGraph "
               f'"{{filename: \'{full_path}\'}}"')
        config = CommandConfig(command=cmd, log_name="map_serialize", timeout=15.0)
        return self.run_map_command(config,
                                    f"Map serialized to {full_path}.posegraph and {map_name}.data",
                                    "Failed to serialize map")

    def list_maps(self) -> CommandResponse:
        self.config.ensure_subdirs()
        maps_dir = self.config.get_maps_dir()

        all_files = list(maps_dir.glob("*"))

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

    def get_robot_status(self) -> CommandResponse:
        linear_speed = self.config.get_variable("linear_speed")
        angular_speed = self.config.get_variable("angular_speed")

        nodes_status = {
            "movement_api": "running" if self.movement_node else "not available",
            "calibration_api": "running" if self.calibration_node else "not available",
            "process_api": "running" if self.process_node else "not available",
        }

        status = {
            "speeds": {"linear": linear_speed, "angular": angular_speed},
            "processes": self.get_launch_status(),
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
        return self.process.list_ros_processes()

    def list_launch_processes(self) -> CommandResponse:
        return self.process.list_launch_processes()

    def kill_ros_process(self, pid: int) -> CommandResponse:
        return self.process.kill_ros_process(pid)

    def publish_intent(self, name: str, slots: dict) -> CommandResponse:
        self.intent.publish(name, "cli", slots)
        return CommandResponse(True, f"Intent published: {name}")

    def publish_intent_stop(self) -> CommandResponse:
        return self.publish_intent("stop", {})

    def publish_intent_explore(self) -> CommandResponse:
        return self.publish_intent("explore", {})

    def publish_intent_describe_scene(self) -> CommandResponse:
        return self.publish_intent("describe_scene", {})

    def publish_intent_count_objects(self, object_type: str) -> CommandResponse:
        return self.publish_intent("count_objects", {"object_type": object_type})

    def publish_intent_navigation_go(self, label: str) -> CommandResponse:
        return self.publish_intent("navigation_go", {"label": label})

    def publish_intent_navigation_cancel(self) -> CommandResponse:
        return self.publish_intent("navigation_cancel", {})

    def publish_intent_exploration_start(self) -> CommandResponse:
        return self.publish_intent("exploration_start", {})

    def publish_intent_exploration_stop(self) -> CommandResponse:
        return self.publish_intent("exploration_stop", {})

    def explore_status(self) -> CommandResponse:
        try:
            result = subprocess.run(
                ["ros2", "topic", "echo", "--once", "/explore/status"],
                capture_output=True, text=True, timeout=3.0
            )
            value = result.stdout.strip().replace("data: ", "").strip("'\"")
            if value:
                return CommandResponse(True, f"explore status: {value}")
            return CommandResponse(False, "No status received from /explore/status (explore_manager_node running?)")
        except subprocess.TimeoutExpired:
            return CommandResponse(False, "Timeout reading /explore/status — explore_manager_node not running")

    def start_survey(self) -> CommandResponse:
        success, message = self.survey.start()
        return CommandResponse(success, message)

    def get_subsystems_status(self) -> CommandResponse:
        subsystems = self.process.get_subsystem_status()
        subsystems["mission"]["state"] = self.process.get_mission_state()
        return CommandResponse(True, "Subsystem status", {"subsystems": subsystems})
