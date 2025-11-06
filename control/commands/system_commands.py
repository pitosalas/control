#!/usr/bin/env python3
"""
System command definitions for configuration, processes, and scripts.

Author: Pito Salas and Claude Code
Open Source Under MIT license
"""
from __future__ import annotations

import control.commands.command_def as cd
import control.commands.parameter_def as pd


def build_system_commands() -> dict[str, cd.CommandDef]:
    return {
        "script.list": cd.CommandDef(
            method_name="script_list",
            parameters=[],
            description="List all available scripts",
            group="script"
        ),
        "script.square": cd.CommandDef(
            method_name="script_square",
            parameters=[
                pd.ParameterDef("meters", float, True, None, "Size of square in meters")
            ],
            description="Execute square movement pattern",
            group="script"
        ),
        "script.rotate_stress": cd.CommandDef(
            method_name="script_rotate_stress",
            parameters=[],
            description="Run continuous rotation stress test",
            group="script"
        ),
        "script.circle_stress": cd.CommandDef(
            method_name="script_circle_stress",
            parameters=[
                pd.ParameterDef("diameter", float, True, None, "Diameter of circle in meters")
            ],
            description="Run continuous circle stress test with specified diameter",
            group="script"
        ),
        "config.set": cd.CommandDef(
            method_name="set_variable",
            parameters=[
                pd.ParameterDef("name", str, True, None, "Variable name"),
                pd.ParameterDef("value", str, True, None, "Variable value")
            ],
            description="Set configuration variable",
            group="config"
        ),
        "config.get": cd.CommandDef(
            method_name="get_variable",
            parameters=[
                pd.ParameterDef("name", str, True, None, "Variable name")
            ],
            description="Get configuration variable",
            group="config"
        ),
        "config.list": cd.CommandDef(
            method_name="get_all_variables",
            parameters=[],
            description="List all configuration variables",
            group="config"
        ),
        "system.topics": cd.CommandDef(
            method_name="list_topics",
            parameters=[],
            description="List active ROS topics",
            group="system"
        ),
        "system.ps": cd.CommandDef(
            method_name="list_ros_processes",
            parameters=[],
            description="List all ROS-related processes on the system",
            group="system"
        ),
        "system.kill": cd.CommandDef(
            method_name="kill_ros_process",
            parameters=[
                pd.ParameterDef("pid", int, True, None, "Process ID (PID) to kill")
            ],
            description="Kill a ROS process by PID",
            group="system"
        ),
        "system.launches": cd.CommandDef(
            method_name="list_launch_processes",
            parameters=[],
            description="List all ros2 launch processes on the system",
            group="system"
        )
    }
