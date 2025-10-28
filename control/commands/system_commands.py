#!/usr/bin/env python3
from typing import Dict
from .command_def import CommandDef
from .parameter_def import ParameterDef


def build_system_commands() -> Dict[str, CommandDef]:
    """Build system command definitions."""
    return {
        "script.square": CommandDef(
            method_name="script_square",
            parameters=[
                ParameterDef("meters", float, description="Size of square in meters")
            ],
            description="Execute square movement pattern",
            group="script"
        ),
        "script.stress_test": CommandDef(
            method_name="script_stress_test",
            parameters=[],
            description="Run continuous stress test with voltage monitoring",
            group="script"
        ),
        "process.running": CommandDef(
            method_name="is_process_running",
            parameters=[
                ParameterDef("process_id", str, description="Process ID to check")
            ],
            description="Check if process is running",
            group="process"
        ),
        "process.kill": CommandDef(
            method_name="kill_process",
            parameters=[
                ParameterDef("process_id", str, description="Process ID to kill")
            ],
            description="Kill a process by ID",
            group="process"
        ),
        "config.set": CommandDef(
            method_name="set_variable",
            parameters=[
                ParameterDef("name", str, description="Variable name"),
                ParameterDef("value", str, description="Variable value")
            ],
            description="Set configuration variable",
            group="config"
        ),
        "config.get": CommandDef(
            method_name="get_variable",
            parameters=[
                ParameterDef("name", str, description="Variable name")
            ],
            description="Get configuration variable",
            group="config"
        ),
        "config.list": CommandDef(
            method_name="get_all_variables",
            parameters=[],
            description="List all configuration variables",
            group="config"
        ),
        "system.topics": CommandDef(
            method_name="list_topics",
            parameters=[],
            description="List active ROS topics",
            group="system"
        )
    }