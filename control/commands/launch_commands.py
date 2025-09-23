#!/usr/bin/env python3
from typing import Dict
from .command_def import CommandDef
from .parameter_def import ParameterDef


def build_launch_commands() -> Dict[str, CommandDef]:
    """Build launch command definitions for unified launch management."""
    return {
        "launch.list": CommandDef(
            method_name="launch_list",
            parameters=[],
            description="List all available launch types with status",
            group="launch"
        ),
        "launch.start": CommandDef(
            method_name="launch_start",
            parameters=[
                ParameterDef("launch_type", str, description="Launch type (nav, slam, map_server)"),
                ParameterDef("use_sim_time", bool, required=False, default=False,
                           description="Use simulation time")
            ],
            description="Start a launch process by type",
            group="launch"
        ),
        "launch.kill": CommandDef(
            method_name="launch_kill",
            parameters=[
                ParameterDef("launch_type", str, description="Launch type to stop")
            ],
            description="Stop a launch process by type",
            group="launch"
        ),
        "launch.status": CommandDef(
            method_name="launch_status",
            parameters=[
                ParameterDef("launch_type", str, required=False,
                           description="Specific launch type (optional)")
            ],
            description="Show status of launch processes",
            group="launch"
        )
    }