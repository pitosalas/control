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
                ParameterDef("launch_type", str, description="Launch type (nav, slam, map)"),
                ParameterDef("use_sim_time", bool, required=False, default=False,
                           description="Use simulation time (use --sim-time flag)"),
                ParameterDef("map_name", str, required=False,
                           description="Map name for map launch type - use --map-name option (without extension)")
            ],
            description="Start a launch process by type (e.g., launch start map --map-name map5)",
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
        ),
        "launch.doctor": CommandDef(
            method_name="launch_doctor",
            parameters=[],
            description="Diagnose launch process conflicts and suggest fixes",
            group="launch"
        ),
        "launch.kill-all": CommandDef(
            method_name="launch_kill_all",
            parameters=[
                ParameterDef("launch_type", str, description="Launch type to kill all instances of")
            ],
            description="Kill ALL instances of a launch type (tracked + external)",
            group="launch"
        )
    }