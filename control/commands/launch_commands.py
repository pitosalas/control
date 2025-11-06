#!/usr/bin/env python3
"""
Launch command definitions for unified launch management.

Author: Pito Salas and Claude Code
Open Source Under MIT license
"""
from __future__ import annotations

import control.commands.command_def as cd
import control.commands.parameter_def as pd


def build_launch_commands() -> dict[str, cd.CommandDef]:
    return {
        "launch.list": cd.CommandDef(
            method_name="launch_list",
            parameters=[],
            description="List all available launch templates from config",
            group="launch"
        ),
        "launch.info": cd.CommandDef(
            method_name="launch_info",
            parameters=[
                pd.ParameterDef("launch_type", str, True, None, "Launch type to show info for")
            ],
            description="Show detailed information about a launch template",
            group="launch"
        ),
        "launch.start": cd.CommandDef(
            method_name="launch_start",
            parameters=[
                pd.ParameterDef("launch_type", str, True, None, "Launch type (nav, slam, map)"),
                pd.ParameterDef("use_sim_time", bool, False, False,
                              "Use simulation time (use --sim-time flag)"),
                pd.ParameterDef("map", str, False, None,
                              "Map filename (without extension) in maps directory"),
                pd.ParameterDef("map_name", str, False, None,
                              "Map name for map launch type - use --map-name option (without extension)")
            ],
            description="Start a launch process by type (e.g., launch start nav --map basement)",
            group="launch"
        ),
        "launch.stop": cd.CommandDef(
            method_name="launch_stop",
            parameters=[
                pd.ParameterDef("launch_type", str, True, None, "Launch type to stop (nav, slam, map)")
            ],
            description="Stop a running launch process by type",
            group="launch"
        )
    }
