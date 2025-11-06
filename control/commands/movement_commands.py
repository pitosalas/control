#!/usr/bin/env python3
"""
Movement command definitions for robot control.

Author: Pito Salas and Claude Code
Open Source Under MIT license
"""
from __future__ import annotations

import control.commands.command_def as cd
import control.commands.parameter_def as pd


def build_movement_commands() -> dict[str, cd.CommandDef]:
    return {
        "move.distance": cd.CommandDef(
            method_name="move_distance",
            parameters=[
                pd.ParameterDef("distance", float, True, None, "Distance in meters")
            ],
            description="Move robot a specific distance",
            group="movement"
        ),
        "move.time": cd.CommandDef(
            method_name="move_for_time",
            parameters=[
                pd.ParameterDef("seconds", float, True, None, "Duration in seconds")
            ],
            description="Move robot for specified time duration",
            group="movement"
        ),
        "turn.time": cd.CommandDef(
            method_name="turn_for_time",
            parameters=[
                pd.ParameterDef("seconds", float, True, None, "Duration in seconds")
            ],
            description="Turn robot for specified time duration",
            group="movement"
        ),
        "turn.radians": cd.CommandDef(
            method_name="turn_radians",
            parameters=[
                pd.ParameterDef("radians", float, True, None, "Angle in radians")
            ],
            description="Turn robot by specified angle in radians",
            group="movement"
        ),
        "turn.degrees": cd.CommandDef(
            method_name="turn_degrees",
            parameters=[
                pd.ParameterDef("degrees", float, True, None, "Angle in degrees")
            ],
            description="Turn robot by specified angle in degrees",
            group="movement"
        ),
        "move.forward": cd.CommandDef(
            method_name="move_forward",
            parameters=[
                pd.ParameterDef("meters", float, True, None, "Distance in meters")
            ],
            description="Move robot forward by distance",
            group="movement"
        ),
        "move.backward": cd.CommandDef(
            method_name="move_backward",
            parameters=[
                pd.ParameterDef("meters", float, True, None, "Distance in meters")
            ],
            description="Move robot backward by distance",
            group="movement"
        ),
        "turn.clockwise": cd.CommandDef(
            method_name="turn_clockwise",
            parameters=[
                pd.ParameterDef("degrees", float, True, None, "Angle in degrees")
            ],
            description="Turn robot clockwise by angle in degrees",
            group="movement"
        ),
        "turn.counterclockwise": cd.CommandDef(
            method_name="turn_counterclockwise",
            parameters=[
                pd.ParameterDef("degrees", float, True, None, "Angle in degrees")
            ],
            description="Turn robot counterclockwise by angle in degrees",
            group="movement"
        )
    }
