#!/usr/bin/env python3
from typing import Dict
from .command_def import CommandDef
from .parameter_def import ParameterDef


def build_movement_commands() -> Dict[str, CommandDef]:
    """Build movement command definitions."""
    return {
        "move.distance": CommandDef(
            method_name="move_distance",
            parameters=[
                ParameterDef("distance", float, description="Distance in meters")
            ],
            description="Move robot a specific distance",
            group="movement"
        ),
        "move.time": CommandDef(
            method_name="move_for_time",
            parameters=[
                ParameterDef("seconds", float, description="Duration in seconds")
            ],
            description="Move robot for specified time duration",
            group="movement"
        ),
        "turn.time": CommandDef(
            method_name="turn_for_time",
            parameters=[
                ParameterDef("seconds", float, description="Duration in seconds")
            ],
            description="Turn robot for specified time duration",
            group="movement"
        ),
        "turn.radians": CommandDef(
            method_name="turn_radians",
            parameters=[
                ParameterDef("radians", float, description="Angle in radians")
            ],
            description="Turn robot by specified angle in radians",
            group="movement"
        ),
        "turn.degrees": CommandDef(
            method_name="turn_degrees",
            parameters=[
                ParameterDef("degrees", float, description="Angle in degrees")
            ],
            description="Turn robot by specified angle in degrees",
            group="movement"
        ),
        "move.forward": CommandDef(
            method_name="move_forward",
            parameters=[
                ParameterDef("meters", float, description="Distance in meters")
            ],
            description="Move robot forward by distance",
            group="movement"
        ),
        "move.backward": CommandDef(
            method_name="move_backward",
            parameters=[
                ParameterDef("meters", float, description="Distance in meters")
            ],
            description="Move robot backward by distance",
            group="movement"
        ),
        "turn.clockwise": CommandDef(
            method_name="turn_clockwise",
            parameters=[
                ParameterDef("degrees", float, description="Angle in degrees")
            ],
            description="Turn robot clockwise by angle in degrees",
            group="movement"
        ),
        "turn.counterclockwise": CommandDef(
            method_name="turn_counterclockwise",
            parameters=[
                ParameterDef("degrees", float, description="Angle in degrees")
            ],
            description="Turn robot counterclockwise by angle in degrees",
            group="movement"
        )
    }