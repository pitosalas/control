#!/usr/bin/env python3
from typing import Dict
from .command_def import CommandDef
from .parameter_def import ParameterDef


def build_control_commands() -> Dict[str, CommandDef]:
    """Build control command definitions."""
    return {
        "robot.stop": CommandDef(
            method_name="stop_robot",
            parameters=[],
            description="Stop robot movement",
            group="control"
        ),
        "robot.status": CommandDef(
            method_name="get_robot_status",
            parameters=[],
            description="Get current robot status",
            group="control"
        )
    }