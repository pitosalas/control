#!/usr/bin/env python3
"""
Control Commands - Robot control command definitions
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""
from __future__ import annotations

import control.commands.command_def as cd


def build_control_commands() -> dict[str, cd.CommandDef]:
    return {
        "robot.stop": cd.CommandDef(
            method_name="stop_robot",
            parameters=[],
            description="Stop robot movement",
            group="control",
        ),
        "robot.status": cd.CommandDef(
            method_name="get_robot_status",
            parameters=[],
            description="Get current robot status",
            group="control",
        ),
    }
