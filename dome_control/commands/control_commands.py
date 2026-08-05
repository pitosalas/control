#!/usr/bin/env python3
"""
Control Commands - Robot control command definitions
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""
import dome_control.commands.command_def as cd
import dome_control.commands.parameter_def as pd


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
        "robot.speak": cd.CommandDef(
            method_name="speak_text",
            parameters=[
                pd.ParameterDef("text", str, True, None, "Text for robot to speak")
            ],
            description="Speak text aloud via speech output",
            group="control",
        ),
        "robot.subsystems": cd.CommandDef(
            method_name="get_subsystems_status",
            parameters=[],
            description="Report running/not-running status for each subsystem "
                         "(gendrv/nav/semantic/control/mission/vision)",
            group="control",
        ),
    }
