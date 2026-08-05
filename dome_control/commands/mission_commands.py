#!/usr/bin/env python3
"""
Mission Commands - dome_mission (Nav2 + exploration) command definitions
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""
import dome_control.commands.command_def as cd
import dome_control.commands.parameter_def as pd


def build_mission_commands() -> dict[str, cd.CommandDef]:
    return {
        "mission.go.start": cd.CommandDef(
            method_name="publish_intent_navigation_go",
            parameters=[pd.ParameterDef("label", str, True, None, "Object label to navigate to")],
            description="Navigate to nearest confirmed object with given label",
            group="mission"
        ),
        "mission.go.stop": cd.CommandDef(
            method_name="publish_intent_navigation_cancel",
            parameters=[],
            description="Cancel current navigation goal",
            group="mission"
        ),
        "mission.explore.start": cd.CommandDef(
            method_name="publish_intent_exploration_start",
            parameters=[],
            description="Start autonomous frontier exploration",
            group="mission"
        ),
        "mission.explore.stop": cd.CommandDef(
            method_name="publish_intent_exploration_stop",
            parameters=[],
            description="Stop autonomous frontier exploration",
            group="mission"
        ),
        "mission.explore.status": cd.CommandDef(
            method_name="explore_status",
            parameters=[],
            description="Read current /explore/status topic value",
            group="mission"
        ),
    }
