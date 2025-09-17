#!/usr/bin/env python3
from typing import Dict
from .command_def import CommandDef
from .parameter_def import ParameterDef


def build_navigation_commands() -> Dict[str, CommandDef]:
    """Build navigation command definitions."""
    return {
        "nav.start": CommandDef(
            method_name="start_navigation_stack",
            parameters=[
                ParameterDef("use_sim_time", bool, required=False, default=False,
                           description="Use simulation time")
            ],
            description="Start the navigation stack",
            group="navigation"
        ),
        "nav.stop": CommandDef(
            method_name="kill_navigation_stack",
            parameters=[],
            description="Stop the navigation stack",
            group="navigation"
        )
    }