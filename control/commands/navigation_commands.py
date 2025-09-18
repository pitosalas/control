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
        ),
        "slam.start": CommandDef(
            method_name="start_slam",
            parameters=[
                ParameterDef("use_sim_time", bool, required=False, default=False,
                           description="Use simulation time")
            ],
            description="Start SLAM",
            group="slam"
        ),
        "slam.stop": CommandDef(
            method_name="stop_slam",
            parameters=[],
            description="Stop SLAM",
            group="slam"
        ),
        "map.save": CommandDef(
            method_name="save_current_map",
            parameters=[
                ParameterDef("filename", str, description="Map filename")
            ],
            description="Save current map to maps/ folder",
            group="map"
        ),
        "map.list": CommandDef(
            method_name="list_maps",
            parameters=[],
            description="List available maps in maps/ folder",
            group="map"
        ),
        "map.load": CommandDef(
            method_name="load_map",
            parameters=[
                ParameterDef("filename", str, description="Map filename (without extension)")
            ],
            description="Load a map from maps/ folder",
            group="map"
        )
    }