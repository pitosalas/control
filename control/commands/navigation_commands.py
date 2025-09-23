#!/usr/bin/env python3
from typing import Dict
from .command_def import CommandDef
from .parameter_def import ParameterDef


def build_navigation_commands() -> Dict[str, CommandDef]:
    """Build navigation command definitions - now only contains map commands."""
    return {
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
        ),
    }