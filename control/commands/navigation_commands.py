#!/usr/bin/env python3
"""
Navigation Commands - Map management command definitions
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""
from __future__ import annotations

import control.commands.command_def as cd
import control.commands.parameter_def as pd


def build_navigation_commands() -> dict[str, cd.CommandDef]:
    return {
        "map.save": cd.CommandDef(
            method_name="map_save",
            parameters=[
                pd.ParameterDef("map_name", str, False, None, "Map name (without extension, defaults to config map_name)")
            ],
            description="Save current map to maps/ folder",
            group="map"
        ),
        "map.list": cd.CommandDef(
            method_name="list_maps",
            parameters=[],
            description="List available maps in maps/ folder",
            group="map"
        ),
    }
