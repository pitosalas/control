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
            parameters=[],
            description="Save current map to maps/ folder (uses map_name variable)",
            group="map"
        ),
        "map.list": cd.CommandDef(
            method_name="list_maps",
            parameters=[],
            description="List available maps in maps/ folder",
            group="map"
        ),
        "map.serialize": cd.CommandDef(
            method_name="map_serialize",
            parameters=[],
            description="Save current map in SLAM Toolbox serialized format (uses map_name variable)",
            group="map"
        ),
    }
