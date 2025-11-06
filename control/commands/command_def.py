#!/usr/bin/env python3
"""
Command Definition - Dataclass for command registry definitions
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

from __future__ import annotations

from dataclasses import dataclass

import control.commands.parameter_def as pd


@dataclass
class CommandDef:
    method_name: str
    parameters: list[pd.ParameterDef]
    description: str
    group: str
