#!/usr/bin/env python3
from dataclasses import dataclass
from typing import List

from .parameter_def import ParameterDef


@dataclass
class CommandDef:
    """Definition of a dispatchable command."""

    method_name: str
    parameters: List[ParameterDef]
    description: str = ""
    group: str = "general"
