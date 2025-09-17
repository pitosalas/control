#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Any


@dataclass
class ParameterDef:
    """Definition of a command parameter."""
    name: str
    param_type: type
    required: bool = True
    default: Any = None
    description: str = ""