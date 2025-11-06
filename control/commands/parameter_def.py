#!/usr/bin/env python3
"""
Parameter Definition - Dataclass for command parameter definitions
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ParameterDef:
    name: str
    param_type: type
    required: bool
    default: object
    description: str
