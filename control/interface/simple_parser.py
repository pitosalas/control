#!/usr/bin/env python3
"""
Simple Command Parser for Robot Control CLI
Author: Pito Salas and Claude Code
Open Source Under MIT license
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


ABBREV_TO_FULL = {
    # Command groups
    "m": "move",
    "t": "turn",
    "r": "robot",
    "lch": "launch",
    "c": "config",
    "sys": "system",
    "scr": "script",
    "map": "map",
    # Movement subcommands
    "fwd": "forward",
    "bak": "backward",
    "dis": "distance",
    "tim": "time",
    # Turn subcommands
    "clk": "clockwise",
    "ccw": "counterclockwise",
    "deg": "degrees",
    "rad": "radians",
    # Robot subcommands
    "stp": "stop",
    "sts": "status",
    # Launch subcommands
    "lst": "list",
    "inf": "info",
    "sta": "start",
    "kil": "kill",
    "doc": "doctor",
    # Config subcommands
    "set": "set",
    "get": "get",
    # System subcommands
    "top": "topics",
    "ps": "ps",
    "lau": "launches",
    # Script subcommands
    "sqr": "square",
    "rot": "rotate_stress",
    "cir": "circle_stress",
    # Map subcommands
    "sav": "save",
    "ser": "serialize",
    # Special commands
    "hlp": "help",
    "q": "exit",
    "x": "exit",
}

FULL_NAMES = set(ABBREV_TO_FULL.values())


# ============================================================================
# DATA STRUCTURES
# ============================================================================


@dataclass
class ParsedCommand:
    command: str
    subcommand: Optional[str]
    arguments: List[Any]


@dataclass
class ParseError:
    message: str
    input_text: str
    position: Optional[int]

class SimpleCommandParser:
    def __init__(self):
        self.abbrev_to_full = ABBREV_TO_FULL
        self.full_names = FULL_NAMES

    def resolve_keyword(self, word: str) -> str:
        # If it's already a full name, return it
        if word in self.full_names:
            return word

        # If it's an abbreviation, return full name
        if word in self.abbrev_to_full:
            return self.abbrev_to_full[word]

        # Unknown keyword - return as-is (error handling elsewhere)
        return word

    def parse_value(self, value_str: str) -> Any:
        # Try boolean
        if value_str.lower() in ("true", "yes", "1"):
            return True
        if value_str.lower() in ("false", "no", "0"):
            return False

        # Try int
        try:
            # Check if it contains decimal point
            if "." not in value_str:
                return int(value_str)
        except ValueError:
            pass

        # Try float
        try:
            return float(value_str)
        except ValueError:
            pass

        # Return as string
        return value_str

    def parse(self, input_text: str) -> tuple[ParsedCommand | None, ParseError | None]:
        tokens = input_text.strip().split()

        if not tokens:
            return None, ParseError("Empty command", input_text, None)

        command = self.resolve_keyword(tokens[0])

        if len(tokens) == 1:
            return ParsedCommand(command, None, []), None

        resolved_token = self.resolve_keyword(tokens[1])
        is_keyword = resolved_token in self.full_names or resolved_token != tokens[1]

        if is_keyword:
            subcommand = resolved_token
            arguments = [self.parse_value(arg) for arg in tokens[2:]]
            return ParsedCommand(command, subcommand, arguments), None
        arguments = [self.parse_value(arg) for arg in tokens[1:]]
        return ParsedCommand(command, None, arguments), None
