#!/usr/bin/env python3
"""
Simple Command Parser for Robot Control CLI
Author: Pito Salas and Claude Code
Open Source Under MIT license

Design Philosophy:
    Replace Click with a simpler, custom parser that:
    1. Handles the specific pattern: command [subcommand] [value...]
    2. Supports abbreviations for ALL keywords (commands AND subcommands)
    3. Naturally handles negative numbers (no special flags)
    4. Provides clear, simple code without decorator boilerplate
    5. Uses the existing CommandDispatcher backend

Command Pattern:
    All commands follow one of these patterns:
    - command subcommand value(s)    e.g., "move forward 1.5", "turn clockwise 90"
    - command subcommand              e.g., "robot stop", "config list"
    - command value(s)                e.g., "help move", "get name"

Abbreviation System:
    Single dictionary maps ALL keywords to abbreviations:
    - Commands: move->mov, turn->trn, robot->rob, config->cfg, etc.
    - Subcommands: forward->fwd, backward->bak, clockwise->clk, etc.

    This allows flexible combinations:
    - "move forward 1.5"  (full command + full subcommand)
    - "mov forward 1.5"   (abbr command + full subcommand)
    - "move fwd 1.5"      (full command + abbr subcommand)
    - "mov fwd 1.5"       (abbr command + abbr subcommand)
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

# ============================================================================
# ABBREVIATIONS - Single source of truth for all keyword abbreviations
# ============================================================================

ABBREVIATIONS = {
    # Command groups
    "move": "m",
    "turn": "t",
    "robot": "r",
    "launch": "lch",
    "config": "c",
    "system": "sys",
    "script": "scr",
    "map": "map",  # Already 3 letters
    # Movement subcommands
    "forward": "fwd",
    "backward": "bak",
    "distance": "dis",
    "time": "tim",
    # Turn subcommands
    "clockwise": "clk",
    "counterclockwise": "ccw",
    "degrees": "deg",
    "radians": "rad",
    # Robot subcommands
    "stop": "stp",
    "status": "sts",
    # Launch subcommands
    "list": "lst",
    "start": "sta",
    "kill": "kil",
    "doctor": "doc",
    # Config subcommands
    "set": "set",  # Already 3 letters
    "get": "get",  # Already 3 letters
    # System subcommands
    "topics": "top",
    # Script subcommands
    "square": "sqr",
    "stress_test": "str",
    # Map subcommands
    "save": "sav",
    # Special commands
    "help": "hlp",
    "exit": "ext",
}

# Create reverse lookup: abbreviation -> full name
FULL_NAMES = {abbr: full for full, abbr in ABBREVIATIONS.items()}


# ============================================================================
# DATA STRUCTURES
# ============================================================================


@dataclass
class ParsedCommand:
    """
    Represents a parsed command ready for dispatch.

    Examples:
        "move forward 1.5" -> ParsedCommand("move", "forward", [1.5])
        "robot stop"       -> ParsedCommand("robot", "stop", [])
        "help move"        -> ParsedCommand("help", None, ["move"])
    """

    command: str  # Full command name (e.g., "move", not "mov")
    subcommand: Optional[str]  # Full subcommand name or None
    arguments: List[Any]  # Parsed argument values (int, float, str, bool)

    def to_dispatcher_format(self) -> Tuple[str, Dict[str, Any]]:
        """
        Convert to CommandDispatcher format.

        Returns:
            (command_name, params_dict)

        Examples:
            ParsedCommand("move", "forward", [1.5])
                -> ("move.forward", {"meters": 1.5})
            ParsedCommand("robot", "stop", [])
                -> ("robot.stop", {})
        """
        # This is a simplified example - actual implementation would need
        # to map arguments to parameter names based on command definitions
        if self.subcommand:
            command_name = f"{self.command}.{self.subcommand}"
        else:
            command_name = self.command

        # Map positional arguments to named parameters
        # (Would use command definitions to get parameter names)
        params = {}
        if self.arguments:
            # Simplified - real version would look up param names
            params = {"arg": self.arguments[0]} if len(self.arguments) == 1 else {}

        return command_name, params


@dataclass
class ParseError:
    """Represents a parsing error with helpful message."""

    message: str
    input_text: str
    position: Optional[int]


# ============================================================================
# PARSER
# ============================================================================


class SimpleCommandParser:
    """
    Simple command parser that handles the robot control command syntax.

    Key Features:
        - Resolves abbreviations automatically
        - Handles negative numbers naturally (they're just values)
        - Simple, readable code
        - Clear error messages
        - No decorator magic
    """

    def __init__(self):
        """Initialize the parser with abbreviation tables."""
        self.abbreviations = ABBREVIATIONS
        self.full_names = FULL_NAMES

    def resolve_keyword(self, word: str) -> str:
        """
        Resolve a keyword to its full name (handles abbreviations).

        Args:
            word: Keyword that might be full or abbreviated

        Returns:
            Full keyword name

        Examples:
            "move" -> "move"
            "mov"  -> "move"
            "fwd"  -> "forward"
        """
        # If it's already a full name, return it
        if word in self.abbreviations:
            return word

        # If it's an abbreviation, return full name
        if word in self.full_names:
            return self.full_names[word]

        # Unknown keyword - return as-is (error handling elsewhere)
        return word

    def parse_value(self, value_str: str) -> Any:
        """
        Parse a value string to appropriate type.

        Handles: int, float, bool, str
        Naturally handles negative numbers!

        Args:
            value_str: String representation of value

        Returns:
            Parsed value with correct type

        Examples:
            "90"    -> 90 (int)
            "1.5"   -> 1.5 (float)
            "-90"   -> -90 (int)  # No special handling needed!
            "-1.5"  -> -1.5 (float)
            "true"  -> True (bool)
            "nav"   -> "nav" (str)
        """
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

    def _is_keyword(self, word: str) -> bool:
        """Check if word is a known keyword (full or abbreviated)."""
        resolved = self.resolve_keyword(word)
        return resolved in self.abbreviations or resolved != word

    def _parse_arguments(self, tokens: List[str]) -> List[Any]:
        """Parse list of token strings into typed values."""
        return [self.parse_value(arg) for arg in tokens]

    def parse(
        self, input_text: str
    ) -> Tuple[Optional[ParsedCommand], Optional[ParseError]]:
        """Parse a command line into a ParsedCommand."""
        tokens = input_text.strip().split()

        if not tokens:
            return None, ParseError("Empty command", input_text, None)

        command = self.resolve_keyword(tokens[0])

        if len(tokens) == 1:
            return ParsedCommand(command, None, []), None

        if self._is_keyword(tokens[1]):
            subcommand = self.resolve_keyword(tokens[1])
            arguments = self._parse_arguments(tokens[2:])
            return ParsedCommand(command, subcommand, arguments), None
        else:
            arguments = self._parse_arguments(tokens[1:])
            return ParsedCommand(command, None, arguments), None

    def parse_tokens(
        self, tokens: List[str]
    ) -> Tuple[Optional[ParsedCommand], Optional[ParseError]]:
        """
        Parse pre-tokenized command.

        Useful for testing and when tokens are already split.

        Args:
            tokens: List of command tokens

        Returns:
            Same as parse()
        """
        return self.parse(" ".join(tokens))


# ============================================================================
# USAGE EXAMPLES
# ============================================================================


def demonstrate_parser():
    """
    Demonstrate the simple parser vs Click.

    This function shows how the simple parser handles all the cases
    that Click struggles with.
    """
    parser = SimpleCommandParser()

    test_cases = [
        # Full names
        "move forward 1.5",
        "turn clockwise 90",
        "robot stop",
        # Abbreviated commands
        "mov forward 1.5",
        "trn clockwise 90",
        "rob stop",
        # Abbreviated subcommands
        "move fwd 1.5",
        "turn clk 90",
        "robot stp",
        # Both abbreviated
        "mov fwd 1.5",
        "trn clk 90",
        "rob stp",
        # Negative numbers (no special handling needed!)
        "move forward -1.5",
        "turn clockwise -90",
        "mov fwd -2",
        "trn clk -45",
        # Multiple arguments
        "config set linear_speed 0.5",
        "cfg set linear_speed 0.5",
        "launch start nav --sim-time",
        "lch sta nav --sim-time",
    ]

    print("Simple Parser Demonstration")
    print("=" * 60)
    print()

    for cmd in test_cases:
        result, error = parser.parse(cmd)
        if error:
            print(f"✗ '{cmd}' -> ERROR: {error.message}")
        else:
            print(f"✓ '{cmd}'")
            print(
                f"  -> command={result.command}, subcommand={result.subcommand}, args={result.arguments}"
            )
        print()


if __name__ == "__main__":
    demonstrate_parser()
